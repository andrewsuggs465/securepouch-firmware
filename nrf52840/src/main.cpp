/*
 * SecurePouch — nRF52840 firmware (BLE relay / application brain)
 * ===============================================================
 *
 * Role: application brain. Owns the arm / lock / alarm state machine and every
 * local peripheral. The nRF9151 is a radio co-processor (RCP) reached over
 * Serial1; it does cellular + GNSS and forwards FMD server commands to us. We
 * never talk to the network directly.
 *
 * Control reaches the state machine from three places, all funnelled through
 * applyCommand():
 *   1. BLE  — companion app writes an SP_CTRL_* opcode to the CONTROL char.
 *   2. LTE  — FMD server command, polled by the 9151, arrives as "CMD:<str>".
 *   3. Local— RFID owner tag toggles lock/arm; accelerometer tamper / BLE
 *             dead-man arm the alarm.
 *
 * Breadboard pinout (confirmed 2026-06-22) — see PIN_* below and
 * firmware/shared/sp_uart_protocol.h for the inter-MCU contract.
 */

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <SPI.h>
#include <MFRC522.h>
#include <mbed.h>

#include "ble_protocol.h"
#include "sp_uart_protocol.h"
#include "ble_bond_store.h"

// ---------------------------------------------------------------------------
// Device configuration
// ---------------------------------------------------------------------------
// Must match the username of this unit's account on the FMD server. Change per
// unit before flashing.
static const char* DEVICE_UID = "securepouch-001";
static const char* FIRMWARE_ROLE = "nRF52840 BLE relay";

static const uint16_t GAP_APPEARANCE_GENERIC_TAG = 0x0200;

// ---------------------------------------------------------------------------
// Pin map  (Arduino Nano 33 BLE "Dx" labels)
// ---------------------------------------------------------------------------
static const uint8_t PIN_BUZZER     = 2;   // D2  passive piezo buzzer bridge leg A
static const uint8_t PIN_BUZZER_GUARD = 3; // D3  passive piezo buzzer bridge leg B
static const uint8_t PIN_LED_R      = 4;   // D4  RGB status LED (active high)
static const uint8_t PIN_LED_G      = 5;   // D5
static const uint8_t PIN_LED_B      = 6;   // D6
static const uint8_t PIN_SERVO      = 7;   // D7  hobby servo signal (demo latch)
static const uint8_t PIN_ACTUATOR_SPARE = 8; // D8 held LOW; spare / future solenoid driver
static const uint8_t PIN_RFID_RST   = 9;   // D9  RC522 RST
static const uint8_t PIN_RFID_SS    = 10;  // D10 RC522 SDA/SS
// RC522 SPI bus uses Nano 33 BLE hardware SPI pins:
//   D11/MOSI -> RC522 MOSI, D12/MISO <- RC522 MISO, D13/SCK -> RC522 SCK.
static const uint8_t PIN_NRF9151_RST = 17; // D17/A3 -> nRF9151 RESET (active low)
// Motion sensing uses the Nano 33 BLE's onboard LSM9DS1 IMU on the internal
// I2C bus (Wire1, P0.30/P0.31) via the Arduino_LSM9DS1 library — no external
// accelerometer and no analog pins needed. A0/A1/A2 and D18/D19 are free.

// Polarity / tuning constants ------------------------------------------------
static const bool LED_ACTIVE_HIGH      = true;   // external common-cathode RGB
static const bool BUZZER_IDLE_HIGH     = false;  // passive piezo: idle LOW
static const uint16_t SIREN_FREQ_HZ    = 2000;   // common Arduino kit piezo buzzer range
static const uint32_t SIREN_HALF_PERIOD_US = 1000000UL / (SIREN_FREQ_HZ * 2UL);
// Servo pulse widths in microseconds (standard hobby servo: 1000–2000 µs).
// Adjust SERVO_LOCK_US / SERVO_UNLOCK_US to match your specific servo travel.
static const uint32_t SERVO_UNLOCK_US  = 1000;   // 0° (retracted/unlocked)
static const uint32_t SERVO_LOCK_US    = 2000;   // 180° (extended/locked)
static const uint32_t SERVO_PERIOD_US  = 20000;  // 50 Hz standard servo frame
static const float    TAMPER_THRESHOLD = 0.12f;  // demo: hand motion trips while armed
static const uint32_t DEADMAN_GRACE_MS = 30000;  // BLE-lost -> alarm delay
static const uint32_t STATUS_PERIOD_MS = 10000;  // push STAT to 9151 cadence
static const uint32_t ALARM_TEST_MS    = 10000;  // web/app/console alarm self-test duration
static const uint32_t ALARM_STROBE_MS  = 120;    // buzzer + red LED flash cadence

// ---------------------------------------------------------------------------
// Peripherals
// ---------------------------------------------------------------------------
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
// Servo driven via mbed::PwmOut for a stable 50Hz signal on the hardware PWM
// peripheral. The mbed Servo library uses software timers at 3.3V which many
// servos reject; PwmOut drives the NRF52840 PWM module directly.
mbed::PwmOut* servoPin = nullptr;
bool rfidOk = false;
bool rfidEnrolled = false;
byte rfidOwnerUid[10] = {0};
byte rfidOwnerUidLen = 0;
bool imuOk = false;

// ---------------------------------------------------------------------------
// BLE service / characteristics
// ---------------------------------------------------------------------------
BLEService spService(SP_SERVICE_UUID);
BLEStringCharacteristic deviceIdChar(SP_CHAR_DEVICE_ID_UUID, BLERead, SP_DEVICE_ID_MAX_LEN);
BLEByteCharacteristic statusChar(SP_CHAR_STATUS_UUID, BLERead | BLENotify);
BLEByteCharacteristic controlChar(SP_CHAR_CONTROL_UUID, BLEWrite);

BLEService batteryService("180F");
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);

// ---------------------------------------------------------------------------
// Device state
// ---------------------------------------------------------------------------
struct PouchState {
  bool locked  = false;   // IBET-style demo: boot visibly unlocked
  bool armed   = false;
  bool alarm   = false;
  bool tamper  = false;
  uint8_t battery = 100;
};
PouchState g_state;

bool g_bleConnected = false;
uint32_t g_bleLostAt = 0;       // millis() when central disconnected while armed

// IMU resting acceleration magnitude (g), captured at boot when undisturbed.
// At rest this is ~1.0 (gravity); tamper = a large transient deviation.
float g_accRestMag = 1.0f;

uint32_t g_lastStatusPush = 0;
uint32_t g_lastSirenToggle = 0;
uint32_t g_alarmStartedAt = 0;
uint32_t g_alarmAutoSilenceAfter = 0;
bool g_sirenPhase = false;
bool g_buzzerPhase = false;
uint32_t g_lastBuzzerToggleUs = 0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void applyCommand(const char* cmd, const char* source);
void pushStatusToRcp();
void uartSendLine(const char* tag, const char* payload);

// ===========================================================================
// LED / status helpers
// ===========================================================================
static void ledWrite(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, (r == LED_ACTIVE_HIGH) ? HIGH : LOW);
  digitalWrite(PIN_LED_G, (g == LED_ACTIVE_HIGH) ? HIGH : LOW);
  digitalWrite(PIN_LED_B, (b == LED_ACTIVE_HIGH) ? HIGH : LOW);
}

// Map the high-level state to a status colour.
static void updateStatusLed() {
  if (g_state.alarm)       ledWrite(true, false, false);   // red   = alarm
  else if (g_state.armed)  ledWrite(true, true, false);    // amber = armed
  else if (g_state.locked) ledWrite(false, false, true);   // blue  = locked/idle
  else                     ledWrite(false, true, false);   // green = unlocked
}

static uint8_t statusByte() {
  uint8_t b = 0;
  if (g_state.locked) b |= SP_STATUS_LOCKED;
  if (g_state.armed)  b |= SP_STATUS_ARMED;
  if (g_state.alarm)  b |= SP_STATUS_ALARM;
  if (g_state.tamper) b |= SP_STATUS_TAMPER;
  return b;
}

// Publish state everywhere that mirrors it: BLE notify + status LED.
static void publishState() {
  statusChar.writeValue(statusByte());
  batteryLevelChar.writeValue(g_state.battery);
  updateStatusLed();
}

// ===========================================================================
// Actuators: servo latch + siren
// ===========================================================================
// Write a pulse width in microseconds to the servo via the hardware PWM module.
// mbed::PwmOut uses the nRF52840 PWM peripheral so the signal is stable 50 Hz
// regardless of CPU load — fixes the 3.3V soft-timer reliability issue.
static void setServoPulse(uint32_t pulseUs, const char* label) {
  if (!servoPin) return;
  // PwmOut::pulsewidth_us sets the high time; period must be set first.
  servoPin->pulsewidth_us(pulseUs);
  Serial.print("[servo] pulse ");
  Serial.print(label);
  Serial.print(" us=");
  Serial.println(pulseUs);
}

static void setupServo() {
  // digitalPinToPinName maps Arduino D7 to the mbed PinName used by PwmOut.
  servoPin = new mbed::PwmOut(digitalPinToPinName(PIN_SERVO));
  servoPin->period_us(SERVO_PERIOD_US);
  setServoPulse(SERVO_UNLOCK_US, "boot/unlock");
}

static void sirenOff() {
  noTone(PIN_BUZZER);
  digitalWrite(PIN_BUZZER, BUZZER_IDLE_HIGH ? HIGH : LOW);
  digitalWrite(PIN_BUZZER_GUARD, LOW);
  g_sirenPhase = false;
  g_buzzerPhase = false;
}

static void serviceBuzzerBridge() {
  uint32_t nowUs = micros();
  if (nowUs - g_lastBuzzerToggleUs < SIREN_HALF_PERIOD_US) return;
  g_lastBuzzerToggleUs = nowUs;
  g_buzzerPhase = !g_buzzerPhase;
  digitalWrite(PIN_BUZZER, g_buzzerPhase ? HIGH : LOW);
  digitalWrite(PIN_BUZZER_GUARD, g_buzzerPhase ? LOW : HIGH);
}

// Non-blocking siren + strobe: call every loop while alarm is active.
// Prototype hardware is a passive piezo buzzer, not a bridge: D2 outputs a
// 2 kHz tone while the RGB LED strobes red. D3 is held LOW so old bridge wiring
// cannot fight the buzzer.
static void serviceSiren() {
  if (!g_state.alarm) return;
  uint32_t now = millis();

  if (g_alarmAutoSilenceAfter > 0 && now - g_alarmStartedAt >= g_alarmAutoSilenceAfter) {
    Serial.println("[alarm] auto-silence after remote/test trigger");
    g_state.alarm = false;
    g_alarmAutoSilenceAfter = 0;
    sirenOff();
    publishState();
    pushStatusToRcp();
    return;
  }

  serviceBuzzerBridge();

  if (now - g_lastSirenToggle >= ALARM_STROBE_MS) {
    g_lastSirenToggle = now;
    g_sirenPhase = !g_sirenPhase;
    if (g_sirenPhase) ledWrite(true, false, false);
    else              ledWrite(false, false, false);
  }
}

// ===========================================================================
// Inter-MCU UART (Serial1 <-> nRF9151)
// ===========================================================================
void uartSendLine(const char* tag, const char* payload) {
  Serial1.print(tag);
  Serial1.print(':');
  Serial1.print(payload);
  Serial1.print(SP_UART_EOL);
}

void pushStatusToRcp() {
  char buf[24];
  snprintf(buf, sizeof(buf), "%u,bat=%u", statusByte(), g_state.battery);
  uartSendLine(SP_MSG_STATUS, buf);
  g_lastStatusPush = millis();
}

static void sendEvent(const char* token) { uartSendLine(SP_MSG_EVENT, token); }

// Parse one complete line received from the nRF9151.
static void handleRcpLine(char* line) {
  char* sep = strchr(line, ':');
  if (!sep) return;                  // not a TAG:payload line — ignore (log noise)
  *sep = '\0';
  const char* tag = line;
  const char* payload = sep + 1;

  if (strcmp(tag, SP_MSG_COMMAND) == 0) {
    applyCommand(payload, "LTE");
  } else if (strcmp(tag, SP_MSG_RCP_STATUS) == 0) {
    Serial.print("[rcp] net="); Serial.println(payload);
  } else if (strcmp(tag, SP_MSG_FIX) == 0) {
    Serial.print("[rcp] fix="); Serial.println(payload);
  }
}

// Accumulate bytes from Serial1 into a line buffer; dispatch on EOL.
static void pollRcp() {
  static char buf[SP_UART_LINE_MAX];
  static size_t len = 0;
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;
    if (c == SP_UART_EOL) {
      buf[len] = '\0';
      if (len > 0) handleRcpLine(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;  // overflow — drop the line
    }
  }
}

// ===========================================================================
// Command dispatch — the single funnel for BLE / LTE / local triggers
// ===========================================================================
static void doLock() {
  setServoPulse(SERVO_LOCK_US, "lock");
  g_state.locked = true;
  sendEvent("LOCK_OK");
}
static void doUnlock() {
  setServoPulse(SERVO_UNLOCK_US, "unlock");
  g_state.locked = false;
  sendEvent("UNLOCK_OK");
}
static void doArm() {
  setServoPulse(SERVO_LOCK_US, "arm/lock");
  g_state.locked = true;
  g_state.armed = true;
  g_bleLostAt = 0;
}
static void doDisarm() {
  setServoPulse(SERVO_UNLOCK_US, "disarm/unlock");
  g_state.locked = false;
  g_state.armed = false;
  g_state.tamper = false;
  if (g_state.alarm) { g_state.alarm = false; sirenOff(); }
}
static void doAlarm(const char* source, uint32_t autoSilenceAfterMs = 0)  {
  g_state.alarm = true;
  g_alarmStartedAt = millis();
  g_alarmAutoSilenceAfter = autoSilenceAfterMs;
  g_lastSirenToggle = 0;
  Serial.print("[alarm] active from ");
  Serial.print(source);
  if (autoSilenceAfterMs > 0) {
    Serial.print(" for ");
    Serial.print(autoSilenceAfterMs / 1000);
    Serial.print("s test window");
  } else {
    Serial.print(" until silence/disarm");
  }
  Serial.println();
  sendEvent("ALARM");
  uartSendLine(SP_MSG_UPLOAD_NOW, "alarm");   // ask 9151 for a location burst
}
static void doSilence() {
  g_state.alarm = false;
  g_alarmAutoSilenceAfter = 0;
  sirenOff();
  // If still armed and BLE is lost, restart the grace window so the dead-man
  // doesn't immediately re-fire after a remote silence command.
  if (g_state.armed && !g_bleConnected) {
    g_bleLostAt = millis();
  }
}
static void doLocate()  { uartSendLine(SP_MSG_UPLOAD_NOW, "locate"); }

// Accepts either an FMD command string (SP_CMD_*) or is called by name.
void applyCommand(const char* cmd, const char* source) {
  Serial.print("[cmd] ");
  Serial.print(source);
  Serial.print(" -> ");
  Serial.println(cmd);

  if      (strcmp(cmd, SP_CMD_LOCK)    == 0) doLock();
  else if (strcmp(cmd, SP_CMD_UNLOCK)  == 0) doUnlock();
  else if (strcmp(cmd, SP_CMD_ARM)     == 0) doArm();
  else if (strcmp(cmd, SP_CMD_DISARM)  == 0) doDisarm();
  else if (strcmp(cmd, SP_CMD_ALARM)   == 0) doAlarm(source, ALARM_TEST_MS);
  else if (strcmp(cmd, SP_CMD_SILENCE) == 0) doSilence();
  else if (strcmp(cmd, SP_CMD_LOCATE)  == 0) doLocate();
  else { Serial.println("[cmd] unknown — ignored"); return; }

  uartSendLine(SP_MSG_ACK, cmd);   // tell the RCP what we executed
  pushStatusToRcp();
  publishState();
}

// Translate a BLE control opcode to the shared command string, then dispatch.
static void applyControlOpcode(uint8_t op) {
  switch (op) {
    case SP_CTRL_LOCK:    applyCommand(SP_CMD_LOCK,    "BLE"); break;
    case SP_CTRL_UNLOCK:  applyCommand(SP_CMD_UNLOCK,  "BLE"); break;
    case SP_CTRL_ARM:     applyCommand(SP_CMD_ARM,     "BLE"); break;
    case SP_CTRL_DISARM:  applyCommand(SP_CMD_DISARM,  "BLE"); break;
    case SP_CTRL_ALARM:   applyCommand(SP_CMD_ALARM,   "BLE"); break;
    case SP_CTRL_SILENCE: applyCommand(SP_CMD_SILENCE, "BLE"); break;
    case SP_CTRL_LOCATE:  applyCommand(SP_CMD_LOCATE,  "BLE"); break;
    default: Serial.print("[ble] unknown ctrl opcode "); Serial.println(op);
  }
}

// ===========================================================================
// Sensors: accelerometer tamper + RFID owner tag
// ===========================================================================
// Average the IMU acceleration-vector magnitude at rest (~1 g, gravity).
static void calibrateAccel() {
  if (!imuOk) return;
  float sum = 0.0f;
  int n = 0;
  uint32_t until = millis() + 300;
  while (millis() < until) {
    if (IMU.accelerationAvailable()) {
      float x, y, z;
      IMU.readAcceleration(x, y, z);   // g
      sum += sqrtf(x * x + y * y + z * z);
      n++;
    }
  }
  if (n > 0) g_accRestMag = sum / n;
}

// |current accel magnitude - resting magnitude| in g. A jerk / cut vibration
// spikes this above TAMPER_THRESHOLD; a still pouch reads ~0.
static float accelDisturbance() {
  if (!imuOk || !IMU.accelerationAvailable()) return 0.0f;
  float x, y, z;
  IMU.readAcceleration(x, y, z);
  return fabsf(sqrtf(x * x + y * y + z * z) - g_accRestMag);
}

static void serviceTamper() {
  if (!g_state.armed) return;
  if (accelDisturbance() > TAMPER_THRESHOLD) {
    if (!g_state.tamper) {
      g_state.tamper = true;
      Serial.println("[tamper] motion detected while armed");
      sendEvent("TAMPER");
      doAlarm("TAMPER");
      publishState();
    }
  }
}

static void printUid(const byte* uid, byte len) {
  for (byte i = 0; i < len; i++) {
    if (uid[i] < 0x10) Serial.print('0');
    Serial.print(uid[i], HEX);
    if (i + 1 < len) Serial.print(':');
  }
}

static bool rfidUidMatchesOwner() {
  if (rfid.uid.size != rfidOwnerUidLen) return false;
  for (byte i = 0; i < rfidOwnerUidLen; i++) {
    if (rfid.uid.uidByte[i] != rfidOwnerUid[i]) return false;
  }
  return true;
}

static void applyOwnerToggle(const char* source) {
  if (g_state.armed || g_state.locked || g_state.alarm) {
    Serial.println("[rfid] owner authorized -> disarm/unlock");
    applyCommand(SP_CMD_DISARM, source);   // doDisarm() unlocks + disarms + clears tamper/alarm
  } else {
    Serial.println("[rfid] owner authorized -> arm/lock");
    applyCommand(SP_CMD_ARM, source);      // doArm() locks + arms
  }
}

static void clearRfidEnrollment() {
  rfidEnrolled = false;
  rfidOwnerUidLen = 0;
  memset(rfidOwnerUid, 0, sizeof(rfidOwnerUid));
  Serial.println("[rfid] owner tag cleared; next card will enroll");
}

static void testServoSweep() {
  Serial.println("[servo] sweep test: unlock -> lock -> unlock");
  setServoPulse(SERVO_UNLOCK_US, "test/unlock");
  delay(700);
  setServoPulse(SERVO_LOCK_US, "test/lock");
  delay(700);
  setServoPulse(SERVO_UNLOCK_US, "test/unlock");
}

static void serviceRfid() {
  if (!rfidOk) return;
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;

  Serial.print("[rfid] card uid=");
  printUid(rfid.uid.uidByte, rfid.uid.size);
  Serial.println();

  if (!rfidEnrolled) {
    rfidOwnerUidLen = min((byte)sizeof(rfidOwnerUid), rfid.uid.size);
    memcpy(rfidOwnerUid, rfid.uid.uidByte, rfidOwnerUidLen);
    rfidEnrolled = true;
    Serial.print("[rfid] owner enrolled uid=");
    printUid(rfidOwnerUid, rfidOwnerUidLen);
    Serial.println();
    applyCommand(SP_CMD_ARM, "RFID");  // doArm() locks + arms in one call
  } else if (rfidUidMatchesOwner()) {
    applyOwnerToggle("RFID");
  } else {
    Serial.println("[rfid] rejected non-owner tag");
    sendEvent("RFID_REJECT");
    ledWrite(false, false, true);
    delay(250);
    publishState();
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

static void printRfidStatus() {
  if (!rfidOk) {
    Serial.println("[rfid] reader NOT found");
    return;
  }
  if (rfidEnrolled) {
    Serial.print("[rfid] owner uid=");
    printUid(rfidOwnerUid, rfidOwnerUidLen);
    Serial.println();
  } else {
    Serial.println("[rfid] reader ready; no owner enrolled");
  }
}

// Simple serial console for breadboard testing without the app.
static void serviceSerialConsole() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'l': applyCommand(SP_CMD_LOCK,    "SER"); break;
    case 'u': applyCommand(SP_CMD_UNLOCK,  "SER"); break;
    case 'a': applyCommand(SP_CMD_ARM,     "SER"); break;
    case 'd': applyCommand(SP_CMD_DISARM,  "SER"); break;
    case '!': applyCommand(SP_CMD_ALARM,   "SER"); break;
    case 's': applyCommand(SP_CMD_SILENCE, "SER"); break;
    case 'g': applyCommand(SP_CMD_LOCATE,  "SER"); break;
    case 'r': clearRfidEnrollment(); break;
    case 'v': testServoSweep(); break;
    case 'c': calibrateAccel();
              Serial.print("[imu] recalibrated rest magnitude=");
              Serial.println(g_accRestMag, 3);
              break;
    case '?': printRfidStatus(); break;
    default: break;
  }
}

// ===========================================================================
// BLE dead-man switch
// ===========================================================================
static void serviceDeadman() {
  if (!g_state.armed || g_state.alarm) return;
  if (g_bleConnected) return;          // phone present — nothing to do
  if (g_bleLostAt == 0) return;        // not armed-while-lost yet
  if (millis() - g_bleLostAt >= DEADMAN_GRACE_MS) {
    Serial.println("[deadman] phone out of range past grace — alarm");
    g_bleLostAt = 0;                   // clear so a silence+reconnect doesn't re-trigger instantly
    sendEvent("DEADMAN");
    doAlarm("DEADMAN");
    publishState();
  }
}

// ===========================================================================
// BLE event handlers
// ===========================================================================
static void onConnect(BLEDevice central) {
  g_bleConnected = true;
  g_bleLostAt = 0;
  Serial.print("[ble] connected: "); Serial.println(central.address());
  publishState();
}

static void onDisconnect(BLEDevice central) {
  g_bleConnected = false;
  if (g_state.armed) g_bleLostAt = millis();   // start dead-man grace timer
  Serial.print("[ble] disconnected: "); Serial.println(central.address());
  BLE.advertise();
  publishState();
}

static void onControlWrite(BLEDevice, BLECharacteristic) {
  applyControlOpcode(controlChar.value());
}

// ===========================================================================
// Setup
// ===========================================================================
static void setupPins() {
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUZZER_GUARD, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_ACTUATOR_SPARE, OUTPUT);
  pinMode(PIN_NRF9151_RST, OUTPUT);

  sirenOff();
  digitalWrite(PIN_ACTUATOR_SPARE, LOW);
  ledWrite(false, false, true);            // boot = blue
}

// Pull the nRF9151 RESET low briefly so the RCP boots in lock-step with us.
static void resetNrf9151() {
  digitalWrite(PIN_NRF9151_RST, LOW);
  delay(50);
  digitalWrite(PIN_NRF9151_RST, HIGH);
}

static void setupImu() {
  // Onboard LSM9DS1 on the internal I2C bus (Wire1); the library handles that.
  imuOk = IMU.begin();
  Serial.print("[imu] LSM9DS1: ");
  Serial.println(imuOk ? "ready" : "NOT found");
}

static void setupRfid() {
  SPI.begin();
  rfid.PCD_Init();
  delay(50);
  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  rfidOk = (version != 0x00 && version != 0xFF);
  Serial.print("[rfid] RC522 version=0x");
  if (version < 0x10) Serial.print('0');
  Serial.println(version, HEX);
  Serial.print("[rfid] reader: ");
  Serial.println(rfidOk ? "ready" : "NOT found");
  if (rfidOk) {
    Serial.println("[rfid] scan first owner tag to enroll and lock/arm");
  }
}

static void setupBle() {
  if (!BLE.begin()) {
    Serial.println("[ble] init failed");
    while (true) { ledWrite(true, false, false); delay(200);
                   ledWrite(false, false, false); delay(200); }
  }
  BLE.setLocalName("SecurePouch-BLE");
  BLE.setDeviceName("SecurePouch BLE relay");
  BLE.setAppearance(GAP_APPEARANCE_GENERIC_TAG);
  BLE.setPairable(Pairable::YES);
  BLE.setAdvertisedService(spService);

  spService.addCharacteristic(deviceIdChar);
  spService.addCharacteristic(statusChar);
  spService.addCharacteristic(controlChar);
  BLE.addService(spService);

  batteryService.addCharacteristic(batteryLevelChar);
  BLE.addService(batteryService);

  deviceIdChar.writeValue(DEVICE_UID);
  statusChar.writeValue(statusByte());
  batteryLevelChar.writeValue(g_state.battery);

  ble_bond_store_init();   // register LTK/IRK flash callbacks before advertising

  BLE.setEventHandler(BLEConnected, onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  controlChar.setEventHandler(BLEWritten, onControlWrite);

  BLE.advertise();
  Serial.print("[ble] advertising as SecurePouch-BLE, addr ");
  Serial.println(BLE.address());
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(SP_UART_BAUD);    // link to nRF9151

  setupPins();
  resetNrf9151();
  setupServo();
  setupImu();
  calibrateAccel();
  setupRfid();
  setupBle();

  Serial.print("[boot] SecurePouch UID="); Serial.print(DEVICE_UID);
  Serial.print(" role="); Serial.println(FIRMWARE_ROLE);
  Serial.println("[boot] console: l/u lock, a/d arm, !/s alarm, g locate, r clear RFID, v servo sweep, c calibrate IMU, ? status");
  publishState();
  pushStatusToRcp();
}

// ===========================================================================
// Main loop
// ===========================================================================
void loop() {
  BLE.poll();              // BLE stack: connections, reads, control writes
  pollRcp();               // inbound lines from the nRF9151
  serviceRfid();           // RFID owner tag lock/arm toggle
  serviceTamper();         // accelerometer tamper while armed
  serviceDeadman();        // BLE dead-man switch
  serviceSiren();          // non-blocking siren/strobe while alarm active
  serviceSerialConsole();  // breadboard test console

  if (millis() - g_lastStatusPush >= STATUS_PERIOD_MS) {
    pushStatusToRcp();
  }
}
