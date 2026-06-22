/*
 * SecurePouch — nRF52840 firmware (Arduino Nano 33 BLE prototype)
 * ==============================================================
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
 *   3. Local— fingerprint match unlocks; accelerometer tamper / BLE dead-man
 *             arm the alarm.
 *
 * Breadboard pinout (confirmed 2026-06-22) — see PIN_* below and
 * firmware/shared/sp_uart_protocol.h for the inter-MCU contract.
 */

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <DFRobot_ID809.h>

#include "ble_protocol.h"
#include "sp_uart_protocol.h"

// ---------------------------------------------------------------------------
// Device configuration
// ---------------------------------------------------------------------------
// Must match the username of this unit's account on the FMD server. Change per
// unit before flashing.
static const char* DEVICE_UID = "securepouch-001";

static const uint16_t GAP_APPEARANCE_GENERIC_TAG = 0x0200;

// ---------------------------------------------------------------------------
// Pin map  (Arduino Nano 33 BLE "Dx" labels)
// ---------------------------------------------------------------------------
static const uint8_t PIN_BUZZER_A   = 2;   // D2  piezo H-bridge leg A
static const uint8_t PIN_BUZZER_B   = 3;   // D3  piezo H-bridge leg B (anti-phase)
static const uint8_t PIN_LED_R      = 4;   // D4  RGB status LED (active high)
static const uint8_t PIN_LED_G      = 5;   // D5
static const uint8_t PIN_LED_B      = 6;   // D6
static const uint8_t PIN_SOLENOID_A = 7;   // D7  lock driver leg A
static const uint8_t PIN_SOLENOID_B = 8;   // D8  lock driver leg B
static const uint8_t PIN_BIO_RX     = 9;   // D9  <- fingerprint TX (we receive)
static const uint8_t PIN_BIO_TX     = 10;  // D10 -> fingerprint RX (we transmit)
static const uint8_t PIN_BIO_EN     = 11;  // D11 fingerprint enable (active high)
static const uint8_t PIN_BIO_IRQ    = 12;  // D12 fingerprint finger-touch (active high)
static const uint8_t PIN_NRF9151_RST = 17; // D17/A3 -> nRF9151 RESET (active low)
// Motion sensing uses the Nano 33 BLE's onboard LSM9DS1 IMU on the internal
// I2C bus (Wire1, P0.30/P0.31) via the Arduino_LSM9DS1 library — no external
// accelerometer and no analog pins needed. A0/A1/A2 and D18/D19 are free.

// Polarity / tuning constants ------------------------------------------------
static const bool LED_ACTIVE_HIGH      = true;   // external common-cathode RGB
static const uint16_t SIREN_FREQ_HZ    = 2900;   // CEB-35FD29 resonance
static const uint16_t SOLENOID_PULSE_MS = 500;   // unlock retract pulse
static const float    TAMPER_THRESHOLD = 0.35f;  // g of |accel - resting| (tune)
static const uint32_t DEADMAN_GRACE_MS = 30000;  // BLE-lost -> alarm delay
static const uint32_t STATUS_PERIOD_MS = 10000;  // push STAT to 9151 cadence

// ---------------------------------------------------------------------------
// Peripherals
// ---------------------------------------------------------------------------
// Second hardware UART (nRF52840 has two UARTE instances; Serial1 uses one).
// The mbed core's arduino::UART can bind to arbitrary pins, which is far more
// reliable at 115200 than bit-banged software serial. Arg order is (TX, RX):
// our TX (D10) -> fingerprint RX; our RX (D9) <- fingerprint TX.
arduino::UART bioSerial(PIN_BIO_TX, PIN_BIO_RX);
DFRobot_ID809 fingerprint;
bool fingerprintOk = false;
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
  bool locked  = true;    // fail-locked by default
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
bool g_sirenPhase = false;

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
// Actuators: lock + siren
// ===========================================================================
// Fail-locked spring; a brief pulse on the H-bridge retracts the pin to unlock.
static void solenoidPulseUnlock() {
  digitalWrite(PIN_SOLENOID_A, HIGH);
  digitalWrite(PIN_SOLENOID_B, LOW);
  delay(SOLENOID_PULSE_MS);
  // De-energise: spring re-extends the pin. Both legs low = coil off.
  digitalWrite(PIN_SOLENOID_A, LOW);
  digitalWrite(PIN_SOLENOID_B, LOW);
}

static void sirenOff() {
  noTone(PIN_BUZZER_A);
  digitalWrite(PIN_BUZZER_A, LOW);
  digitalWrite(PIN_BUZZER_B, LOW);
}

// Non-blocking siren + strobe: call every loop while alarm is active.
// Drives the two H-bridge legs in anti-phase to get ~2x p-p across the disc and
// flashes the RGB LED red as a strobe stand-in.
static void serviceSiren() {
  if (!g_state.alarm) return;
  uint32_t now = millis();
  // 2.9 kHz on leg A via tone(); manually toggle leg B in anti-phase strobe.
  tone(PIN_BUZZER_A, SIREN_FREQ_HZ);
  if (now - g_lastSirenToggle >= 60) {       // ~8 Hz visible strobe
    g_lastSirenToggle = now;
    g_sirenPhase = !g_sirenPhase;
    digitalWrite(PIN_BUZZER_B, g_sirenPhase ? HIGH : LOW);
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
static void doLock()   { g_state.locked = true; }
static void doUnlock() {
  solenoidPulseUnlock();
  g_state.locked = false;
  sendEvent("UNLOCK_OK");
}
static void doArm()    { g_state.armed = true;  g_bleLostAt = 0; }
static void doDisarm() {
  g_state.armed = false;
  g_state.tamper = false;
  if (g_state.alarm) { g_state.alarm = false; sirenOff(); }
}
static void doAlarm()  {
  g_state.alarm = true;
  sendEvent("ALARM");
  uartSendLine(SP_MSG_UPLOAD_NOW, "alarm");   // ask 9151 for a location burst
}
static void doSilence() { g_state.alarm = false; sirenOff(); }
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
  else if (strcmp(cmd, SP_CMD_ALARM)   == 0) doAlarm();
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
// Sensors: accelerometer tamper + fingerprint unlock
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
      doAlarm();
      publishState();
    }
  }
}

// Try a 1:N fingerprint match. Returns matched ID (1-80) or 0.
static uint8_t tryFingerprintMatch() {
  if (!fingerprintOk) return 0;
  if (fingerprint.collectionFingerprint(/*timeout s=*/5) == ERR_ID809) return 0;
  uint8_t id = fingerprint.search();
  if (id) {
    fingerprint.ctrlLED(DFRobot_ID809::eKeepsOn, DFRobot_ID809::eLEDGreen, 0);
  } else {
    fingerprint.ctrlLED(DFRobot_ID809::eKeepsOn, DFRobot_ID809::eLEDRed, 0);
    sendEvent("UNLOCK_FAIL");
  }
  delay(600);
  fingerprint.ctrlLED(DFRobot_ID809::eNormalClose, DFRobot_ID809::eLEDBlue, 0);
  return id;
}

// On finger touch (IRQ high), attempt biometric unlock.
static void serviceFingerprint() {
  if (!fingerprintOk) return;
  if (digitalRead(PIN_BIO_IRQ) != HIGH) return;
  Serial.println("[bio] finger detected — matching");
  uint8_t id = tryFingerprintMatch();
  if (id) {
    Serial.print("[bio] match id="); Serial.println(id);
    applyCommand(SP_CMD_UNLOCK, "BIO");
    applyCommand(SP_CMD_DISARM, "BIO");
  } else {
    Serial.println("[bio] no match");
  }
}

// Enroll a new fingerprint into the next free slot. Called from the serial
// console ('e') for breadboard provisioning; mirrors the DFRobot example.
static void enrollFingerprint() {
  if (!fingerprintOk) { Serial.println("[bio] sensor not present"); return; }
  uint8_t id = fingerprint.getEmptyID();
  if (id == ERR_ID809) { Serial.println("[bio] no free slot"); return; }
  Serial.print("[bio] enrolling into id="); Serial.println(id);
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print("  press finger ("); Serial.print(i + 1); Serial.println("/3)");
    fingerprint.ctrlLED(DFRobot_ID809::eBreathing, DFRobot_ID809::eLEDBlue, 0);
    if (fingerprint.collectionFingerprint(/*timeout s=*/10) == ERR_ID809) {
      Serial.println("  capture failed — retry");
      i--; continue;
    }
    fingerprint.ctrlLED(DFRobot_ID809::eFastBlink, DFRobot_ID809::eLEDYellow, 3);
    Serial.println("  release finger");
    while (fingerprint.detectFinger()) delay(50);
  }
  if (fingerprint.storeFingerprint(id) != ERR_ID809) {
    Serial.print("[bio] enrolled id="); Serial.println(id);
    fingerprint.ctrlLED(DFRobot_ID809::eKeepsOn, DFRobot_ID809::eLEDGreen, 0);
  } else {
    Serial.println("[bio] store failed");
  }
  delay(800);
  fingerprint.ctrlLED(DFRobot_ID809::eNormalClose, DFRobot_ID809::eLEDBlue, 0);
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
    case 'e': enrollFingerprint(); break;
    case 'x': if (fingerprintOk) { fingerprint.delFingerprint(DELALL);
                                   Serial.println("[bio] all templates deleted"); } break;
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
    sendEvent("DEADMAN");
    doAlarm();
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
  pinMode(PIN_BUZZER_A, OUTPUT);
  pinMode(PIN_BUZZER_B, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_SOLENOID_A, OUTPUT);
  pinMode(PIN_SOLENOID_B, OUTPUT);
  pinMode(PIN_BIO_EN, OUTPUT);
  pinMode(PIN_BIO_IRQ, INPUT);
  pinMode(PIN_NRF9151_RST, OUTPUT);

  sirenOff();
  digitalWrite(PIN_SOLENOID_A, LOW);
  digitalWrite(PIN_SOLENOID_B, LOW);
  digitalWrite(PIN_BIO_EN, HIGH);          // power/enable the fingerprint module
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

static void setupFingerprint() {
  // SEN0348 ships at 115200; bioSerial is a real hardware UART so that is fine.
  bioSerial.begin(115200);
  fingerprint.begin(bioSerial);
  fingerprintOk = fingerprint.isConnected();
  Serial.print("[bio] fingerprint sensor: ");
  Serial.println(fingerprintOk ? "ready" : "NOT found");
}

static void setupBle() {
  if (!BLE.begin()) {
    Serial.println("[ble] init failed");
    while (true) { ledWrite(true, false, false); delay(200);
                   ledWrite(false, false, false); delay(200); }
  }
  BLE.setLocalName("SecurePouch");
  BLE.setDeviceName("SecurePouch");
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

  BLE.setEventHandler(BLEConnected, onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  controlChar.setEventHandler(BLEWritten, onControlWrite);

  BLE.advertise();
  Serial.print("[ble] advertising as SecurePouch, addr ");
  Serial.println(BLE.address());
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(SP_UART_BAUD);    // link to nRF9151

  setupPins();
  resetNrf9151();
  setupImu();
  calibrateAccel();
  setupFingerprint();
  setupBle();

  Serial.print("[boot] SecurePouch UID="); Serial.println(DEVICE_UID);
  Serial.println("[boot] console: l/u lock, a/d arm, !/s alarm, g locate, e enroll, x wipe");
  publishState();
  pushStatusToRcp();
}

// ===========================================================================
// Main loop
// ===========================================================================
void loop() {
  BLE.poll();              // BLE stack: connections, reads, control writes
  pollRcp();               // inbound lines from the nRF9151
  serviceFingerprint();    // biometric unlock on touch
  serviceTamper();         // accelerometer tamper while armed
  serviceDeadman();        // BLE dead-man switch
  serviceSiren();          // non-blocking siren/strobe while alarm active
  serviceSerialConsole();  // breadboard test console

  if (millis() - g_lastStatusPush >= STATUS_PERIOD_MS) {
    pushStatusToRcp();
  }
}
