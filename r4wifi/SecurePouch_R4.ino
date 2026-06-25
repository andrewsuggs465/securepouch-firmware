/*
 * SecurePouch — Arduino UNO R4 WiFi prototype
 * ============================================
 *
 * Hardware changes from the nRF52840 build:
 *   - RFID reader  -> momentary button (active-low, D4) — tap to toggle lock/arm
 *   - Solenoid     -> 2N2222 on D7; HIGH = energised = locked, LOW = unlocked
 *   - Button       -> arm/disarm toggle; silences alarm if active (does NOT disarm on silence)
 *   - Siren        -> passive buzzer, single pin (D3~, tone() capable)
 *   - RGB LED      -> common-cathode, D9/D10/D11 (all PWM-capable)
 *   - Accelerometer-> DFRobot triple-axis analog (A0=X, A1=Y, A2=Z)
 *   - BLE          -> built-in ESP32-S3 co-processor via ArduinoBLE
 *   - UART to 9151 -> Serial1 = D0(RX) / D1(TX)
 *
 * Required libraries (install via Library Manager):
 *   ArduinoBLE      >= 1.3.6
 *   (no other libraries needed — solenoid is plain GPIO, IMU is analog reads)
 *
 * Solenoid wiring (energise-to-lock, NPN 2N2222 driver):
 *   D7 ──── 1kΩ ──── 2N2222 Base
 *   2N2222 Collector ──── solenoid (−)
 *   solenoid (+) ──── 12V supply
 *   12V GND ──── Arduino GND  (shared ground essential)
 *   1N4007 flyback diode across solenoid terminals (cathode to 12V)
 *
 * RGB LED wiring (common-cathode):
 *   LED R anode -> 220Ω -> D9
 *   LED G anode -> 220Ω -> D10
 *   LED B anode -> 220Ω -> D11
 *   LED common cathode -> GND
 *
 * Button wiring:
 *   D4 -> one leg of button -> GND
 *   (uses internal pull-up; LOW = pressed)
 *
 * Buzzer wiring (passive piezo):
 *   D3 -> buzzer (+) -> buzzer (-) -> GND
 *   (or 100Ω in series if buzzer is loud enough)
 *
 * UART to nRF9151 DK:
 *   R4 D1 (TX) -> nRF9151 DK P0.01 (RX)
 *   R4 D0 (RX) <- nRF9151 DK P0.00 (TX)
 *   GND shared between boards
 *   NOTE: R4 WiFi GPIO is 3.3V; nRF9151 DK is also 3.3V — direct connection is fine.
 *
 * DFRobot triple-axis accelerometer wiring (analog):
 *   VCC -> 3.3V
 *   GND -> GND
 *   X   -> A0
 *   Y   -> A1
 *   Z   -> A2
 *   (no library needed; outputs are analog voltages centred at VCC/2)
 */

#include <ArduinoBLE.h>
#include <EEPROM.h>

// Shared protocol headers (relative path — copy to same sketch folder or adjust)
#include "ble_protocol.h"
#include "sp_uart_protocol.h"
#include "ble_bond_store.h"

// ---------------------------------------------------------------------------
// Device configuration
// ---------------------------------------------------------------------------
static const char* DEVICE_UID    = "securepouch-001";
static const char* FIRMWARE_ROLE = "R4 WiFi BLE relay";

static const uint16_t GAP_APPEARANCE_GENERIC_TAG = 0x0200;

// ---------------------------------------------------------------------------
// Pin map — Arduino UNO R4 WiFi
// ---------------------------------------------------------------------------
static const uint8_t PIN_BUZZER      = 3;   // D3~ (PWM / tone)
static const uint8_t PIN_BUTTON      = 4;   // D4  arm/disarm toggle button (active-low)
static const uint8_t PIN_SOLENOID    = 7;   // D7  2N2222 base drive (HIGH = energised = locked)
static const uint8_t PIN_LED_R       = 9;   // D9~  RGB red   (PWM)
static const uint8_t PIN_LED_G       = 10;  // D10~ RGB green (PWM)
static const uint8_t PIN_LED_B       = 11;  // D11~ RGB blue  (PWM)
static const uint8_t PIN_NRF9151_RST = 5;   // D5   nRF9151 RESET (active-low pulse at boot)
// SDA = A4, SCL = A5 (Wire default on R4)

// ---------------------------------------------------------------------------
// Timing / tuning constants
// ---------------------------------------------------------------------------
static const uint16_t SIREN_HI_HZ       = 2000;   // two-tone alarm frequencies
static const uint16_t SIREN_LO_HZ       = 1000;
static const uint32_t SIREN_HALF_MS     = 250;     // swap tone every 250 ms
static const uint32_t STROBE_MS         = 150;     // LED flash cadence during alarm
static const uint32_t DEADMAN_GRACE_MS  = 30000;   // BLE-lost -> alarm delay
static const uint32_t STATUS_PERIOD_MS  = 10000;   // push STAT to 9151 cadence
static const uint32_t ALARM_TEST_MS     = 10000;   // remote/test alarm auto-silence
static const float    TAMPER_THRESHOLD  = 200.0f;  // ADC counts deviation from rest magnitude (~0.04g at 14-bit/3.3V); tune up if false trips
static const uint32_t BTN_DEBOUNCE_MS   = 200;

// ---------------------------------------------------------------------------
// DFRobot triple-axis analog accelerometer (A0=X, A1=Y, A2=Z)
// ---------------------------------------------------------------------------
// The R4 WiFi ADC is 14-bit (0–16383) referenced to 3.3V.
// The DFRobot board outputs VCC/2 (~1.65V) at 0g, swings ~0.33V/g.
// We work in raw ADC counts throughout to avoid float division; the tamper
// threshold is expressed in counts so it's independent of actual g scaling.
static const uint8_t PIN_ACC_X = A0;
static const uint8_t PIN_ACC_Y = A1;
static const uint8_t PIN_ACC_Z = A2;

// Resting magnitude in ADC counts (calibrated at boot). At rest the vector
// magnitude = sqrt(0² + 0² + 1g²) worth of counts — we capture the baseline
// and alarm on deviation, same logic as the nRF52840 LSM9DS1 build.
static float g_accRestMag = 0.0f;

// ---------------------------------------------------------------------------
// BLE service / characteristics (same UUIDs as nRF52840 build)
// ---------------------------------------------------------------------------
BLEService spService(SP_SERVICE_UUID);
BLEStringCharacteristic deviceIdChar(SP_CHAR_DEVICE_ID_UUID, BLERead, SP_DEVICE_ID_MAX_LEN);
BLEByteCharacteristic  statusChar(SP_CHAR_STATUS_UUID,   BLERead | BLENotify);
BLEByteCharacteristic  controlChar(SP_CHAR_CONTROL_UUID, BLEWrite);

BLEService             batteryService("180F");
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);

// ---------------------------------------------------------------------------
// Device state
// ---------------------------------------------------------------------------
struct PouchState {
  bool locked  = false;
  bool armed   = false;
  bool alarm   = false;
  bool tamper  = false;
  uint8_t battery = 100;
};
PouchState g_state;

bool     g_bleConnected = false;
uint32_t g_bleLostAt    = 0;

// Alarm timing
uint32_t g_alarmStartedAt       = 0;
uint32_t g_alarmAutoSilenceAfter = 0;
uint32_t g_lastSirenToggle      = 0;
uint32_t g_lastStrobe           = 0;
bool     g_strobePhase          = false;
bool     g_sirenPhase           = false;   // false = lo tone, true = hi tone

// Button state
bool     g_lastBtnState = HIGH;
uint32_t g_lastBtnMs    = 0;

// Status push cadence
uint32_t g_lastStatusPush = 0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void applyCommand(const char* cmd, const char* source);
void pushStatusToRcp();
void uartSendLine(const char* tag, const char* payload);

// ===========================================================================
// LED helpers (common-cathode, active-high)
// ===========================================================================
static void ledWrite(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

static void updateStatusLed() {
  if (g_state.alarm)       ledWrite(true,  false, false);  // red   = alarm
  else if (g_state.armed)  ledWrite(false, false, true);   // blue  = armed
  else                     ledWrite(false, true,  false);  // green = unarmed
}

// ===========================================================================
// Solenoid latch (energise-to-lock)
// ===========================================================================
static void solenoidLock() {
  digitalWrite(PIN_SOLENOID, HIGH);
  Serial.print("[sol] D7 = "); Serial.println(digitalRead(PIN_SOLENOID));
}
static void solenoidUnlock() {
  digitalWrite(PIN_SOLENOID, LOW);
  Serial.print("[sol] D7 = "); Serial.println(digitalRead(PIN_SOLENOID));
}

// ===========================================================================
// Siren + strobe (non-blocking, called every loop while alarm is active)
// ===========================================================================
static void sirenOff() {
  noTone(PIN_BUZZER);
  digitalWrite(PIN_BUZZER, LOW);
  g_sirenPhase  = false;
  g_strobePhase = false;
}

static void serviceSiren() {
  if (!g_state.alarm) return;
  uint32_t now = millis();

  // Auto-silence after a timed test alarm
  if (g_alarmAutoSilenceAfter > 0 && now - g_alarmStartedAt >= g_alarmAutoSilenceAfter) {
    Serial.println("[alarm] auto-silence");
    g_state.alarm            = false;
    g_alarmAutoSilenceAfter  = 0;
    sirenOff();
    updateStatusLed();
    pushStatusToRcp();
    statusChar.writeValue(statusByte());
    return;
  }

  // Two-tone siren: swap every SIREN_HALF_MS
  if (now - g_lastSirenToggle >= SIREN_HALF_MS) {
    g_lastSirenToggle = now;
    g_sirenPhase      = !g_sirenPhase;
    tone(PIN_BUZZER, g_sirenPhase ? SIREN_HI_HZ : SIREN_LO_HZ);
  }

  // Strobe red / off every STROBE_MS
  if (now - g_lastStrobe >= STROBE_MS) {
    g_lastStrobe  = now;
    g_strobePhase = !g_strobePhase;
    ledWrite(g_strobePhase, false, false);
  }
}

// ===========================================================================
// Status byte
// ===========================================================================
uint8_t statusByte() {
  uint8_t b = 0;
  if (g_state.locked) b |= SP_STATUS_LOCKED;
  if (g_state.armed)  b |= SP_STATUS_ARMED;
  if (g_state.alarm)  b |= SP_STATUS_ALARM;
  if (g_state.tamper) b |= SP_STATUS_TAMPER;
  return b;
}

static void publishState() {
  statusChar.writeValue(statusByte());
  batteryLevelChar.writeValue(g_state.battery);
  updateStatusLed();
}

// ===========================================================================
// Inter-MCU UART (Serial1 = D0/D1 on R4 WiFi)
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

static void handleRcpLine(char* line) {
  char* sep = strchr(line, ':');
  if (!sep) return;
  *sep = '\0';
  const char* tag     = line;
  const char* payload = sep + 1;

  if      (strcmp(tag, SP_MSG_COMMAND)    == 0) applyCommand(payload, "LTE");
  else if (strcmp(tag, SP_MSG_RCP_STATUS) == 0) { Serial.print("[rcp] net="); Serial.println(payload); }
  else if (strcmp(tag, SP_MSG_FIX)        == 0) { Serial.print("[rcp] fix="); Serial.println(payload); }
}

static void pollRcp() {
  static char   buf[SP_UART_LINE_MAX];
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
      len = 0;
    }
  }
}

// ===========================================================================
// Command dispatch
// ===========================================================================
static void doLock() {
  solenoidLock();
  g_state.locked = true;
  sendEvent("LOCK_OK");
  Serial.println("[lock] locked");
}

static void doUnlock() {
  solenoidUnlock();
  g_state.locked = false;
  sendEvent("UNLOCK_OK");
  Serial.println("[unlock] unlocked");
}

static void doArm() {
  solenoidLock();
  g_state.locked = true;
  g_state.armed  = true;
  g_bleLostAt    = 0;
  Serial.println("[arm] armed + locked");
}

static void doDisarm() {
  solenoidUnlock();
  g_state.locked = false;
  g_state.armed  = false;
  g_state.tamper = false;
  if (g_state.alarm) { g_state.alarm = false; sirenOff(); }
  Serial.println("[disarm] disarmed + unlocked");
}

static void doAlarm(const char* source, uint32_t autoSilenceAfterMs = 0) {
  g_state.alarm             = true;
  g_alarmStartedAt          = millis();
  g_alarmAutoSilenceAfter   = autoSilenceAfterMs;
  g_lastSirenToggle         = 0;
  g_lastStrobe              = 0;
  Serial.print("[alarm] active from "); Serial.println(source);
  sendEvent("ALARM");
  uartSendLine(SP_MSG_UPLOAD_NOW, "alarm");
}

static void doSilence() {
  g_state.alarm            = false;
  g_alarmAutoSilenceAfter  = 0;
  sirenOff();
  if (g_state.armed && !g_bleConnected) g_bleLostAt = millis();
}

static void doLocate() { uartSendLine(SP_MSG_UPLOAD_NOW, "locate"); }

void applyCommand(const char* cmd, const char* source) {
  Serial.print("[cmd] "); Serial.print(source); Serial.print(" -> "); Serial.println(cmd);

  if      (strcmp(cmd, SP_CMD_LOCK)    == 0) doLock();
  else if (strcmp(cmd, SP_CMD_UNLOCK)  == 0) doUnlock();
  else if (strcmp(cmd, SP_CMD_ARM)     == 0) doArm();
  else if (strcmp(cmd, SP_CMD_DISARM)  == 0) doDisarm();
  else if (strcmp(cmd, SP_CMD_ALARM)   == 0) doAlarm(source, ALARM_TEST_MS);
  else if (strcmp(cmd, SP_CMD_SILENCE) == 0) doSilence();
  else if (strcmp(cmd, SP_CMD_LOCATE)  == 0) doLocate();
  else { Serial.println("[cmd] unknown"); return; }

  uartSendLine(SP_MSG_ACK, cmd);
  pushStatusToRcp();
  publishState();
}

static void applyControlOpcode(uint8_t op) {
  switch (op) {
    case SP_CTRL_LOCK:    applyCommand(SP_CMD_LOCK,    "BLE"); break;
    case SP_CTRL_UNLOCK:  applyCommand(SP_CMD_UNLOCK,  "BLE"); break;
    case SP_CTRL_ARM:     applyCommand(SP_CMD_ARM,     "BLE"); break;
    case SP_CTRL_DISARM:  applyCommand(SP_CMD_DISARM,  "BLE"); break;
    case SP_CTRL_ALARM:   applyCommand(SP_CMD_ALARM,   "BLE"); break;
    case SP_CTRL_SILENCE: applyCommand(SP_CMD_SILENCE, "BLE"); break;
    case SP_CTRL_LOCATE:  applyCommand(SP_CMD_LOCATE,  "BLE"); break;
    default: Serial.print("[ble] unknown opcode "); Serial.println(op);
  }
}

// ===========================================================================
// Button — tap to toggle lock/arm state
// ===========================================================================
static void serviceButton() {
  bool btn = digitalRead(PIN_BUTTON);
  if (btn == LOW && g_lastBtnState == HIGH && millis() - g_lastBtnMs > BTN_DEBOUNCE_MS) {
    g_lastBtnMs = millis();
    if (g_state.alarm) {
      applyCommand(SP_CMD_SILENCE, "BTN");   // silence only; stays armed
    } else if (g_state.armed) {
      applyCommand(SP_CMD_DISARM, "BTN");
    } else {
      applyCommand(SP_CMD_ARM, "BTN");
    }
  }
  g_lastBtnState = btn;
}

// ===========================================================================
// Accelerometer tamper detection (DFRobot analog triple-axis)
// ===========================================================================
static float readAccMag() {
  float x = analogRead(PIN_ACC_X);
  float y = analogRead(PIN_ACC_Y);
  float z = analogRead(PIN_ACC_Z);
  return sqrtf(x * x + y * y + z * z);
}

static void calibrateAccel() {
  // Average 50 samples over ~100 ms with board undisturbed.
  float sum = 0.0f;
  for (int i = 0; i < 50; i++) {
    sum += readAccMag();
    delay(2);
  }
  g_accRestMag = sum / 50.0f;
  Serial.print("[imu] rest magnitude="); Serial.println(g_accRestMag, 1);
}

static void serviceTamper() {
  if (!g_state.armed || g_state.alarm) return;
  float mag = readAccMag();
  float delta = fabsf(mag - g_accRestMag);
  if (delta > TAMPER_THRESHOLD && !g_state.tamper) {
    g_state.tamper = true;
    Serial.print("[tamper] accel spike delta="); Serial.println(delta, 1);
    sendEvent("TAMPER");
    doAlarm("TAMPER");
    publishState();
  }
}

// ===========================================================================
// BLE dead-man switch
// ===========================================================================
static void serviceDeadman() {
  if (!g_state.armed || g_state.alarm) return;
  if (g_bleConnected || g_bleLostAt == 0) return;
  if (millis() - g_bleLostAt >= DEADMAN_GRACE_MS) {
    Serial.println("[deadman] phone out of range past grace — alarm");
    g_bleLostAt = 0;
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
  g_bleLostAt    = 0;
  Serial.print("[ble] connected: "); Serial.println(central.address());
  publishState();
}

static void onDisconnect(BLEDevice central) {
  g_bleConnected = false;
  if (g_state.armed) g_bleLostAt = millis();
  Serial.print("[ble] disconnected: "); Serial.println(central.address());
  BLE.advertise();
  publishState();
}

static void onControlWrite(BLEDevice, BLECharacteristic) {
  applyControlOpcode(controlChar.value());
}

// ===========================================================================
// Serial console (USB Serial — breadboard testing)
// ===========================================================================
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
    case 't': {
      float x = analogRead(PIN_ACC_X);
      float y = analogRead(PIN_ACC_Y);
      float z = analogRead(PIN_ACC_Z);
      float mag = sqrtf(x*x + y*y + z*z);
      Serial.print("[imu] x="); Serial.print(x, 0);
      Serial.print(" y="); Serial.print(y, 0);
      Serial.print(" z="); Serial.print(z, 0);
      Serial.print(" mag="); Serial.print(mag, 0);
      Serial.print(" rest="); Serial.print(g_accRestMag, 0);
      Serial.print(" delta="); Serial.println(fabsf(mag - g_accRestMag), 0);
      break;
    }
    case 'c': calibrateAccel(); break;
    default: break;
  }
}

// ===========================================================================
// Setup helpers
// ===========================================================================
static void setupPins() {
  pinMode(PIN_BUZZER,      OUTPUT);
  pinMode(PIN_BUTTON,      INPUT_PULLUP);
  pinMode(PIN_LED_R,       OUTPUT);
  pinMode(PIN_LED_G,       OUTPUT);
  pinMode(PIN_LED_B,       OUTPUT);
  pinMode(PIN_NRF9151_RST, OUTPUT);

  pinMode(PIN_SOLENOID, OUTPUT);
  solenoidUnlock();                      // boot = de-energised = unlocked
  digitalWrite(PIN_NRF9151_RST, HIGH);
  noTone(PIN_BUZZER);
  ledWrite(false, true, false);          // boot = green (unarmed)
}

static void resetNrf9151() {
  digitalWrite(PIN_NRF9151_RST, LOW);
  delay(50);
  digitalWrite(PIN_NRF9151_RST, HIGH);
}

static void setupImu() {
  // Analog pins are inputs by default; set ADC resolution to 14-bit for R4.
  analogReadResolution(14);
  calibrateAccel();
  Serial.println("[imu] DFRobot analog accel ready (hold still during boot)");
}

static void setupBle() {
  if (!BLE.begin()) {
    Serial.println("[ble] init failed — halting");
    while (true) { ledWrite(true, false, false); delay(200); ledWrite(false, false, false); delay(200); }
  }
  BLE.setLocalName("SecurePouch-BLE");
  BLE.setDeviceName("SecurePouch BLE relay");
  BLE.setAppearance(GAP_APPEARANCE_GENERIC_TAG);

  spService.addCharacteristic(deviceIdChar);
  spService.addCharacteristic(statusChar);
  spService.addCharacteristic(controlChar);
  BLE.addService(spService);

  batteryService.addCharacteristic(batteryLevelChar);
  BLE.addService(batteryService);

  BLE.setAdvertisedService(spService);

  deviceIdChar.writeValue(DEVICE_UID);
  statusChar.writeValue(statusByte());
  batteryLevelChar.writeValue(g_state.battery);

  BLE.setPairable(Pairable::YES);
  ble_bond_store_init();   // register LTK/IRK EEPROM callbacks before advertising

  BLE.setEventHandler(BLEConnected,    onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  controlChar.setEventHandler(BLEWritten, onControlWrite);

  BLE.advertise();
  Serial.print("[ble] advertising as SecurePouch-BLE  addr=");
  Serial.println(BLE.address());
}

// ===========================================================================
// Setup / Loop
// ===========================================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(SP_UART_BAUD);   // UART link to nRF9151 DK

  setupPins();
  resetNrf9151();
  setupImu();
  setupBle();

  Serial.print("[boot] SecurePouch UID="); Serial.print(DEVICE_UID);
  Serial.print(" role="); Serial.println(FIRMWARE_ROLE);
  Serial.println("[boot] console: l/u lock, a/d arm, !/s alarm, g locate, t imu, c calibrate");
  publishState();
  pushStatusToRcp();
}

void loop() {
  BLE.poll();
  pollRcp();
  serviceButton();
  serviceTamper();
  serviceDeadman();
  serviceSiren();
  serviceSerialConsole();

  if (millis() - g_lastStatusPush >= STATUS_PERIOD_MS) {
    pushStatusToRcp();
  }
}
