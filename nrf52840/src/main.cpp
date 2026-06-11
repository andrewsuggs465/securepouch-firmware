#include <ArduinoBLE.h>
#include "ble_protocol.h"

// --- Device configuration ---
// Set this to the UID registered for this device in the FMD server.
// Each unit needs a unique value flashed before deployment.
static const char* DEVICE_UID = "securepouch-001";

// Standard GAP appearance: Generic Tag (0x0200) — lets Android categorise
// the device in its Bluetooth settings instead of showing "Unknown device"
static const uint16_t GAP_APPEARANCE_GENERIC_TAG = 0x0200;

// --- BLE objects ---
BLEService spService(SP_SERVICE_UUID);

// Device UID readable by the companion phone app
BLEStringCharacteristic deviceIdChar(SP_CHAR_DEVICE_ID_UUID, BLERead, SP_DEVICE_ID_MAX_LEN);

// Status byte: SP_STATUS_LOCKED | SP_STATUS_ARMED flags
// Reserved for future use (lock/alarm GPIO not wired on prototype)
BLEByteCharacteristic statusChar(SP_CHAR_STATUS_UUID, BLERead | BLENotify);

// Standard Battery Service (0x180F) — gives the OS a profile it understands,
// so a bonded SecurePouch shows a battery level in Android settings
BLEService batteryService("180F");
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);

static void onConnect(BLEDevice central) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Central connected: ");
    Serial.println(central.address());
}

static void onDisconnect(BLEDevice central) {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Central disconnected: ");
    Serial.println(central.address());
    // ArduinoBLE resumes advertising automatically, but be explicit so a
    // failed OS pairing attempt can never leave the device invisible
    BLE.advertise();
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    if (!BLE.begin()) {
        Serial.println("BLE init failed — check board and library");
        while (true);
    }

    // Advertised name (scan response) and GAP device name (Generic Access
    // service). Without setDeviceName, Android settings shows "Arduino".
    BLE.setLocalName("SecurePouch");
    BLE.setDeviceName("SecurePouch");
    BLE.setAppearance(GAP_APPEARANCE_GENERIC_TAG);

    // Allow OS-level bonding (Just Works — no display or keypad on the pouch).
    // The companion app connects over plain GATT and does not require this,
    // but it lets Android's Bluetooth settings pair without an error.
    BLE.setPairable(Pairable::YES);

    BLE.setAdvertisedService(spService);

    spService.addCharacteristic(deviceIdChar);
    spService.addCharacteristic(statusChar);
    BLE.addService(spService);

    batteryService.addCharacteristic(batteryLevelChar);
    BLE.addService(batteryService);

    deviceIdChar.writeValue(DEVICE_UID);
    statusChar.writeValue((uint8_t)0x00);
    batteryLevelChar.writeValue(100);  // prototype is USB-powered; no fuel gauge yet

    BLE.setEventHandler(BLEConnected, onConnect);
    BLE.setEventHandler(BLEDisconnected, onDisconnect);

    BLE.advertise();

    Serial.print("SecurePouch BLE advertising — address: ");
    Serial.println(BLE.address());
    Serial.print("Device UID: ");
    Serial.println(DEVICE_UID);
}

void loop() {
    // Keep BLE stack alive; handles connections, reads, and notify callbacks
    BLE.poll();
}
