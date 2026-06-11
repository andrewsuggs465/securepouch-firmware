#include <ArduinoBLE.h>
#include "ble_protocol.h"

// --- Device configuration ---
// Set this to the UID registered for this device in the FMD server.
// Each unit needs a unique value flashed before deployment.
static const char* DEVICE_UID = "securepouch-001";

// --- BLE objects ---
BLEService spService(SP_SERVICE_UUID);

// Device UID readable by the companion phone app
BLEStringCharacteristic deviceIdChar(SP_CHAR_DEVICE_ID_UUID, BLERead, SP_DEVICE_ID_MAX_LEN);

// Status byte: SP_STATUS_LOCKED | SP_STATUS_ARMED flags
// Reserved for future use (lock/alarm GPIO not wired on prototype)
BLEByteCharacteristic statusChar(SP_CHAR_STATUS_UUID, BLERead | BLENotify);

void setup() {
    Serial.begin(115200);

    if (!BLE.begin()) {
        Serial.println("BLE init failed — check board and library");
        while (true);
    }

    BLE.setLocalName("SecurePouch");
    BLE.setAdvertisedService(spService);

    spService.addCharacteristic(deviceIdChar);
    spService.addCharacteristic(statusChar);
    BLE.addService(spService);

    deviceIdChar.writeValue(DEVICE_UID);
    statusChar.writeValue((uint8_t)0x00);

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
