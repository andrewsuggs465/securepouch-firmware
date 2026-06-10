#pragma once

// SecurePouch custom BLE service and characteristic UUIDs.
// Must be kept in sync with the FMD Android companion app BLE scanner.

// Primary service — advertised in scan response so phones can filter on it
#define SP_SERVICE_UUID         "af19b3e4-d279-4a2a-9d3f-2f5e8a6bc051"

// UTF-8 device UID registered in the FMD server (read-only)
#define SP_CHAR_DEVICE_ID_UUID  "af19b3e4-d279-4a2a-9d3f-2f5e8a6bc052"

// Device status byte (read + notify) — see SP_STATUS_* flags below
#define SP_CHAR_STATUS_UUID     "af19b3e4-d279-4a2a-9d3f-2f5e8a6bc053"

#define SP_DEVICE_ID_MAX_LEN 64

// Status byte bit flags
#define SP_STATUS_LOCKED  0x01
#define SP_STATUS_ARMED   0x02
