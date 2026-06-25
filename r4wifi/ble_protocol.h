#pragma once

// SecurePouch custom BLE service and characteristic UUIDs.
// Must be kept in sync with the FMD Android companion app BLE scanner.

// Primary service — advertised in scan response so phones can filter on it
#define SP_SERVICE_UUID         "af19b3e4-d279-4a2a-9d3f-2f5e8a6bc051"

// UTF-8 device UID registered in the FMD server (read-only)
#define SP_CHAR_DEVICE_ID_UUID  "af19b3e4-d279-4a2a-9d3f-2f5e8a6bc052"

// Device status byte (read + notify) — see SP_STATUS_* flags below
#define SP_CHAR_STATUS_UUID     "af19b3e4-d279-4a2a-9d3f-2f5e8a6bc053"

// Control byte (write) — the companion app writes one SP_CTRL_* opcode here to
// command the pouch directly over BLE (arm/disarm/lock/unlock/alarm/silence).
// This is the local path; the same actions also arrive from the FMD server via
// the nRF9151 (see firmware/shared/sp_uart_protocol.h, SP_CMD_*).
#define SP_CHAR_CONTROL_UUID    "af19b3e4-d279-4a2a-9d3f-2f5e8a6bc054"

#define SP_DEVICE_ID_MAX_LEN 64

// Status byte bit flags (also used by SP_MSG_STATUS over the inter-MCU UART)
#define SP_STATUS_LOCKED  0x01
#define SP_STATUS_ARMED   0x02
#define SP_STATUS_ALARM   0x04  // siren/strobe currently active
#define SP_STATUS_TAMPER  0x08  // tamper motion detected since last clear

// Control opcodes written to SP_CHAR_CONTROL_UUID by the companion app.
// Mirror the FMD command strings in sp_uart_protocol.h (SP_CMD_*).
#define SP_CTRL_LOCK     0x01
#define SP_CTRL_UNLOCK   0x02
#define SP_CTRL_ARM      0x03
#define SP_CTRL_DISARM   0x04
#define SP_CTRL_ALARM    0x05  // trigger siren/strobe now
#define SP_CTRL_SILENCE  0x06  // stop an active alarm
#define SP_CTRL_LOCATE   0x07  // force a GNSS fix + upload (relayed to nRF9151)
