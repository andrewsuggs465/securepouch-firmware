/*
 * ble_bond_store.h — persistent BLE bond key storage for the UNO R4 WiFi.
 *
 * Wires up the four ArduinoBLE bonding callbacks (storeLTK / getLTK /
 * storeIRK / getIRKs) to the R4's virtualEEPROM so pairing keys survive
 * power cycles.
 *
 * EEPROM layout (all offsets in bytes):
 *   0x000                   — magic byte (0xB0); 0xFF = uninitialised
 *   0x001                   — LTK count (uint8_t, 0–BLE_BOND_MAX_DEVICES)
 *   0x002                   — IRK count  (uint8_t, 0–BLE_BOND_MAX_DEVICES)
 *   0x010 + n*LTK_REC_SIZE  — LTK record n: addr[6] | LTK[16]  = 22 bytes each
 *   0x070 + n*IRK_REC_SIZE  — IRK record n: addr[6] | IRK[16] | addrType[1] = 23 bytes each
 *
 * Total: ~0x0D3 (211 bytes). The R4 virtualEEPROM has at least 256 bytes.
 *
 * Usage (in setupBle(), after BLE.begin(), before BLE.advertise()):
 *   ble_bond_store_init();
 */

#pragma once
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <EEPROM.h>
#include <stdlib.h>

#define BLE_BOND_MAX_DEVICES  4
#define BLE_BOND_MAGIC        0xB0

// Record sizes
#define LTK_REC_SIZE  22   // addr[6] + LTK[16]
#define IRK_REC_SIZE  23   // addr[6] + IRK[16] + addrType[1]

// EEPROM offsets
#define EE_MAGIC       0x000
#define EE_LTK_COUNT   0x001
#define EE_IRK_COUNT   0x002
#define EE_LTK_BASE    0x010
#define EE_IRK_BASE    0x070

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void bondAddrToHex(const uint8_t* addr, char* out) {
    snprintf(out, 13, "%02x%02x%02x%02x%02x%02x",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

static void eeWriteBytes(int offset, const uint8_t* src, size_t len) {
    for (size_t i = 0; i < len; i++) EEPROM.update(offset + i, src[i]);
}
static void eeReadBytes(int offset, uint8_t* dst, size_t len) {
    for (size_t i = 0; i < len; i++) dst[i] = EEPROM.read(offset + i);
}

static bool g_bondFlashInited = false;

static void bondEnsureInit() {
    if (!g_bondFlashInited) {
        // EEPROM.length() forces the underlying DataFlashBlockDevice lazy-init
        // before any read/write. Must be called before the first EEPROM.read().
        (void)EEPROM.length();
        g_bondFlashInited = true;
    }
    if (EEPROM.read(EE_MAGIC) != BLE_BOND_MAGIC) {
        EEPROM.write(EE_MAGIC,     BLE_BOND_MAGIC);
        EEPROM.write(EE_LTK_COUNT, 0);
        EEPROM.write(EE_IRK_COUNT, 0);
    }
}

// ---------------------------------------------------------------------------
// storeLTK — called after successful pairing. addr: 6-byte peer MAC; LTK: 16 bytes.
// ---------------------------------------------------------------------------
static int ble_storeLTK(uint8_t* addr, uint8_t* ltk) {
    bondEnsureInit();
    uint8_t count = EEPROM.read(EE_LTK_COUNT);
    char hex[13]; bondAddrToHex(addr, hex);

    // Check if this address already has a slot — update in place.
    for (uint8_t i = 0; i < count; i++) {
        int off = EE_LTK_BASE + i * LTK_REC_SIZE;
        uint8_t stored[6]; eeReadBytes(off, stored, 6);
        if (memcmp(stored, addr, 6) == 0) {
            eeWriteBytes(off + 6, ltk, 16);
            Serial.print("[bond] LTK updated for "); Serial.println(hex);
            return 0;
        }
    }
    // LRU evict if full.
    if (count >= BLE_BOND_MAX_DEVICES) {
        for (uint8_t i = 0; i < count - 1; i++) {
            int src = EE_LTK_BASE + (i + 1) * LTK_REC_SIZE;
            int dst = EE_LTK_BASE + i * LTK_REC_SIZE;
            uint8_t rec[LTK_REC_SIZE]; eeReadBytes(src, rec, LTK_REC_SIZE);
            eeWriteBytes(dst, rec, LTK_REC_SIZE);
        }
        count = BLE_BOND_MAX_DEVICES - 1;
    }
    int off = EE_LTK_BASE + count * LTK_REC_SIZE;
    eeWriteBytes(off, addr, 6);
    eeWriteBytes(off + 6, ltk, 16);
    EEPROM.write(EE_LTK_COUNT, count + 1);
    Serial.print("[bond] LTK stored for "); Serial.println(hex);
    return 0;
}

// ---------------------------------------------------------------------------
// getLTK — called on reconnect. Fills ltk[16]; returns 1 if found, 0 if not.
// ---------------------------------------------------------------------------
static int ble_getLTK(uint8_t* addr, uint8_t* ltk) {
    bondEnsureInit();
    uint8_t count = EEPROM.read(EE_LTK_COUNT);
    for (uint8_t i = 0; i < count; i++) {
        int off = EE_LTK_BASE + i * LTK_REC_SIZE;
        uint8_t stored[6]; eeReadBytes(off, stored, 6);
        if (memcmp(stored, addr, 6) == 0) {
            eeReadBytes(off + 6, ltk, 16);
            char hex[13]; bondAddrToHex(addr, hex);
            Serial.print("[bond] LTK loaded for "); Serial.println(hex);
            return 1;
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// storeIRK — called when a resolvable private address is resolved.
// ---------------------------------------------------------------------------
static int ble_storeIRK(uint8_t* addr, uint8_t* irk) {
    bondEnsureInit();
    uint8_t count = EEPROM.read(EE_IRK_COUNT);
    char hex[13]; bondAddrToHex(addr, hex);

    for (uint8_t i = 0; i < count; i++) {
        int off = EE_IRK_BASE + i * IRK_REC_SIZE;
        uint8_t stored[6]; eeReadBytes(off, stored, 6);
        if (memcmp(stored, addr, 6) == 0) {
            eeWriteBytes(off + 6, irk, 16);
            Serial.print("[bond] IRK updated for "); Serial.println(hex);
            return 0;
        }
    }
    if (count >= BLE_BOND_MAX_DEVICES) {
        for (uint8_t i = 0; i < count - 1; i++) {
            int src = EE_IRK_BASE + (i + 1) * IRK_REC_SIZE;
            int dst = EE_IRK_BASE + i * IRK_REC_SIZE;
            uint8_t rec[IRK_REC_SIZE]; eeReadBytes(src, rec, IRK_REC_SIZE);
            eeWriteBytes(dst, rec, IRK_REC_SIZE);
        }
        count = BLE_BOND_MAX_DEVICES - 1;
    }
    int off = EE_IRK_BASE + count * IRK_REC_SIZE;
    eeWriteBytes(off, addr, 6);
    eeWriteBytes(off + 6, irk, 16);
    EEPROM.write(off + 22, 0);   // addrType = 0 (public)
    EEPROM.write(EE_IRK_COUNT, count + 1);
    Serial.print("[bond] IRK stored for "); Serial.println(hex);
    return 0;
}

// ---------------------------------------------------------------------------
// getIRKs — called at connection time to resolve private addresses.
// ArduinoBLE frees the allocations after use.
// ---------------------------------------------------------------------------
static int ble_getIRKs(uint8_t* nIRKs, uint8_t** BDAddrTypes,
                        uint8_t*** BDAddrs, uint8_t*** IRKs) {
    bondEnsureInit();
    uint8_t count = EEPROM.read(EE_IRK_COUNT);
    if (count == 0) { *nIRKs = 0; return 0; }

    // ArduinoBLE's HCI layer frees these with free() after use, so we must
    // allocate with malloc/calloc (not new) to keep the heap consistent.
    *BDAddrTypes = (uint8_t*)  calloc(count, sizeof(uint8_t));
    *BDAddrs     = (uint8_t**) calloc(count, sizeof(uint8_t*));
    *IRKs        = (uint8_t**) calloc(count, sizeof(uint8_t*));
    uint8_t loaded = 0;

    for (uint8_t i = 0; i < count; i++) {
        int off = EE_IRK_BASE + i * IRK_REC_SIZE;
        (*IRKs)[loaded]    = (uint8_t*) malloc(16);
        (*BDAddrs)[loaded] = (uint8_t*) malloc(6);
        eeReadBytes(off,      (*BDAddrs)[loaded], 6);
        eeReadBytes(off + 6,  (*IRKs)[loaded],    16);
        (*BDAddrTypes)[loaded] = EEPROM.read(off + 22);
        loaded++;
    }
    *nIRKs = loaded;
    return 0;
}

// ---------------------------------------------------------------------------
// ble_bond_store_init — register callbacks. Call after BLE.begin().
// ---------------------------------------------------------------------------
static void ble_bond_store_init() {
    bondEnsureInit();
    BLE.setStoreLTK(ble_storeLTK);
    BLE.setGetLTK(ble_getLTK);
    BLE.setStoreIRK(ble_storeIRK);
    BLE.setGetIRKs(ble_getIRKs);

    uint8_t ltks = EEPROM.read(EE_LTK_COUNT);
    uint8_t irks = EEPROM.read(EE_IRK_COUNT);
    Serial.print("[bond] store init — ");
    Serial.print(ltks); Serial.print(" LTK(s), ");
    Serial.print(irks); Serial.println(" IRK(s) in EEPROM");
}
