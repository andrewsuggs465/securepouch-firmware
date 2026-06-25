/*
 * ble_bond_store.h — persistent BLE bond key storage for the nRF52840.
 *
 * Wires up the four ArduinoBLE bonding callbacks (storeLTK / getLTK /
 * storeIRK / getIRKs) to mbed's KVStore so pairing keys survive power cycles.
 *
 * Keys are stored under /kv/ (the mbed global KVStore, backed by internal flash
 * on the Nano 33 BLE):
 *   /kv/sp_ltk_<6-byte MAC hex>   — 16-byte LTK
 *   /kv/sp_irk_<6-byte MAC hex>   — 16-byte IRK + 6-byte address + 1-byte type
 *     (IRK records pack: peer IRK [16], BDAddr [6], BDAddrType [1] = 23 bytes)
 *   /kv/sp_irk_count              — uint8_t count of stored IRK records
 *
 * Usage (in setup(), after BLE.begin() but before BLE.advertise()):
 *   ble_bond_store_init();
 *
 * The callbacks are plain C functions to match the function-pointer API.
 */

#pragma once
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <kvstore_global_api/kvstore_global_api.h>

// Max bonded devices to remember.  Each costs 23 bytes of KVStore space.
#define BLE_BOND_MAX_DEVICES 4

// IRK record layout in flash: peerIRK[16] | BDAddr[6] | BDAddrType[1]
#define BLE_IRK_RECORD_SIZE 23

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void bondAddrToHex(const uint8_t* addr, char* out) {
    // out must be at least 13 bytes (12 hex + null)
    snprintf(out, 13, "%02x%02x%02x%02x%02x%02x",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

// ---------------------------------------------------------------------------
// storeLTK  — called by ArduinoBLE after a successful pairing exchange.
//             addr: 6-byte peer MAC; LTK: 16-byte long-term key.
//             Return 0 on success.
// ---------------------------------------------------------------------------
static int ble_storeLTK(uint8_t* addr, uint8_t* ltk) {
    char key[32];
    char hex[13];
    bondAddrToHex(addr, hex);
    snprintf(key, sizeof(key), "/kv/sp_ltk_%s", hex);
    int err = kv_set(key, ltk, 16, 0);
    if (err != 0) {
        Serial.print("[bond] storeLTK failed for ");
        Serial.print(hex);
        Serial.print(" err=");
        Serial.println(err);
    } else {
        Serial.print("[bond] LTK stored for ");
        Serial.println(hex);
    }
    return err == 0 ? 0 : -1;
}

// ---------------------------------------------------------------------------
// getLTK  — called by ArduinoBLE when a bonded central reconnects.
//           Fills LTK[16]; return 1 if found, 0 if not found.
// ---------------------------------------------------------------------------
static int ble_getLTK(uint8_t* addr, uint8_t* ltk) {
    char key[32];
    char hex[13];
    bondAddrToHex(addr, hex);
    snprintf(key, sizeof(key), "/kv/sp_ltk_%s", hex);
    size_t actual = 0;
    int err = kv_get(key, ltk, 16, &actual);
    if (err == 0 && actual == 16) {
        Serial.print("[bond] LTK loaded for ");
        Serial.println(hex);
        return 1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// storeIRK  — called by ArduinoBLE after resolving a resolvable private addr.
//             addr: 6-byte peer MAC (may change); IRK: 16-byte identity key.
//             We store a compact record so getIRKs can rebuild the list.
// ---------------------------------------------------------------------------
static int ble_storeIRK(uint8_t* addr, uint8_t* irk) {
    // Read current count
    uint8_t count = 0;
    size_t actual = 0;
    kv_get("/kv/sp_irk_count", &count, 1, &actual);

    // Check if this address is already stored — update in place
    char key[32];
    char hex[13];
    bondAddrToHex(addr, hex);
    for (uint8_t i = 0; i < count; i++) {
        snprintf(key, sizeof(key), "/kv/sp_irk_%u", i);
        uint8_t rec[BLE_IRK_RECORD_SIZE];
        size_t sz = 0;
        if (kv_get(key, rec, sizeof(rec), &sz) == 0 && sz == BLE_IRK_RECORD_SIZE) {
            if (memcmp(rec + 16, addr, 6) == 0) {
                // Same address — overwrite the IRK
                memcpy(rec, irk, 16);
                kv_set(key, rec, sizeof(rec), 0);
                Serial.print("[bond] IRK updated for "); Serial.println(hex);
                return 0;
            }
        }
    }

    if (count >= BLE_BOND_MAX_DEVICES) {
        Serial.println("[bond] IRK store full — evicting oldest entry");
        // Shift records [1..count-1] down to [0..count-2]
        for (uint8_t i = 0; i < count - 1; i++) {
            char src[32], dst[32];
            snprintf(src, sizeof(src), "/kv/sp_irk_%u", (uint8_t)(i + 1));
            snprintf(dst, sizeof(dst), "/kv/sp_irk_%u", i);
            uint8_t rec[BLE_IRK_RECORD_SIZE];
            size_t sz = 0;
            if (kv_get(src, rec, sizeof(rec), &sz) == 0) {
                kv_set(dst, rec, sz, 0);
            }
        }
        count = BLE_BOND_MAX_DEVICES - 1;
    }

    // Append new record: IRK[16] | addr[6] | addrType[1]=0
    uint8_t rec[BLE_IRK_RECORD_SIZE] = {0};
    memcpy(rec, irk, 16);
    memcpy(rec + 16, addr, 6);
    rec[22] = 0;  // addrType: 0=public; the library passes resolved addr

    snprintf(key, sizeof(key), "/kv/sp_irk_%u", count);
    kv_set(key, rec, sizeof(rec), 0);
    count++;
    kv_set("/kv/sp_irk_count", &count, 1, 0);

    Serial.print("[bond] IRK stored for "); Serial.println(hex);
    return 0;
}

// ---------------------------------------------------------------------------
// getIRKs  — called by ArduinoBLE at connection time to resolve private addrs.
//            Fills nIRKs, BDAddrType[n], BDAddrs[n][6], IRKs[n][16].
//            ArduinoBLE frees the allocations after use.
//            Return 0 on success.
// ---------------------------------------------------------------------------
static int ble_getIRKs(uint8_t* nIRKs, uint8_t** BDAddrTypes,
                        uint8_t*** BDAddrs, uint8_t*** IRKs) {
    uint8_t count = 0;
    size_t actual = 0;
    kv_get("/kv/sp_irk_count", &count, 1, &actual);
    if (count == 0) {
        *nIRKs = 0;
        return 0;
    }

    *BDAddrTypes = new uint8_t[count];
    *BDAddrs     = new uint8_t*[count];
    *IRKs        = new uint8_t*[count];
    uint8_t loaded = 0;

    for (uint8_t i = 0; i < count; i++) {
        char key[32];
        snprintf(key, sizeof(key), "/kv/sp_irk_%u", i);
        uint8_t rec[BLE_IRK_RECORD_SIZE];
        size_t sz = 0;
        if (kv_get(key, rec, sizeof(rec), &sz) != 0 || sz != BLE_IRK_RECORD_SIZE) {
            continue;
        }
        (*IRKs)[loaded]        = new uint8_t[16];
        (*BDAddrs)[loaded]     = new uint8_t[6];
        memcpy((*IRKs)[loaded],    rec,      16);
        memcpy((*BDAddrs)[loaded], rec + 16, 6);
        (*BDAddrTypes)[loaded] = rec[22];
        loaded++;
    }

    *nIRKs = loaded;
    return 0;
}

// ---------------------------------------------------------------------------
// ble_bond_store_init  — register callbacks with ArduinoBLE.
//                        Call after BLE.begin(), before BLE.advertise().
// ---------------------------------------------------------------------------
static void ble_bond_store_init() {
    BLE.setStoreLTK(ble_storeLTK);
    BLE.setGetLTK(ble_getLTK);
    BLE.setStoreIRK(ble_storeIRK);
    BLE.setGetIRKs(ble_getIRKs);

    uint8_t count = 0;
    size_t actual = 0;
    kv_get("/kv/sp_irk_count", &count, 1, &actual);
    Serial.print("[bond] store init — ");
    Serial.print(count);
    Serial.println(" bonded device(s) in flash");
}
