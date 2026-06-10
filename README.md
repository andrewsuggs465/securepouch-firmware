# SecurePouch Firmware

Firmware for the SecurePouch smart anti-theft travel pouch. The device uses two Nordic chips:

| Chip | Board (prototype) | Role | Status |
|---|---|---|---|
| nRF52840 | Arduino Nano 33 BLE | BLE advertiser; drives sensors, lock solenoid, siren, LEDs | Prototype working |
| nRF9151 | nRF9151 DK | LTE-M modem + GNSS; uploads location to FMD server; polls commands | Stub — hardware not yet available |

The two chips communicate over UART. The nRF9151 is the network-facing side; the nRF52840 handles all local peripherals.

---

## How it works (AirTag-style proxy flow)

The Nano 33 BLE prototype has no network connection. Instead it uses the user's phone as a location proxy:

```
Nano 33 BLE                  Android companion app         FMD Server
    │                               │                           │
    │── advertise SP_SERVICE_UUID ──▶│                           │
    │                               │ (scan detects pouch)      │
    │◀──────────── connect ─────────│                           │
    │── DEVICE_ID char ────────────▶│                           │
    │◀──────────── disconnect ──────│                           │
    │                               │── POST /api/v1/locations ▶│
    │                               │   (phone GPS, pouch creds)│
    │                               │                    [pouch shown on map]
```

The pouch has its own account on the FMD server. When the companion app sees the pouch's BLE advertisement, it reads the `DEVICE_ID` characteristic, then posts the phone's current GPS coordinates to the server *using the pouch's credentials*. The pouch appears as a separate tracked device on the web dashboard.

---

## Repository layout

```
firmware/
├── shared/
│   └── ble_protocol.h      ← BLE UUIDs and status flags (shared between chips and Android app)
├── nrf52840/               ← Arduino/PlatformIO project
│   ├── platformio.ini
│   └── src/main.cpp
└── nrf9151/                ← Zephyr/NCS project (stub — hardware not yet available)
    ├── CMakeLists.txt
    ├── prj.conf
    └── src/main.c
```

---

## nRF52840 — BLE advertiser (prototype)

### BLE protocol

Defined in [shared/ble_protocol.h](shared/ble_protocol.h).

| UUID | Type | Description |
|---|---|---|
| `af19b3e4-...-bc051` | Service | SecurePouch primary service (advertised) |
| `af19b3e4-...-bc052` | Characteristic (Read) | Device UID string — matches FMD server account |
| `af19b3e4-...-bc053` | Characteristic (Read + Notify) | Status byte: `0x01` = locked, `0x02` = armed |

No BLE pairing or bonding required — the characteristics are unauthenticated read-only.

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)

### Per-unit setup

Before flashing each unit, set the device UID in [nrf52840/src/main.cpp](nrf52840/src/main.cpp):

```cpp
static const char* DEVICE_UID = "securepouch-001";  // unique per unit
```

This must match the username of the account registered on the FMD server for this device.

### Build and flash

```bash
cd nrf52840
pio run --target upload
pio device monitor          # confirm BLE advertising on serial output
```

Expected serial output:
```
SecurePouch BLE advertising — address: xx:xx:xx:xx:xx:xx
Device UID: securepouch-001
```

### Verify without the Android app

Install **nRF Connect** (Nordic Semiconductor, free on Android/iOS). Scan for `SecurePouch`, connect, and read the `DEVICE_ID` characteristic to confirm the firmware is running correctly.

---

## nRF9151 — LTE-M/GNSS (planned)

> Hardware not yet available. This is a Zephyr/NCS stub.

Build system is [nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/getting_started.html) (west + Zephyr).

Planned responsibilities:
- Acquire GNSS fix via integrated modem
- `POST /api/v1/locations/{uid}` to FMD server over LTE-M
- `GET /api/v1/commands/{uid}` — poll for lock/unlock/arm/disarm/alarm commands
- Relay commands to nRF52840 over UART

```bash
cd nrf9151
west build -b nrf9151dk/nrf9151
west flash
```

---

## Prototype TODO

Steps remaining to get the Nano 33 BLE working as a full AirTag-style tracker on the FMD server map.

### 1 — Firmware

- [ ] Set `DEVICE_UID` to a real value (e.g. `securepouch-001`) in `nrf52840/src/main.cpp` and flash
- [ ] Confirm BLE advertising via serial monitor and nRF Connect app

### 2 — Server: register the device account

The pouch needs its own FMD server account so it appears as a separate device on the map.

- [ ] Register account via curl (run from any machine):
  ```bash
  curl -X POST https://lintu-server.duckdns.org/api/v1/register \
    -H "Content-Type: application/json" \
    -d '{
      "username": "securepouch-001",
      "password": "<choose a password>",
      "registrationToken": "<vault_lintu_registration_token>"
    }'
  ```
  Save the returned access token — the Android app will use it to post locations.
- [ ] Confirm the device account appears in the FMD web dashboard

### 3 — Android app: BLE scanner service

This is the main remaining work. The fmd-android fork needs a background service that:

- [ ] **`BLETrackerService`** — foreground service using `BluetoothLeScanner`
  - Scans for devices advertising `SP_SERVICE_UUID` (`af19b3e4-d279-4a2a-9d3f-2f5e8a6bc051`)
  - On detection: connect → read `DEVICE_ID` char → disconnect
  - Looks up stored access token for that `DEVICE_ID`
  - POSTs current phone GPS to `POST /api/v1/locations` using the pouch's access token
  - Configurable scan interval (default: 30 s — balance between latency and battery)

- [ ] **Paired device storage** — `SharedPreferences` map of `DEVICE_ID → access_token`
  - For the prototype, this can be a single hardcoded entry

- [ ] **Pairing UI** (can be deferred for prototype — hardcode credentials first)
  - "Add SecurePouch" screen: scan for nearby SP_SERVICE_UUID devices
  - User selects device → enters server password → app fetches and stores access token

- [ ] **Out-of-range notification** — alert when a previously-seen pouch hasn't been detected for N minutes

### 4 — End-to-end test

- [ ] Nano 33 advertising → phone running BLETrackerService → location appears on map at `https://lintu-server.duckdns.org`
- [ ] Walk away from the Nano → verify map location stays pinned at last-seen position
- [ ] Kill and restart the companion app → verify service restarts and resumes reporting

---

## License

AGPL-3.0-or-later
