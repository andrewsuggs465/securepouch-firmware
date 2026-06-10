# SecurePouch Firmware

Firmware for the SecurePouch smart anti-theft travel pouch. The device uses two Nordic chips:

| Chip | Board (prototype) | Role |
|---|---|---|
| nRF52840 | Arduino Nano 33 BLE | BLE advertiser; drives sensors, lock solenoid, siren, LEDs |
| nRF9151 | nRF9151 DK | LTE-M modem + GNSS; uploads location to FMD server; polls commands |

The two chips communicate over UART. The nRF9151 is the network-facing side; the nRF52840 handles all local peripherals.

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

## nRF52840 — BLE tracker (prototype)

### What it does

Advertises a custom BLE service so the companion Android app can detect the pouch is nearby. When the app sees the service UUID, it connects, reads the `DEVICE_ID` characteristic, then posts the phone's GPS location to the FMD server on behalf of the pouch.

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

This must match the username registered in the FMD server for this device.

### Build and flash

```bash
cd nrf52840
pio run --target upload
pio device monitor          # confirm BLE advertising on serial output
```

### Verify without the Android app

Install **nRF Connect** (Nordic Semiconductor, free on Android/iOS). Scan for `SecurePouch`, connect, and read the `DEVICE_ID` characteristic to confirm the firmware is running.

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

## License

AGPL-3.0-or-later
