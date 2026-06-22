# SecurePouch Firmware

Firmware for the SecurePouch smart anti-theft travel pouch. The device uses two
Nordic chips, prototyped on a breadboard:

| Chip | Board (prototype) | Role | Status |
|---|---|---|---|
| nRF52840 | Arduino Nano 33 BLE | Application brain: BLE + all GPIO (IMU, lock, siren, LEDs, fingerprint); owns the arm/lock/alarm state machine | Builds + runs |
| nRF9151 | nRF9151 DK | Radio co-processor (RCP): LTE-M + GNSS; FMD HTTP client; forwards server commands to the 52840 over UART | Builds (GNSS-only, plaintext upload) |

The two chips talk over a UART. The **nRF52840 owns all application logic**; the
**nRF9151 is a dumb network pipe** (RCP). See
[shared/sp_uart_protocol.h](shared/sp_uart_protocol.h) for the inter-MCU contract
and [shared/ble_protocol.h](shared/ble_protocol.h) for the BLE GATT contract.

---

## Control paths

Every arm / disarm / lock / unlock / alarm / silence / locate action funnels
through one dispatcher (`applyCommand()` on the nRF52840). It can be triggered
from three places:

1. **BLE** — the companion app writes an `SP_CTRL_*` opcode to the CONTROL
   characteristic (`…bc054`).
2. **LTE** — the web dashboard / app POSTs an FMD command (`lock`, `unlock`,
   `arm`, `disarm`, `alarm`, `silence`, `locate`); the nRF9151 polls it from the
   server and forwards it over UART as `CMD:<string>`.
3. **Local** — fingerprint match unlocks + disarms; accelerometer tamper or the
   BLE dead-man switch raises the alarm.

```
 Web dashboard / app ──POST /api/v1/command──► FMD server
                                                  │  (nRF9151 polls PUT /command)
 Companion app ──BLE CONTROL write──┐             ▼
                                    ├──► nRF52840 state machine ──► lock / siren / LEDs
 Fingerprint / accel / dead-man ────┘             ▲
                                                  │  status + GNSS fix
 nRF9151 ──UART (STAT/FIX/CMD/UP/EVT)─────────────┘
   │  POST /api/v1/location (phone-free GNSS)
   ▼
 FMD server ──► web map dashboard
```

---

## Breadboard wiring

### nRF52840 (Arduino Nano 33 BLE)

| Pin | Net | Notes |
|---|---|---|
| D1 / TX1 | → nRF9151 P0.01 (RX) | `Serial1` hardware UART |
| D0 / RX0 | ← nRF9151 P0.00 (TX) | `Serial1` |
| D2 / D3 | Buzzer + / − | piezo H-bridge legs, anti-phase ~2.9 kHz |
| D4 / D5 / D6 | RGB LED R / G / B | active-high, external common-cathode |
| D7 / D8 | Solenoid + / − | fail-locked spring; pulse to unlock |
| D9 / D10 | Fingerprint RX / TX | SEN0348 over a **2nd hardware UART** (`arduino::UART`) |
| D11 | Fingerprint EN | active-high enable |
| D12 | Fingerprint IRQ | finger-touch, active-high |
| D17 / A3 | → nRF9151 RESET | active-low |
| (internal) | Motion sensing | onboard **LSM9DS1** IMU on internal I²C (`Wire1`); no external accel, A0–A2 free |

> The Nano 33 BLE has two UARTE peripherals. `Serial1` (D0/D1) is the nRF9151
> link; the fingerprint sensor uses a second `arduino::UART` bound to D9/D10 — a
> real hardware UART, not bit-banged software serial, so the SEN0348 runs at its
> native 115200.
>
> Tamper detection uses the board's built-in LSM9DS1 (via `Arduino_LSM9DS1`),
> which lives on the internal I²C bus, so it needs no external wiring and frees
> the analog pins. Tamper = transient deviation of the acceleration-vector
> magnitude from the at-rest baseline (calibrated at boot).

### nRF9151 DK

| Pin | Net |
|---|---|
| P0.00 | TX → nRF52840 D0 (RX0) |
| P0.01 | RX ← nRF52840 D1 (TX1) |
| RESET | ← nRF52840 D17 |

UART pins are assigned to `uart2` in
[nrf9151/boards/nrf9151dk_nrf9151_ns.overlay](nrf9151/boards/nrf9151dk_nrf9151_ns.overlay).

---

## nRF52840 — build & flash (PlatformIO / Arduino)

Per-unit: set `DEVICE_UID` in [nrf52840/src/main.cpp](nrf52840/src/main.cpp) to
the FMD server account name for this unit.

```bash
cd nrf52840
~/.platformio/penv/bin/pio run                 # build  (pio not on PATH)
~/.platformio/penv/bin/pio run -t upload       # flash
~/.platformio/penv/bin/pio device monitor      # 115200 baud
```

The SEN0348 driver (`DFRobot_ID809`) is vendored in
[nrf52840/lib/DFRobot_ID809/](nrf52840/lib/DFRobot_ID809/); ArduinoBLE +
Adafruit SSD1306/GFX are pulled by `platformio.ini`.

### Serial test console (no app needed)

The 52840 exposes a console on USB serial for breadboard bring-up:

| Key | Action |
|---|---|
| `l` / `u` | lock / unlock (pulses solenoid) |
| `a` / `d` | arm / disarm |
| `!` / `s` | alarm / silence |
| `g` | locate (asks the 9151 for a fix + upload) |
| `e` | enroll a new fingerprint (3 presses) |
| `x` | delete all fingerprint templates |

Status is shown on the RGB LED (blue=locked-idle, green=unlocked, amber=armed,
red=alarm) and logged over USB serial (RCP net status, GNSS fixes, events).

---

## nRF9151 — build & flash (nRF Connect SDK / Zephyr)

Built and verified against **NCS v3.3.1**. Board target is `nrf9151dk/nrf9151/ns`
(the only variant; TF-M is mandatory). The inter-MCU UART is `uart2` on
P0.00/P0.01 via the board overlay.

```bash
cd nrf9151
west build -b nrf9151dk/nrf9151/ns --build-dir build
west flash --build-dir build
```

Provision the FMD access token and (optionally) host at build time:

```bash
west build -b nrf9151dk/nrf9151/ns --build-dir build -- \
  -DCONFIG... \
  -DSP_FMD_TOKEN='"<access-token>"' \
  -DSP_DEVICE_UID='"securepouch-001"' \
  -DSP_FMD_HOST='"lintu-server.myvnc.com"'
```

(Defaults are in [nrf9151/src/main.c](nrf9151/src/main.c). With an empty token
the modem still attaches and fixes but skips uploads/polls.)

### Prototype scope / TODO (matches roadmap P3)

- **Plaintext location upload.** The FMD server stores the POSTed `Data` field
  verbatim; we send plaintext JSON so the location is recorded and visible via
  the API. The web dashboard expects hybrid RSA-OAEP+AES-GCM ciphertext, so it
  will not *decrypt* a plaintext payload on the map yet — see the crypto stub in
  `build_location_json()`. **P3:** implement the hybrid scheme (mbedTLS) to match
  `fmd-server/web/src/lib/crypto.ts`.
- **Token only (no on-device login).** Registration/login crypto (Argon2 → `/salt`,
  `/requestAccess`) is a P3 item; provision a token via `SP_FMD_TOKEN`, or refresh
  it on a 401 (also P3).
- **GNSS-only fix.** Cellular/Wi-Fi fallback needs an nRF Cloud location service
  account; disabled for the standalone prototype.
- **Plaintext HTTP.** Port 80. For HTTPS, provision the CA to a modem sec-tag and
  open an `IPPROTO_TLS_1_2` socket (see NCS `modem_key_mgmt`); Caddy fronts the
  server with Let's Encrypt.
- **Command signature unverified.** `CmdSig` (RSA-PSS over `UnixTime:Data`) is not
  yet checked before acting on a command.

---

## Inter-MCU UART protocol

Newline-delimited ASCII, `TAG:payload\n`, 115200 8N1. Full definition in
[shared/sp_uart_protocol.h](shared/sp_uart_protocol.h).

| Dir | Tag | Payload | Meaning |
|---|---|---|---|
| 9151→52840 | `CMD` | command string | server command to execute |
| 9151→52840 | `RCP` | `BOOT`/`LTE_UP`/`GNSS_SEARCH`/`REG`… | RCP/link status (logged) |
| 9151→52840 | `FIX` | `lat,lon` | last GNSS fix (logged) |
| 52840→9151 | `STAT` | `<statusByte>,bat=<n>` | device status mirror |
| 52840→9151 | `UP` | reason | request an immediate location upload |
| 52840→9151 | `EVT` | `TAMPER`/`DEADMAN`/`ALARM`… | event to surface to the server |
| 52840→9151 | `ACK` | command string | command was applied |

---

## Server-side fork changes still needed (P2)

The firmware already speaks the FMD protocol; these are the **fmd-server /
fmd-android** changes that complete the loop (do not exist upstream):

- Dashboard controls per tracker tab: `lock` / `unlock` / `arm` / `disarm` /
  `alarm` / `silence` / `locate` buttons that call `sendCommand()` with the
  matching `SP_CMD_*` string.
- (Optional) server-side recognition of the new command strings for the event
  log; the command channel itself is already free-form so polling works as-is.

---

## License

AGPL-3.0-or-later
