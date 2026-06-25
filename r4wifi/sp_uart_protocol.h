#pragma once

/*
 * SecurePouch inter-MCU UART protocol
 * ===================================
 *
 * Link:  nRF52840 (Arduino Nano 33 BLE)  <-- UART -->  nRF9151 DK
 *
 *   nRF52840 Serial1 TX1 (P1.03 / D1) ----> nRF9151 P0.01 (RX)
 *   nRF52840 Serial1 RX0 (P1.10 / D0) <---- nRF9151 P0.00 (TX)
 *   nRF52840 D17/A3 --------------------> nRF9151 RESET  (active low, drive low to reset)
 *
 * Baud: 115200 8N1, no flow control.
 *
 * Authority model (confirmed): the nRF52840 owns the application / alarm /
 * lock / arm state machine and all GPIO. The nRF9151 is a radio co-processor
 * (RCP): it owns the cellular link + GNSS, forwards inbound server commands to
 * the 52840, and uploads whatever location / status the 52840 hands it.
 *
 * Wire format
 * -----------
 * One message per line, ASCII, terminated by '\n'. A message is:
 *
 *     <TAG>:<payload>\n
 *
 * The TAG is a short uppercase token. Payload format is per-tag (see below).
 * Lines that do not parse are ignored by both sides (forward-compatible).
 * Either side may emit human-readable log lines that do NOT contain ':' before
 * a space; the other side ignores anything it doesn't recognise.
 *
 * Direction legend:  [9151->52840]  inbound from network/RCP to the brain
 *                    [52840->9151]  outbound from the brain to the RCP
 */

/* ----- 9151 -> 52840 : inbound from the network / RCP --------------------- */

/* RCP link / network status. payload = one of: BOOT, LTE_UP, LTE_DOWN,
 * GNSS_FIX, GNSS_SEARCH, REG (registered to FMD server). The 52840 uses this
 * to drive the OLED "connectivity" line and to know when uploads are possible.
 *   e.g.  "RCP:LTE_UP\n"   "RCP:GNSS_FIX\n" */
#define SP_MSG_RCP_STATUS   "RCP"

/* A command pulled from the FMD server (or forwarded from BLE-via-9151, unused
 * for now). payload = the FMD command string, see SP_CMD_* below.
 *   e.g.  "CMD:unlock\n"   "CMD:arm\n" */
#define SP_MSG_COMMAND      "CMD"

/* Last known GNSS fix, so the 52840 can show coords on the OLED.
 * payload = "<lat>,<lon>" decimal degrees (or "none").
 *   e.g.  "FIX:37.4219,-122.0841\n" */
#define SP_MSG_FIX          "FIX"

/* ----- 52840 -> 9151 : outbound to the RCP -------------------------------- */

/* Push current device status so the 9151 can include it in the next upload and
 * so a future server-side status mirror works. payload = decimal status byte
 * (same bit layout as the BLE STATUS characteristic, SP_STATUS_* in
 * ble_protocol.h) optionally followed by ",bat=<0-100>".
 *   e.g.  "STAT:3,bat=87\n"   (3 = LOCKED|ARMED) */
#define SP_MSG_STATUS       "STAT"

/* Ask the 9151 to upload a location now (e.g. on alarm / tamper burst). The
 * 9151 attaches its own GNSS fix; the 52840 does not have GPS. Optional payload
 * = reason string for logging ("alarm","periodic","wake").
 *   e.g.  "UP:alarm\n" */
#define SP_MSG_UPLOAD_NOW   "UP"

/* Acknowledge that a forwarded command was applied, so the RCP can log it.
 * payload = the command string that was executed.
 *   e.g.  "ACK:unlock\n" */
#define SP_MSG_ACK          "ACK"

/* An asynchronous event the 9151 should surface to the server as an alarm /
 * tamper note on the next upload. payload = event token: TAMPER, DEADMAN,
 * ALARM, UNLOCK_OK, UNLOCK_FAIL.
 *   e.g.  "EVT:TAMPER\n" */
#define SP_MSG_EVENT        "EVT"

/* ----- Shared command vocabulary (FMD command strings) -------------------- *
 *
 * These are the exact strings POSTed to the FMD server's /api/v1/command
 * endpoint (Data field) by the web dashboard / Android app, polled by the 9151,
 * and forwarded over UART as "CMD:<string>". They are SecurePouch fork
 * additions; upstream FMD does not define them. Keep in sync with the dashboard
 * controls and the Android app.
 */
#define SP_CMD_LOCK     "lock"
#define SP_CMD_UNLOCK   "unlock"
#define SP_CMD_ARM      "arm"
#define SP_CMD_DISARM   "disarm"
#define SP_CMD_ALARM    "alarm"      /* trigger siren/strobe now */
#define SP_CMD_SILENCE  "silence"    /* stop an active alarm */
#define SP_CMD_LOCATE   "locate"     /* force a GNSS fix + location upload */

/* Line terminator and limits */
#define SP_UART_EOL        '\n'
#define SP_UART_BAUD       115200
#define SP_UART_LINE_MAX   96
