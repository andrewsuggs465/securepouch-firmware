/*
 * SecurePouch — nRF9151 firmware (RCP: LTE-M + GNSS + FMD HTTP client)
 * ===================================================================
 *
 * Role: radio co-processor. The nRF52840 owns all application logic; this chip
 * only talks to the outside world:
 *   - brings up LTE-M and (via the Location library) gets a position fix
 *   - uploads the fix to the FMD server (POST /api/v1/location)
 *   - polls the FMD server for commands (PUT /api/v1/command) and forwards them
 *     to the nRF52840 over uart2 as "CMD:<string>"
 *   - takes status / upload-now / event lines from the nRF52840 over uart2
 *
 * Inter-MCU protocol: firmware/shared/sp_uart_protocol.h.
 * FMD wire format (verified against fmd-server/backend/apiv1.go):
 *   POST /api/v1/location   body {"IDT":<token>,"Data":<location-json-or-cipher>}
 *   PUT  /api/v1/command    body {"IDT":<token>,"Data":""}
 *                           -> {"IDT":..,"Data":<cmd>,"UnixTime":..,"CmdSig":..}
 *
 * Auth: an access token (FMD "IDT"). For the prototype the token is provisioned
 * as a build define (SP_FMD_TOKEN); login/registration crypto is a P3 item.
 *
 * Encryption: the FMD server stores the POSTed Data field verbatim and the web
 * dashboard tries to RSA/AES-decrypt it. For the prototype we send PLAINTEXT
 * JSON (see build_location_json) — locations are stored and visible in the API,
 * and full hybrid encryption is the documented next step (P3). This is the
 * agreed "plaintext POST, crypto stub" milestone.
 *
 * Build: west build -b nrf9151dk/nrf9151/ns
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/tls_credentials.h>
#include <zephyr/net/http/client.h>

#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/location.h>
#include <modem/modem_key_mgmt.h>
#include <nrf_modem_at.h>
#include <date_time.h>

#include <string.h>
#include <stdio.h>

#include "sp_uart_protocol.h"

LOG_MODULE_REGISTER(securepouch_rcp, LOG_LEVEL_INF);

/* ---- Server / device configuration (override at build time) -------------- */
#ifndef SP_FMD_HOST
#define SP_FMD_HOST   "lintu-server.myvnc.com"
#endif
/* HTTPS on 443 — Caddy on the server handles TLS termination. */
#ifndef SP_FMD_PORT
#define SP_FMD_PORT   "443"
#endif
#ifndef SP_DEVICE_UID
#define SP_DEVICE_UID "securepouch-001"
#endif
#ifndef SP_FIRMWARE_ROLE
#define SP_FIRMWARE_ROLE "nRF9151-DK standalone LTE/GNSS"
#endif
/* FMD access token (IDT). Provision via -DSP_FMD_TOKEN=... or set here.
 * Empty token => uploads/polls are skipped (modem still attaches + fixes). */
#ifndef SP_FMD_TOKEN
#define SP_FMD_TOKEN  ""
#endif
/* APN for the IoT SIM (e.g. "onomondo", "melita", a Conexa APN). Empty => let
 * the modem use the network-provided default. Set via -DSP_APN='"onomondo"'. */
#ifndef SP_APN
#define SP_APN  ""
#endif
/* OpenCellID API token for cellular location fallback.
 * Pass at build time: -DSP_OPENCELLID_TOKEN=pk.xxxxx
 * Without this the cellular fallback request is skipped. */
#ifndef SP_OPENCELLID_TOKEN
#define SP_OPENCELLID_TOKEN  ""
#endif

#define LOCATION_PATH "/api/v1/location"
#define COMMAND_PATH  "/api/v1/command"

/* TLS security tag under which we store the modem's peer-verify-none session.
 * We use the modem's built-in CA roots (TLS_PEER_VERIFY_NONE for demo) so no
 * certificate provisioning is required here. */
#define SP_TLS_SEC_TAG  42

#define UPLOAD_PERIOD_MS  (2 * 60 * 1000)   /* location upload interval (2 min for demo) */
#define POLL_PERIOD_MS    (30 * 1000)        /* command poll cadence */

/* ---- UART to the nRF52840 ----------------------------------------------- */
#define MCU_UART_NODE DT_CHOSEN(securepouch_mcu_link)
static const struct device *mcu_uart = DEVICE_DT_GET(MCU_UART_NODE);

static char rx_line[SP_UART_LINE_MAX];
static size_t rx_len;
K_MSGQ_DEFINE(uart_lines, SP_UART_LINE_MAX, 8, 4);

/* ---- Forward declarations ------------------------------------------------ */
#if defined(CONFIG_LOCATION_METHOD_CELLULAR)
static void cellular_location_request(const struct location_data_cloud *req);
#endif

/* ---- State shared between threads ---------------------------------------- */
static K_SEM_DEFINE(lte_connected, 0, 1);
static atomic_t lte_registered = ATOMIC_INIT(0);
static struct location_data last_fix;
static bool have_fix;
static atomic_t location_in_progress = ATOMIC_INIT(0);
static atomic_t pouch_status = ATOMIC_INIT(0);   /* mirror of 52840 status byte */
static atomic_t pouch_battery = ATOMIC_INIT(100);
static atomic_t upload_request = ATOMIC_INIT(0); /* set by "UP:" from 52840 */

/* Cached resolved address for SP_FMD_HOST — resolved once after LTE attach. */
static struct sockaddr_storage fmd_server_addr;
static bool fmd_addr_cached = false;

/* ========================================================================= */
/* UART bridge                                                               */
/* ========================================================================= */
static void uart_send_line(const char *tag, const char *payload)
{
	char buf[SP_UART_LINE_MAX];
	int n = snprintf(buf, sizeof(buf), "%s:%s%c", tag, payload, SP_UART_EOL);
	for (int i = 0; i < n; i++) {
		uart_poll_out(mcu_uart, buf[i]);
	}
}

/* ISR: accumulate bytes into a line, push complete lines to the work queue. */
static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t c;

	if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) {
		return;
	}
	while (uart_fifo_read(dev, &c, 1) == 1) {
		if (c == '\r') {
			continue;
		}
		if (c == SP_UART_EOL) {
			rx_line[rx_len] = '\0';
			if (rx_len > 0) {
				(void)k_msgq_put(&uart_lines, rx_line, K_NO_WAIT);
			}
			rx_len = 0;
		} else if (rx_len < sizeof(rx_line) - 1) {
			rx_line[rx_len++] = c;
		} else {
			rx_len = 0; /* overflow — drop */
		}
	}
}

/* Parse one line received from the nRF52840. */
static void handle_mcu_line(char *line)
{
	char *sep = strchr(line, ':');
	if (!sep) {
		return;
	}
	*sep = '\0';
	const char *tag = line;
	const char *payload = sep + 1;

	if (strcmp(tag, SP_MSG_STATUS) == 0) {
		/* payload: "<status>" or "<status>,bat=<n>" */
		unsigned int st = 0, bat = 100;
		sscanf(payload, "%u", &st);
		const char *bat_p = strstr(payload, "bat=");
		if (bat_p) {
			sscanf(bat_p, "bat=%u", &bat);
		}
		atomic_set(&pouch_status, st);
		atomic_set(&pouch_battery, bat);
	} else if (strcmp(tag, SP_MSG_UPLOAD_NOW) == 0) {
		atomic_set(&upload_request, 1);
		LOG_INF("upload requested by MCU (%s)", payload);
	} else if (strcmp(tag, SP_MSG_EVENT) == 0) {
		LOG_INF("MCU event: %s", payload);
		atomic_set(&upload_request, 1); /* surface alarms/tamper promptly */
	} else if (strcmp(tag, SP_MSG_ACK) == 0) {
		LOG_INF("MCU ack: %s", payload);
	}
}

/* ========================================================================= */
/* LTE                                                                       */
/* ========================================================================= */
static void lte_handler(const struct lte_lc_evt *const evt)
{
	if (evt->type != LTE_LC_EVT_NW_REG_STATUS) {
		return;
	}
	switch (evt->nw_reg_status) {
	case LTE_LC_NW_REG_REGISTERED_HOME:
	case LTE_LC_NW_REG_REGISTERED_ROAMING:
		LOG_INF("LTE registered");
		atomic_set(&lte_registered, 1);
		k_sem_give(&lte_connected);
		break;
	case LTE_LC_NW_REG_SEARCHING:
		LOG_INF("LTE searching for a network...");
		break;
	case LTE_LC_NW_REG_NOT_REGISTERED:
		LOG_WRN("LTE not registered (no SIM / no service?)");
		break;
	case LTE_LC_NW_REG_REGISTRATION_DENIED:
		LOG_ERR("LTE registration denied (SIM not provisioned for this network?)");
		break;
	case LTE_LC_NW_REG_UNKNOWN:
		LOG_WRN("LTE registration status unknown");
		break;
	default:
		break;
	}
}

/* Set the APN on the default PDN context (CID 0). Must be done with the modem
 * offline (CFUN=0) — call before lte_lc_connect(). No-op if SP_APN is empty. */
static void configure_apn(void)
{
	if (strlen(SP_APN) == 0) {
		LOG_INF("APN: using network default");
		return;
	}
	/* Modem must be offline to change CID 0. nrf_modem_lib_init() leaves it in
	 * CFUN=0, but be explicit in case of a warm restart. */
	(void)nrf_modem_at_printf("AT+CFUN=0");
	int err = nrf_modem_at_printf("AT+CGDCONT=0,\"IP\",\"%s\"", SP_APN);
	if (err) {
		LOG_ERR("failed to set APN '%s' (err %d)", SP_APN, err);
	} else {
		LOG_INF("APN set to '%s'", SP_APN);
	}
}

/* Dump the modem's raw, unfiltered replies to the SIM-related AT commands.
 * This is the ground-truth diagnostic: if %XSIM stays 0 here, it is a physical
 * contact / card-holder / SIM-power issue, not firmware or SW1. */
static void dump_raw_sim_at(void)
{
	static const char *cmds[] = {
		"AT+CFUN?",      /* functional mode — must be 1/4 to power the UICC */
		"AT%XSIM?",      /* 1 = SIM present & initialised */
		"AT+CPIN?",      /* READY / SIM PIN / SIM not inserted */
		"AT%XICCID",     /* card serial — only answers if a card responds */
		"AT+CIMI",       /* IMSI — only with a live SIM */
	};
	char buf[160];
	for (size_t i = 0; i < ARRAY_SIZE(cmds); i++) {
		int err = nrf_modem_at_cmd(buf, sizeof(buf), "%s", cmds[i]);
		if (err == 0) {
			LOG_INF("RAW %s -> %s", cmds[i], buf);
		} else {
			LOG_WRN("RAW %s -> error %d", cmds[i], err);
		}
	}
}

/* Log SIM + modem state so a missing/inactive SIM is visible instead of a
 * silent forever-block on attach. Returns true if a SIM appears usable.
 *
 * The SIM subsystem can take a moment to come up after boot, so we poll
 * %XSIM (1 = card present & initialised) and CPIN with a few retries before
 * giving up, and log the raw responses so the failure cause is visible. */
static bool log_modem_diagnostics(void)
{
	char buf[96];
	bool sim_ok = false;
	int xsim = 0;

	if (nrf_modem_at_cmd(buf, sizeof(buf), "AT+CGSN=1") == 0) {
		LOG_INF("modem IMEI: %s", buf);
	}

	for (int attempt = 0; attempt < 10 && !sim_ok; attempt++) {
		/* %XSIM: <0|1> — 1 means a SIM is present and initialised. */
		if (nrf_modem_at_scanf("AT%%XSIM?", "%%XSIM: %d", &xsim) == 1 && xsim == 1) {
			sim_ok = true;
			break;
		}
		/* Fall back to CPIN; READY also means usable. */
		if (nrf_modem_at_scanf("AT+CPIN?", "+CPIN: %95[^\r\n]", buf) == 1 &&
		    strstr(buf, "READY") != NULL) {
			sim_ok = true;
			break;
		}
		/* Halfway through, force the modem to power-cycle and re-read the SIM.
		 * CFUN=1 (full functional) powers the UICC; CFUN=41 explicitly
		 * re-activates the UICC without enabling the radio. A debugger
		 * pin-reset alone does not re-power the SIM. */
		if (attempt == 4) {
			LOG_INF("re-initialising SIM (CFUN power-cycle)...");
			(void)nrf_modem_at_printf("AT+CFUN=0");
			k_sleep(K_MSEC(500));
			(void)nrf_modem_at_printf("AT+CFUN=1"); /* full: powers UICC */
			k_sleep(K_MSEC(2000));
		}
		k_sleep(K_MSEC(500));
	}

	if (sim_ok) {
		if (nrf_modem_at_scanf("AT+CPIN?", "+CPIN: %95[^\r\n]", buf) == 1) {
			LOG_INF("SIM ready (CPIN: %s, XSIM: %d)", buf, xsim);
		} else {
			LOG_INF("SIM ready (XSIM: %d)", xsim);
		}
		/* ICCID — confirms WHICH of several SIMs is inserted. */
		if (nrf_modem_at_scanf("AT%%XICCID", "%%XICCID: %95[^\r\n]", buf) == 1) {
			LOG_INF("SIM ICCID: %s", buf);
		}
	} else {
		/* Log the raw CPIN response so we can tell no-SIM from PIN-locked/busy. */
		if (nrf_modem_at_cmd(buf, sizeof(buf), "AT+CPIN?") == 0) {
			LOG_WRN("SIM not ready (XSIM:%d) — CPIN raw: %s", xsim, buf);
		} else {
			LOG_WRN("SIM not detected (XSIM:%d, CPIN errored)", xsim);
		}
		/* Raw AT dump for ground-truth: distinguishes physical contact / power
		 * problems (all error) from card-state issues (CPIN reports a reason). */
		dump_raw_sim_at();
	}
	return sim_ok;
}

/* Log network attach details (operator, band, signal) for debugging which
 * network the SIM roamed onto and how strong the signal is. Safe to call
 * repeatedly; fields are only populated once registered. */
static void log_network_status(void)
{
	char op[32] = "";
	int band = 0;
	char raw[160];

	/* %XMONITOR returns reg_status, full/short op name, PLMN, TAC, AcT, band,
	 * cell id, ... — log the raw line; it's the single most useful diagnostic. */
	if (nrf_modem_at_cmd(raw, sizeof(raw), "AT%%XMONITOR") == 0) {
		LOG_INF("XMONITOR: %s", raw);
	}
	/* Operator name + band, parsed for a concise line. */
	if (nrf_modem_at_scanf("AT%%XMONITOR",
			       "%%XMONITOR: %*d,\"%31[^\"]\",%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%d",
			       op, &band) >= 1) {
		LOG_INF("registered on '%s' band %d", op, band);
	}
}

/* ========================================================================= */
/* Location (GNSS + cellular fallback)                                        */
/* ========================================================================= */
static void location_event_handler(const struct location_event_data *event_data)
{
	switch (event_data->id) {
	case LOCATION_EVT_LOCATION:
		last_fix = event_data->location;
		have_fix = true;
		atomic_set(&location_in_progress, 0);
		LOG_INF("fix: %.06f, %.06f (+/- %d m) method=%d",
			last_fix.latitude, last_fix.longitude,
			(int)last_fix.accuracy, (int)event_data->method);
		{
			char fix[40];
			snprintf(fix, sizeof(fix), "%.5f,%.5f",
				 last_fix.latitude, last_fix.longitude);
			uart_send_line(SP_MSG_FIX, fix);
		}
		break;
	case LOCATION_EVT_TIMEOUT:
		atomic_set(&location_in_progress, 0);
		LOG_WRN("location request timed out (method=%d)", (int)event_data->method);
		break;
	case LOCATION_EVT_ERROR:
		atomic_set(&location_in_progress, 0);
		LOG_WRN("location request error (method=%d)", (int)event_data->method);
		break;
	case LOCATION_EVT_FALLBACK:
		LOG_INF("location: falling back to method %d", (int)event_data->method);
		break;
#if defined(CONFIG_LOCATION_METHOD_CELLULAR)
	case LOCATION_EVT_CLOUD_LOCATION_EXT_REQUEST:
		/* GNSS timed out; library wants us to resolve the cell data via our
		 * own cloud service. Call OpenCellID and feed the result back. */
		LOG_INF("location: GNSS timed out — querying OpenCellID for cellular fix");
		cellular_location_request(&event_data->cloud_location_request);
		break;
#endif
	default:
		break;
	}
}

static void request_location(void)
{
	if (atomic_get(&location_in_progress) == 1) {
		LOG_INF("location request already in progress; reusing pending fix attempt");
		return;
	}

	struct location_config config;
	enum location_method methods[] = {
		LOCATION_METHOD_GNSS,
#if defined(CONFIG_LOCATION_METHOD_CELLULAR)
		/* Cellular fallback via OpenCellID — fires
		 * LOCATION_EVT_CLOUD_LOCATION_EXT_REQUEST when GNSS times out. */
		LOCATION_METHOD_CELLULAR,
#endif
	};

	location_config_defaults_set(&config, ARRAY_SIZE(methods), methods);
	/* GNSS timeout: 30 s indoors is enough to know it won't work; fall through
	 * to cellular. Outdoors a cold-start fix arrives well within this window. */
	config.methods[0].gnss.timeout = 30 * MSEC_PER_SEC;

	atomic_set(&location_in_progress, 1);
	int err = location_request(&config);
	if (err) {
		atomic_set(&location_in_progress, 0);
		LOG_ERR("location_request failed: %d", err);
	}
}

/* ========================================================================= */
/* HTTPS                                                                      */
/* ========================================================================= */
static uint8_t http_recv_buf[1024];
static int http_status;
static char http_body[512];
static size_t http_body_len;

static int http_response_cb(struct http_response *rsp,
			    enum http_final_call final, void *user_data)
{
	ARG_UNUSED(user_data);
	if (rsp->body_frag_start && rsp->body_frag_len) {
		size_t n = MIN(rsp->body_frag_len, sizeof(http_body) - 1 - http_body_len);
		memcpy(http_body + http_body_len, rsp->body_frag_start, n);
		http_body_len += n;
		http_body[http_body_len] = '\0';
	}
	if (final == HTTP_DATA_FINAL) {
		http_status = rsp->http_status_code;
	}
	return 0;
}

/* Resolve SP_FMD_HOST once and cache the result. Returns 0 on success. */
static int resolve_fmd_host(void)
{
	if (fmd_addr_cached) {
		return 0;
	}
	struct zsock_addrinfo *res = NULL;
	struct zsock_addrinfo hints = {
		.ai_family   = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	int err = zsock_getaddrinfo(SP_FMD_HOST, SP_FMD_PORT, &hints, &res);
	if (err || !res) {
		LOG_ERR("DNS: getaddrinfo(%s) failed: %d", SP_FMD_HOST, err);
		return -EIO;
	}
	memcpy(&fmd_server_addr, res->ai_addr, res->ai_addrlen);
	zsock_freeaddrinfo(res);
	fmd_addr_cached = true;
	LOG_INF("DNS: %s resolved and cached", SP_FMD_HOST);
	return 0;
}

/* One blocking HTTPS request to the cached FMD server address.
 * Returns HTTP status code, or negative errno on network/TLS error.
 * Response body (if any) is left in http_body / http_body_len.
 *
 * TLS: modem-offloaded TLS 1.2 with TLS_PEER_VERIFY_NONE (demo mode —
 * no CA cert provisioning needed; upgrade to REQUIRED for production). */
static int http_request(enum http_method method, const char *path,
			const char *json)
{
	http_status = 0;
	http_body_len = 0;
	http_body[0] = '\0';

	if (resolve_fmd_host() != 0) {
		return -EIO;
	}

	int sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TLS_1_2);
	if (sock < 0) {
		LOG_ERR("TLS socket() failed: %d", -errno);
		fmd_addr_cached = false;   /* force re-resolve next time */
		return -errno;
	}

	/* TLS_PEER_VERIFY_NONE — demo convenience; no cert to provision.
	 * For production: provision ISRG Root X1 PEM via modem_key_mgmt_write()
	 * at sec_tag SP_TLS_SEC_TAG and switch verify to TLS_PEER_VERIFY_REQUIRED. */
	int verify = TLS_PEER_VERIFY_NONE;
	(void)zsock_setsockopt(sock, SOL_TLS, TLS_PEER_VERIFY, &verify, sizeof(verify));

	/* SNI: required for virtual-hosted servers (Caddy checks the hostname). */
	(void)zsock_setsockopt(sock, SOL_TLS, TLS_HOSTNAME,
			       SP_FMD_HOST, strlen(SP_FMD_HOST));

	if (zsock_connect(sock, (struct sockaddr *)&fmd_server_addr,
			  sizeof(struct sockaddr_in)) < 0) {
		LOG_ERR("TLS connect() failed: %d (errno %d)", -errno, errno);
		zsock_close(sock);
		fmd_addr_cached = false;   /* address may have changed — re-resolve */
		return -errno;
	}

	const char *headers[] = {
		"Content-Type: application/json\r\n",
		NULL,
	};
	struct http_request req = {
		.method          = method,
		.url             = path,
		.host            = SP_FMD_HOST,
		.protocol        = "HTTP/1.1",
		.header_fields   = headers,
		.response        = http_response_cb,
		.recv_buf        = http_recv_buf,
		.recv_buf_len    = sizeof(http_recv_buf),
	};
	if (json) {
		req.payload     = json;
		req.payload_len = strlen(json);
	}

	int ret = http_client_req(sock, &req, 15 * MSEC_PER_SEC, NULL);
	zsock_close(sock);

	if (ret < 0) {
		LOG_ERR("http_client_req(%s) failed: %d", path, ret);
		return ret;
	}
	return http_status;
}

/* ========================================================================= */
/* OpenCellID — cellular location fallback                                    */
/* ========================================================================= */
/* Called from the location event handler when GNSS times out and the library
 * fires LOCATION_EVT_CLOUD_LOCATION_EXT_REQUEST with serving-cell data.
 * Queries the OpenCellID REST API and feeds the result back to the library. */
#if defined(CONFIG_LOCATION_METHOD_CELLULAR)
static void cellular_location_request(const struct location_data_cloud *req)
{
	if (strlen(SP_OPENCELLID_TOKEN) == 0) {
		LOG_WRN("No SP_OPENCELLID_TOKEN built in — cellular location skipped");
		location_cloud_location_ext_result_set(LOCATION_EXT_RESULT_ERROR, NULL);
		return;
	}

	const struct lte_lc_cell *cell = &req->cell_data->current_cell;
	if (cell->id == LTE_LC_CELL_EUTRAN_ID_INVALID) {
		LOG_WRN("No valid serving cell — cellular location skipped");
		location_cloud_location_ext_result_set(LOCATION_EXT_RESULT_ERROR, NULL);
		return;
	}

	/* Build OpenCellID request body.
	 * API: https://us1.unwiredlabs.com/v2/process
	 * Required fields: token, radio, mcc, mnc, cells[{lac, cid}] */
	char body[256];
	int n = snprintf(body, sizeof(body),
		"{\"token\":\"%s\",\"radio\":\"lte\",\"mcc\":%d,\"mnc\":%d,"
		"\"cells\":[{\"lac\":%u,\"cid\":%u}],\"address\":1}",
		SP_OPENCELLID_TOKEN,
		cell->mcc, cell->mnc,
		(unsigned)cell->tac, (unsigned)cell->id);
	if (n <= 0 || (size_t)n >= sizeof(body)) {
		LOG_ERR("OpenCellID body truncated");
		location_cloud_location_ext_result_set(LOCATION_EXT_RESULT_ERROR, NULL);
		return;
	}

	/* Send HTTPS POST to OpenCellID.
	 * We build a one-off connection here (different host than FMD server). */
	static uint8_t ocid_recv_buf[512];
	static char ocid_body[256];
	static int ocid_status;
	static size_t ocid_body_len;
	ocid_status = 0; ocid_body_len = 0; ocid_body[0] = '\0';

	struct zsock_addrinfo *res = NULL;
	struct zsock_addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
	if (zsock_getaddrinfo("us1.unwiredlabs.com", "443", &hints, &res) || !res) {
		LOG_ERR("DNS: us1.unwiredlabs.com failed");
		location_cloud_location_ext_result_set(LOCATION_EXT_RESULT_ERROR, NULL);
		return;
	}

	int sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TLS_1_2);
	if (sock < 0) {
		zsock_freeaddrinfo(res);
		location_cloud_location_ext_result_set(LOCATION_EXT_RESULT_ERROR, NULL);
		return;
	}
	int verify = TLS_PEER_VERIFY_NONE;
	(void)zsock_setsockopt(sock, SOL_TLS, TLS_PEER_VERIFY, &verify, sizeof(verify));
	static const char ocid_host[] = "us1.unwiredlabs.com";
	(void)zsock_setsockopt(sock, SOL_TLS, TLS_HOSTNAME, ocid_host, strlen(ocid_host));

	if (zsock_connect(sock, res->ai_addr, res->ai_addrlen) < 0) {
		LOG_ERR("OpenCellID connect failed: %d", errno);
		zsock_close(sock);
		zsock_freeaddrinfo(res);
		location_cloud_location_ext_result_set(LOCATION_EXT_RESULT_ERROR, NULL);
		return;
	}
	zsock_freeaddrinfo(res);

	/* Re-use the same http_response_cb pattern but with local buffers. */
	struct http_response ocid_rsp_data = { 0 };
	ARG_UNUSED(ocid_rsp_data);

	/* Simple inline response accumulator for OpenCellID. */
	static struct { uint8_t *buf; char *body; size_t *body_len; int *status; } ocid_ctx;
	ocid_ctx.buf      = ocid_recv_buf;
	ocid_ctx.body     = ocid_body;
	ocid_ctx.body_len = &ocid_body_len;
	ocid_ctx.status   = &ocid_status;

	const char *ocid_headers[] = { "Content-Type: application/json\r\n", NULL };
	struct http_request ocid_req = {
		.method        = HTTP_POST,
		.url           = "/v2/process",
		.host          = ocid_host,
		.protocol      = "HTTP/1.1",
		.header_fields = ocid_headers,
		.payload       = body,
		.payload_len   = strlen(body),
		.response      = http_response_cb,  /* reuses global http_body / http_status */
		.recv_buf      = http_recv_buf,
		.recv_buf_len  = sizeof(http_recv_buf),
	};

	/* Note: http_response_cb writes into the global http_body / http_status.
	 * This is safe because cellular_location_request() is only called from the
	 * location event handler (main thread context), not concurrently with an FMD
	 * HTTP request. */
	http_status = 0; http_body_len = 0; http_body[0] = '\0';
	int ret = http_client_req(sock, &ocid_req, 15 * MSEC_PER_SEC, NULL);
	zsock_close(sock);

	if (ret < 0 || http_status != 200) {
		LOG_WRN("OpenCellID request failed: ret=%d status=%d", ret, http_status);
		location_cloud_location_ext_result_set(LOCATION_EXT_RESULT_ERROR, NULL);
		return;
	}

	/* Parse {"status":"ok","lat":...,"lon":...,"accuracy":...} */
	double lat = 0.0, lon = 0.0, acc = 1000.0;
	char status_str[8] = "";
	/* Simple sscanf extraction — fields may come in any order so try each. */
	char *p;
	if ((p = strstr(http_body, "\"lat\":"))  != NULL) { sscanf(p, "\"lat\":%lf",  &lat); }
	if ((p = strstr(http_body, "\"lon\":"))  != NULL) { sscanf(p, "\"lon\":%lf",  &lon); }
	if ((p = strstr(http_body, "\"accuracy\":")) != NULL) { sscanf(p, "\"accuracy\":%lf", &acc); }
	if ((p = strstr(http_body, "\"status\":\"")) != NULL) {
		sscanf(p, "\"status\":\"%7[^\"]\"", status_str);
	}

	if (strcmp(status_str, "ok") != 0) {
		LOG_WRN("OpenCellID: non-ok status '%s' body: %.100s", status_str, http_body);
		location_cloud_location_ext_result_set(LOCATION_EXT_RESULT_ERROR, NULL);
		return;
	}

	struct location_data fix = {
		.latitude  = lat,
		.longitude = lon,
		.accuracy  = (float)acc,
	};
	LOG_INF("OpenCellID fix: %.5f, %.5f (+/- %.0f m) mcc=%d mnc=%d tac=%u cid=%u",
		lat, lon, acc, cell->mcc, cell->mnc,
		(unsigned)cell->tac, (unsigned)cell->id);
	location_cloud_location_ext_result_set(LOCATION_EXT_RESULT_SUCCESS, &fix);
}
#endif /* CONFIG_LOCATION_METHOD_CELLULAR */

/* Build the FMD location JSON. PLAINTEXT for the prototype (crypto stub).
 * Field names match fmd-server web client Location interface
 * (web/src/lib/api.ts): lat, lon, bat, date, time, provider, accuracy. */
static int build_location_json(char *out, size_t out_len)
{
	int64_t now_ms = 0;
	(void)date_time_now(&now_ms);

	/* TODO(P3): replace this plaintext payload with the hybrid scheme the web
	 * dashboard expects: base64( RSA-OAEP(AES key)[384] | IV[12] | AES-GCM
	 * ciphertext ) of this same JSON. The server stores Data verbatim either
	 * way; only the dashboard's decrypt step differs. */
	int n = snprintf(out, out_len,
		"{\"IDT\":\"%s\",\"Data\":\"{"
		"\\\"lat\\\":%.6f,"
		"\\\"lon\\\":%.6f,"
		"\\\"bat\\\":%d,"
		"\\\"date\\\":%lld,"
			"\\\"provider\\\":\\\"nrf9151-gnss\\\","
		"\\\"accuracy\\\":%d}\"}",
		SP_FMD_TOKEN,
		last_fix.latitude, last_fix.longitude,
		(int)atomic_get(&pouch_battery),
		(long long)now_ms,
		(int)last_fix.accuracy);
	return (n > 0 && (size_t)n < out_len) ? n : -ENOMEM;
}

static void upload_location(void)
{
	if (!have_fix) {
		LOG_WRN("no fix yet — skipping upload");
		return;
	}
	if (strlen(SP_FMD_TOKEN) == 0) {
		LOG_WRN("no FMD token built in — skipping upload");
		return;
	}
	char json[512];
	if (build_location_json(json, sizeof(json)) < 0) {
		LOG_ERR("location json too long");
		return;
	}
	int code = http_request(HTTP_POST, LOCATION_PATH, json);
	LOG_INF("9151 standalone location upload -> HTTP %d", code);
	if (code == 200) {
		uart_send_line(SP_MSG_RCP_STATUS, "REG");
	} else if (code == 401) {
		/* Token expired. Full re-auth requires P3 crypto; for the demo build
		 * just notify the 52840 so the status LED reflects the failure. The
		 * fix is to re-flash with a fresh SP_FMD_TOKEN (./west build ... --
		 * -DSP_FMD_TOKEN=<new_token>). */
		LOG_ERR("location upload 401 — token expired; reflash with a fresh SP_FMD_TOKEN");
		uart_send_line(SP_MSG_RCP_STATUS, "NO_TOKEN");
	} else if (code >= 300 && code < 400) {
		LOG_WRN("location upload redirected; production server likely requires HTTPS/TLS");
	} else if (code < 0) {
		LOG_ERR("location upload network error: %d", code);
	}
}

/* Poll FMD for a pending command; forward it to the 52840. */
static void poll_command(void)
{
	if (strlen(SP_FMD_TOKEN) == 0) {
		return;
	}
	char body[128];
	snprintf(body, sizeof(body), "{\"IDT\":\"%s\",\"Data\":\"\"}", SP_FMD_TOKEN);

	int code = http_request(HTTP_PUT, COMMAND_PATH, body);
	LOG_INF("9151 standalone command poll -> HTTP %d (body_len=%u)",
		code, (unsigned int)http_body_len);
	if (code == 401) {
		LOG_ERR("command poll 401 — token expired; reflash with a fresh SP_FMD_TOKEN");
		uart_send_line(SP_MSG_RCP_STATUS, "NO_TOKEN");
		return;
	}
	if (code != 200 || http_body_len == 0) {
		if (code >= 300 && code < 400) {
			LOG_WRN("command poll redirected; production server likely requires HTTPS/TLS");
		} else if (code != 200) {
			LOG_WRN("command poll failed (HTTP %d); check token or server reachability", code);
		}
		return;
	}

	/* Response: {"IDT":..,"Data":"<cmd>","UnixTime":..,"CmdSig":..}.
	 * Minimal extraction of the Data field (no full JSON parse needed). */
	char *p = strstr(http_body, "\"Data\":\"");
	if (!p) {
		LOG_WRN("command poll response missing Data field");
		return;
	}
	p += strlen("\"Data\":\"");
	char *end = strchr(p, '"');
	if (!end || end == p) {
		LOG_INF("command poll: no pending command");
		return;   /* empty Data => no pending command */
	}
	*end = '\0';
	LOG_INF("server command via 9151 LTE path: %s", p);
	uart_send_line(SP_MSG_COMMAND, p);
	/* TODO(P3): verify CmdSig (RSA-PSS over "UnixTime:Data") before acting. */
}

/* ========================================================================= */
/* Worker thread: handle UART lines from the 52840 off the ISR              */
/* ========================================================================= */
static void uart_worker(void)
{
	char line[SP_UART_LINE_MAX];
	while (1) {
		k_msgq_get(&uart_lines, line, K_FOREVER);
		handle_mcu_line(line);
	}
}
K_THREAD_DEFINE(uart_worker_id, 2048, uart_worker, NULL, NULL, NULL, 7, 0, 0);

/* ========================================================================= */
/* main                                                                      */
/* ========================================================================= */
int main(void)
{
	LOG_INF("SecurePouch starting uid=%s role=%s host=%s apn=%s token=%s",
		SP_DEVICE_UID, SP_FIRMWARE_ROLE, SP_FMD_HOST, SP_APN,
		strlen(SP_FMD_TOKEN) == 0 ? "missing" : "set");

	if (!device_is_ready(mcu_uart)) {
		LOG_ERR("MCU-link UART not ready");
		return -1;
	}
	uart_irq_callback_user_data_set(mcu_uart, uart_isr, NULL);
	uart_irq_rx_enable(mcu_uart);
	uart_send_line(SP_MSG_RCP_STATUS, "BOOT");
	uart_send_line(SP_MSG_RCP_STATUS, "mode=9151-lte");  /* informational; 52840 ignores unknown tokens */
	if (strlen(SP_FMD_TOKEN) == 0) {
		uart_send_line(SP_MSG_RCP_STATUS, "NO_TOKEN");
		LOG_WRN("no FMD token built in — standalone server polling/uploads disabled");
	}

	int err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("nrf_modem_lib_init failed: %d", err);
		return -1;
	}

	err = location_init(location_event_handler);
	if (err) {
		LOG_ERR("location_init failed: %d", err);
	}

	bool sim_ok = log_modem_diagnostics();
	if (!sim_ok) {
		LOG_WRN("no usable SIM — LTE uploads/polls disabled; GNSS + UART still run");
		uart_send_line(SP_MSG_RCP_STATUS, "NO_SIM");
	}

	configure_apn();   /* must run while modem is offline (before connect) */

	lte_lc_register_handler(lte_handler);
	LOG_INF("connecting to LTE-M...");
	uart_send_line(SP_MSG_RCP_STATUS, "LTE_SEARCH");
	err = lte_lc_connect_async(lte_handler);
	if (err) {
		LOG_ERR("lte_lc_connect_async failed: %d", err);
	}
	/* Bounded wait: don't hang forever if there's no SIM / no coverage. The
	 * device still runs (GNSS, UART bridge); network-dependent steps no-op and
	 * are retried each loop once registration eventually completes. */
	bool lte_up = (k_sem_take(&lte_connected, K_SECONDS(120)) == 0);
	if (lte_up) {
		uart_send_line(SP_MSG_RCP_STATUS, "LTE_UP");
		log_network_status();
		(void)date_time_update_async(NULL);
		/* Pre-resolve the FMD server hostname right after attach so the first
		 * upload doesn't pay a DNS RTT while the location is still warm. */
		resolve_fmd_host();
	} else {
		LOG_WRN("LTE not up after 120s — continuing; will retry uploads when registered");
	}

	int64_t last_upload = 0;
	int64_t last_poll = 0;
	bool announced_up = lte_up;

	while (1) {
		int64_t now = k_uptime_get();
		bool registered = atomic_get(&lte_registered) == 1;

		/* If LTE came up late (e.g. SIM inserted after boot), announce it once
		 * and sync time so uploads/polls can begin. */
		if (registered && !announced_up) {
			announced_up = true;
			uart_send_line(SP_MSG_RCP_STATUS, "LTE_UP");
			log_network_status();
			(void)date_time_update_async(NULL);
			resolve_fmd_host();
		}

		bool forced = atomic_set(&upload_request, 0) == 1;
		if (forced || now - last_upload >= UPLOAD_PERIOD_MS) {
			uart_send_line(SP_MSG_RCP_STATUS, "GNSS_SEARCH");
			request_location();   /* async; location_event_handler fires on result */

			/* Wait for the fix attempt to complete (GNSS up to 30 s timeout,
			 * then cellular via OpenCellID). Poll in 1 s ticks; total wait cap
			 * = GNSS timeout (30 s) + cellular RTT (~5 s) + headroom. */
			for (int wait = 0;
			     wait < 45 && atomic_get(&location_in_progress) == 1;
			     wait++) {
				k_sleep(K_SECONDS(1));
			}

			if (registered) {
				upload_location();
			}
			last_upload = k_uptime_get();
		}

		if (registered && now - last_poll >= POLL_PERIOD_MS) {
			poll_command();
			last_poll = now;
		}

		k_sleep(K_SECONDS(1));
	}
}
