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
#include <zephyr/net/http/client.h>

#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/location.h>
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
#ifndef SP_FMD_PORT
#define SP_FMD_PORT   "80"           /* plaintext for the prototype */
#endif
#ifndef SP_DEVICE_UID
#define SP_DEVICE_UID "securepouch-001"
#endif
/* FMD access token (IDT). Provision via -DSP_FMD_TOKEN=... or set here.
 * Empty token => uploads/polls are skipped (modem still attaches + fixes). */
#ifndef SP_FMD_TOKEN
#define SP_FMD_TOKEN  ""
#endif

#define LOCATION_PATH "/api/v1/location"
#define COMMAND_PATH  "/api/v1/command"

#define UPLOAD_PERIOD_MS  (5 * 60 * 1000)   /* periodic location upload */
#define POLL_PERIOD_MS    (30 * 1000)       /* command poll cadence */

/* ---- UART to the nRF52840 ----------------------------------------------- */
#define MCU_UART_NODE DT_CHOSEN(securepouch_mcu_link)
static const struct device *mcu_uart = DEVICE_DT_GET(MCU_UART_NODE);

static char rx_line[SP_UART_LINE_MAX];
static size_t rx_len;
K_MSGQ_DEFINE(uart_lines, SP_UART_LINE_MAX, 8, 4);

/* ---- State shared between threads ---------------------------------------- */
static K_SEM_DEFINE(lte_connected, 0, 1);
static atomic_t lte_registered = ATOMIC_INIT(0);
static struct location_data last_fix;
static bool have_fix;
static atomic_t pouch_status = ATOMIC_INIT(0);   /* mirror of 52840 status byte */
static atomic_t pouch_battery = ATOMIC_INIT(100);
static atomic_t upload_request = ATOMIC_INIT(0); /* set by "UP:" from 52840 */

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
		/* payload: "<status>,bat=<n>" */
		unsigned int st = 0, bat = 100;
		sscanf(payload, "%u,bat=%u", &st, &bat);
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

/* Log SIM + modem state so a missing/inactive SIM is visible instead of a
 * silent forever-block on attach. Returns true if a SIM appears usable. */
static bool log_modem_diagnostics(void)
{
	char buf[64];
	bool sim_ok = false;

	if (nrf_modem_at_scanf("AT+CPIN?", "+CPIN: %63[^\r\n]", buf) == 1) {
		LOG_INF("SIM (CPIN): %s", buf);
		sim_ok = (strstr(buf, "READY") != NULL);
	} else {
		LOG_WRN("SIM not detected (AT+CPIN? failed) — insert/activate a SIM");
	}

	if (nrf_modem_at_cmd(buf, sizeof(buf), "AT+CGSN=1") == 0) {
		LOG_INF("modem IMEI: %s", buf);
	}
	return sim_ok;
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
		LOG_INF("fix: %.06f, %.06f (+/- %d m)",
			last_fix.latitude, last_fix.longitude,
			(int)last_fix.accuracy);
		{
			char fix[40];
			snprintf(fix, sizeof(fix), "%.5f,%.5f",
				 last_fix.latitude, last_fix.longitude);
			uart_send_line(SP_MSG_FIX, fix);
		}
		break;
	case LOCATION_EVT_TIMEOUT:
		LOG_WRN("location request timed out");
		break;
	case LOCATION_EVT_ERROR:
		LOG_WRN("location request error");
		break;
	default:
		break;
	}
}

static void request_location(void)
{
	struct location_config config;
	enum location_method methods[] = {
		LOCATION_METHOD_GNSS,       /* TODO(P3): add CELLULAR fallback + nRF Cloud */
	};

	location_config_defaults_set(&config, ARRAY_SIZE(methods), methods);
	int err = location_request(&config);
	if (err) {
		LOG_ERR("location_request failed: %d", err);
	}
}

/* ========================================================================= */
/* HTTP                                                                       */
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

/* One blocking HTTP request. Returns HTTP status code, or negative on error.
 * Response body (if any) is left in http_body / http_body_len. */
static int http_request(enum http_method method, const char *path,
			const char *json)
{
	struct zsock_addrinfo *res = NULL;
	struct zsock_addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};

	http_status = 0;
	http_body_len = 0;
	http_body[0] = '\0';

	int err = zsock_getaddrinfo(SP_FMD_HOST, SP_FMD_PORT, &hints, &res);
	if (err) {
		LOG_ERR("getaddrinfo(%s) failed: %d", SP_FMD_HOST, err);
		return -EIO;
	}

	int sock = zsock_socket(res->ai_family, res->ai_socktype, IPPROTO_TCP);
	if (sock < 0) {
		LOG_ERR("socket() failed: %d", -errno);
		zsock_freeaddrinfo(res);
		return -errno;
	}

	if (zsock_connect(sock, res->ai_addr, res->ai_addrlen) < 0) {
		LOG_ERR("connect() failed: %d", -errno);
		zsock_close(sock);
		zsock_freeaddrinfo(res);
		return -errno;
	}

	const char *headers[] = {
		"Content-Type: application/json\r\n",
		NULL,
	};
	struct http_request req = {
		.method = method,
		.url = path,
		.host = SP_FMD_HOST,
		.protocol = "HTTP/1.1",
		.header_fields = headers,
		.response = http_response_cb,
		.recv_buf = http_recv_buf,
		.recv_buf_len = sizeof(http_recv_buf),
	};
	if (json) {
		req.payload = json;
		req.payload_len = strlen(json);
	}

	int ret = http_client_req(sock, &req, 15 * MSEC_PER_SEC, NULL);
	zsock_close(sock);
	zsock_freeaddrinfo(res);

	if (ret < 0) {
		LOG_ERR("http_client_req(%s) failed: %d", path, ret);
		return ret;
	}
	return http_status;
}

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
		"\\\"provider\\\":\\\"gps\\\","
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
	LOG_INF("location upload -> HTTP %d", code);
	if (code == 200) {
		uart_send_line(SP_MSG_RCP_STATUS, "REG");
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
	if (code != 200 || http_body_len == 0) {
		return;
	}

	/* Response: {"IDT":..,"Data":"<cmd>","UnixTime":..,"CmdSig":..}.
	 * Minimal extraction of the Data field (no full JSON parse needed). */
	char *p = strstr(http_body, "\"Data\":\"");
	if (!p) {
		return;
	}
	p += strlen("\"Data\":\"");
	char *end = strchr(p, '"');
	if (!end || end == p) {
		return;   /* empty Data => no pending command */
	}
	*end = '\0';
	LOG_INF("server command: %s", p);
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
	LOG_INF("SecurePouch RCP starting (uid=%s)", SP_DEVICE_UID);

	if (!device_is_ready(mcu_uart)) {
		LOG_ERR("MCU-link UART not ready");
		return -1;
	}
	uart_irq_callback_user_data_set(mcu_uart, uart_isr, NULL);
	uart_irq_rx_enable(mcu_uart);
	uart_send_line(SP_MSG_RCP_STATUS, "BOOT");

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
		(void)date_time_update_async(NULL);
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
			(void)date_time_update_async(NULL);
		}

		bool forced = atomic_set(&upload_request, 0) == 1;
		if (forced || now - last_upload >= UPLOAD_PERIOD_MS) {
			/* GNSS works without LTE; attempt a fix regardless. The upload
			 * itself no-ops without registration / a token. */
			uart_send_line(SP_MSG_RCP_STATUS, "GNSS_SEARCH");
			request_location();           /* async; handler fills last_fix */
			k_sleep(K_SECONDS(8));         /* give the fix a chance to land */
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
