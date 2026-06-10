/*
 * nRF9151 firmware — LTE-M/GNSS + FMD Server HTTP client
 *
 * Planned responsibilities:
 *  - Acquire GNSS fix via nRF9151 integrated modem
 *  - POST location to FMD server: POST /api/v1/locations/{uid}
 *  - Poll FMD server for commands: GET /api/v1/commands/{uid}
 *  - Relay lock/unlock/arm/disarm/alarm commands to nRF52840 over UART
 *
 * Build system: NCS (Zephyr). See prj.conf for Kconfig options.
 */
#include <zephyr/kernel.h>

int main(void)
{
    return 0;
}
