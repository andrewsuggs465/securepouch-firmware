cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(securepouch_nrf9151 VERSION 0.1.0)

target_sources(app PRIVATE src/main.c)
# Shared inter-MCU protocol header (firmware/shared/sp_uart_protocol.h)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../shared)

# SecurePouch build-time provisioning. Override on the west command line, e.g.
#   west build ... -- -DSP_APN=melita -DSP_FMD_TOKEN=abc123
# These are forwarded to the compiler as quoted string macros so main.c picks
# them up via its #ifndef defaults.
set(SP_APN "" CACHE STRING "APN for the IoT SIM — empty uses network default; override with -DSP_APN=onomondo etc.")
set(SP_FMD_TOKEN "" CACHE STRING "FMD server access token (IDT)")
set(SP_DEVICE_UID "securepouch-001" CACHE STRING "FMD device UID / username")
set(SP_FMD_HOST "lintu-server.myvnc.com" CACHE STRING "FMD server host")
set(SP_FIRMWARE_ROLE "nRF9151-DK standalone LTE/GNSS" CACHE STRING "Human-readable firmware role")

if(DEFINED ENV{SP_FMD_TOKEN} AND NOT "$ENV{SP_FMD_TOKEN}" STREQUAL "")
    set(SP_FMD_TOKEN "$ENV{SP_FMD_TOKEN}")
endif()

target_compile_definitions(app PRIVATE
    SP_APN="${SP_APN}"
    SP_FMD_TOKEN="${SP_FMD_TOKEN}"
    SP_DEVICE_UID="${SP_DEVICE_UID}"
    SP_FMD_HOST="${SP_FMD_HOST}"
    SP_FIRMWARE_ROLE="${SP_FIRMWARE_ROLE}"
)
