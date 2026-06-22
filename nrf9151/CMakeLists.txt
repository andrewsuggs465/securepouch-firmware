cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(securepouch_nrf9151 VERSION 0.1.0)

target_sources(app PRIVATE src/main.c)
# Shared inter-MCU protocol header (firmware/shared/sp_uart_protocol.h)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../shared)
