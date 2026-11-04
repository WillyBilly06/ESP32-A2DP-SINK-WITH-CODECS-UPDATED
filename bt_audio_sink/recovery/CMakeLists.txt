# Recovery partition - minimal OTA-only firmware
# This builds standalone WiFi recovery firmware with encrypted OTA support.
# 
# To build:
#   cd recovery
#   idf.py build
# 
# The binary will be at: recovery/build/recovery.bin
# Flash it explicitly to the recovery (ota_2) partition at 0x10000.

cmake_minimum_required(VERSION 3.22)

# Use parent's partition table
set(EXTRA_COMPONENT_DIRS "../main/config")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
if(NOT IDF_VERSION_MAJOR EQUAL 6 OR NOT IDF_VERSION_MINOR EQUAL 1)
    message(FATAL_ERROR "Use the bundled patched ESP-IDF v6.1 SDK; activate esp-idf/export first")
endif()
project(recovery)
