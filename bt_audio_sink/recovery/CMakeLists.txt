# Recovery partition - minimal OTA-only firmware
# This builds a standalone recovery binary that only supports BLE OTA
# 
# To build:
#   cd recovery
#   idf.py build
# 
# The binary will be at: recovery/build/recovery.bin
# Flash it to factory partition at 0x10000

cmake_minimum_required(VERSION 3.5)

# Use parent's partition table
set(EXTRA_COMPONENT_DIRS "../main/config")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(recovery)
