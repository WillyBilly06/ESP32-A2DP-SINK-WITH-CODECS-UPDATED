cmake_minimum_required(VERSION 3.22)

# Set IDF path to our local copy
set(IDF_PATH "$ENV{IDF_PATH}" CACHE PATH "Path to ESP-IDF")
if(NOT IDF_PATH)
    set(IDF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../esp-idf")
endif()

# Include IDF build system
include("${IDF_PATH}/tools/cmake/project.cmake")

if(NOT IDF_VERSION_MAJOR EQUAL 6 OR NOT IDF_VERSION_MINOR EQUAL 1)
    message(FATAL_ERROR "Use the bundled patched ESP-IDF v6.1 SDK; activate esp-idf/export first")
endif()

project(bt_audio_sink)
if(NOT CONFIG_APP_PROJECT_VER_FROM_CONFIG)
    message(FATAL_ERROR "Enable APP_PROJECT_VER_FROM_CONFIG so the image and BLE report the same firmware version")
endif()
