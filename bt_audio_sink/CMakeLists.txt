cmake_minimum_required(VERSION 3.16)

# Set IDF path to our local copy
set(IDF_PATH "$ENV{IDF_PATH}" CACHE PATH "Path to ESP-IDF")
if(NOT IDF_PATH)
    set(IDF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../esp-idf")
endif()

# Include IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(bt_audio_sink)
