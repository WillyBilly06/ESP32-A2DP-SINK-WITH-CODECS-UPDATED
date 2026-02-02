cmake_minimum_required(VERSION 3.16)

if (DEFINED ESP_PLATFORM)
  # idf component
  idf_component_register(
    SRC_DIRS src src/libAACdec src/libFDK src/libSYS src/libArithCoding src/libMpegTPDec src/libSACdec src/libDRCdec src/libSBRdec src/libPCMutils
    INCLUDE_DIRS src src/libAACdec src/libFDK src/libSYS src/libArithCoding src/libMpegTPDec src/libSACdec src/libDRCdec src/libSBRdec src/libPCMutils
    REQUIRES arduino-esp32
  )

  target_compile_options(${COMPONENT_LIB} INTERFACE -Wno-error -Wno-format -Wno-format-security -Wformat=0)
  target_compile_options(${COMPONENT_LIB} PRIVATE -DUSE_DEFAULT_STDLIB)
  add_compile_definitions(ESP32)
else()
    # set the project name
    project(fdk_aac)

    # lots of warnings and all warnings as errors
    ## add_compile_options(-Wall -Wextra )
    set(CMAKE_CXX_STANDARD 17)

    file(GLOB_RECURSE SRC_LIST_CPP CONFIGURE_DEPENDS  "${PROJECT_SOURCE_DIR}/src/*.cpp" )

    # define libraries
    add_library (fdk_aac ${SRC_LIST_CPP})

    # prevent compile errors
    #target_compile_options(fdk_aac PRIVATE -DUSE_DEFAULT_STDLIB)

    # define location for header files
    target_include_directories(fdk_aac PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src )

    # build examples
    add_subdirectory( "${CMAKE_CURRENT_SOURCE_DIR}/examples/decode")
    add_subdirectory( "${CMAKE_CURRENT_SOURCE_DIR}/examples/encode")
endif()
