# SDK freeaptx is included exactly once by the native backend translation unit.
# It does not bring in Bluedroid or the SDK's A2DP wrapper.
if(CMAKE_BUILD_EARLY_EXPANSION)
    idf_component_register(REQUIRES native_bt codec_core)
    return()
endif()
get_filename_component(workspace "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(owned "${workspace}/native_bt/codecs/aptx")
set(aptx "${workspace}/esp-idf/components/bt/host/bluedroid/external/libfreeaptx/main")
if(NOT EXISTS "${aptx}/freeaptx.c" OR NOT EXISTS "${aptx}/freeaptx.h")
    message(FATAL_ERROR "The existing SDK freeaptx decoder sources were not found")
endif()
idf_component_register(SRCS "${owned}/dbt_aptx.c" "${owned}/dbt_aptx_backend.c"
    INCLUDE_DIRS "${owned}" PRIV_INCLUDE_DIRS "${aptx}" REQUIRES native_bt codec_core)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c11 -fwrapv)
# The existing compact adapter has two GCC indentation warnings; their control
# flow is explicit and host-tested.
set_source_files_properties("${owned}/dbt_aptx.c" PROPERTIES COMPILE_OPTIONS "-Wno-error=misleading-indentation")
