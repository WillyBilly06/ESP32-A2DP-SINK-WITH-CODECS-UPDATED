# The one LHDC component uses the native codec-worker adapter. The older raw
# session adapter has separate global cache ownership and is not linked here.
if(CMAKE_BUILD_EARLY_EXPANSION)
    idf_component_register(REQUIRES native_bt codec_core PRIV_REQUIRES log esp_common esp_timer esp_hw_support heap freertos)
    return()
endif()
if(NOT DBT_PROBE_LHDC_BUILD)
    idf_component_register()
    return()
endif()
get_filename_component(workspace "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(owned "${workspace}/native_bt/codecs/lhdcv5")
set(default_source "${owned}/source_snapshot")
set(LHDCV5_SOURCE_DIR "${default_source}" CACHE PATH "Pinned portable LHDC V5 snapshot or audited source override")
set(decoder "${LHDCV5_SOURCE_DIR}/decoder")
set(port "${CMAKE_CURRENT_BINARY_DIR}/port")
if(NOT EXISTS "${decoder}/lhdc_dec.c")
    message(FATAL_ERROR "Set LHDCV5_SOURCE_DIR to a source snapshot containing decoder/lhdc_dec.c")
endif()
# Regenerate the source-pinned port whenever its transformer or audited
# inputs change; incremental builds must not silently link an older port.
set(decoder_headers
    "${decoder}/lhdc_bit_reader.h"
    "${decoder}/lhdc_dec.h"
    "${decoder}/lhdc_dec_internal.h"
    "${decoder}/lhdc_dec_slot.h"
    "${decoder}/lhdc_diag_config.h"
    "${decoder}/lhdc_entropy_dec.h"
    "${decoder}/lhdc_imdct.h"
    "${decoder}/lhdc_sns_synth.h"
    "${decoder}/lhdc_tables.h"
)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${owned}/prepare_port.py" "${decoder}/lhdc_dec.c" "${decoder}/lhdc_imdct.c"
    "${decoder}/lhdc_entropy_dec.c" "${decoder}/lhdc_sns_synth.c" ${decoder_headers})
idf_build_get_property(python PYTHON)
execute_process(COMMAND "${python}" "${owned}/prepare_port.py" --source "${decoder}" --output "${port}"
    RESULT_VARIABLE port_result OUTPUT_QUIET)
if(NOT port_result EQUAL 0)
    message(FATAL_ERROR "The source-pinned LHDC V5 port could not be generated")
endif()
idf_component_register(SRCS "${owned}/dbt_lhdcv5_codec.c"
    "${port}/lhdc_dec.c" "${port}/lhdc_imdct.c" "${port}/lhdc_entropy_dec.c" "${port}/lhdc_sns_synth.c"
    "${decoder}/lhdc_bit_reader.c" "${decoder}/lhdc_diag_config.c" "${decoder}/lhdc_tables.c"
    INCLUDE_DIRS "${owned}" PRIV_INCLUDE_DIRS "${port}" "${decoder}"
    REQUIRES native_bt codec_core PRIV_REQUIRES log esp_common esp_timer esp_hw_support heap freertos)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu11 -fwrapv -O3)
target_compile_definitions(${COMPONENT_LIB} PRIVATE DBT_LHDC_QUALITY=1)
if(DBT_PROBE_LHDC_STAGE_PROFILE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DBT_LHDC_STAGE_PROFILE=1 DBT_LHDC_STAGE_CLOCK_CYCLES=1)
endif()

if(DBT_PROBE_LHDC_FAST_MANTISSA)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DBT_LHDC_FAST_MANTISSA=1)
endif()
