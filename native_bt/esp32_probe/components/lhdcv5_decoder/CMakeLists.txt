if(CMAKE_BUILD_EARLY_EXPANSION)
    idf_component_register(PRIV_REQUIRES log esp_common esp_timer heap freertos)
    return()
endif()
if(NOT DBT_PROBE_LHDC_BUILD)
    idf_component_register()
    return()
endif()
get_filename_component(default_lhdc_source "${CMAKE_CURRENT_LIST_DIR}/../../../../../LHDC-V5-Decoder-main" ABSOLUTE)
set(LHDCV5_SOURCE_DIR "${default_lhdc_source}" CACHE PATH "User-owned portable LHDC V5 decoder")
if(NOT EXISTS "${LHDCV5_SOURCE_DIR}/decoder/lhdc_dec.c")
    message(FATAL_ERROR "Set LHDCV5_SOURCE_DIR to the user's LHDC-V5-Decoder-main checkout")
endif()
idf_component_register(
    SRCS
        "${LHDCV5_SOURCE_DIR}/decoder/lhdc_bit_reader.c"
        "${LHDCV5_SOURCE_DIR}/decoder/lhdc_dec.c"
        "${LHDCV5_SOURCE_DIR}/decoder/lhdc_diag_config.c"
        "${LHDCV5_SOURCE_DIR}/decoder/lhdc_entropy_dec.c"
        "${LHDCV5_SOURCE_DIR}/decoder/lhdc_imdct.c"
        "${LHDCV5_SOURCE_DIR}/decoder/lhdc_sns_synth.c"
        "${LHDCV5_SOURCE_DIR}/decoder/lhdc_tables.c"
        "../../../codecs/lhdcv5/dbt_lhdcv5.c"
    INCLUDE_DIRS "${LHDCV5_SOURCE_DIR}/decoder" "../../../codecs/lhdcv5" "../../../include"
    PRIV_REQUIRES log esp_common esp_timer heap freertos
)
target_compile_options(${COMPONENT_LIB} PRIVATE -O3)
