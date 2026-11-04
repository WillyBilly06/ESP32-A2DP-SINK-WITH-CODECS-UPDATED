# Codec-only component; its consumer must separately provide dbt_codec.c and
# the independently authored native AVDTP types. No protocol wrapper is linked.
if(CMAKE_BUILD_EARLY_EXPANSION)
    idf_component_register()
    return()
endif()
set(DBT_APTX_SDK_DIR "${IDF_PATH}/components/bt/host/bluedroid/external/libfreeaptx/main")
if(NOT EXISTS "${DBT_APTX_SDK_DIR}/freeaptx.c")
    message(FATAL_ERROR "The existing IDF freeaptx codec source was not found")
endif()
idf_component_register(SRCS "dbt_aptx.c" "dbt_aptx_backend.c"
    INCLUDE_DIRS "." "../common" "../../include"
    PRIV_INCLUDE_DIRS "${DBT_APTX_SDK_DIR}")
