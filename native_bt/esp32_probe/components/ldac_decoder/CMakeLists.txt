# Experimental standalone LDAC decoder, using only codec-library sources.
# This component does not register or advertise an A2DP endpoint.
if(CMAKE_BUILD_EARLY_EXPANSION)
    idf_component_register(REQUIRES native_bt codec_core)
    return()
endif()
get_filename_component(workspace "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(owned "${workspace}/native_bt/codecs/ldac")
set(ldac "${workspace}/esp-idf/components/bt/host/bluedroid/external/libldac-dec")
if(NOT EXISTS "${ldac}/src/ldaclib.c" OR NOT EXISTS "${ldac}/inc/ldacBT.h")
    message(FATAL_ERROR "The existing SDK LDAC decoder sources were not found")
endif()
idf_build_get_property(python PYTHON)
execute_process(COMMAND "${python}" "${owned}/prepare_port.py" --source "${ldac}/src"
    --output "${CMAKE_CURRENT_BINARY_DIR}/port" RESULT_VARIABLE port_result OUTPUT_QUIET)
if(NOT port_result EQUAL 0)
    message(FATAL_ERROR "The source-pinned LDAC codec port could not be generated")
endif()
idf_component_register(SRCS "${owned}/dbt_ldac.c" "${owned}/dbt_ldac_decoder.c"
    "${owned}/dbt_ldac_bt.c" "${owned}/dbt_ldac_lib.c"
    INCLUDE_DIRS "${owned}"
    PRIV_INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}/port" "${ldac}/inc" "${ldac}/src"
    REQUIRES native_bt codec_core)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c11 -fwrapv)
