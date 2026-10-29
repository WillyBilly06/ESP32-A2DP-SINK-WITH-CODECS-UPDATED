if(CMAKE_BUILD_EARLY_EXPANSION)
    idf_component_register(REQUIRES native_bt)
    return()
endif()
get_filename_component(workspace "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(codec "${workspace}/esp-idf/components/bt/host/bluedroid/external/sbc/decoder")
set(owned "${workspace}/native_bt/codecs/sbc")
idf_build_get_property(python PYTHON)
execute_process(COMMAND "${python}" "${owned}/prepare_port.py" --source "${codec}/srce"
                --output "${CMAKE_CURRENT_BINARY_DIR}/port" RESULT_VARIABLE port_result OUTPUT_QUIET)
if(NOT port_result EQUAL 0)
    message(FATAL_ERROR "Standalone SBC arithmetic port requires source review")
endif()
idf_component_register(
    SRCS "${owned}/dbt_sbc_decoder.c" "${owned}/dbt_sbc_worker.c"
         "${codec}/srce/alloc.c" "${codec}/srce/bitalloc-sbc.c" "${codec}/srce/bitalloc.c"
         "${codec}/srce/bitstream-decode.c" "${codec}/srce/decoder-oina.c" "${codec}/srce/decoder-private.c"
         "${codec}/srce/decoder-sbc.c" "${codec}/srce/dequant.c" "${codec}/srce/framing-sbc.c"
         "${codec}/srce/framing.c" "${codec}/srce/oi_codec_version.c" "${CMAKE_CURRENT_BINARY_DIR}/port/synthesis-8-generated.c"
         "${CMAKE_CURRENT_BINARY_DIR}/port/synthesis-dct8.c" "${CMAKE_CURRENT_BINARY_DIR}/port/synthesis-sbc.c"
    INCLUDE_DIRS "${owned}"
    PRIV_INCLUDE_DIRS "${owned}/port" "${codec}/include"
    REQUIRES native_bt
)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c11 -fwrapv)
