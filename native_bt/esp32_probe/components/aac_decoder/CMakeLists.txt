# Standalone guarded AAC-LC codec only. The Bluetooth controller/host component
# is not a dependency; codec_core supplies the single owned worker framework.
if(CMAKE_BUILD_EARLY_EXPANSION)
    idf_component_register(REQUIRES native_bt codec_core)
    return()
endif()
get_filename_component(workspace "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(owned "${workspace}/native_bt/codecs/aac")
set(helix "${workspace}/esp-idf/components/bt/host/bluedroid/external/libaac-lc/libhelix_aac")
if(NOT EXISTS "${helix}/aacdec.c")
    message(FATAL_ERROR "The existing SDK AAC decoder sources were not found")
endif()
idf_build_get_property(python PYTHON)
execute_process(COMMAND "${python}" "${owned}/prepare_port.py" --source "${helix}/aacdec.c"
    --output "${CMAKE_CURRENT_BINARY_DIR}/port" RESULT_VARIABLE port_result OUTPUT_QUIET)
if(NOT port_result EQUAL 0)
    message(FATAL_ERROR "The source-pinned guarded AAC decoder could not be generated")
endif()
set(aac_sdk_units
    "${CMAKE_CURRENT_BINARY_DIR}/port/aacdec_guarded.c"
    "${helix}/aactabs.c" "${helix}/bitstream.c" "${helix}/buffers.c"
    "${helix}/dct4.c" "${helix}/decelmnt.c" "${helix}/dequant.c"
    "${helix}/fft.c" "${helix}/filefmt.c" "${helix}/huffman.c"
    "${helix}/hufftabs.c" "${helix}/imdct.c" "${helix}/noiseless.c"
    "${helix}/pns.c" "${helix}/stproc.c" "${helix}/tns.c" "${helix}/trigtabs.c")
idf_component_register(SRCS "${owned}/dbt_aac_decoder.c" "${owned}/dbt_aac_transport.c" ${aac_sdk_units}
    INCLUDE_DIRS "${owned}" PRIV_INCLUDE_DIRS "${helix}"
    REQUIRES native_bt codec_core)
target_compile_definitions(${COMPONENT_LIB} PRIVATE ESP32=1 USE_DEFAULT_STDLIB AAC_MAX_NCHANS=2)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c11 -fwrapv)
