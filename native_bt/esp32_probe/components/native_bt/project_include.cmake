# The image is built for the admitted ota_1 slot. The smaller recovery app is
# preserved; its size is not the capacity of this candidate's destination.
function(partition_table_add_check_size_target target_name)
    cmake_parse_arguments(CHECK "" "BINARY_PATH;PARTITION_TYPE;PARTITION_SUBTYPE" "DEPENDS" ${ARGN})
    set(selected_subtype ${CHECK_PARTITION_SUBTYPE})
    if(target_name STREQUAL "app_check_size")
        set(selected_subtype ota_1)
    endif()
    set(subtype_args)
    if(selected_subtype)
        set(subtype_args --subtype ${selected_subtype})
    endif()
    idf_build_get_property(check_python PYTHON)
    idf_build_get_property(check_table PARTITION_TABLE_BIN_PATH)
    add_custom_target(${target_name}
        COMMAND ${check_python} ${PARTITION_TABLE_CHECK_SIZES_TOOL_PATH}
            --offset ${PARTITION_TABLE_OFFSET} partition --type ${CHECK_PARTITION_TYPE}
            ${subtype_args} ${check_table} ${CHECK_BINARY_PATH}
        DEPENDS ${CHECK_DEPENDS} partition_table_bin VERBATIM)
endfunction()
