if(CMAKE_BUILD_EARLY_EXPANSION)
    idf_component_register(REQUIRES native_bt)
    return()
endif()
get_filename_component(native_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
idf_component_register(SRCS "${native_root}/codecs/common/dbt_codec.c"
    INCLUDE_DIRS "${native_root}/codecs/common" REQUIRES native_bt)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c11 -Wall -Wextra -Werror)
