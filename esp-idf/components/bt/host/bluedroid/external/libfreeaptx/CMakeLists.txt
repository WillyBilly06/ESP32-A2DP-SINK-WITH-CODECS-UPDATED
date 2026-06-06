# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

if (DEFINED ESP_PLATFORM)

    idf_build_get_property(target IDF_TARGET)

    set(srcs "main/freeaptx.c")
    idf_component_register(SRCS "${srcs}"
                           INCLUDE_DIRS main)

else()

    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(freeaptx)

endif()
