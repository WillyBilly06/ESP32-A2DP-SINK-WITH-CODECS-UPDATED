cmake_minimum_required(VERSION 3.15)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror=implicit-function-declaration")


option(Only_compile_decoder "Only compile decoder" ON)
option(Used_CSI_DSP "Used CSI DSP" OFF)
option(Used_double "Used double" OFF)

include_directories(
    ./inc
)
set(LDAC_SRC_DIC ./src)
add_library(
    ldac_dec_lib STATIC
    ${LDAC_SRC_DIC}/ldacBT.c
    ${LDAC_SRC_DIC}/ldaclib.c
)

add_library(
    ldacBT_dec SHARED
    ${LDAC_SRC_DIC}/ldacBT.c
    ${LDAC_SRC_DIC}/ldaclib.c
)

target_link_libraries(ldacBT_dec m)

if(Used_CSI_DSP)
add_definitions(-DCONFIG_USED_CSI_DSP)
endif()

if(Used_double)
add_definitions(-DCONFIG_SCALAR_F64)
endif()

if(NOT Only_compile_decoder)
set(lib_enc_dic ../encode/libldac)#from aosp


set(libldac_enc ${lib_enc_dic})
include_directories(
    ${libldac_enc}/inc
)

set(enc_dic ${libldac_enc}/src)

add_library(
    ldac_enc_lib STATIC
    ${enc_dic}/ldacBT.c
    ${enc_dic}/ldaclib.c
)

target_link_libraries(ldac_enc_lib m)

add_executable(ldac_encoder ldac_enc_test.c)
add_executable(ldac_decoder ldac_dec_test.c)
add_executable(auto_test auto_test.c)


target_link_libraries(ldac_encoder ldac_enc_lib)
target_link_libraries(ldac_decoder ldac_dec_lib m)
endif()
