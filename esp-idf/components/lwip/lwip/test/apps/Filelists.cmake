# This file is indented to be included in end-user CMakeLists.txt
# include(/path/to/Filelists.cmake)
# It assumes the variable LWIP_DIR is defined pointing to the
# root path of lwIP sources.
#
# This file is NOT designed (on purpose) to be used as cmake
# subdir via add_subdirectory()
# The intention is to provide greater flexibility to users to
# create their own targets using the *_SRCS variables.

set(LWIP_TESTDIR ${LWIP_DIR}/test/apps)
set(LWIP_TESTFILES
	${LWIP_TESTDIR}/test_apps.c
	${LWIP_CONTRIB_DIR}/ports/unix/port/sys_arch.c
)

if("${TEST_APP}" MATCHES "^socket_")
	list(APPEND LWIP_TESTFILES ${LWIP_TESTDIR}/test_sockets.c)
elseif("${TEST_APP}" STREQUAL "tcp_socket_reuse")
	list(APPEND LWIP_TESTFILES ${LWIP_TESTDIR}/tcp_socket_reuse/test_sockets.c)
	list(APPEND LWIP_TESTFILES ${LWIP_DIR}/test/unit/tcp/tcp_helper.c)
endif()
