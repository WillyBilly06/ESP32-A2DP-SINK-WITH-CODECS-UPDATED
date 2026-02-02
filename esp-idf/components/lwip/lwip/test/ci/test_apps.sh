#!/bin/bash
set -e

export LWIP_DIR=`pwd`
export LWIP_CONTRIB_DIR=`pwd`/contrib

TEST_APPS="socket_no_linger socket_linger socket_linger_reuse tcp_socket_reuse"
STRESS_TEST_APPS="socket_linger"

cd test/apps
# Prepare a failing report in case we get stuck (check in no-fork mode)
python socket_linger_stress_test.py failed > ${LWIP_DIR}/socket_linger_stress_test.xml
for app in $TEST_APPS; do
    cmake -DCI_BUILD=1 -DTEST_APP=${app} -B ${app} -G Ninja .
    cmake --build ${app}/
    timeout 10 ./${app}/lwip_test_apps
    [ -f ${LWIP_DIR}/check2junit.py ] &&
        python ${LWIP_DIR}/check2junit.py lwip_test_apps.xml > ${LWIP_DIR}/${app}.xml
done

# Run the stress test(s) multiple times
for stress_cfg in $STRESS_TEST_APPS; do
  for run in {1..1000}; do ( timeout 10 ./$stress_cfg/lwip_test_apps ) || exit 1 ; done;
done
# All good, regenerate the stress test-report, since the test succeeded
python socket_linger_stress_test.py > ${LWIP_DIR}/socket_linger_stress_test.xml
