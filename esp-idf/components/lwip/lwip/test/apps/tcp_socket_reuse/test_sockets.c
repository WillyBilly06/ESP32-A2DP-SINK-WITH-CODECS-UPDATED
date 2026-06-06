#include <pthread.h>
#include <tcp/tcp_helper.h>

#include "lwip_check.h"
#include "lwip/opt.h"
#include "lwip/sockets.h"
#include "lwip/priv/sockets_priv.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/api.h"

Suite *sockets_suite(void);

static int
test_sockets_get_used_count(void)
{
    int used = 0;
    int i;

    for (i = 0; i < NUM_SOCKETS; i++) {
        struct lwip_sock* s = lwip_socket_dbg_get_socket(i);
        if (s != NULL) {
            if (s->fd_used) {
                used++;
            }
        }
    }
    return used;
}

/* Setups/teardown functions */
static void
sockets_setup(void)
{
    fail_unless(test_sockets_get_used_count() == 0);
}

static void
sockets_teardown(void)
{
    fail_unless(test_sockets_get_used_count() == 0);
    /* poll until all memory is released... */
    while (tcp_tw_pcbs) {
        tcp_abort(tcp_tw_pcbs);
    }
}

START_TEST(test_tcp_active_socket_limit)
{
    int i;
    int sock;
    LWIP_UNUSED_ARG(_i);
    for (i = 0; i < MEMP_NUM_TCP_PCB; i++) {
        sock = lwip_socket(AF_INET, SOCK_STREAM, 0);
        fail_unless(sock != -1);
    }
    sock = lwip_socket(AF_INET, SOCK_STREAM, 0);
    fail_unless(sock == -1);
    for (i = 0; i < MEMP_NUM_TCP_PCB; i++) {
        lwip_close(i + LWIP_SOCKET_OFFSET);
    }
}
END_TEST

static void create_tcp_pcbs_and_kill_state(u8_t recycle_test, enum tcp_state state)
{
    struct tcp_pcb* pcb[MEMP_NUM_TCP_PCB + 1];
    int i;
    err_t err;
    mem_size_t prev_used, one_pcb_size;

    prev_used = lwip_stats.mem.used;
    pcb[0] = tcp_new();
    fail_unless(pcb[0] != NULL);
    one_pcb_size = lwip_stats.mem.used - prev_used;

    for(i = 1; i < MEMP_NUM_TCP_PCB; i++) {
        prev_used = lwip_stats.mem.used;
        pcb[i] = tcp_new();
        fail_unless(pcb[i] != NULL);
        fail_unless(one_pcb_size == lwip_stats.mem.used - prev_used);
        /* bind the pcb, so lwip expects they're active */
        err = tcp_bind(pcb[i], &test_local_ip, 3333+i);
        fail_unless(err == ERR_OK);
    }
    pcb[MEMP_NUM_TCP_PCB] = tcp_new();
    fail_unless(pcb[MEMP_NUM_TCP_PCB] == NULL);

    if (recycle_test) {
        /* Trying to remove the oldest pcb in TIME_WAIT,LAST_ACK,CLOSING state when pcb full */
        if (state == TIME_WAIT) {
            tcp_set_state(pcb[0], TIME_WAIT, &test_local_ip, &test_remote_ip, TEST_LOCAL_PORT, TEST_REMOTE_PORT);
        } else {
            tcp_set_state(pcb[0], ESTABLISHED, &test_local_ip, &test_remote_ip, TEST_LOCAL_PORT, TEST_REMOTE_PORT);
            pcb[0]->state = state;
        }
        fail_unless(pcb[0]->state == state);

        pcb[MEMP_NUM_TCP_PCB] = tcp_new();
        fail_unless(pcb[MEMP_NUM_TCP_PCB] != NULL);
        pcb[0] = tcp_new();
        fail_unless(pcb[0] == NULL);
    }

    for (i = 0; i <= MEMP_NUM_TCP_PCB; i++)
    {
        if (pcb[i]) {
            tcp_abort(pcb[i]);
        }
    }
}

START_TEST(test_tcp_new_max_num)
{
    LWIP_UNUSED_ARG(_i);
    create_tcp_pcbs_and_kill_state(0, /* N/A */ CLOSED);
}
END_TEST

START_TEST(test_tcp_pcb_recycle_TIME_WAIT)
{
    LWIP_UNUSED_ARG(_i);
    create_tcp_pcbs_and_kill_state(1, TIME_WAIT);
}
END_TEST

START_TEST(test_tcp_pcb_recycle_LAST_ACK)
{
    LWIP_UNUSED_ARG(_i);
    create_tcp_pcbs_and_kill_state(1, LAST_ACK);
}
END_TEST

START_TEST(test_tcp_pcb_recycle_CLOSING)
{
    LWIP_UNUSED_ARG(_i);
    create_tcp_pcbs_and_kill_state(1, CLOSING);
}
END_TEST

START_TEST(test_tcp_pcb_recycle_FIN_WAIT_1)
{
    LWIP_UNUSED_ARG(_i);
    create_tcp_pcbs_and_kill_state(1, FIN_WAIT_1);
}
END_TEST

START_TEST(test_tcp_pcb_recycle_FIN_WAIT_2)
{
    LWIP_UNUSED_ARG(_i);
    create_tcp_pcbs_and_kill_state(1, FIN_WAIT_2);
}
END_TEST

/** Create the suite including all tests for this module */
Suite *
sockets_suite(void)
{
    testfunc tests[] = {
            TESTFUNC(test_tcp_active_socket_limit),
            TESTFUNC(test_tcp_new_max_num),
            TESTFUNC(test_tcp_pcb_recycle_TIME_WAIT),
            TESTFUNC(test_tcp_pcb_recycle_LAST_ACK),
            TESTFUNC(test_tcp_pcb_recycle_CLOSING),
            TESTFUNC(test_tcp_pcb_recycle_FIN_WAIT_1),
            TESTFUNC(test_tcp_pcb_recycle_FIN_WAIT_2),
    };
    return create_suite("TCP_ACTIVE_SOCKETS", tests, sizeof(tests)/sizeof(testfunc), sockets_setup, sockets_teardown);
}
