#ifndef PROBE_SECURITY_H
#define PROBE_SECURITY_H
#include "dbt_classic_security.h"
#include "dbt_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

dbt_result_t probe_security_init(dbt_runtime_t *runtime, dbt_gap_t *gap, TaskHandle_t owner);
bool probe_security_accept(void *context, const uint8_t address[6], const uint8_t device_class[3]);
bool probe_security_authorize(void *context, const dbt_link_t *link, uint16_t psm,
                              dbt_security_t level);
void probe_security_link(const dbt_link_change_t *change);
dbt_result_t probe_security_receive(const dbt_hci_packet_t *packet, uint32_t now_ms);
void probe_security_poll(uint32_t now_ms);
uint32_t probe_security_wait_ms(uint32_t now_ms);
unsigned probe_security_bytes(void);
unsigned probe_security_loaded_bonds(void);
dbt_result_t probe_security_request(void *context, const dbt_link_t *link, uint16_t psm,
                                    dbt_security_t level, bool start);
dbt_result_t probe_security_forget_peer(const dbt_link_t *link);
/* Bounded deterministic echo diagnostics, formatted only by the logger task. */
void probe_security_trace(uint8_t stage, uint16_t size, uint16_t offset,
                          uint8_t expected, uint8_t actual);
void probe_security_trace_hash(uint32_t before,uint32_t after,uint32_t owner,uint32_t ordinal,
                               uint16_t size,uint16_t offset,bool after_ready);
#endif
