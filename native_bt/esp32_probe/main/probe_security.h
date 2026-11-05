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
/* Optional metadata only; never changes the failed procedure's policy. */
void probe_security_le_failure(uint16_t handle,uint32_t generation);
void probe_security_le_state(uint16_t handle,uint32_t generation,uint8_t flags,uint8_t key_size);
void probe_security_oversized(const dbt_channel_oversize_t *);
void probe_security_smp_trace(uint16_t,uint32_t,uint8_t stage,int result,uint8_t code,uint16_t size,uint32_t window);
void probe_security_pairing_features(uint16_t,uint32_t,const uint8_t fields[7]);
void probe_security_channel(dbt_channel_event_t,const dbt_channel_t *,dbt_channel_reason_t,uint16_t result);
void probe_security_sdp_error(const dbt_channel_t *,uint16_t bytes,uint8_t pdu,dbt_result_t,bool client);
dbt_result_t probe_security_receive(const dbt_hci_packet_t *packet, uint32_t now_ms);
void probe_security_poll(uint32_t now_ms);
uint32_t probe_security_wait_ms(uint32_t now_ms);
unsigned probe_security_bytes(void);
unsigned probe_security_loaded_bonds(void);
/* Owner-only; accepted NVS jobs remain non-quiescent until their completion
 * has been consumed, including a worker blocked publishing its receipt. */
bool probe_security_quiescent(void);
dbt_result_t probe_security_request(void *context, const dbt_link_t *link, uint16_t psm,
                                    dbt_security_t level, bool start);
dbt_result_t probe_security_forget_peer(const dbt_link_t *link);
/* Bounded deterministic echo diagnostics, formatted only by the logger task. */
void probe_security_trace(uint8_t stage, uint16_t size, uint16_t offset,
                          uint8_t expected, uint8_t actual);
void probe_security_trace_hash(uint32_t before,uint32_t after,uint32_t owner,uint32_t ordinal,
                               uint16_t size,uint16_t offset,bool after_ready);
#endif
