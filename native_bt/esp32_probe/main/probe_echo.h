#ifndef PROBE_ECHO_H
#define PROBE_ECHO_H
#include "dbt_gap.h"
#include "probe_ingress.h"
dbt_result_t probe_echo_init(dbt_runtime_t *runtime, dbt_gap_t *gap);
void probe_echo_event(void *context, dbt_channel_event_t event, const dbt_channel_t *channel,
                      dbt_channel_reason_t reason, uint16_t result);
void probe_echo_data(void *context, const dbt_channel_t *channel, const uint8_t *data,
                     uint16_t size);
void probe_echo_poll(void);
uint32_t probe_echo_wait_ms(void);
void probe_echo_transmit(const uint8_t *data, uint16_t size);
void probe_echo_receive_packet(const dbt_hci_packet_t *packet,const probe_ingress_sample_t *sample);
#endif
