#ifndef PROBE_AUDIO_H
#define PROBE_AUDIO_H
#include "dbt_a2dp_acceptor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/* Host-owner control/data hooks; decoder and PCM work run on the separate
 * generic codec worker. Optional Opus, AAC and aptX endpoints are present only
 * when their corresponding DBT_PROBE_*_BUILD components are enabled. */
dbt_result_t probe_audio_init(dbt_runtime_t *runtime,TaskHandle_t owner);
bool probe_audio_accept(void *context,const dbt_link_t *link,uint16_t psm);
void probe_audio_event(dbt_channel_event_t event,const dbt_channel_t *channel);
void probe_audio_receive(const dbt_channel_t *channel,const uint8_t *data,uint16_t size);
void probe_audio_poll(uint32_t now_ms);
uint32_t probe_audio_wait_ms(uint32_t now_ms);
#endif
