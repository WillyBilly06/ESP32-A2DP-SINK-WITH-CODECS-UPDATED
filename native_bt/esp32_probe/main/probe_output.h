#ifndef PROBE_OUTPUT_H
#define PROBE_OUTPUT_H
#include "dbt_audio.h"
typedef struct {
    uint32_t running,failed,rate,requested_bytes,accepted_frames,rendered_frames,ready_frames,input_ready_frames;
    uint32_t dma_eofs,dma_source_frames,underrun_frames,descriptor_errors,max_eof_gap_us;
    uint32_t maximum_render_us,stop_fill_frames;
    uint32_t first_underrun_accepted_frames,first_underrun_dma_frames;
} probe_output_stats_t;
dbt_result_t probe_output_init(void (*wake_decoder)(void *),void *context);
/* Decoder worker only; bounded synchronous handshake with the render task. */
dbt_result_t probe_output_stream(bool start,uint32_t rate,uint8_t channels);
dbt_result_t probe_output_pcm(const int16_t *samples,uint16_t frames,uint8_t channels,uint32_t rate);
void probe_output_stats(probe_output_stats_t *stats);
uint16_t probe_output_delay_100us(uint32_t rate);
dbt_result_t probe_output_controls(int8_t bass,int8_t mid,int8_t treble,uint8_t volume,uint8_t flags);
#endif
