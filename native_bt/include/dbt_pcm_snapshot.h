#ifndef DBT_PCM_SNAPSHOT_H
#define DBT_PCM_SNAPSHOT_H
/* Opt-in, one-shot diagnostic. No storage or runtime hooks in ordinary builds.
 * Codec, copied-input producer and renderer each publish immutable blobs once.
 * No interrupt hooks, allocation, waits, or UART calls on audio paths. */
#ifdef DBT_APTX_SNAPSHOT
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fixed_dsp.h"
void dbt_pcm_snapshot_arm(void);
void dbt_pcm_snapshot_retire(void);
bool dbt_pcm_snapshot_wanted(bool hd,uint32_t now_ms);
bool dbt_pcm_snapshot_decode_begin(const void *context,size_t bytes,uint32_t schema,
    const uint8_t *input,size_t input_bytes,const void *output,uint32_t rate,uint32_t now_ms);
void dbt_pcm_snapshot_decode_end(size_t processed,size_t written);
void dbt_pcm_snapshot_handoff(const void *origin,uint32_t frames,uint32_t stride,
    const void *first,size_t first_bytes,const void *second,size_t second_bytes);
bool dbt_pcm_snapshot_dsp_begin(const fixed_dsp_t *,int32_t *,unsigned frames,bool mix);
void dbt_pcm_snapshot_dsp_end(const int32_t *);
void dbt_pcm_snapshot_final(const fixed_dsp_t *);
/* Logger owner only, at most two 64-byte chunks/call, only after output stops. */
void dbt_pcm_snapshot_log(bool paused);
uint32_t dbt_pcm_snapshot_dsp_schema(void);
#endif
#endif
