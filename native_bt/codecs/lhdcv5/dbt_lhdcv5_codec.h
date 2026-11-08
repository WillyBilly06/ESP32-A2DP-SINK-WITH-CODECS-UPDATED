#ifndef DBT_LHDCV5_CODEC_H
#define DBT_LHDCV5_CODEC_H
#include "dbt_codec.h"
dbt_result_t dbt_lhdcv5_configure(const uint8_t *cie,uint16_t size,dbt_codec_config_t *config);
uint8_t dbt_a2dp_lhdcv5_validate(const dbt_avdtp_endpoint_t *endpoint,const dbt_avdtp_cap_t *codec);
extern const dbt_codec_ops_t dbt_lhdcv5_codec;
/* Pure bounded framing validation; returns PCM samples per channel.
 * Zero means unknown/invalid, not an empty accepted packet. */
uint32_t dbt_lhdcv5_packet_frames(const dbt_codec_config_t *,const dbt_avdtp_media_t *);

enum { DBT_LHDC_CHANNEL0_ERROR=1u, DBT_LHDC_CHANNEL1_SILENCED=2u,
    DBT_LHDC_PEAK_CONCEALED=4u, DBT_LHDC_CLIPPED=8u, DBT_LHDC_ALLOCATION_FAILED=16u,
    DBT_LHDC_FORMAT_MISMATCH=32u, DBT_LHDC_QUALITY_SCANNED=64u, DBT_LHDC_FRAME_REJECTED=128u, DBT_LHDC_NONFINITE=256u };
typedef struct {
    uint32_t revision,attempts,rejected,channel0_errors,channel1_silenced,peak_concealed,nonfinite_frames,clipped_frames,clipped_samples;
    uint32_t event_attempt,generation,frame_index,rtp_timestamp,ssrc,sequence,frame_bytes,flags,peak_f32_bits;
    int32_t result,channel0_result,channel1_result;
} dbt_lhdcv5_quality_t;
/* Product startup only, after cold checks and before decoder admission. Counts
 * then cover decoder-owner attempts for this boot; they wrap as uint32_t.
 * Snapshot uses at most three attempts and leaves output unchanged on false.
 * Counters are cumulative; metadata describes the last rejected/clipped/concealed/channel-error attempt.
 * Peak is raw IEEE754 binary32 output-container amplitude, not a bitrate.
 * No allocation, UART, waiting, or worker-owned pointer access. */
void dbt_lhdcv5_quality_start(void);
bool dbt_lhdcv5_quality_snapshot(dbt_lhdcv5_quality_t *);
#endif
