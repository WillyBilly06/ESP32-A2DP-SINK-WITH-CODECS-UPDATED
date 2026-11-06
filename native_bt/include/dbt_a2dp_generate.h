#ifndef DBT_A2DP_GENERATE_H
#define DBT_A2DP_GENERATE_H
#include "dbt_a2dp_select.h"

typedef enum {DBT_A2DP_FAMILY_SBC=1,DBT_A2DP_FAMILY_AAC,DBT_A2DP_FAMILY_APTX,
    DBT_A2DP_FAMILY_APTX_HD,DBT_A2DP_FAMILY_APTX_LL,DBT_A2DP_FAMILY_LDAC,
    DBT_A2DP_FAMILY_ANDROID_OPUS,DBT_A2DP_FAMILY_OPUS05,DBT_A2DP_FAMILY_LHDCV5} dbt_a2dp_family_t;
typedef enum {DBT_A2DP_AAC_VBR_AUTO,DBT_A2DP_AAC_CBR,DBT_A2DP_AAC_VBR_REQUIRED} dbt_a2dp_aac_vbr_t;
typedef struct {
    dbt_a2dp_family_t family; /* Independent of Android enums and SEID numbers. */
    uint32_t rate; /* Exact nonzero Hz, never rounded or reduced. */
    uint8_t channels; /* Exact 1 or2. */
    uint16_t avdtp_version;
    dbt_a2dp_delay_policy_t delay;
    union {
        struct {uint8_t mode,blocks,subbands,allocation,minimum_bitpool,maximum_bitpool;} sbc;
        struct {uint32_t maximum_bitrate_bps;uint8_t object;dbt_a2dp_aac_vbr_t vbr;} aac;
        struct {uint8_t mode;} ldac;
        struct {uint32_t duration_us;} android_opus;
        struct {uint32_t duration_us,location;uint16_t maximum_bitrate_field;} opus05;
        struct {uint16_t minimum_kbps,maximum_kbps;uint8_t bits,features;} lhdc;
    } codec;
} dbt_a2dp_generate_request_t;
typedef enum {DBT_A2DP_GENERATE_NONE,DBT_A2DP_GENERATE_SUCCESS,DBT_A2DP_GENERATE_PEER_NOT_ELIGIBLE,
    DBT_A2DP_GENERATE_PEER_CAPS_INVALID,DBT_A2DP_GENERATE_LOCAL_DESCRIPTOR_INVALID,
    DBT_A2DP_GENERATE_NO_LOCAL_FAMILY,DBT_A2DP_GENERATE_PEER_FAMILY_MISMATCH,
    DBT_A2DP_GENERATE_LAYOUT_UNSUPPORTED,DBT_A2DP_GENERATE_PARAMETERS_UNSUPPORTED,
    DBT_A2DP_GENERATE_VALIDATION_FAILED,DBT_A2DP_GENERATE_NO_SPACE} dbt_a2dp_generate_outcome_t;
typedef struct {
    dbt_a2dp_generate_outcome_t outcome;
    dbt_avdtp_cap_error_t capability_error;
    uint8_t local_seid,codec_size;
    bool match_attempted;
    dbt_a2dp_select_result_t selection;
} dbt_a2dp_generate_result_t;

/* Pure bounded requested-family format generation, then exact matching.
 * Family/rate/channel requests are strict. Zero codec-specific fields choose
 * a documented mutually supported default: SBC joint/stereo/dual,16/12/8/4
 * blocks,8/4 bands,loudness/SNR,max legal pool; AAC MPEG4/2LC and mutual VBR;
 * Opus20/10/5/2.5/40ms as the format permits; LHDC highest common depth and
 * mutual allowed bitrate bounds. Nonzero options are never silently relaxed.
 * SBC mode/allocation and LDAC mode use their CIE bit values. AAC object is
 * 0x40/0x80. Opus05 bitrate is the raw protocol field, not a guessed unit;
 * LHDC bounds are kbps and features may select0x40 if mutually advertised.
 * Encoder quality/actual bitrate and source PCM depth absent from a codec CIE
 * cannot be set or inferred here. Only LHDC exposes a negotiated depth field.
 * Current supported layouts stay strict; vendor extensions are not trimmed.
 * Outputs are disjoint from all inputs/descriptors and each other. No heap,
 * decoder work, task, borrow beyond return or RF. No partial configuration on
 * failure/no-space; capacity0 reports selection.required. A proposal reserves
 * nothing. Cross-family ranking/reconnect/default caller remains separate. */
dbt_result_t dbt_a2dp_generate(const dbt_avdtp_endpoint_t *,uint8_t,
    const dbt_avdtp_sep_t *,const uint8_t *,uint16_t,const dbt_a2dp_generate_request_t *,
    uint8_t *,uint16_t,dbt_a2dp_generate_result_t *);
#endif
