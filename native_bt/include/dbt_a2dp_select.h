#ifndef DBT_A2DP_SELECT_H
#define DBT_A2DP_SELECT_H
#include "dbt_avdtp_query.h"
#include "dbt_avdtp_sink.h"

#define DBT_A2DP_SELECT_MAX_CAPABILITIES 1024u
#define DBT_A2DP_SELECT_MAX_CODEC 255u

typedef enum {DBT_A2DP_DELAY_DISABLED,DBT_A2DP_DELAY_IF_SUPPORTED,DBT_A2DP_DELAY_REQUIRED} dbt_a2dp_delay_policy_t;
typedef struct {
    const uint8_t *codec; /* Exact Media Codec category body, not Android masks. */
    uint16_t codec_size,avdtp_version; /* 0 means version unknown: no Delay. */
    dbt_a2dp_delay_policy_t delay;
} dbt_a2dp_select_request_t;
typedef enum {
    DBT_A2DP_SELECT_NONE,DBT_A2DP_SELECT_SUCCESS,DBT_A2DP_SELECT_PEER_UNAVAILABLE,
    DBT_A2DP_SELECT_NOT_AUDIO_SOURCE,DBT_A2DP_SELECT_PEER_CAPS_INVALID,
    DBT_A2DP_SELECT_MISSING_SERVICE,DBT_A2DP_SELECT_LOCAL_DESCRIPTOR_INVALID,
    DBT_A2DP_SELECT_NO_LOCAL_CODEC,DBT_A2DP_SELECT_LOCAL_INCOMPATIBLE,
    DBT_A2DP_SELECT_PEER_INCOMPATIBLE,DBT_A2DP_SELECT_DELAY_UNSUPPORTED,DBT_A2DP_SELECT_NO_SPACE
} dbt_a2dp_select_outcome_t;
typedef struct {
    dbt_a2dp_select_outcome_t outcome;
    dbt_result_t peer_status;
    dbt_avdtp_cap_error_t peer_error,local_error;
    uint16_t configuration_size,required;
    uint8_t local_seid,remote_seid,local_codec_error,peer_codec_error;
    bool delay_enabled;
} dbt_a2dp_select_result_t;

/* Pure bounded exact-format proposal for a new stream. All local descriptors
 * are immutable Sink endpoints. The peer must be an available audio Source.
 * Both sides require Transport+Codec and must accept the exact requested CIE
 * through the existing codec validator; no codec/rate/quality fallback occurs.
 * Peer codec media-type RFU bits are ignored, output RFU bits stay zero.
 * Delay requires an explicit policy, known AVDTP>=1.3 and both advertisements.
 * Input views are borrowed only for this call. Outputs must be disjoint from
 * every input/descriptor buffer and each other. Invalid pointers/aliases leave
 * outputs untouched. Valid mismatch/no-space returns detailed result but never
 * partial configuration bytes. NULL/size0 output supports required-size query.
 * No heap, decoder work, callbacks with external effects, tasks or RF. The
 * result reserves nothing; the peer may still reject subsequent configuration.
 * Existing-stream reconfigure and automatic candidate generation/ranking are
 * separate caller policy, not supplied by this exact-format matcher. */
dbt_result_t dbt_a2dp_select_exact(const dbt_avdtp_endpoint_t *,uint8_t count,
    const dbt_avdtp_sep_t *,const uint8_t *capabilities,uint16_t capabilities_size,
    const dbt_a2dp_select_request_t *,uint8_t *configuration,uint16_t capacity,
    dbt_a2dp_select_result_t *);
#endif
