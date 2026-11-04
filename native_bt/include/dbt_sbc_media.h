#ifndef DBT_SBC_MEDIA_H
#define DBT_SBC_MEDIA_H

#include "dbt_avdtp.h"

/* Legal SBC header dimensions, before the profile bitrate bound. */
#define DBT_SBC_MAX_FRAME_BYTES 524u
typedef struct {
    uint32_t rate;
    uint16_t samples;
    uint8_t format,channels,minimum_bitpool,maximum_bitpool;
} dbt_sbc_config_t;
typedef struct {
    const uint8_t *data;
    uint16_t size,samples;
    uint8_t channels,bitpool;
} dbt_sbc_frame_t;
typedef struct {
    void *context;
    uint8_t *(*allocate)(void *context,uint16_t size);
    void (*release)(void *context,uint8_t *data);
    /* Worker only, borrowed exact validated frame. Codec/PCM work may run here.
     * Timestamp belongs to this frame, with samples added for each following
     * frame in an unfragmented packet. A callback error stops the packet. */
    dbt_result_t (*frame)(void *context,const dbt_sbc_frame_t *frame,uint32_t timestamp);
} dbt_sbc_media_hooks_t;
typedef struct {
    dbt_sbc_media_hooks_t hooks;
    dbt_sbc_config_t config;
    uint8_t *fragment;
    uint32_t timestamp,ssrc,deadline,fragment_ms;
    uint32_t packets,frames,rejected,gaps;
    uint16_t size,capacity,chunk_size,next_sequence;
    uint8_t remaining;
    bool initialized;
} dbt_sbc_media_t;

/* The Media Codec category body: audio/type SBC + four CIE bytes. No allocation. */
dbt_result_t dbt_sbc_configure(const uint8_t *codec,uint16_t size,dbt_sbc_config_t *config);
/* Header dimensions, negotiated format/bitpool, legal bitrate and SBC CRC over
 * header/joint/scalefactors. Does not validate all quantized audio payload bits.
 * size may include subsequent frames; view.size identifies this exact frame. */
dbt_result_t dbt_sbc_frame_decode(const dbt_sbc_config_t *config,const uint8_t *data,
    uint16_t size,dbt_sbc_frame_t *frame);
dbt_result_t dbt_sbc_frame_bytes(const dbt_sbc_config_t *config,uint8_t bitpool,uint16_t *size);
/* One decoder worker owns init/receive/expire/reset. No retained RTP pointers.
 * Unfragmented media allocates nothing and validates the entire packet before
 * dispatch. Fragment storage is lazy, capped by the actual frame header, and
 * released on completion, error, discontinuity or expiry. */
dbt_result_t dbt_sbc_media_init(dbt_sbc_media_t *stream,dbt_sbc_media_hooks_t hooks,
    dbt_sbc_config_t config,uint32_t fragment_ms);
dbt_result_t dbt_sbc_media_receive(dbt_sbc_media_t *stream,const dbt_avdtp_media_t *media,uint32_t now_ms);
void dbt_sbc_media_expire(dbt_sbc_media_t *stream,uint32_t now_ms);
void dbt_sbc_media_reset(dbt_sbc_media_t *stream);

#endif
