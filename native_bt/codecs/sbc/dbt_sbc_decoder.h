#ifndef DBT_SBC_DECODER_H
#define DBT_SBC_DECODER_H
#include "dbt_sbc_media.h"

typedef struct {
    void *context;
    uint8_t *(*allocate)(void *context,uint16_t size);
    void (*release)(void *context,uint8_t *bytes);
    dbt_result_t (*pcm)(void *context,const int16_t *pcm,uint16_t samples,
                        uint8_t channels,uint32_t rate,uint32_t timestamp);
} dbt_sbc_decoder_hooks_t;
typedef struct {
    dbt_sbc_decoder_hooks_t hooks;
    dbt_sbc_config_t config;
    void *state;
    uint32_t frames,failures;
    uint16_t allocated_bytes;
} dbt_sbc_decoder_t;

/* Decoder-worker owner only. Init is allocation-free. open() releases all old
 * state before constructing a replacement. Its failure leaves it closed: a
 * reconfiguration provider must restore the old config or report Fatal.
 * The only SDK dependency is its standalone Apache-licensed SBC codec; neither
 * Bluedroid protocol code nor the BTstack reference is included or called. */
dbt_result_t dbt_sbc_decoder_init(dbt_sbc_decoder_t *decoder,dbt_sbc_decoder_hooks_t hooks);
dbt_result_t dbt_sbc_decoder_open(dbt_sbc_decoder_t *decoder,dbt_sbc_config_t config);
void dbt_sbc_decoder_close(dbt_sbc_decoder_t *decoder);
dbt_result_t dbt_sbc_decoder_reset(dbt_sbc_decoder_t *decoder);
/* Validated frames only; verifies again before entering the external codec. */
dbt_result_t dbt_sbc_decoder_frame(void *decoder,const dbt_sbc_frame_t *frame,uint32_t timestamp);

#endif
