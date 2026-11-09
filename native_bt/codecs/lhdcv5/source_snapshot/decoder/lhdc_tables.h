#ifndef LHDC_TABLES_H
#define LHDC_TABLES_H

#include "lhdc_dec_internal.h"

/* Band configuration indices */
typedef enum {
    LHDC_BAND_CFG_480    = 0,
    LHDC_BAND_CFG_480_HR = 1,
    LHDC_BAND_CFG_480_LB = 2,
    LHDC_BAND_CFG_960    = 3,
    LHDC_BAND_CFG_1920   = 4,
    LHDC_BAND_CFG_COUNT  = 5,
} lhdc_band_cfg_idx_t;

/* Band configuration descriptor */
typedef struct lhdc_band_cfg_desc {
    uint8_t   cfg_idx;
    uint16_t  num_sfb;
    uint16_t  mdct_size;
    const uint16_t *band_off;
    const uint16_t *band_scale;
    /* FAC moving-average window sizes (band_cfg[0x28]/[0x2c] in the encoder).
     * Dumped from liblhdcv5.so; the encoder switches its adaptive model from the
     * fill phase to the sliding-window phase at these counts. Wrong windows
     * desync the range decoder mid-stream. 48k/5ms (spf=240): win1=57, win2=63. */
    uint16_t  ma_win1;
    uint16_t  ma_win2;
} lhdc_band_cfg_desc_t;

extern const lhdc_band_cfg_desc_t lhdc_band_cfgs[LHDC_BAND_CFG_COUNT];
extern const uint16_t lhdc_step_size_table[128];
extern const uint32_t lhdc_bitrate_table_44k[10];
extern const uint32_t lhdc_bitrate_table_48k[10];
extern const uint32_t lhdc_bitrate_table_96k[10];
extern const uint32_t lhdc_bitrate_table_192k[10];
extern const uint32_t lhdc_band_stop_table[36];

const lhdc_band_cfg_desc_t *lhdc_get_band_cfg(uint32_t sample_rate, uint8_t frame_duration_ms);
/* Per-frame variant: also keys on the per-channel payload bytes, mirroring the
 * encoder's bitrate-dependent segment-config selection at 44.1/48 kHz. */
const lhdc_band_cfg_desc_t *lhdc_get_band_cfg_frame(uint32_t sample_rate,
                                                    uint8_t frame_duration_ms,
                                                    int per_ch_bytes);
uint16_t lhdc_get_samples_per_frame(uint32_t sample_rate, uint8_t frame_duration_ms);
uint16_t lhdc_get_mdct_size(uint32_t sample_rate, uint8_t frame_duration_ms);
const uint32_t *lhdc_get_bitrate_table(uint32_t sample_rate);
void lhdc_gen_kbd_window(float *window, int mdct_size);

/* FAC (Fast Arithmetic Coding) decoder */
#define LHDC_FAC_MAX_SYMBOLS 64

typedef struct {
    uint32_t range;
    uint32_t low;
    uint32_t sym_count[LHDC_FAC_MAX_SYMBOLS];
    uint32_t total_count;
    uint8_t  num_symbols;
} lhdc_fac_dec_t;

void lhdc_fac_dec_init(lhdc_fac_dec_t *fac, uint8_t num_symbols);
int   lhdc_fac_dec_read(lhdc_fac_dec_t *fac, lhdc_dec_bit_reader_t *br);

/* Const (flash) MDCT window for mdct_size, or NULL if none precomputed. */
const float *lhdc_get_window_const(int mdct_size);

#endif /* LHDC_TABLES_H */
