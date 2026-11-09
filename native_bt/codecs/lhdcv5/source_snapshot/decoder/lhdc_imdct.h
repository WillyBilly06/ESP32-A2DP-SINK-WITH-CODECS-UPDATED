#ifndef LHDC_IMDCT_H
#define LHDC_IMDCT_H

#include "lhdc_dec_internal.h"

/*
 * Initialize IMDCT tables for a given size.
 * Returns 0 on success.
 */
int lhdc_imdct_init(int mdct_size);

/*
 * Perform IMDCT.
 * in:  MDCT coefficients (mdct_size elements)
 * out: time-domain samples (mdct_size elements)
 * mdct_size: transform size
 */
void lhdc_imdct_transform(const float *in, float *out, int mdct_size);

/*
 * Free the lazily-allocated N=960 (96k) fast-IMDCT tables (~15 KB DRAM). Call
 * when reconfiguring to a non-960 rate or tearing down the decoder so the block
 * isn't resident while 96k is not in use. Safe to call when nothing is
 * allocated.
 */
void lhdc_imdct_free_960(void);

/*
 * Free the lazily-allocated N=480 (44.1/48k) fast-IMDCT tables (~4.2 KB DRAM).
 * Call when reconfiguring to a non-480 rate or tearing down the decoder. Safe
 * to call when nothing is allocated.
 */
void lhdc_imdct_free_480(void);

/*
 * Free the lazily-allocated N=1920 (192k) fast-IMDCT tables (~13 KB DRAM). Call
 * when reconfiguring to a non-1920 rate or tearing down the decoder. Safe to
 * call when nothing is allocated.
 */
void lhdc_imdct_free_1920(void);

/* Free the active reference cosine table (4*N floats, up to ~30 KB). */
void lhdc_imdct_free_cos(void);

#endif /* LHDC_IMDCT_H */
