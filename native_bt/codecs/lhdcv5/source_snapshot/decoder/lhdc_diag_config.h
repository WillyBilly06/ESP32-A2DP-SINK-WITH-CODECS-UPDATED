#ifndef LHDC_DIAG_CONFIG_H
#define LHDC_DIAG_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Runtime LHDC V5 diagnostics.
 *
 * These are intentionally runtime variables, not #define switches, so you can
 * flash ONE firmware and change tests from your wrapper/console while playing
 * the same 50 Hz / 96 kHz stream.
 *
 * Default values keep the current best-sounding shifted low-overlap OLA path.
 */

typedef enum {
    LHDC_DIAG_SNS_CURRENT  = 0,  /* current decoder path: divide by SNS gain */
    LHDC_DIAG_SNS_MULTIPLY = 1,  /* test alternate direction */
    LHDC_DIAG_SNS_BYPASS   = 2,  /* leave spectrum unshaped */
} lhdc_diag_sns_mode_t;

typedef enum {
    LHDC_DIAG_OLA_SHIFTED  = 0,  /* current best shifted low-overlap OLA */
    LHDC_DIAG_OLA_LEGACY   = 1,  /* old full 50 percent OLA, A/B only */
    LHDC_DIAG_OLA_DELAYED  = 2,  /* shifted OLA delayed-transition test */
} lhdc_diag_ola_mode_t;

extern volatile int g_lhdc_diag_force_ref_imdct_960;
extern volatile int g_lhdc_diag_sns_mode;
extern volatile int g_lhdc_diag_disable_coeff_rev;
extern volatile int g_lhdc_diag_ola_mode;

/* Extra runtime isolation for remaining low-frequency artifact.
 * out: 0=normal stereo, 1=ch0 to both, 2=ch1 to both, 3=left only,
 *      4=right only, 5=swap L/R.
 * sel0/sel1: 0=auto selector, 1=force descramble selector 0, 2=force selector 1.
 */
extern volatile int g_lhdc_diag_output_mode;
extern volatile int g_lhdc_diag_sel0_mode;
extern volatile int g_lhdc_diag_sel1_mode;

void lhdc_diag_set_modes(int force_ref_imdct_960,
                         int sns_mode,
                         int disable_coeff_rev,
                         int ola_mode);

void lhdc_diag_get_modes(int *force_ref_imdct_960,
                         int *sns_mode,
                         int *disable_coeff_rev,
                         int *ola_mode);

/* Small helper for console/UART commands. Examples:
 *   lhdc_diag_set_one("ref", 1);
 *   lhdc_diag_set_one("sns", 2);
 *   lhdc_diag_set_one("rev", 1);
 *   lhdc_diag_set_one("ola", 0);
 *   lhdc_diag_set_one("out", 1);   // ch0 to both outputs
 *   lhdc_diag_set_one("sel0", 2);  // force ch0 selector 1
 * Returns 0 on success, -1 for an unknown key.
 */
int lhdc_diag_set_one(const char *key, int value);

#ifdef __cplusplus
}
#endif

#endif /* LHDC_DIAG_CONFIG_H */
