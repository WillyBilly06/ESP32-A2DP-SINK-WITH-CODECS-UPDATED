#include "lhdc_diag_config.h"
#include <string.h>

volatile int g_lhdc_diag_force_ref_imdct_960 = 0;
volatile int g_lhdc_diag_sns_mode = LHDC_DIAG_SNS_CURRENT;
volatile int g_lhdc_diag_disable_coeff_rev = 0;
volatile int g_lhdc_diag_ola_mode = LHDC_DIAG_OLA_SHIFTED;
volatile int g_lhdc_diag_output_mode = 0;
volatile int g_lhdc_diag_sel0_mode = 0;
volatile int g_lhdc_diag_sel1_mode = 0;

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void lhdc_diag_set_modes(int force_ref_imdct_960,
                         int sns_mode,
                         int disable_coeff_rev,
                         int ola_mode)
{
    g_lhdc_diag_force_ref_imdct_960 = force_ref_imdct_960 ? 1 : 0;
    g_lhdc_diag_sns_mode = clamp_int(sns_mode, 0, 2);
    g_lhdc_diag_disable_coeff_rev = disable_coeff_rev ? 1 : 0;
    g_lhdc_diag_ola_mode = clamp_int(ola_mode, 0, 2);
}

void lhdc_diag_get_modes(int *force_ref_imdct_960,
                         int *sns_mode,
                         int *disable_coeff_rev,
                         int *ola_mode)
{
    if (force_ref_imdct_960) *force_ref_imdct_960 = g_lhdc_diag_force_ref_imdct_960;
    if (sns_mode)            *sns_mode = g_lhdc_diag_sns_mode;
    if (disable_coeff_rev)   *disable_coeff_rev = g_lhdc_diag_disable_coeff_rev;
    if (ola_mode)            *ola_mode = g_lhdc_diag_ola_mode;
}

int lhdc_diag_set_one(const char *key, int value)
{
    if (!key) return -1;

    if (strcmp(key, "ref") == 0 || strcmp(key, "imdct_ref") == 0) {
        g_lhdc_diag_force_ref_imdct_960 = value ? 1 : 0;
        return 0;
    }
    if (strcmp(key, "sns") == 0) {
        g_lhdc_diag_sns_mode = clamp_int(value, 0, 2);
        return 0;
    }
    if (strcmp(key, "rev") == 0 || strcmp(key, "coeff_rev") == 0) {
        g_lhdc_diag_disable_coeff_rev = value ? 1 : 0;
        return 0;
    }
    if (strcmp(key, "ola") == 0) {
        g_lhdc_diag_ola_mode = clamp_int(value, 0, 2);
        return 0;
    }
    if (strcmp(key, "out") == 0 || strcmp(key, "output") == 0) {
        g_lhdc_diag_output_mode = clamp_int(value, 0, 5);
        return 0;
    }
    if (strcmp(key, "sel0") == 0 || strcmp(key, "selector0") == 0) {
        g_lhdc_diag_sel0_mode = clamp_int(value, 0, 2);
        return 0;
    }
    if (strcmp(key, "sel1") == 0 || strcmp(key, "selector1") == 0) {
        g_lhdc_diag_sel1_mode = clamp_int(value, 0, 2);
        return 0;
    }

    return -1;
}
