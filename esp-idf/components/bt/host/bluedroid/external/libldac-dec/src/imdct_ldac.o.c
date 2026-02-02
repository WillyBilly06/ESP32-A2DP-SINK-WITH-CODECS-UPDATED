#include "ldac.h"

/* Optimized IMDCT core for ESP32 - uses restrict pointers and manual loop unrolling */
static void proc_imdct_core_ldac(SCALAR* __restrict__ p_y, SCALAR* __restrict__ p_x, int nlnn) {
  int i, j, k;
  int loop1, loop2;
  int coef, index0, index1, offset;
  const int nsmpl = npow2_ldac(nlnn);
  const int nsmpl_half = nsmpl >> 1;
  const int* __restrict__ p_rp;
  const SCALAR * __restrict__ p_w, * __restrict__ p_c, * __restrict__ p_s;
  SCALAR a_work[LDAC_MAXLSU];
  SCALAR a, b, c, d;
  SCALAR cc, cs;

  i = nlnn - LDAC_1FSLNN;
  p_w = gaa_bwin_ldac[i];
#if !(CONFIG_USED_CSI_DSP)
  p_c = gaa_wcos_ldac[i];
  p_s = gaa_wsin_ldac[i];
  p_rp = gaa_rev_perm_ldac[i];
  int tmp0;
  if (nsmpl >= 0)
    tmp0 = nsmpl >> 2;
  else
    tmp0 = (nsmpl + 3) >> 2;

  cc = p_c[0];
  cs = p_s[0];
  
  /* Unrolled first stage - process 8 elements at a time when possible */
  if (tmp0 >= 2) {
    for (i = 0; i < (tmp0 & ~1) << 2; i += 8) {
      SCALAR f1, f2, f3, f4, f5, f6, f7, f8;
      /* First group of 4 */
      f1 = p_y[p_rp[i + 0]];
      f2 = p_y[p_rp[i + 1]];
      f3 = p_y[p_rp[i + 2]];
      f4 = p_y[p_rp[i + 3]];
      a = f1 * cc + f2 * cs;
      b = f1 * cs - f2 * cc;
      a_work[i + 0] = a + f3;
      a_work[i + 1] = b + f4;
      a_work[i + 2] = f3 - a;
      a_work[i + 3] = f4 - b;
      /* Second group of 4 */
      f5 = p_y[p_rp[i + 4]];
      f6 = p_y[p_rp[i + 5]];
      f7 = p_y[p_rp[i + 6]];
      f8 = p_y[p_rp[i + 7]];
      a = f5 * cc + f6 * cs;
      b = f5 * cs - f6 * cc;
      a_work[i + 4] = a + f7;
      a_work[i + 5] = b + f8;
      a_work[i + 6] = f7 - a;
      a_work[i + 7] = f8 - b;
    }
    /* Handle remaining elements */
    for (; i < (tmp0 << 2); i += 4) {
      SCALAR f1, f2, f3, f4;
      f1 = p_y[p_rp[i + 0]];
      f2 = p_y[p_rp[i + 1]];
      f3 = p_y[p_rp[i + 2]];
      f4 = p_y[p_rp[i + 3]];
      a = f1 * cc + f2 * cs;
      b = f1 * cs - f2 * cc;
      a_work[i + 0] = a + f3;
      a_work[i + 1] = b + f4;
      a_work[i + 2] = f3 - a;
      a_work[i + 3] = f4 - b;
    }
  } else if (tmp0 > 0) {
    for (i = 0; i < (tmp0 << 2); i += 4) {
      SCALAR f1, f2, f3, f4;
      f1 = p_y[p_rp[i + 0]];
      f2 = p_y[p_rp[i + 1]];
      f3 = p_y[p_rp[i + 2]];
      f4 = p_y[p_rp[i + 3]];
      a = f1 * cc + f2 * cs;
      b = f1 * cs - f2 * cc;
      a_work[i + 0] = a + f3;
      a_work[i + 1] = b + f4;
      a_work[i + 2] = f3 - a;
      a_work[i + 3] = f4 - b;
    }
  }
  
  coef = 1;
  for (i = 1; i < (nlnn - 1); ++i) {
    loop1 = nsmpl >> (i + 2);
    loop2 = 1 << i;
    index0 = 0;
    index1 = 1 << (i + 1);
    offset = 1 << (i + 2);

    for (k = 0; k < loop2; ++k) {
      cc = p_c[coef];
      cs = p_s[coef++];
      for (j = 0; j < loop1; j++) {
        a = a_work[index0 + 0];
        b = a_work[index0 + 1];
        c = a_work[index1 + 0] * cc + a_work[index1 + 1] * cs;
        d = a_work[index1 + 0] * cs - a_work[index1 + 1] * cc;

        a_work[index0 + 0] = a + c;
        a_work[index0 + 1] = b + d;
        a_work[index1 + 0] = a - c;
        a_work[index1 + 1] = b - d;
        index0 += offset;
        index1 += offset;
      }
      index0 += 2 - nsmpl;
      index1 += 2 - nsmpl;
    }
  }

  /* Final stage - unroll by 2 */
  for (i = 0; i < nsmpl_half - 1; i += 2) {
    p_y[2 * i] = p_s[coef + i] * a_work[2 * i + 1] + p_c[coef + i] * a_work[2 * i];
    p_y[nsmpl - 2 * i - 1] = p_s[coef + i] * a_work[2 * i] - a_work[2 * i + 1] * p_c[coef + i];
    p_y[2 * (i + 1)] = p_s[coef + i + 1] * a_work[2 * (i + 1) + 1] + p_c[coef + i + 1] * a_work[2 * (i + 1)];
    p_y[nsmpl - 2 * (i + 1) - 1] = p_s[coef + i + 1] * a_work[2 * (i + 1)] - a_work[2 * (i + 1) + 1] * p_c[coef + i + 1];
  }
  /* Handle odd remaining element */
  if (i < nsmpl_half) {
    p_y[2 * i] = p_s[coef + i] * a_work[2 * i + 1] + p_c[coef + i] * a_work[2 * i];
    p_y[nsmpl - 2 * i - 1] = p_s[coef + i] * a_work[2 * i] - a_work[2 * i + 1] * p_c[coef + i];
  }
#else
  csi_dct4_f32(&dct4f32, a_work, p_y);
#endif

  /* Windowing stage - unroll by 2 */
  for (i = 0; i < nsmpl_half - 1; i += 2) {
    p_x[i] = p_y[nsmpl_half + i] * p_w[i] - p_w[nsmpl - 1 - i] * p_x[nsmpl + i];
    p_x[nsmpl_half + i] = -p_w[nsmpl_half - 1 - i] * p_x[nsmpl + nsmpl_half + i] -
                          p_y[nsmpl - 1 - i] * p_w[nsmpl_half + i];
    p_x[nsmpl + i] = p_y[nsmpl_half - 1 - i];
    p_x[nsmpl + nsmpl_half + i] = p_y[i];
    
    p_x[i + 1] = p_y[nsmpl_half + i + 1] * p_w[i + 1] - p_w[nsmpl - 2 - i] * p_x[nsmpl + i + 1];
    p_x[nsmpl_half + i + 1] = -p_w[nsmpl_half - 2 - i] * p_x[nsmpl + nsmpl_half + i + 1] -
                              p_y[nsmpl - 2 - i] * p_w[nsmpl_half + i + 1];
    p_x[nsmpl + i + 1] = p_y[nsmpl_half - 2 - i];
    p_x[nsmpl + nsmpl_half + i + 1] = p_y[i + 1];
  }
  /* Handle odd remaining element */
  if (i < nsmpl_half) {
    p_x[i] = p_y[nsmpl_half + i] * p_w[i] - p_w[nsmpl - 1 - i] * p_x[nsmpl + i];
    p_x[nsmpl_half + i] = -p_w[nsmpl_half - 1 - i] * p_x[nsmpl + nsmpl_half + i] -
                          p_y[nsmpl - 1 - i] * p_w[nsmpl_half + i];
    p_x[nsmpl + i] = p_y[nsmpl_half - 1 - i];
    p_x[nsmpl + nsmpl_half + i] = p_y[i];
  }
}

DECLFUNC void proc_imdct_ldac(SFINFO* p_sfinfo, int nlnn) {
  AC* p_ac;
  int ich;
  int nchs = p_sfinfo->cfg.ch;
  if (nchs > 0) {
    for (ich = 0; ich < nchs; ich++) {
      p_ac = p_sfinfo->ap_ac[ich];
      proc_imdct_core_ldac(p_ac->p_acsub->a_spec, p_ac->p_acsub->a_time, nlnn);
    }
  }
  return;
}
