#include "ldac.h"

/* Optimized dequantization with restrict pointers and loop unrolling
 * Updated for int16_t a_qspec/a_rspec arrays */
__attribute__((always_inline)) 
static inline void dequant_spectrum_core_ldac(AC* __restrict__ p_ac, int iqu) {
  int i;
  const int isp = ga_isp_ldac[iqu];
  const int nsps = ga_nsps_ldac[iqu];
  const int16_t* __restrict__ p_qspec = p_ac->a_qspec + isp;
  const SCALAR iqf = ga_iqf_ldac[p_ac->a_idwl1[iqu]];
  const SCALAR sf = ga_sf_ldac[p_ac->a_idsf[iqu]];
  SCALAR* __restrict__ p_nspec = p_ac->p_acsub->a_spec + isp;
  const SCALAR scale = iqf * sf;

  /* Unroll by 4 for most common case (nsps is typically 2,4,8,16) */
  const int nsps4 = nsps & ~3;
  for (i = 0; i < nsps4; i += 4) {
    p_nspec[i + 0] = (SCALAR)p_qspec[i + 0] * scale;
    p_nspec[i + 1] = (SCALAR)p_qspec[i + 1] * scale;
    p_nspec[i + 2] = (SCALAR)p_qspec[i + 2] * scale;
    p_nspec[i + 3] = (SCALAR)p_qspec[i + 3] * scale;
  }
  /* Handle remaining elements */
  for (; i < nsps; i++) {
    p_nspec[i] = (SCALAR)p_qspec[i] * scale;
  }
}

DECLFUNC void dequant_spectrum_ldac(AC* p_ac) {
  int iqu;
  const int nqus = p_ac->p_ab->nqus;
  for (iqu = 0; iqu < nqus; iqu++) {
    dequant_spectrum_core_ldac(p_ac, iqu);
  }
}

__attribute__((always_inline))
static inline void dequant_residual_core_ldac(AC* __restrict__ p_ac, int iqu) {
  int i;
  const int nsps = ga_nsps_ldac[iqu];
  const int isp = ga_isp_ldac[iqu];
  const int16_t* __restrict__ p_rspec = p_ac->a_rspec + isp;
  const SCALAR iqf = ga_iqf_ldac[p_ac->a_idwl2[0]];
  const SCALAR sf = ga_sf_ldac[p_ac->a_idsf[0]];
  const SCALAR rsf = ga_rsf_ldac[LDAC_MAXIDWL1];
  const SCALAR tmp = rsf * iqf * sf;
  SCALAR* __restrict__ p_nspec = p_ac->p_acsub->a_spec + isp;

  /* Unroll by 4 */
  const int nsps4 = nsps & ~3;
  for (i = 0; i < nsps4; i += 4) {
    p_nspec[i + 0] += tmp * (SCALAR)p_rspec[i + 0];
    p_nspec[i + 1] += tmp * (SCALAR)p_rspec[i + 1];
    p_nspec[i + 2] += tmp * (SCALAR)p_rspec[i + 2];
    p_nspec[i + 3] += tmp * (SCALAR)p_rspec[i + 3];
  }
  /* Handle remaining elements */
  for (; i < nsps; i++) {
    p_nspec[i] += tmp * (SCALAR)p_rspec[i];
  }
}

DECLFUNC void dequant_residual_ldac(AC* p_ac) {
  int nqus;
  int iqu;
  nqus = p_ac->p_ab->nqus;
  for (iqu = 0; iqu < nqus; iqu++) {
    if (__builtin_expect(p_ac->a_idwl2[iqu] > 0, 0)) {
      dequant_residual_core_ldac(p_ac, iqu);
    }
  }
}

DECLFUNC void clear_spectrum_ldac(AC* p_ac, int nsps) {
  clear_data_ldac(p_ac->p_acsub->a_spec, sizeof(SCALAR) * nsps);
}
