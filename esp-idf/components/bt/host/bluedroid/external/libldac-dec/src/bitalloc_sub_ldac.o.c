#include "ldac.h"
/***************************************************************************************************
    Calculate Additional Word Length Data - Optimized for int8_t types
***************************************************************************************************/
LDAC_HOT void calc_add_word_length_ldac(AC* __restrict__ p_ac) {
  const int nqus = p_ac->p_ab->nqus;
  const int8_t* __restrict__ p_idsf = p_ac->a_idsf;
  int8_t* __restrict__ p_addwl = p_ac->a_addwl;

  /* Use memset for faster clearing */
  memset(p_addwl, 0, LDAC_MAXNQUS * sizeof(int8_t));

  if (LDAC_LIKELY(p_ac->p_ab->grad_mode != LDAC_MODE_0)) {
    for (int iqu = 1; iqu < nqus; iqu++) {
      const int dif = (int)p_idsf[iqu] - (int)p_idsf[iqu - 1];

      /* Branchless version using min/max */
      if (dif > 0) {
        p_addwl[iqu] += (dif > 5) ? 5 : ((dif > 1) ? dif - 1 : 0);
      } else if (dif < 0) {
        const int neg_dif = -dif;
        p_addwl[iqu - 1] += (neg_dif > 5) ? 5 : ((neg_dif > 1) ? neg_dif - 1 : 0);
      }
    }
  }
}
LDAC_HOT DECLFUNC void reconst_gradient_ldac(AB* __restrict__ p_ab, int lqu, int hqu) {
  const int grad_qu_h = p_ab->grad_qu_h;
  const int grad_qu_l = p_ab->grad_qu_l;
  const int grad_os_l = p_ab->grad_os_l;
  const int grad_os_h = p_ab->grad_os_h;
  const int tmp_range = grad_qu_h - grad_qu_l;
  int8_t* __restrict__ p_grad = p_ab->a_grad;
  
  /* Initialize gradients */
  for (int iqu = 0; iqu < grad_qu_h; iqu++) {
    p_grad[iqu] = (int8_t)(-grad_os_l);
  }
  for (int iqu = grad_qu_h; iqu < hqu; iqu++) {
    p_grad[iqu] = (int8_t)(-grad_os_h);
  }
  
  if (LDAC_LIKELY(tmp_range > 0)) {
    const unsigned char* __restrict__ p_t = gaa_resamp_grad_ldac[tmp_range - 1];
    const int tmp = grad_os_h - grad_os_l;
    
    if (tmp > 0) {
      const int tmp_adj = tmp - 1;
      for (int iqu = grad_qu_l; iqu < grad_qu_h; iqu++) {
        p_grad[iqu] -= (int8_t)(((*p_t++ * tmp_adj) >> 8) + 1);
      }
    } else if (tmp < 0) {
      const int tmp_adj = -tmp - 1;
      for (int iqu = grad_qu_l; iqu < grad_qu_h; iqu++) {
        p_grad[iqu] += (int8_t)(((*p_t++ * tmp_adj) >> 8) + 1);
      }
    }
  }
}

LDAC_HOT DECLFUNC void reconst_word_length_ldac(AC* __restrict__ p_ac) {
  AB* __restrict__ p_ab = p_ac->p_ab;
  const int hqu = p_ab->nqus;
  const int grad_mode = p_ab->grad_mode;
  const int8_t* __restrict__ p_grad = p_ab->a_grad;
  const int8_t* __restrict__ p_idsf = p_ac->a_idsf;
  const int8_t* __restrict__ p_addwl = p_ac->a_addwl;
  int8_t* __restrict__ p_idwl1 = p_ac->a_idwl1;
  int8_t* __restrict__ p_idwl2 = p_ac->a_idwl2;

  /* Use switch for better branch prediction */
  switch (grad_mode) {
    case LDAC_MODE_0:
      for (int iqu = 0; iqu < hqu; iqu++) {
        int idwl1 = (int)p_idsf[iqu] + (int)p_grad[iqu];
        p_idwl1[iqu] = (int8_t)((idwl1 <= 0) ? 1 : idwl1);
      }
      break;
    case LDAC_MODE_1:
      for (int iqu = 0; iqu < hqu; iqu++) {
        int idwl1 = (int)p_idsf[iqu] + (int)p_grad[iqu] + (int)p_addwl[iqu];
        idwl1 = (idwl1 > 0) ? (idwl1 >> 1) : idwl1;
        p_idwl1[iqu] = (int8_t)((idwl1 <= 0) ? 1 : idwl1);
      }
      break;
    case LDAC_MODE_2:
      for (int iqu = 0; iqu < hqu; iqu++) {
        int idwl1 = (int)p_idsf[iqu] + (int)p_grad[iqu] + (int)p_addwl[iqu];
        idwl1 = (idwl1 > 0) ? ((idwl1 * 3) >> 3) : idwl1;
        p_idwl1[iqu] = (int8_t)((idwl1 <= 0) ? 1 : idwl1);
      }
      break;
    case LDAC_MODE_3:
      for (int iqu = 0; iqu < hqu; iqu++) {
        int idwl1 = (int)p_idsf[iqu] + (int)p_grad[iqu] + (int)p_addwl[iqu];
        idwl1 = (idwl1 > 0) ? (idwl1 >> 2) : idwl1;
        p_idwl1[iqu] = (int8_t)((idwl1 <= 0) ? 1 : idwl1);
      }
      break;
  }
  
  /* Apply nadjqus adjustment */
  const int nadjqus = p_ab->nadjqus;
  for (int iqu = 0; iqu < nadjqus; iqu++) {
    p_idwl1[iqu] += 1;
  }

  /* Calculate idwl2 with overflow handling */
  for (int iqu = 0; iqu < hqu; iqu++) {
    p_idwl2[iqu] = 0;
    if (LDAC_UNLIKELY(p_idwl1[iqu] > LDAC_MAXIDWL1)) {
      int idwl2 = (int)p_idwl1[iqu] - LDAC_MAXIDWL1;
      p_idwl2[iqu] = (int8_t)((idwl2 >= LDAC_MAXIDWL2) ? LDAC_MAXIDWL2 : idwl2);
      p_idwl1[iqu] = (int8_t)LDAC_MAXIDWL1;
    }
  }
}
