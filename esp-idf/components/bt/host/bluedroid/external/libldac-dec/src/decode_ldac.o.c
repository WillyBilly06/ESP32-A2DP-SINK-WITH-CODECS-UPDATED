#include "ldac.h"
DECLFUNC LDAC_RESULT init_decode_ldac(SFINFO* p_sfinfo) {
  AB* p_ab;
  LDAC_RESULT result = LDAC_S_OK;
  CFG* p_cfg = &p_sfinfo->cfg;
  int nchs = p_cfg->ch;
  int nbks = gaa_block_setting_ldac[p_cfg->chconfig_id][1];
  int ch_offset = 0;

  for (int ich = 0; ich < nchs; ich++) {
    p_sfinfo->ap_ac[ich] = (AC*)calloc_ldac(p_sfinfo, 1u, sizeof(AC));
    if (p_sfinfo->ap_ac[ich] != (AC*)NULL) {
      p_sfinfo->ap_ac[ich]->p_acsub = (ACSUB*)calloc_ldac(p_sfinfo, 1u, sizeof(ACSUB));
      if (p_sfinfo->ap_ac[ich]->p_acsub == (ACSUB*)NULL) {
        p_sfinfo->error_code = LDAC_ERR_ALLOC_MEMORY;
        result = LDAC_E_FAIL;
        break;
      }
    } else {
      p_sfinfo->error_code = LDAC_ERR_ALLOC_MEMORY;
      result = LDAC_E_FAIL;
      break;
    }
  }

  p_ab = (AB*)calloc_ldac(p_sfinfo, nbks, sizeof(AB));
  p_sfinfo->p_ab = p_ab;
  if (p_sfinfo->p_ab == (AB*)NULL) {
    p_sfinfo->error_code = LDAC_ERR_ALLOC_MEMORY;
    return LDAC_E_FAIL;
  }
  p_sfinfo->error_code = LDAC_ERR_NONE;
  int blk_type, blk_nchs;

  /* Set AB information */
  p_ab = p_sfinfo->p_ab;
  for (int ibk = 0; ibk < nbks; ibk++) {
    p_ab->blk_type = blk_type = gaa_block_setting_ldac[p_cfg->chconfig_id][ibk + 2];
    p_ab->blk_nchs = blk_nchs = get_block_nchs_ldac(blk_type);
    p_ab->p_smplrate_id = &p_cfg->smplrate_id;
    p_ab->p_error_code = &p_sfinfo->error_code;

    /* Set AC Information */
    for (int ich = 0; ich < blk_nchs; ich++) {
      p_ab->ap_ac[ich] = p_sfinfo->ap_ac[ch_offset++];
      p_ab->ap_ac[ich]->p_ab = p_ab;
      p_ab->ap_ac[ich]->ich = ich;
      p_ab->ap_ac[ich]->frmana_cnt = 0;
    }

    p_ab++;
  }

  return result;
}
DECLFUNC void free_decode_ldac(SFINFO* p_sfinfo) {
  int ich;
  int nchs = p_sfinfo->cfg.ch;

  /* Free AB */
  if (p_sfinfo->p_ab != (AB*)NULL) {
    free(p_sfinfo->p_ab);
    p_sfinfo->p_ab = (AB*)NULL;
  }

  /* Free AC */
  for (ich = 0; ich < nchs; ich++) {
    if (p_sfinfo->ap_ac[ich] != (AC*)NULL) {
      if (p_sfinfo->ap_ac[ich]->p_acsub != (ACSUB*)NULL) {
        free(p_sfinfo->ap_ac[ich]->p_acsub);
        p_sfinfo->ap_ac[ich]->p_acsub = (ACSUB*)NULL;
      }
      free(p_sfinfo->ap_ac[ich]);
      p_sfinfo->ap_ac[ich] = (AC*)NULL;
    }
  }

  return;
}

/* Hot path - mark for optimization */
LDAC_HOT DECLFUNC void decode_ldac(SFINFO* __restrict__ p_sfinfo) {
  AB* __restrict__ p_ab;
  int nbks;
  int ibk;
  int nchs;
  int ich;

  p_ab = p_sfinfo->p_ab;
  nbks = gaa_block_setting_ldac[p_sfinfo->cfg.chconfig_id][1];
  for (ibk = 0; ibk < nbks; ++ibk) {
    nchs = p_ab->blk_nchs;
    for (ich = 0; ich < nchs; ++ich) {
      AC* __restrict__ p_ac = p_ab->ap_ac[ich];
      clear_spectrum_ldac(p_ac, LDAC_MAXLSU);
      dequant_spectrum_ldac(p_ac);
      dequant_residual_ldac(p_ac);
    }
    ++p_ab;
  }
}
