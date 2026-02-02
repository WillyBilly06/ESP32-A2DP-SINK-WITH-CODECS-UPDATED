#include "ldacBT_internal.h"

/* Clear LDAC handle parameters */
DECLFUNC void ldacBT_param_clear(HANDLE_LDAC_BT hLdacBT) {
  if (hLdacBT == NULL) {
    return;
  }
  hLdacBT->sfid = UNSET;
  hLdacBT->pcm.sf = UNSET;
  hLdacBT->tx.mtu = UNSET;
  hLdacBT->tx.tx_size = UNSET;
  hLdacBT->tx.pkt_hdr_sz = UNSET;
  hLdacBT->frmlen_tx = UNSET;
  hLdacBT->tx.nfrm_in_pkt = UNSET;
  hLdacBT->frmlen = UNSET;
  hLdacBT->tgt_nfrm_in_pkt = UNSET;
  hLdacBT->tgt_frmlen = UNSET;
  hLdacBT->tgt_eqmid = UNSET;
  hLdacBT->cm = UNSET;
  hLdacBT->cci = UNSET;
  hLdacBT->eqmid = UNSET;
  hLdacBT->transport = UNSET;
  hLdacBT->proc_mode = LDACBT_PROCMODE_UNSET;
  hLdacBT->error_code = 0;
  hLdacBT->error_code_api = 0;
  hLdacBT->frm_samples = 0;
  hLdacBT->pcm.ch = 0;
  hLdacBT->nshift = 0;
  hLdacBT->frm_status = 0;
  hLdacBT->bitrate = 0;
  hLdacBT->stat_alter_op = 0;
  hLdacBT->flg_decode_inited = 0;
  hLdacBT->pcm.fmt = LDACBT_SMPL_FMT_S24;
  hLdacBT->ap_pcm[0] = &hLdacBT->a_pcm[0];
  hLdacBT->ap_pcm[1] = &hLdacBT->a_pcm[LDACBT_MAX_LSU * LDACBT_PCM_WLEN_MAX];
  hLdacBT->pp_pcm = hLdacBT->ap_pcm;
  clear_data_ldac(hLdacBT->a_pcm, LDAC_PRCNCH * LDACBT_MAX_LSU * LDACBT_PCM_WLEN_MAX);
}

/* get ldaclib error code */
DECLFUNC int ldacBT_check_ldaclib_error_code(HANDLE_LDAC_BT hLdacBT) {
  HANDLE_LDAC hData;
  int error_code, internal_error_code;

  if (hLdacBT == NULL) {
    return LDACBT_E_FAIL;
  }
  if ((hData = hLdacBT->hLDAC) == NULL) {
    return LDACBT_E_FAIL;
  }

  ldaclib_get_error_code(hData, &error_code);

  ldaclib_get_internal_error_code(hData, &internal_error_code);

  hLdacBT->error_code = error_code << 10 | internal_error_code;

  return LDACBT_S_OK;
}

/* Assertions. */
DECLFUNC int ldacBT_assert_cm(int cm) {
  if ((cm != LDACBT_CHANNEL_MODE_STEREO) && (cm != LDACBT_CHANNEL_MODE_DUAL_CHANNEL) &&
      (cm != LDACBT_CHANNEL_MODE_MONO)) {
    return LDACBT_ERR_ASSERT_CHANNEL_MODE;
  }
  return LDACBT_ERR_NONE;
}
DECLFUNC int ldacBT_assert_cci(int cci) {
  if ((cci != LDAC_CCI_STEREO) && (cci != LDAC_CCI_DUAL_CHANNEL) && (cci != LDAC_CCI_MONO)) {
    return LDACBT_ERR_ASSERT_CHANNEL_CONFIG;
  }
  return LDACBT_ERR_NONE;
}

DECLFUNC int ldacBT_assert_sample_format(LDACBT_SMPL_FMT_T fmt) {
  if ((fmt != LDACBT_SMPL_FMT_S16) && (fmt != LDACBT_SMPL_FMT_S24) &&
      (fmt != LDACBT_SMPL_FMT_S32) && (fmt != LDACBT_SMPL_FMT_F32)) {
    return LDACBT_ERR_ILL_SMPL_FORMAT;
  }
  return LDACBT_ERR_NONE;
}

DECLFUNC int ldacBT_assert_pcm_sampling_freq(int sampling_freq) {
  if ((sampling_freq != 1 * 44100) && (sampling_freq != 1 * 48000) &&
      (sampling_freq != 2 * 44100) && (sampling_freq != 2 * 48000)) {
    return LDACBT_ERR_ILL_SAMPLING_FREQ;
  }
  return LDACBT_ERR_NONE;
}

DECLFUNC int ldacBT_interleave_pcm(
    unsigned char* p_pcm, const char** pp_pcm, int nsmpl, int nch, LDACBT_SMPL_FMT_T fmt) {
  if (nsmpl <= 0) {
    return 0;
  }
  const void* p_lch = pp_pcm[0];
  const void* p_rch = pp_pcm[1];
  if (nch == 2) {
    switch (fmt) {
      case LDACBT_SMPL_FMT_S16:
        for (int i = 0; i < nsmpl; i++) {
          ((short*)p_pcm)[(i << 1) + 0] = ((const short*)p_lch)[i];
          ((short*)p_pcm)[(i << 1) + 1] = ((const short*)p_rch)[i];
        }
        return 2 * 2 * nsmpl;
      case LDACBT_SMPL_FMT_S24:
        for (int i = 0; i < nsmpl * 3; i += 3) {
          p_pcm[(i << 1) + 0] = ((const unsigned char*)p_lch)[i + 0];
          p_pcm[(i << 1) + 1] = ((const unsigned char*)p_lch)[i + 1];
          p_pcm[(i << 1) + 2] = ((const unsigned char*)p_lch)[i + 2];
          p_pcm[(i << 1) + 3] = ((const unsigned char*)p_rch)[i + 0];
          p_pcm[(i << 1) + 4] = ((const unsigned char*)p_rch)[i + 1];
          p_pcm[(i << 1) + 5] = ((const unsigned char*)p_rch)[i + 2];
        }
        return 2 * 3 * nsmpl;
      case LDACBT_SMPL_FMT_S32:
      case LDACBT_SMPL_FMT_F32:
        for (int i = 0; i < nsmpl; i++) {
          ((unsigned int*)p_pcm)[(i << 1) + 0] = ((const unsigned int*)p_lch)[i];
          ((unsigned int*)p_pcm)[(i << 1) + 1] = ((const unsigned int*)p_rch)[i];
        }
        return 2 * 4 * nsmpl;
      default:
        return 0;
    }
  } else {
    if (nch != 1) return 0;
    switch (fmt) {
      case LDACBT_SMPL_FMT_S16:
        copy_data_ldac(*pp_pcm, p_pcm, 2 * nsmpl);
        return 2 * nsmpl;
      case LDACBT_SMPL_FMT_S24:
        copy_data_ldac(*pp_pcm, p_pcm, 3 * nsmpl);
        return 3 * nsmpl;
      case LDACBT_SMPL_FMT_S32:
      case LDACBT_SMPL_FMT_F32:
        copy_data_ldac(*pp_pcm, p_pcm, 4 * nsmpl);
        return 4 * nsmpl;
      default:
        return 0;
    }
  }
}

/* Get channel_config_index from channel_mode.
 * The argument cm, channel_mode, must be checked by function ldacBT_assert_cm()
 * before calling this function.
 */
DECLFUNC int ldacBT_cm_to_cci(int cm) {
  if (cm == LDACBT_CHANNEL_MODE_STEREO) {
    return LDAC_CCI_STEREO;
  } else if (cm == LDACBT_CHANNEL_MODE_DUAL_CHANNEL) {
    return LDAC_CCI_DUAL_CHANNEL;
  } else /* if( cm == LDACBT_CHANNEL_MODE_MONO )*/ {
    return LDAC_CCI_MONO;
  }
}

/* Get channel_mode from channel_config_index.
 * The argument cci, channel_config_index, must be checked by the function
 * ldacBT_assert_cci() before calling this function.
 */
UNUSED_ATTR DECLFUNC int ldacBT_cci_to_cm(int cci) {
  if (cci == LDAC_CCI_STEREO) {
    return LDACBT_CHANNEL_MODE_STEREO;
  } else if (cci == LDAC_CCI_DUAL_CHANNEL) {
    return LDACBT_CHANNEL_MODE_DUAL_CHANNEL;
  } else /* if( cci == LDAC_CCI_MONO )*/ {
    return LDACBT_CHANNEL_MODE_MONO;
  }
}
