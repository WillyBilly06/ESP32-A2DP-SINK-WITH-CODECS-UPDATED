#include <malloc.h>

#include "ldacBT_internal.h"
#define LDACBT_LIB_VER_MAJOR  2
#define LDACBT_LIB_VER_MINOR  0
#define LDACBT_LIB_VER_BRANCH 33

enum {
  /* 2-DH5 */
  LDACBT_2DH5_02,
  LDACBT_2DH5_03,
  LDACBT_2DH5_04,
  LDACBT_2DH5_05,
  LDACBT_2DH5_06,
  LDACBT_2DH5_07,
  LDACBT_2DH5_08,
  LDACBT_2DH5_09,
  LDACBT_2DH5_10,
  LDACBT_2DH5_11,
  LDACBT_2DH5_12,
  LDACBT_2DH5_13,
  LDACBT_2DH5_14,
};
/* LDAC config table
 *  - NFRM/PCKT must be less than 16.
 */
DECLFUNC const LDACBT_CONFIG tbl_ldacbt_config[] = {
    /*
     *   index          , NFRM , LDAC  , FRM
     *                  , ---- ,  FRM  ,  LEN
     *                  , PCKT ,   LEN ,    /CH
     *                  ,      , [byte], [byte]
     */
    {LDACBT_2DH5_02, 2, 330, 165},
    {LDACBT_2DH5_03, 3, 220, 110},
    {LDACBT_2DH5_04, 4, 164, 82},
    {LDACBT_2DH5_05, 5, 132, 66},
    {LDACBT_2DH5_06, 6, 110, 55},
    {LDACBT_2DH5_07, 7, 94, 47},
    {LDACBT_2DH5_08, 8, 82, 41},
    {LDACBT_2DH5_09, 9, 72, 36},
    {LDACBT_2DH5_10, 10, 66, 33},
    {LDACBT_2DH5_11, 11, 60, 30},
    {LDACBT_2DH5_12, 12, 54, 27},
    {LDACBT_2DH5_13, 13, 50, 25},
    {LDACBT_2DH5_14, 14, 46, 23},
};
LDACBT_API int ldacBT_get_version() {
  // return 131105;
  return (((LDACBT_LIB_VER_MAJOR) << 16) | ((LDACBT_LIB_VER_MINOR) << 8) | (LDACBT_LIB_VER_BRANCH));
}

LDACBT_API void ldacBT_free_handle(HANDLE_LDAC_BT hLdacBT) {
  if (hLdacBT == NULL) return;

  if (hLdacBT->hLDAC != NULL) {
    /* close ldaclib handle */
    if (hLdacBT->proc_mode == LDACBT_PROCMODE_DECODE) {
      ldaclib_free_decode(hLdacBT->hLDAC);
    }
    /* free ldaclib handle */
    ldaclib_free_handle(hLdacBT->hLDAC);
    hLdacBT->hLDAC = NULL;
  }
  /* free ldacbt handle */
  free(hLdacBT);
}
/* Get LDAC handle */
LDACBT_API HANDLE_LDAC_BT ldacBT_get_handle(void) {
  HANDLE_LDAC_BT hLdacBT;
  hLdacBT = (HANDLE_LDAC_BT)malloc(sizeof(STRUCT_LDACBT_HANDLE));
  if (hLdacBT == NULL) return NULL;

  //* Get ldaclib Handler
  if ((hLdacBT->hLDAC = ldaclib_get_handle()) == NULL) {
    ldacBT_free_handle(hLdacBT);
    return NULL;
  }

  ldacBT_param_clear(hLdacBT);
  return hLdacBT;
}
LDACBT_API void ldacBT_close_handle(HANDLE_LDAC_BT hLdacBT) {
  if (hLdacBT == NULL) return;

  if (hLdacBT->hLDAC != NULL) {
    /* close ldaclib handle */
    if (hLdacBT->proc_mode == LDACBT_PROCMODE_DECODE) {
      ldaclib_free_decode(hLdacBT->hLDAC);
    }
    /* clear error code */
    ldaclib_clear_error_code(hLdacBT->hLDAC);
    ldaclib_clear_internal_error_code(hLdacBT->hLDAC);
  }
  /* clear ldacbt handle */
  ldacBT_param_clear(hLdacBT);
}

/* Get ERROR CODE */
LDACBT_API int ldacBT_get_error_code(HANDLE_LDAC_BT hLdacBT) {
  int error_code;
  if (hLdacBT == NULL) return LDACBT_ERR_FATAL_HANDLE << 10;

  ldacBT_check_ldaclib_error_code(hLdacBT);
  if (hLdacBT->error_code_api == LDACBT_GET_LDACLIB_ERROR_CODE) {
    error_code = LDACBT_ERR_FATAL << 20 | hLdacBT->error_code;
  } else if (hLdacBT->error_code_api != LDACBT_ERR_NONE) {
    error_code = hLdacBT->error_code_api << 20 | hLdacBT->error_code;
  } else {
    error_code = hLdacBT->error_code_api << 20;
  }
  return error_code;
}

LDACBT_API int ldacBT_get_sampling_freq(HANDLE_LDAC_BT hLdacBT) {
  if (!hLdacBT) return -1;
  if (hLdacBT->proc_mode == LDACBT_PROCMODE_DECODE) return hLdacBT->pcm.sf;
  hLdacBT->error_code_api = LDACBT_ERR_HANDLE_NOT_INIT;
  return -1;
}

LDACBT_API int ldacBT_get_bitrate(HANDLE_LDAC_BT hLdacBT) {
  if (!hLdacBT) return -1;
  if (hLdacBT->proc_mode == LDACBT_PROCMODE_DECODE) return hLdacBT->bitrate;
  hLdacBT->error_code_api = LDACBT_ERR_HANDLE_NOT_INIT;
  return -1;
}

LDACBT_API int ldacBT_init_handle_decode(
    HANDLE_LDAC_BT hLdacBT, int cm, int sf, int nshift, int var0, int var1) {
  int cci;
  int frmlen;
  int sfid;
  int channel;
  LDAC_RESULT result;
  if (hLdacBT == NULL) {
    return LDACBT_E_FAIL;
  }
  if ((hLdacBT->error_code_api = ldacBT_assert_cm(cm)) != LDACBT_ERR_NONE) {
    return LDACBT_E_FAIL;
  }
  if ((hLdacBT->error_code_api = ldacBT_assert_pcm_sampling_freq(sf)) != LDACBT_ERR_NONE) {
    return LDACBT_E_FAIL;
  }
  ldacBT_close_handle(hLdacBT);
  hLdacBT->flg_decode_inited = 0;
  hLdacBT->pcm.sf = sf;
  hLdacBT->nshift = nshift;
  hLdacBT->proc_mode = LDACBT_PROCMODE_DECODE;
  /* initilize ldac encode */
  /* Get sampling frequency index */
  result = ldaclib_get_sampling_rate_index(hLdacBT->pcm.sf, &sfid);
  if (LDAC_FAILED(result)) {
    hLdacBT->error_code_api = LDACBT_ERR_ILL_SAMPLING_FREQ;
    return LDACBT_E_FAIL;
  }
  hLdacBT->sfid = sfid;
  cci = ldacBT_cm_to_cci(cm);
  result = ldaclib_get_channel(cci, &channel);
  if (LDAC_FAILED(result)) {
    hLdacBT->error_code_api = LDACBT_GET_LDACLIB_ERROR_CODE;
    return LDACBT_E_FAIL;
  }
  hLdacBT->pcm.ch = channel;
  // tbl_ldacbt_config[0].frmlen_1ch;
  // frmlen = 165 * channel - LDACBT_FRMHDRBYTES;
  frmlen = tbl_ldacbt_config[0].frmlen_1ch * channel - LDACBT_FRMHDRBYTES;
  /* Set Configuration Information */
  result = ldaclib_set_config_info(hLdacBT->hLDAC, sfid, cci, frmlen, 0);
  if (LDAC_FAILED(result)) {
    hLdacBT->error_code_api = LDACBT_GET_LDACLIB_ERROR_CODE;
    return LDACBT_E_FAIL;
  } else if (result != LDAC_S_OK) {
    hLdacBT->error_code_api = LDACBT_GET_LDACLIB_ERROR_CODE;
  }
  hLdacBT->frmlen = frmlen;
  hLdacBT->cm = cm;
  hLdacBT->cci = cci;
  hLdacBT->frm_status = 0;
  hLdacBT->sfid = sfid;
  result = ldaclib_init_decode(hLdacBT->hLDAC, nshift);
  if (LDAC_FAILED(result)) {
    hLdacBT->error_code_api = LDACBT_GET_LDACLIB_ERROR_CODE;
    return LDACBT_E_FAIL;
  }
  hLdacBT->flg_decode_inited = 1;
  result = ldaclib_get_frame_samples(hLdacBT->sfid, &hLdacBT->frm_samples);
  if (LDAC_FAILED(result)) {
    hLdacBT->error_code_api = LDACBT_GET_LDACLIB_ERROR_CODE;
    return LDACBT_E_FAIL;
  }
  return LDACBT_S_OK;
}

LDACBT_API int ldacBT_decode(
    HANDLE_LDAC_BT hLdacBT,
    unsigned char* p_bs,
    unsigned char* p_pcm,
    LDACBT_SMPL_FMT_T fmt,
    int bs_bytes,
    int* used_bytes,
    int* wrote_bytes) {
  int cm;
  int tmp;
  int frm_samples;
  int sfid;
  int cci;
  int frmlen;
  int frm_status;

  if (!hLdacBT) return 1;
  if (hLdacBT->proc_mode != LDACBT_PROCMODE_DECODE) {
    hLdacBT->error_code_api = LDACBT_ERR_HANDLE_NOT_INIT;
    return 2;
  }
  if (!(p_bs && p_pcm && used_bytes && wrote_bytes)) {
    hLdacBT->error_code_api = LDACBT_ERR_ILL_PARAM;
    return 3;
  }
  if (bs_bytes <= 4) {
    hLdacBT->error_code_api = LDACBT_ERR_INPUT_BUFFER_SIZE;
    return 4;
  }
  hLdacBT->error_code_api = ldacBT_assert_sample_format(fmt);
  if (hLdacBT->error_code_api) {
    return 5;
  }
  *used_bytes = 0;
  *wrote_bytes = 0;
  if (LDAC_FAILED(
          ldaclib_get_frame_header(hLdacBT->hLDAC, p_bs, &sfid, &cci, &frmlen, &frm_status))) {
    hLdacBT->error_code_api = LDACBT_GET_LDACLIB_ERROR_CODE;
    return 6;
  }
  hLdacBT->error_code_api = ldacBT_assert_cci(cci);
  if (hLdacBT->error_code_api) {
    return 7;
  }
  cm = ldacBT_cci_to_cm(cci);
  if (hLdacBT->flg_decode_inited &&
      LDAC_FAILED(ldaclib_check_frame_header(hLdacBT->hLDAC, sfid, cci))) {
    ldaclib_get_sampling_rate(sfid, &hLdacBT->pcm.sf);
    if (ldacBT_init_handle_decode(hLdacBT, cm, hLdacBT->pcm.sf, hLdacBT->nshift, 0, 0)) return 9;
    hLdacBT->error_code_api = 40;
    tmp = 8;
  }
  tmp = 0;

  if (LDAC_FAILED(ldaclib_set_config_info(hLdacBT->hLDAC, sfid, cci, frmlen, frm_status))) {
    hLdacBT->error_code_api = LDACBT_GET_LDACLIB_ERROR_CODE;
    return 10;
  }
  hLdacBT->sfid = sfid;
  hLdacBT->frmlen = frmlen;
  hLdacBT->cm = cm;
  hLdacBT->cci = cci;
  hLdacBT->frm_status = frm_status;
  if (!hLdacBT->flg_decode_inited) {
    if (LDAC_FAILED(ldaclib_init_decode(hLdacBT->hLDAC, hLdacBT->nshift))) {
      hLdacBT->error_code_api = LDACBT_GET_LDACLIB_ERROR_CODE;
      return 11;
    }
    hLdacBT->flg_decode_inited = 1;
    if (LDAC_FAILED(ldaclib_get_frame_samples(hLdacBT->sfid, &hLdacBT->frm_samples))) {
      hLdacBT->error_code_api = LDACBT_ERR_ILL_SAMPLING_FREQ;
      return 12;
    }
    if (!hLdacBT->flg_decode_inited) return 13;
  }

  if (LDAC_FAILED(ldaclib_decode(
          hLdacBT->hLDAC,
          p_bs + LDACBT_FRMHDRBYTES,
          (void **) hLdacBT->pp_pcm,
          bs_bytes - LDACBT_FRMHDRBYTES,
          used_bytes,
          fmt))) {
    hLdacBT->error_code_api = LDACBT_GET_LDACLIB_ERROR_CODE;
    return 14;
  }
  if (*used_bytes) *used_bytes += LDACBT_FRMHDRBYTES;
  frm_samples = hLdacBT->frm_samples;
  hLdacBT->bitrate = *used_bytes * hLdacBT->pcm.sf / frm_samples / (1000 / 8);
  *wrote_bytes = ldacBT_interleave_pcm(
      p_pcm, (const char**)hLdacBT->pp_pcm, frm_samples, hLdacBT->pcm.ch, fmt);
  return tmp;
}
