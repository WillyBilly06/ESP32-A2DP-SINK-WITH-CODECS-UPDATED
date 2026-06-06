#include "ldac.h"
#include "ldaclib.h"
#include <malloc.h>

#define LDACLIB_MAJOR_VERSION  01
#define LDACLIB_MINOR_VERSION  00
#define LDACLIB_BRANCH_VERSION 00
/***************************************************************************************************
    Local Assert Functions
***************************************************************************************************/
int ldaclib_assert_sampling_rate_index(int smplrate_id) {
  if ((LDAC_SMPLRATEID_0 <= smplrate_id) && (smplrate_id < LDAC_NSMPLRATEID)) {
    return LDAC_TRUE;
  } else {
    return LDAC_FALSE;
  }
}
int ldaclib_assert_supported_sampling_rate_index(int smplrate_id) {
  if ((LDAC_SMPLRATEID_0 <= smplrate_id) && (smplrate_id < LDAC_NSUPSMPLRATEID)) {
    return LDAC_TRUE;
  } else {
    return LDAC_FALSE;
  }
}
static int ldaclib_assert_channel_config_index(int chconfig_id) {
  if ((chconfig_id == LDAC_CHCONFIGID_MN) || (chconfig_id == LDAC_CHCONFIGID_DL) ||
      (chconfig_id == LDAC_CHCONFIGID_ST)) {
    return LDAC_TRUE;
  } else {
    return LDAC_FALSE;
  }
}
static int ldaclib_assert_channel(int ch) {
  if ((ch == LDAC_CHANNEL_1CH) || (ch == LDAC_CHANNEL_2CH)) {
    return LDAC_TRUE;
  } else {
    return LDAC_FALSE;
  }
}
static int ldaclib_assert_frame_length(int frame_length) {
  if ((0 < frame_length) && (frame_length <= LDAC_MAXNBYTES)) {
    return LDAC_TRUE;
  } else {
    return LDAC_FALSE;
  }
}
static int ldaclib_assert_supported_frame_length(int frame_length, int chconfig_id) {
  if (chconfig_id == LDAC_CHCONFIGID_MN) {
    if ((LDAC_MINSUPNBYTES / 2 <= frame_length) && (frame_length <= LDAC_MAXSUPNBYTES / 2)) {
      return LDAC_TRUE;
    } else {
      return LDAC_FALSE;
    }
  } else if ((chconfig_id == LDAC_CHCONFIGID_DL) || (chconfig_id == LDAC_CHCONFIGID_ST)) {
    if ((LDAC_MINSUPNBYTES <= frame_length) && (frame_length <= LDAC_MAXSUPNBYTES)) {
      return LDAC_TRUE;
    } else {
      return LDAC_FALSE;
    }
  } else {
    return LDAC_FALSE;
  }
}
static int ldaclib_assert_frame_status(int frame_status) {
  if ((LDAC_FRMSTAT_LEV_0 <= frame_status) && (frame_status <= LDAC_FRMSTAT_LEV_3)) {
    return LDAC_TRUE;
  } else {
    return LDAC_FALSE;
  }
}
int ldaclib_assert_nlnn_shift(int nlnn_shift) {
  if ((-2 <= nlnn_shift) && (nlnn_shift < LDAC_NSFTSTEP - 2)) {
    return LDAC_TRUE;
  } else {
    return LDAC_FALSE;
  }
}
/***************************************************************************************************
    Common API Functions
***************************************************************************************************/

/***************************************************************************************************
    Get Library Version
***************************************************************************************************/
DECLSPEC int ldaclib_get_version(void) {
  return (LDACLIB_MAJOR_VERSION << 16) | (LDACLIB_MINOR_VERSION << 8) | LDACLIB_BRANCH_VERSION;
}

DECLSPEC int ldaclib_get_major_version(void) {
  return LDACLIB_MAJOR_VERSION;
}

DECLSPEC int ldaclib_get_minor_version(void) {
  return LDACLIB_MINOR_VERSION;
}

DECLSPEC int ldaclib_get_branch_version(void) {
  return LDACLIB_BRANCH_VERSION;
}
/***************************************************************************************************
    Get Basic Parameters
***************************************************************************************************/
DECLSPEC LDAC_RESULT ldaclib_get_sampling_rate_index(int smplrate, int* p_smplrate_id) {
  if (smplrate == 44100) {
    *p_smplrate_id = LDAC_SMPLRATEID_0;
  } else if (smplrate == 48000) {
    *p_smplrate_id = LDAC_SMPLRATEID_1;
  } else if (smplrate == 88200) {
    *p_smplrate_id = LDAC_SMPLRATEID_2;
  } else if (smplrate == 96000) {
    *p_smplrate_id = LDAC_SMPLRATEID_3;
  } else {
    return LDAC_E_FAIL;
  }

  return LDAC_S_OK;
}

DECLSPEC LDAC_RESULT ldaclib_get_sampling_rate(int smplrate_id, int* p_smplrate) {
  if (!ldaclib_assert_sampling_rate_index(smplrate_id)) {
    return LDAC_E_FAIL;
  }
  if (!ldaclib_assert_supported_sampling_rate_index(smplrate_id)) {
    return LDAC_E_FAIL;
  }

  *p_smplrate = ga_smplrate_ldac[smplrate_id];

  return LDAC_S_OK;
}

DECLSPEC LDAC_RESULT ldaclib_get_frame_samples(int smplrate_id, int* p_framesmpls) {
  if (!ldaclib_assert_sampling_rate_index(smplrate_id)) {
    return LDAC_E_FAIL;
  }
  if (!ldaclib_assert_supported_sampling_rate_index(smplrate_id)) {
    return LDAC_E_FAIL;
  }

  *p_framesmpls = ga_framesmpls_ldac[smplrate_id];

  return LDAC_S_OK;
}

DECLSPEC LDAC_RESULT ldaclib_get_nlnn(int smplrate_id, int* p_nlnn) {
  if (!ldaclib_assert_sampling_rate_index(smplrate_id)) {
    return LDAC_E_FAIL;
  }
  if (!ldaclib_assert_supported_sampling_rate_index(smplrate_id)) {
    return LDAC_E_FAIL;
  }

  *p_nlnn = ga_ln_framesmpls_ldac[smplrate_id];

  return LDAC_S_OK;
}
DECLSPEC LDAC_RESULT ldaclib_get_channel(int chconfig_id, int* p_ch) {
  if (!ldaclib_assert_channel_config_index(chconfig_id)) {
    return LDAC_E_FAIL;
  }

  *p_ch = ga_ch_ldac[chconfig_id];

  return LDAC_S_OK;
}

DECLSPEC LDAC_RESULT ldaclib_get_channel_config_index(int ch, int* p_chconfig_id) {
  if (!ldaclib_assert_channel(ch)) {
    return LDAC_E_FAIL;
  }

  *p_chconfig_id = ga_chconfig_id_ldac[ch];

  return LDAC_S_OK;
}

DECLSPEC LDAC_RESULT ldaclib_check_nlnn_shift(int smplrate_id, int nlnn_shift) {
  if (!ldaclib_assert_sampling_rate_index(smplrate_id)) {
    return LDAC_E_FAIL;
  }
  if (!ldaclib_assert_supported_sampling_rate_index(smplrate_id)) {
    return LDAC_E_FAIL;
  }
  if (!ldaclib_assert_nlnn_shift(nlnn_shift)) {
    return LDAC_E_FAIL;
  }

  if (gaa_nlnn_shift_ldac[smplrate_id][nlnn_shift + 2] < 0) {
    return LDAC_E_FAIL;
  }

  return LDAC_S_OK;
}

/***************************************************************************************************
    Get Handle
***************************************************************************************************/
DECLSPEC HANDLE_LDAC ldaclib_get_handle(void) {
  HANDLE_LDAC hData;

  hData = (HANDLE_LDAC)malloc(sizeof(HANDLE_LDAC_STRUCT));
  if (hData != (HANDLE_LDAC)NULL) {
    clear_data_ldac(hData, sizeof(HANDLE_LDAC_STRUCT));
    hData->sfinfo.p_mempos = (char*)NULL;
    hData->error_code = LDAC_ERR_NONE;
  }

  return hData;
}

/***************************************************************************************************
    Free Handle
***************************************************************************************************/
DECLSPEC LDAC_RESULT ldaclib_free_handle(HANDLE_LDAC hData) {
  if (hData != (HANDLE_LDAC)NULL) {
    if (hData->sfinfo.p_mempos != (char*)NULL) {
      return LDAC_S_OK;
    }

    free(hData);
  }

  return LDAC_S_OK;
}
/***************************************************************************************************
    Set Configuration Information
***************************************************************************************************/
DECLSPEC LDAC_RESULT ldaclib_set_config_info(
    HANDLE_LDAC hData, int smplrate_id, int chconfig_id, int frame_length, int frame_status) {
  CFG* p_cfg = &hData->sfinfo.cfg;

  if (!ldaclib_assert_sampling_rate_index(smplrate_id)) {
    hData->error_code = LDAC_ERR_ASSERT_SAMPLING_RATE;
    return LDAC_E_FAIL;
  }

  if (!ldaclib_assert_supported_sampling_rate_index(smplrate_id)) {
    hData->error_code = LDAC_ERR_ASSERT_SUP_SAMPLING_RATE;
    return LDAC_E_FAIL;
  }

  if (!ldaclib_assert_channel_config_index(chconfig_id)) {
    hData->error_code = LDAC_ERR_ASSERT_CHANNEL_CONFIG;
    return LDAC_E_FAIL;
  }

  if (!ldaclib_assert_frame_length(frame_length)) {
    hData->error_code = LDAC_ERR_ASSERT_FRAME_LENGTH;
    return LDAC_E_FAIL;
  }

  if (!ldaclib_assert_supported_frame_length(frame_length, chconfig_id)) {
    hData->error_code = LDAC_ERR_ASSERT_SUP_FRAME_LENGTH;
    return LDAC_E_FAIL;
  }

  if (!ldaclib_assert_frame_status(frame_status)) {
    hData->error_code = LDAC_ERR_ASSERT_FRAME_STATUS;
    return LDAC_E_FAIL;
  }

  p_cfg->smplrate_id = smplrate_id;
  p_cfg->chconfig_id = chconfig_id;
  p_cfg->frame_length = frame_length;
  p_cfg->frame_status = frame_status;

  ldaclib_get_channel(chconfig_id, &p_cfg->ch);

  return LDAC_S_OK;
}

/***************************************************************************************************
    Get Configuration Information
***************************************************************************************************/
DECLSPEC LDAC_RESULT ldaclib_get_config_info(
    HANDLE_LDAC hData,
    int* p_smplrate_id,
    int* p_chconfig_id,
    int* p_frame_length,
    int* p_frame_status) {
  CFG* p_cfg = &hData->sfinfo.cfg;

  *p_smplrate_id = p_cfg->smplrate_id;
  *p_chconfig_id = p_cfg->chconfig_id;
  *p_frame_length = p_cfg->frame_length;
  *p_frame_status = p_cfg->frame_status;

  return LDAC_S_OK;
}

LDAC_RESULT ldaclib_get_frame_header(
    HANDLE_LDAC hData,
    uint8_t* p_stream,
    int* p_smplrate_id,
    int* p_chconfig_id,
    int* p_frame_length,
    int* p_frame_status) {
  if (unpack_frame_header_ldac(
          p_smplrate_id, p_chconfig_id, p_frame_length, p_frame_status, p_stream))
    return 0;
  hData->error_code = LDAC_ERR_ILL_SYNCWORD;
  return LDAC_E_FAIL;
}

LDAC_RESULT ldaclib_check_frame_header(HANDLE_LDAC hData, int smplrate_id, int chconfig_id) {
  LDAC_RESULT result;

  if ((unsigned int)smplrate_id <= 5) {
    if ((unsigned int)smplrate_id > 3) {
      hData->error_code = LDAC_ERR_ASSERT_SUP_SAMPLING_RATE;
      return LDAC_E_FAIL;
    } else if ((unsigned int)chconfig_id > 2) {
      result = LDAC_E_FAIL;
      hData->error_code = LDAC_ERR_ASSERT_CHANNEL_CONFIG;
    } else if (smplrate_id == hData->sfinfo.cfg.smplrate_id) {
      if (chconfig_id == hData->sfinfo.cfg.chconfig_id) {
        return 0;
      } else {
        result = LDAC_E_FAIL;
        hData->error_code = LDAC_ERR_CHECK_CHANNEL_CONFIG;
      }
    } else {
      hData->error_code = LDAC_ERR_CHECK_SAMPLING_RATE;
      return LDAC_E_FAIL;
    }
  } else {
    hData->error_code = LDAC_ERR_ASSERT_SAMPLING_RATE;
    return LDAC_E_FAIL;
  }
  return result;
}
LDAC_RESULT ldaclib_init_decode(HANDLE_LDAC hData, int nlnn_shift) {
  unsigned int smplrate_id;
  LDAC_RESULT result;
  SFINFO* p_sfinfo;
  int nlnn;

  smplrate_id = hData->sfinfo.cfg.smplrate_id;
  if (smplrate_id > 3 || (unsigned int)(nlnn_shift + 2) > 4 ||
      gaa_nlnn_shift_ldac[smplrate_id][nlnn_shift + 2] < 0) {
    hData->error_code = LDAC_ERR_ASSERT_NSHIFT;
    return LDAC_E_FAIL;
  } else {
    p_sfinfo = &hData->sfinfo;
    nlnn = ga_ln_framesmpls_ldac[smplrate_id] + nlnn_shift;
    hData->nlnn = nlnn;
    set_imdct_table_ldac(nlnn);
    result = init_decode_ldac(p_sfinfo);
    if (result) {
      hData->error_code = LDAC_ERR_DEC_INIT_ALLOC;
      return LDAC_E_FAIL;
    }
  }
  return result;
}

LDAC_RESULT ldaclib_free_decode(HANDLE_LDAC hData) {
  if (!hData->sfinfo.p_mempos) free_decode_ldac(&hData->sfinfo);
  return 0;
}

LDAC_RESULT ldaclib_decode(
    HANDLE_LDAC hData,
    uint8_t* p_stream,
    void** ap_pcm,
    int frame_length,
    int* p_nbytes_used,
    LDAC_SMPL_FMT_T sample_format) {
  SFINFO* p_sfinfo;
  int error_code;
  int nlnn;
  int loc;

  loc = 0;
  if (hData->sfinfo.cfg.frame_length > frame_length) {
    hData->error_code = LDAC_ERR_INPUT_BUFFER_SIZE;
    return LDAC_E_FAIL;
  }
  if ((unsigned int)(sample_format - 2) > 3) {
    hData->error_code = LDAC_ERR_ILL_SMPL_FORMAT;
    return LDAC_E_FAIL;
  } else {
    p_sfinfo = &hData->sfinfo;
    error_code = unpack_raw_data_frame_ldac(&hData->sfinfo, p_stream, &loc, p_nbytes_used);
    if (error_code <= 0) {
      decode_ldac(p_sfinfo);
      proc_imdct_ldac(p_sfinfo, hData->nlnn);
      nlnn = hData->nlnn;
      set_output_pcm_ldac(p_sfinfo, ap_pcm, sample_format, nlnn);
      return hData->sfinfo.error_code < LDAC_ERR_FATAL;
    }
    hData->error_code = error_code;
    return LDAC_E_FAIL;
  }
}
LDAC_RESULT ldaclib_clear_error_code(HANDLE_LDAC hData) {
  hData->error_code = LDAC_ERR_NONE;
  return 0;
}

LDAC_RESULT ldaclib_get_error_code(HANDLE_LDAC hData, int* p_error_code) {
  *p_error_code = hData->error_code;
  return 0;
}

LDAC_RESULT ldaclib_clear_internal_error_code(HANDLE_LDAC hData) {
  hData->sfinfo.error_code = LDAC_ERR_NONE;
  return 0;
}

LDAC_RESULT ldaclib_get_internal_error_code(HANDLE_LDAC hData, int* p_error_code) {
  *p_error_code = hData->sfinfo.error_code;
  return 0;
}
