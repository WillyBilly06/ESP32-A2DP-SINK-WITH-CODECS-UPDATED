#include "ldacBT.h"
#include <malloc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Convert LDAC Error Code to string */
#define CASE_RETURN_STR(const)                                                 \
  case const:                                                                  \
    return #const;
static const char *ldac_ErrCode2Str(int ErrCode) {
  switch (ErrCode) {
    CASE_RETURN_STR(LDACBT_ERR_NONE);
    CASE_RETURN_STR(LDACBT_ERR_NON_FATAL);
    CASE_RETURN_STR(LDACBT_ERR_BIT_ALLOCATION);
    CASE_RETURN_STR(LDACBT_ERR_NOT_IMPLEMENTED);
    CASE_RETURN_STR(LDACBT_ERR_NON_FATAL_ENCODE);
    CASE_RETURN_STR(LDACBT_ERR_FATAL);
    CASE_RETURN_STR(LDACBT_ERR_SYNTAX_BAND);
    CASE_RETURN_STR(LDACBT_ERR_SYNTAX_GRAD_A);
    CASE_RETURN_STR(LDACBT_ERR_SYNTAX_GRAD_B);
    CASE_RETURN_STR(LDACBT_ERR_SYNTAX_GRAD_C);
    CASE_RETURN_STR(LDACBT_ERR_SYNTAX_GRAD_D);
    CASE_RETURN_STR(LDACBT_ERR_SYNTAX_GRAD_E);
    CASE_RETURN_STR(LDACBT_ERR_SYNTAX_IDSF);
    CASE_RETURN_STR(LDACBT_ERR_SYNTAX_SPEC);
    CASE_RETURN_STR(LDACBT_ERR_BIT_PACKING);
    CASE_RETURN_STR(LDACBT_ERR_ALLOC_MEMORY);
    CASE_RETURN_STR(LDACBT_ERR_FATAL_HANDLE);
    CASE_RETURN_STR(LDACBT_ERR_ILL_SYNCWORD);
    CASE_RETURN_STR(LDACBT_ERR_ILL_SMPL_FORMAT);
    CASE_RETURN_STR(LDACBT_ERR_ILL_PARAM);
    CASE_RETURN_STR(LDACBT_ERR_ASSERT_SAMPLING_FREQ);
    CASE_RETURN_STR(LDACBT_ERR_ASSERT_SUP_SAMPLING_FREQ);
    CASE_RETURN_STR(LDACBT_ERR_CHECK_SAMPLING_FREQ);
    CASE_RETURN_STR(LDACBT_ERR_ASSERT_CHANNEL_CONFIG);
    CASE_RETURN_STR(LDACBT_ERR_CHECK_CHANNEL_CONFIG);
    CASE_RETURN_STR(LDACBT_ERR_ASSERT_FRAME_LENGTH);
    CASE_RETURN_STR(LDACBT_ERR_ASSERT_SUP_FRAME_LENGTH);
    CASE_RETURN_STR(LDACBT_ERR_ASSERT_FRAME_STATUS);
    CASE_RETURN_STR(LDACBT_ERR_ASSERT_NSHIFT);
    CASE_RETURN_STR(LDACBT_ERR_ASSERT_CHANNEL_MODE);
    CASE_RETURN_STR(LDACBT_ERR_ENC_INIT_ALLOC);
    CASE_RETURN_STR(LDACBT_ERR_ENC_ILL_GRADMODE);
    CASE_RETURN_STR(LDACBT_ERR_ENC_ILL_GRADPAR_A);
    CASE_RETURN_STR(LDACBT_ERR_ENC_ILL_GRADPAR_B);
    CASE_RETURN_STR(LDACBT_ERR_ENC_ILL_GRADPAR_C);
    CASE_RETURN_STR(LDACBT_ERR_ENC_ILL_GRADPAR_D);
    CASE_RETURN_STR(LDACBT_ERR_ENC_ILL_NBANDS);
    CASE_RETURN_STR(LDACBT_ERR_PACK_BLOCK_FAILED);
    CASE_RETURN_STR(LDACBT_ERR_DEC_INIT_ALLOC);
    CASE_RETURN_STR(LDACBT_ERR_INPUT_BUFFER_SIZE);
    CASE_RETURN_STR(LDACBT_ERR_UNPACK_BLOCK_FAILED);
    CASE_RETURN_STR(LDACBT_ERR_UNPACK_BLOCK_ALIGN);
    CASE_RETURN_STR(LDACBT_ERR_UNPACK_FRAME_ALIGN);
    CASE_RETURN_STR(LDACBT_ERR_FRAME_LENGTH_OVER);
    CASE_RETURN_STR(LDACBT_ERR_FRAME_ALIGN_OVER);
    CASE_RETURN_STR(LDACBT_ERR_ALTER_EQMID_LIMITED);
    CASE_RETURN_STR(LDACBT_ERR_ILL_EQMID);
    CASE_RETURN_STR(LDACBT_ERR_ILL_SAMPLING_FREQ);
    CASE_RETURN_STR(LDACBT_ERR_ILL_NUM_CHANNEL);
    CASE_RETURN_STR(LDACBT_ERR_ILL_MTU_SIZE);
    CASE_RETURN_STR(LDACBT_ERR_HANDLE_NOT_INIT);
  default:
    return "unknown-error-code";
  }
}
char a_ErrorCodeStr[128];
const char *get_error_code_string(int error_code) {
  int errApi, errHdl, errBlk;

  errApi = LDACBT_API_ERR(error_code);
  errHdl = LDACBT_HANDLE_ERR(error_code);
  errBlk = LDACBT_BLOCK_ERR(error_code);

  a_ErrorCodeStr[0] = '\0';
  strcat(a_ErrorCodeStr, "API:");
  strcat(a_ErrorCodeStr, ldac_ErrCode2Str(errApi));
  strcat(a_ErrorCodeStr, " Handle:");
  strcat(a_ErrorCodeStr, ldac_ErrCode2Str(errHdl));
  strcat(a_ErrorCodeStr, " Block:");
  strcat(a_ErrorCodeStr, ldac_ErrCode2Str(errBlk));
  return a_ErrorCodeStr;
}
LDACBT_SMPL_FMT_T kFormat[] = {LDACBT_SMPL_FMT_S16, LDACBT_SMPL_FMT_S24,
                               LDACBT_SMPL_FMT_S32, LDACBT_SMPL_FMT_F32};
int32_t kEqmidValue[] = {LDACBT_EQMID_HQ, LDACBT_EQMID_SQ, LDACBT_EQMID_MQ};
int32_t kChannel[] = {LDACBT_CHANNEL_MODE_STEREO,
                      LDACBT_CHANNEL_MODE_DUAL_CHANNEL,
                      LDACBT_CHANNEL_MODE_MONO};
int32_t kSamplingFrequency[] = {44100, 48000, 2 * 44100, 2 * 48000};
int32_t kLdacBtRequiredMtu = 679;
int32_t kMaxWlValue = 4;
int32_t kMinChValue = 1;
int32_t kMaxChValue = 2;
int32_t kOutputSize = 1024;

// Typedefs for the wave file
typedef char ID[4];

// Stores the file header information
typedef union {
  struct {
    ID groupID;
    uint32_t size;
    ID riffType;
  };
  uint8_t data[0xc];

} WAVRIFFHEADER;
typedef union {
  struct {
    ID chunkID;
    uint32_t chunkSize;
    uint16_t wFormatTag;
    uint16_t wChannels;
    uint32_t dwSamplesPerSec;
    uint32_t dwAvgBytesPerSec;
    uint16_t wBlockAlign;
    uint16_t wBitsPerSample;
  };
  uint8_t data[0x18];

} FORMATCHUNK;
typedef union {
  struct {
    ID chunkID;
    uint32_t chunkSize;
  };
  uint8_t data[8];
} DATACHUNK;
typedef struct {
  WAVRIFFHEADER WAVRIFF_HEADER;
  FORMATCHUNK FORMAT_CHUNK;
  DATACHUNK DATA_CHUNK;
} WAV_INF;

int main(int argc, char *argv[]) {

  FILE *fp = NULL;
  uint32_t input_pcm_size, SamplesPerSec, AvgBytesPerSec;
  uint16_t BitsPerSample, FormatTag, Channels;
  char filepath[128] = {0};
  char filename[128] = "512kMeasSweep_0_to_24000_-12_dBV_48k_PCM16_LR";
  if (argc > 1) {
    memset(filename, 0, sizeof(filename));
    strcat(filename, argv[1]);
  }

  strcat(filepath, "../");
  strcat(filepath, filename);
  strcat(filepath, ".wav");
  strcat(filepath, "\0");
  fp = fopen(filepath, "rb");
  WAV_INF *wav_info = malloc(sizeof(WAV_INF));
  fread(wav_info, 1, sizeof(WAV_INF), fp);
  SamplesPerSec = wav_info->FORMAT_CHUNK.dwSamplesPerSec;
  BitsPerSample = wav_info->FORMAT_CHUNK.wBitsPerSample;
  AvgBytesPerSec = wav_info->FORMAT_CHUNK.dwAvgBytesPerSec;
  FormatTag = wav_info->FORMAT_CHUNK.wFormatTag;
  Channels = wav_info->FORMAT_CHUNK.wChannels;
  input_pcm_size = wav_info->DATA_CHUNK.chunkSize;
  int32_t Eqmid_index = 0;
  FILE *testfile_already_exist = NULL;
  char testfilepath[128] = {0};
  for (int i = 0; i < 3; i += 1) {
    strcat(testfilepath, "../");
    strcat(testfilepath, filename);
    strcat(testfilepath, "_enc");
    if (i == 0) {
      strcat(testfilepath, "_HQ");
    } else if (i == 1) {
      strcat(testfilepath, "_SQ");
    } else if (i == 2) {
      strcat(testfilepath, "_MQ");
    }
    strcat(testfilepath, ".bin");
    testfile_already_exist = fopen(testfilepath, "rb");
    memset(testfilepath, 0, sizeof(testfilepath));
    if (testfile_already_exist != 0) {
      fclose(testfile_already_exist);
      testfile_already_exist = NULL;
      if (i == 2) {
        return -1;
      }
    } else {
      Eqmid_index = i;
      break;
    }
  }

  HANDLE_LDAC_BT ldacBT_enc_handle = ldacBT_get_handle();

  int32_t pcmUsed = 0, pcmUsedcount = 0, streamSize, frameNum,
          streamoutputcount = 0;

  int32_t kChannel_index = 0, kFormat_index = 0;
  kChannel_index = (Channels == 2) ? 1 : 2;
  switch (BitsPerSample) {
  case 16:
    kFormat_index = 0;
    break;
  case 24:
    kFormat_index = 1;
    break;
  case 32:
    kFormat_index = 2;
    break;
  default:
    break;
  }
  if (FormatTag == 3) {
    kFormat_index = 3;
  }

  uint8_t *input_pcm = malloc(input_pcm_size);
  memset(input_pcm, 0, input_pcm_size);
  fread(input_pcm, 1, input_pcm_size, fp);
  fclose(fp);
  uint32_t pStream_size = (float)input_pcm_size *
                          ((3 - kEqmidValue[Eqmid_index]) * 3 * 110 * 1000) /
                          (float)(AvgBytesPerSec * 8);
  uint8_t *pStream = malloc(pStream_size);
  memset(pStream, 0, pStream_size);

  ldacBT_init_handle_encode(ldacBT_enc_handle, kLdacBtRequiredMtu,
                            kEqmidValue[Eqmid_index], kChannel[kChannel_index],
                            kFormat[kFormat_index],
                            SamplesPerSec // kSamplingFrequency[1]
  );
  printf("%s\r\n",
         get_error_code_string(ldacBT_get_error_code(ldacBT_enc_handle)));
  int result = 0;
  char outpath[128] = {0};

  strcat(outpath, "../");
  strcat(outpath, filename);
  strcat(outpath, "_enc");
  switch (Eqmid_index) {
  case 0:
    strcat(outpath, "_HQ");
    break;
  case 1:
    strcat(outpath, "_SQ");
    break;
  case 2:
    strcat(outpath, "_MQ");
    break;
  default:
    break;
  }
  strcat(outpath, ".bin");
  strcat(outpath, "\0");
#if  __WIN32
  unsigned __int64 t0 = *(
      unsigned __int64 *)(0x7FFE0000 + 0x8); // like QueryUnbiasedInterruptTime
  __int64 npack = 0;
#endif
  while ((input_pcm_size - pcmUsedcount) >= pcmUsed) {
    result |=
        ldacBT_encode(ldacBT_enc_handle, input_pcm + pcmUsedcount, &pcmUsed,
                      pStream + streamoutputcount, &streamSize, &frameNum);
    pcmUsedcount += pcmUsed;
    streamoutputcount += streamSize;
#if  __WIN32
    npack += 1;
#endif
  }
  // ldacBT_encode(ldacBT_enc_handle, input_pcm + 44 + pcmUsed, &pcmUsed,
  // pStream, &streamSize, &frameNum);

  if (result != 0) {
    printf("error\r\n");
  }
#if  __WIN32
  unsigned __int64 t = *(unsigned __int64 *)(0x7FFE0000 + 0x8) - t0;
  printf("used %lfs,%lfus per pack\n", ((double)t) / 10000000.0,
         ((double)t / (double)npack) / 10.0);
#endif
  free(input_pcm);
  printf("%s\r\n",
         get_error_code_string(ldacBT_get_error_code(ldacBT_enc_handle)));
  printf("pcmUsed:%d\r\n", pcmUsed);
  FILE *LDAC_fp = fopen(outpath, "wb");
  fwrite(pStream, 1, streamoutputcount, LDAC_fp);
  fclose(LDAC_fp);
  free(pStream);
  ldacBT_close_handle(ldacBT_enc_handle);
  ldacBT_free_handle(ldacBT_enc_handle);

  return 0;
}
