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
  uint32_t enc_times = 1536, SamplesPerSec;

  uint16_t FormatTag, BitsPerSample, Channels;
  FILE *out_fp = NULL;
  FILE *rawfile = NULL;
  char filepath[128] = {0};
  char filename[128] = "512kMeasSweep_0_to_24000_-12_dBV_48k_PCM16_LR";
  char rawflie_path[128] = {0};
  if (argc > 1) {
    memset(filename, 0, sizeof(filename));
    strcat(filename, argv[1]);
  }
  strcat(rawflie_path, "../");
  strcat(rawflie_path, filename);
  strcat(rawflie_path, ".wav");
  strcat(rawflie_path, "\0");
  rawfile = fopen(rawflie_path, "rb");
  WAV_INF *rawfileinfo = malloc(sizeof(WAV_INF));
  if (rawfileinfo == NULL) {
    printf("wav文件不存在\r\n");
    return -1;
  }
  fread(rawfileinfo, 1, sizeof(WAV_INF), rawfile);
  fclose(rawfile);

  SamplesPerSec = rawfileinfo->FORMAT_CHUNK.dwSamplesPerSec;
  BitsPerSample = rawfileinfo->FORMAT_CHUNK.wBitsPerSample;
  Channels = rawfileinfo->FORMAT_CHUNK.wChannels;
  FormatTag = rawfileinfo->FORMAT_CHUNK.wFormatTag;

  char testfilepath[128] = {0};
  uint32_t Eqmid_index = -1;
  int32_t streamSize;
  for (int i = 0; i < 3; i += 1) {
    memset(testfilepath, 0, sizeof(testfilepath));
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
    fp = fopen(testfilepath, "rb");
    if (fp != NULL) {
      testfilepath[strlen(testfilepath) - 11 + 1] = 'd'; //_enc_HQ.bin
      testfilepath[strlen(testfilepath) - 11 + 2] = 'e';
      testfilepath[strlen(testfilepath) - 3 + 0] = 'w';
      testfilepath[strlen(testfilepath) - 3 + 1] = 'a';
      testfilepath[strlen(testfilepath) - 3 + 2] = 'v';
      FILE *fp_dec = fopen(testfilepath, "rb");
      // 获取文件大小
      if (fp_dec) {
        fclose(fp);
        fclose(fp_dec);
        if (i == 2) {
          printf("已完成所有解码\n");
          return -1;
        } else {
          continue;
        }
      }
      Eqmid_index = i;
      fseek(fp, 0, SEEK_END);
      streamSize = ftell(fp);
      fseek(fp, 0, SEEK_SET);
      printf("File size: %ld bytes\n", streamSize);
      break;
    }
  }
  if (Eqmid_index == -1) {
    return -1;
  }

  uint8_t *pStream = malloc(streamSize);
  memset(pStream, 0, streamSize);
  fread(pStream, 1, streamSize, fp);
  fclose(fp);
  HANDLE_LDAC_BT ldacBT_dec_handle = ldacBT_get_handle();
  ldacBT_init_handle_decode(ldacBT_dec_handle,
                            kChannel[(Channels == 2) ? 1 : 2],
                            SamplesPerSec, // kSamplingFrequency[3],
                            0, 0, 0);

  // rawfileinfo->DATA_CHUNK.chunkSize;

  uint8_t *out_pcm = malloc(rawfileinfo->DATA_CHUNK.chunkSize);
  memset(out_pcm, 0, rawfileinfo->DATA_CHUNK.chunkSize);
  uint32_t kFormat_index;
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
  int32_t streamUsed = 0, out_pcm_szie = 0, out_pcm_count = 0,
          used_Stream_count = 0,
          frame_size = (3 - kEqmidValue[Eqmid_index]) * 110;
  int result = 0;
#if  __WIN32
  unsigned __int64 t0 = *(
      unsigned __int64 *)(0x7FFE0000 + 0x8); // like QueryUnbiasedInterruptTime
  __int64 npack = 0;
#endif


  while ((streamSize - used_Stream_count) > streamUsed) {

    result = ldacBT_decode(ldacBT_dec_handle, pStream + used_Stream_count,
                           out_pcm + out_pcm_count, kFormat[kFormat_index],
                           frame_size, &streamUsed, &out_pcm_szie);
    out_pcm_count += out_pcm_szie;
    used_Stream_count += streamUsed;
#if  __WIN32
    npack += 1;
#endif
  }
  if (result != 0) {
    printf("%s\r\n",get_error_code_string(ldacBT_get_error_code(ldacBT_dec_handle)));
    printf("Error generated in the %dth stage of decoding\r\n", result);
    printf("error\r\n");
  }
#if  __WIN32
  unsigned __int64 t = *(unsigned __int64 *)(0x7FFE0000 + 0x8) - t0;
  printf("used %lfs,%lfus per pack\n", ((double)t) / 10000000.0,
         ((double)t / (double)npack) / 10.0);
#endif


  free(pStream);
  ldacBT_close_handle(ldacBT_dec_handle);
  ldacBT_free_handle(ldacBT_dec_handle);

  // return 0;
  //  out_pcm_count -= 512;//  前512字节空白

  WAVRIFFHEADER wav_head = {
      .groupID = "RIFF", .riffType = "WAVE", .size = out_pcm_count + 36};

  FORMATCHUNK aFORMATCHUNK = {
      .chunkID = "fmt\x20",
      .chunkSize = 16,
      .wFormatTag = FormatTag,
      .wChannels = Channels,
      .dwSamplesPerSec = SamplesPerSec,
      .dwAvgBytesPerSec = SamplesPerSec * (BitsPerSample >> 3) * Channels,
      .wBlockAlign = (BitsPerSample >> 3) * Channels,
      .wBitsPerSample = BitsPerSample};
  // sizeof(aFORMATCHUNK);
  DATACHUNK aDATACHUNK = {.chunkID = "data", .chunkSize = out_pcm_count};

  char outpath[128] = {0};
  strcat(outpath, "../");
  strcat(outpath, filename);
  strcat(outpath, "_dec");
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
  strcat(outpath, ".wav");
  strcat(outpath, "\0");
  out_fp = fopen(outpath, "wb");
  sizeof(aFORMATCHUNK);
  /**/
  fwrite(wav_head.data, 1, sizeof(wav_head), out_fp);
  fwrite(aFORMATCHUNK.data, 1, sizeof(aFORMATCHUNK), out_fp);
  fwrite(aDATACHUNK.data, 1, sizeof(aDATACHUNK), out_fp);

  // fwrite(rawfileinfo, 1, sizeof(WAV_INF), out_fp);
  fwrite(out_pcm, 1, out_pcm_count, out_fp);
  fclose(out_fp);

  free(out_pcm);
  free(rawfileinfo);

  return 0;
}