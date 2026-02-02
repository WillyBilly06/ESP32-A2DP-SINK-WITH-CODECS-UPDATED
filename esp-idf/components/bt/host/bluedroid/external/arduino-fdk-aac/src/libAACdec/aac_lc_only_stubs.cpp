/* -----------------------------------------------------------------------------
 * AAC-LC Only Stub Implementations
 * 
 * When building with FDK_AACDEC_LC_ONLY and excluding the heavy sub-libraries
 * (libSBRdec, libSACdec, libDRCdec, libPCMutils), we need stub implementations
 * for the functions that the core AAC decoder references.
 * 
 * These stubs ensure the decoder compiles and links without those libraries,
 * returning appropriate "not available" or "disabled" results.
 * ----------------------------------------------------------------------------- */

#include "libSYS/FDK_audio.h"
#include "libFDK/FDK_bitstream.h"
#include "libFDK/common_fix.h"

#if defined(FDK_AAC_NO_SBRDEC)

/* SBR Decoder Stubs */
#include "libSBRdec/sbrdecoder.h"

SBR_ERROR sbrDecoder_Open(HANDLE_SBRDECODER *pSelf, HANDLE_FDK_QMF_DOMAIN pQmfDomain) {
  (void)pQmfDomain;
  if (pSelf) *pSelf = NULL;
  return SBRDEC_OK; /* Return OK but handle is NULL - decoder will check handle */
}

SBR_ERROR sbrDecoder_Close(HANDLE_SBRDECODER *pSelf) {
  if (pSelf) *pSelf = NULL;
  return SBRDEC_OK;
}

SBR_ERROR sbrDecoder_FreeMem(HANDLE_SBRDECODER *pSelf) {
  if (pSelf) *pSelf = NULL;
  return SBRDEC_OK;
}

SBR_ERROR sbrDecoder_InitElement(HANDLE_SBRDECODER self, const int sampleRateIn,
                                  const int sampleRateOut, const int samplesPerFrame,
                                  const AUDIO_OBJECT_TYPE coreCodec,
                                  const MP4_ELEMENT_ID elementID, const int elementIndex,
                                  const UCHAR harmonicSBR, const UCHAR stereoConfigIndex,
                                  const UCHAR configMode, UCHAR *configChanged,
                                  const INT downscaleFactor) {
  (void)self; (void)sampleRateIn; (void)sampleRateOut; (void)samplesPerFrame;
  (void)coreCodec; (void)elementID; (void)elementIndex; (void)harmonicSBR;
  (void)stereoConfigIndex; (void)configMode; (void)configChanged; (void)downscaleFactor;
  return SBRDEC_NOT_INITIALIZED;
}

SBR_ERROR sbrDecoder_SetParam(HANDLE_SBRDECODER self, const SBRDEC_PARAM param, const INT value) {
  (void)self; (void)param; (void)value;
  return SBRDEC_OK;
}

INT sbrDecoder_Header(HANDLE_SBRDECODER self, HANDLE_FDK_BITSTREAM hBs,
                      const INT sampleRateIn, const INT sampleRateOut,
                      const INT samplesPerFrame, const AUDIO_OBJECT_TYPE coreCodec,
                      const MP4_ELEMENT_ID elementID, const INT elementIndex,
                      const UCHAR harmonicSBR, const UCHAR stereoConfigIndex,
                      const UCHAR configMode, UCHAR *configChanged,
                      const INT downscaleFactor) {
  (void)self; (void)hBs; (void)sampleRateIn; (void)sampleRateOut;
  (void)samplesPerFrame; (void)coreCodec; (void)elementID; (void)elementIndex;
  (void)harmonicSBR; (void)stereoConfigIndex; (void)configMode; (void)configChanged;
  (void)downscaleFactor;
  return -1; /* Signal no SBR header found */
}

SBR_ERROR sbrDecoder_Parse(HANDLE_SBRDECODER self, HANDLE_FDK_BITSTREAM hBs,
                           UCHAR *pDrmBsBuffer, USHORT drmBsBufferSize,
                           int *count, int bsPayLen, int crcFlag,
                           MP4_ELEMENT_ID prev_element, int element_index,
                           UINT acFlags, UINT acElFlags[]) {
  (void)self; (void)hBs; (void)pDrmBsBuffer; (void)drmBsBufferSize;
  (void)count; (void)bsPayLen; (void)crcFlag; (void)prev_element;
  (void)element_index; (void)acFlags; (void)acElFlags;
  return SBRDEC_NOT_INITIALIZED;
}

SBR_ERROR sbrDecoder_Apply(HANDLE_SBRDECODER self, LONG *input, LONG *timeData,
                           const int timeDataSize, int *numChannels, int *sampleRate,
                           const FDK_channelMapDescr *const mapDescr,
                           const int mapIdx, const int coreDecodedOk,
                           UCHAR *psDecoded, const INT inDataHeadroom,
                           INT *outDataHeadroom) {
  (void)self; (void)input; (void)timeData; (void)timeDataSize;
  (void)numChannels; (void)sampleRate; (void)mapDescr; (void)mapIdx;
  (void)coreDecodedOk; (void)psDecoded; (void)inDataHeadroom; (void)outDataHeadroom;
  return SBRDEC_NOT_INITIALIZED;
}

UINT sbrDecoder_GetDelay(const HANDLE_SBRDECODER self) {
  (void)self;
  return 0;
}

INT sbrDecoder_GetLibInfo(LIB_INFO *info) {
  (void)info;
  return 0;
}

SBR_ERROR sbrDecoder_drcFeedChannel(HANDLE_SBRDECODER self, INT ch,
                                    UINT numBands, FIXP_DBL *pNextFact_mag,
                                    INT nextFact_exp,
                                    SHORT drcInterpolationScheme,
                                    UCHAR winSequence, USHORT *pBandTop) {
  (void)self; (void)ch; (void)numBands; (void)pNextFact_mag;
  (void)nextFact_exp; (void)drcInterpolationScheme; (void)winSequence; (void)pBandTop;
  return SBRDEC_OK;
}

void sbrDecoder_drcDisable(HANDLE_SBRDECODER self, INT ch) {
  (void)self; (void)ch;
}

#endif /* FDK_AAC_NO_SBRDEC */


#if defined(FDK_AAC_NO_SACDEC)

/* MPEG Surround (SAC) Decoder Stubs */
#include "libSACdec/sac_dec_lib.h"

SACDEC_ERROR mpegSurroundDecoder_Open(CMpegSurroundDecoder **pMpegSurroundDecoder,
                                      INT stereoConfigIndex,
                                      HANDLE_FDK_QMF_DOMAIN pQmfDomain) {
  if (pMpegSurroundDecoder) *pMpegSurroundDecoder = NULL;
  (void)stereoConfigIndex; (void)pQmfDomain;
  return MPS_OK;
}

void mpegSurroundDecoder_Close(CMpegSurroundDecoder *pMpegSurroundDecoder) {
  (void)pMpegSurroundDecoder;
}

SACDEC_ERROR mpegSurroundDecoder_FreeMem(CMpegSurroundDecoder *pMpegSurroundDecoder) {
  (void)pMpegSurroundDecoder;
  return MPS_OK;
}

SACDEC_ERROR mpegSurroundDecoder_Config(CMpegSurroundDecoder *pMpegSurroundDecoder,
                                        HANDLE_FDK_BITSTREAM hBs,
                                        AUDIO_OBJECT_TYPE coreCodec,
                                        INT samplingRate, INT frameSize,
                                        INT stereoConfigIndex, INT coreSbrFrameLengthIndex,
                                        INT configBytes, const UCHAR configMode,
                                        UCHAR *configChanged) {
  (void)pMpegSurroundDecoder; (void)hBs; (void)coreCodec; (void)samplingRate;
  (void)frameSize; (void)stereoConfigIndex; (void)coreSbrFrameLengthIndex;
  (void)configBytes; (void)configMode; (void)configChanged;
  return MPS_OK;
}

SACDEC_ERROR mpegSurroundDecoder_Apply(CMpegSurroundDecoder *pMpegSurroundDecoder,
                                       PCM_MPS *input, PCM_MPS *output,
                                       const INT inputTimeDataSize,
                                       INT timeDataSize, INT *nChannels,
                                       INT *frameSize, INT *sampleRate,
                                       UINT flags, UINT *sampleRateSBR,
                                       UINT *sampleRateMPS) {
  (void)pMpegSurroundDecoder; (void)input; (void)output; (void)inputTimeDataSize;
  (void)timeDataSize; (void)nChannels; (void)frameSize; (void)sampleRate;
  (void)flags; (void)sampleRateSBR; (void)sampleRateMPS;
  return MPS_NOTOK;
}

SACDEC_ERROR mpegSurroundDecoder_SetParam(CMpegSurroundDecoder *pMpegSurroundDecoder,
                                          const SACDEC_PARAM param, const INT value) {
  (void)pMpegSurroundDecoder; (void)param; (void)value;
  return MPS_OK;
}

void mpegSurroundDecoder_GetLibInfo(LIB_INFO *info) {
  (void)info;
}

INT mpegSurroundDecoder_GetDelay(CMpegSurroundDecoder *pMpegSurroundDecoder) {
  (void)pMpegSurroundDecoder;
  return 0;
}

void mpegSurroundDecoder_ConfigureQmfDomain(CMpegSurroundDecoder *pMpegSurroundDecoder,
                                            SAC_INPUT_CONFIG sac_input,
                                            UINT coreSamplingRate,
                                            AUDIO_OBJECT_TYPE coreCodec) {
  (void)pMpegSurroundDecoder; (void)sac_input; (void)coreSamplingRate; (void)coreCodec;
}

INT mpegSurroundDecoder_IsFullMpegSurroundDecoderInstanceAvailable(
    CMpegSurroundDecoder *pMpegSurroundDecoder) {
  (void)pMpegSurroundDecoder;
  return 0;
}

#endif /* FDK_AAC_NO_SACDEC */


#if defined(FDK_AAC_NO_DRCDEC)

/* DRC Decoder Stubs */
#include "libDRCdec/FDK_drcDecLib.h"

DRC_DEC_ERROR FDK_drcDec_Open(HANDLE_DRC_DECODER *phDrcDec, const DRC_DEC_FUNCTIONAL_RANGE functionalRange) {
  if (phDrcDec) *phDrcDec = NULL;
  (void)functionalRange;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_Close(HANDLE_DRC_DECODER *phDrcDec) {
  if (phDrcDec) *phDrcDec = NULL;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_SetCodecMode(HANDLE_DRC_DECODER hDrcDec, const DRC_DEC_CODEC_MODE codecMode) {
  (void)hDrcDec; (void)codecMode;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_SetParam(HANDLE_DRC_DECODER hDrcDec, const DRC_DEC_USERPARAM requestType, const FIXP_DBL requestValue) {
  (void)hDrcDec; (void)requestType; (void)requestValue;
  return DRC_DEC_OK;
}

LONG FDK_drcDec_GetParam(HANDLE_DRC_DECODER hDrcDec, const DRC_DEC_USERPARAM requestType) {
  (void)hDrcDec; (void)requestType;
  return 0;
}

DRC_DEC_ERROR FDK_drcDec_ReadUniDrcConfig(HANDLE_DRC_DECODER hDrcDec, HANDLE_FDK_BITSTREAM hBs) {
  (void)hDrcDec; (void)hBs;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_ReadLoudnessInfoSet(HANDLE_DRC_DECODER hDrcDec, HANDLE_FDK_BITSTREAM hBs) {
  (void)hDrcDec; (void)hBs;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_ReadLoudnessBox(HANDLE_DRC_DECODER hDrcDec, HANDLE_FDK_BITSTREAM hBs) {
  (void)hDrcDec; (void)hBs;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_ReadDownmixInstructions_Box(HANDLE_DRC_DECODER hDrcDec, HANDLE_FDK_BITSTREAM hBs) {
  (void)hDrcDec; (void)hBs;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_ReadUniDrcInstructions_Box(HANDLE_DRC_DECODER hDrcDec, HANDLE_FDK_BITSTREAM hBs) {
  (void)hDrcDec; (void)hBs;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_ReadUniDrcCoefficients_Box(HANDLE_DRC_DECODER hDrcDec, HANDLE_FDK_BITSTREAM hBs) {
  (void)hDrcDec; (void)hBs;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_Preprocess(HANDLE_DRC_DECODER hDrcDec) {
  (void)hDrcDec;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_ProcessTime(HANDLE_DRC_DECODER hDrcDec, const INT delaySamples,
                                     const DRC_DEC_LOCATION drcLocation,
                                     const INT channelOffset, const INT drcChannelOffset,
                                     const INT numChannelsProcessed,
                                     FIXP_DBL *realBuffer, const INT timeDataChannelOffset) {
  (void)hDrcDec; (void)delaySamples; (void)drcLocation; (void)channelOffset;
  (void)drcChannelOffset; (void)numChannelsProcessed; (void)realBuffer; (void)timeDataChannelOffset;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_ApplyDownmix(HANDLE_DRC_DECODER hDrcDec, INT *reverseInChannelMap,
                                      INT *reverseOutChannelMap, FIXP_DBL *realBuffer,
                                      INT *numChannels) {
  (void)hDrcDec; (void)reverseInChannelMap; (void)reverseOutChannelMap;
  (void)realBuffer; (void)numChannels;
  return DRC_DEC_OK;
}

DRC_DEC_ERROR FDK_drcDec_SetChannelGains(HANDLE_DRC_DECODER hDrcDec, const INT numChannels,
                                         const INT frameSize, FIXP_DBL *channelGainDb,
                                         FIXP_DBL *audioBuffer, const INT audioBufferChannelOffset) {
  (void)hDrcDec; (void)numChannels; (void)frameSize; (void)channelGainDb;
  (void)audioBuffer; (void)audioBufferChannelOffset;
  return DRC_DEC_OK;
}

#endif /* FDK_AAC_NO_DRCDEC */


#if defined(FDK_AAC_NO_PCMUTILS)

/* PCM Utils (Downmix, Limiter) Stubs */
#include "libPCMutils/pcmdmx_lib.h"
#include "libPCMutils/limiter.h"
#include "libPCMutils/pcm_utils.h"

PCMDMX_ERROR pcmDmx_Open(HANDLE_PCM_DOWNMIX *pSelf) {
  if (pSelf) *pSelf = NULL;
  return PCMDMX_OK;
}

PCMDMX_ERROR pcmDmx_Close(HANDLE_PCM_DOWNMIX *pSelf) {
  if (pSelf) *pSelf = NULL;
  return PCMDMX_OK;
}

PCMDMX_ERROR pcmDmx_Reset(HANDLE_PCM_DOWNMIX self, UINT flags) {
  (void)self; (void)flags;
  return PCMDMX_OK;
}

PCMDMX_ERROR pcmDmx_SetParam(HANDLE_PCM_DOWNMIX self, const PCMDMX_PARAM param, const INT value) {
  (void)self; (void)param; (void)value;
  return PCMDMX_OK;
}

PCMDMX_ERROR pcmDmx_GetParam(HANDLE_PCM_DOWNMIX self, const PCMDMX_PARAM param, INT *pValue) {
  (void)self; (void)param;
  if (pValue) *pValue = 0;
  return PCMDMX_OK;
}

PCMDMX_ERROR pcmDmx_ApplyFrame(HANDLE_PCM_DOWNMIX self, DMX_PCM *pPcmBuf, const INT pcmBufSize,
                               const INT frameSize, INT *nChannels, const INT fInterleaved,
                               const AUDIO_CHANNEL_TYPE channelType[],
                               const UCHAR channelIndices[],
                               const FDK_channelMapDescr *const mapDescr,
                               INT *pDmxOutScale) {
  (void)self; (void)pPcmBuf; (void)pcmBufSize; (void)frameSize;
  (void)fInterleaved; (void)channelType; (void)channelIndices;
  (void)mapDescr; (void)pDmxOutScale;
  /* Pass through unchanged for stereo */
  return PCMDMX_OK;
}

void pcmDmx_GetLibInfo(LIB_INFO *info) {
  (void)info;
}

/* PCM Limiter stubs */
TDLimiterPtr pcmLimiter_Create(unsigned int maxAttackMs, unsigned int releaseMs,
                               FIXP_DBL threshold, unsigned int maxChannels,
                               UINT maxSampleRate) {
  (void)maxAttackMs; (void)releaseMs; (void)threshold;
  (void)maxChannels; (void)maxSampleRate;
  return NULL;
}

TDLIMITER_ERROR pcmLimiter_Destroy(TDLimiterPtr limiter) {
  (void)limiter;
  return TDLIMIT_OK;
}

TDLIMITER_ERROR pcmLimiter_Reset(TDLimiterPtr limiter) {
  (void)limiter;
  return TDLIMIT_OK;
}

TDLIMITER_ERROR pcmLimiter_SetAttack(TDLimiterPtr limiter, unsigned int attackMs) {
  (void)limiter; (void)attackMs;
  return TDLIMIT_OK;
}

TDLIMITER_ERROR pcmLimiter_SetRelease(TDLimiterPtr limiter, unsigned int releaseMs) {
  (void)limiter; (void)releaseMs;
  return TDLIMIT_OK;
}

TDLIMITER_ERROR pcmLimiter_SetNChannels(TDLimiterPtr limiter, unsigned int nChannels) {
  (void)limiter; (void)nChannels;
  return TDLIMIT_OK;
}

TDLIMITER_ERROR pcmLimiter_SetSampleRate(TDLimiterPtr limiter, UINT sampleRate) {
  (void)limiter; (void)sampleRate;
  return TDLIMIT_OK;
}

TDLIMITER_ERROR pcmLimiter_Apply(TDLimiterPtr limiter, PCM_LIM *samplesIn, INT_PCM *samplesOut,
                                 FIXP_DBL *pGainPerSample, const INT scaling,
                                 const UINT nSamples) {
  (void)limiter; (void)pGainPerSample; (void)scaling;
  /* Simple pass-through (no limiting) */
  if (samplesIn && samplesOut && nSamples > 0) {
    for (UINT i = 0; i < nSamples; i++) {
      samplesOut[i] = (INT_PCM)(samplesIn[i] >> (DFRACT_BITS - PCM_OUT_BITS - 1));
    }
  }
  return TDLIMIT_OK;
}

unsigned int pcmLimiter_GetDelay(TDLimiterPtr limiter) {
  (void)limiter;
  return 0;
}

void pcmLimiter_GetLibInfo(LIB_INFO *info) {
  (void)info;
}

#endif /* FDK_AAC_NO_PCMUTILS */
