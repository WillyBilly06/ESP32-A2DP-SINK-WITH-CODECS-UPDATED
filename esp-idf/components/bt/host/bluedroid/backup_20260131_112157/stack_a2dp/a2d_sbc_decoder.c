/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arpa/inet.h>
#include "common/bt_trace.h"
#include "esp_log.h"
#include "stack/a2d_sbc.h"
#include "stack/a2d_sbc_decoder.h"
#include "stack/bt_types.h"
#include "oi_codec_sbc.h"
#include "oi_status.h"
#include "osi/future.h"

#if (defined(SBC_DEC_INCLUDED) && SBC_DEC_INCLUDED == TRUE)

typedef struct {
  OI_CODEC_SBC_DECODER_CONTEXT decoder_context;
  OI_UINT32 context_data[CODEC_DATA_WORDS(2, SBC_CODEC_FAST_FILTER_BUFFERS)];
  OI_UINT8 maxChannels;
  OI_UINT8 pcmStride;
  decoded_data_callback_t decode_callback;
} tA2DP_SBC_DECODER_CB;

static tA2DP_SBC_DECODER_CB a2dp_sbc_decoder_cb;

bool a2dp_sbc_decoder_init(decoded_data_callback_t decode_callback) {
  a2dp_sbc_decoder_cb.maxChannels = 2;
  a2dp_sbc_decoder_cb.pcmStride = 2;
  a2dp_sbc_decoder_reset();
  a2dp_sbc_decoder_cb.decode_callback = decode_callback;
  return true;
}

void a2dp_sbc_decoder_cleanup(void) {
  // Do nothing.
}

bool a2dp_sbc_decoder_reset() {
  OI_STATUS status = OI_CODEC_SBC_DecoderReset(
      &a2dp_sbc_decoder_cb.decoder_context, a2dp_sbc_decoder_cb.context_data,
      sizeof(a2dp_sbc_decoder_cb.context_data),
      a2dp_sbc_decoder_cb.maxChannels, a2dp_sbc_decoder_cb.pcmStride,
      false, false);
  if (!OI_SUCCESS(status)) {
    LOG_ERROR("%s: OI_CODEC_SBC_DecoderReset failed with error code %d",
              __func__, status);
    return false;
  }
  return true;
}

void a2dp_sbc_decoder_configure(const uint8_t* p_codec_info) {
  tA2D_SBC_CIE cie;
  tA2D_STATUS status;
  status = A2D_ParsSbcInfo(&cie, (uint8_t*)p_codec_info, false);
  if (status != A2D_SUCCESS) {
    LOG_ERROR("%s: failed parsing codec info. %d", __func__, status);
    return;
  }

  uint32_t sr = 0;
  if (cie.samp_freq == A2D_SBC_IE_SAMP_FREQ_16) {
    sr = 16000;
  } else if (cie.samp_freq == A2D_SBC_IE_SAMP_FREQ_32) {
    sr = 32000;
  } else if (cie.samp_freq == A2D_SBC_IE_SAMP_FREQ_44) {
    sr = 44100;
  } else if (cie.samp_freq == A2D_SBC_IE_SAMP_FREQ_48) {
    sr = 48000;
  }
  LOG_INFO("%s: SBC Sampling frequency = %lu", __func__, sr);

  if (cie.ch_mode == A2D_SBC_IE_CH_MD_MONO) {
    LOG_INFO("%s: SBC Channel mode: Mono", __func__);
  } else if (cie.ch_mode == A2D_SBC_IE_CH_MD_DUAL) {
    LOG_INFO("%s: SBC Channel mode: Dual", __func__);
  } else if (cie.ch_mode == A2D_SBC_IE_CH_MD_STEREO) {
    LOG_INFO("%s: SBC Channel mode: Stereo", __func__);
  } else if (cie.ch_mode == A2D_SBC_IE_CH_MD_JOINT) {
    LOG_INFO("%s: SBC Channel mode: Joint", __func__);
  }

  uint8_t block_len = 0;
  if (cie.block_len == A2D_SBC_IE_BLOCKS_4) {
    block_len = 4;
  } else if (cie.block_len == A2D_SBC_IE_BLOCKS_8) {
    block_len = 8;
  } else if (cie.block_len == A2D_SBC_IE_BLOCKS_12) {
    block_len = 12;
  } else if (cie.block_len == A2D_SBC_IE_BLOCKS_16) {
    block_len = 16;
  }
  LOG_INFO("%s: SBC Block length = %lu", __func__, block_len);

  uint8_t subbands = 0;
  if (cie.num_subbands == A2D_SBC_IE_SUBBAND_4) {
    subbands = 4;
  } else if (cie.num_subbands == A2D_SBC_IE_SUBBAND_8) {
    subbands = 8;
  }
  LOG_INFO("%s: SBC Number of subbands = %lu", __func__, subbands);

  if (cie.alloc_mthd == A2D_SBC_IE_ALLOC_MD_S) {
    LOG_INFO("%s: SBC Allocation method: SNR", __func__);
  } else if (cie.alloc_mthd == A2D_SBC_IE_ALLOC_MD_L) {
    LOG_INFO("%s: SBC Allocation method: Loudness", __func__);
  }

  LOG_INFO("%s: SBC Maximum bitpool = %lu", __func__, cie.max_bitpool);
  LOG_INFO("%s: SBC Minimum bitpool = %lu", __func__, cie.min_bitpool);

  if (cie.ch_mode == A2D_SBC_IE_CH_MD_MONO) {
    a2dp_sbc_decoder_cb.maxChannels = 1;
  } else {
    a2dp_sbc_decoder_cb.maxChannels = 2;
  }

  a2dp_sbc_decoder_reset();
}

ssize_t a2dp_sbc_decoder_decode_packet_header(BT_HDR* p_buf) {
  UINT8 *data;
  struct media_packet_header *header;
  size_t csrc_len, header_len;

  data  = (UINT8 *)(p_buf + 1) + p_buf->offset;
  header = (struct media_packet_header*)(data);

  /* calculate total header size */
  csrc_len = (header->cc * sizeof(uint32_t));
  header_len = sizeof(struct media_packet_header) + csrc_len;

  /* check for and skip over extension header */
  if (header->x) {
    uint16_t *p_ex_len = (uint16_t*)(header + header_len + 2);
    uint16_t ex_len = ntohs(*p_ex_len) * 4;
    header_len += ex_len;
  }

  /* adjust length for any padding at end of packet */
  if (header->p) {
    /* padding length in last byte of packet */
    UINT8 pad_len = 0;
    pad_len =  *((UINT8*)header + p_buf->len);
    p_buf->len -= pad_len;
  }

  p_buf->offset += header_len;
  p_buf->len -= header_len;

  /* report sequence number */
  p_buf->layer_specific = ntohs(header->seq);

  return 0;
}

bool a2dp_sbc_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf, size_t buf_len) {
    UINT8 *data;
    struct sbc_header *header;
    UINT8 *src;
    UINT32 src_len;
    size_t num_frames;
    OI_INT16 *dst;
    INT32 avail;
    UINT32 written;
    OI_STATUS status;
    int count;

    data  = (UINT8 *)(p_buf + 1) + p_buf->offset;
    header = (struct sbc_header*)data;
    num_frames = header->num_frames;

    src = data + sizeof(struct sbc_header);
    src_len = p_buf->len - sizeof(struct sbc_header);

    dst = (OI_INT16*)buf;
    avail = buf_len;

    APPL_TRACE_DEBUG("Number of sbc frames %d, frame_len %d\n", num_frames, src_len);

    for (count = 0; count < num_frames && src_len != 0 && avail > 0; count ++) {
        written = avail;
        status = OI_CODEC_SBC_DecodeFrame(&a2dp_sbc_decoder_cb.decoder_context,
                                          (const OI_BYTE **)&src,
                                          (OI_UINT32 *)&src_len,
                                          (OI_INT16 *)dst,
                                          (OI_UINT32 *)&written);
        if (!OI_SUCCESS(status)) {
            APPL_TRACE_ERROR("Decoding failure: %d\n", status);
            return false;
        }
        avail -= written;
        dst += written / 2;
    }

    p_buf->offset = p_buf->offset + p_buf->len - src_len;
    p_buf->len = src_len;

    size_t out_used = buf_len - avail;
    out_used = out_used <= buf_len ? out_used : buf_len;
    a2dp_sbc_decoder_cb.decode_callback((uint8_t*)buf, out_used);
    return true;
}

#endif /* #if (defined(SBC_DEC_INCLUDED) && SBC_DEC_INCLUDED == TRUE) */
