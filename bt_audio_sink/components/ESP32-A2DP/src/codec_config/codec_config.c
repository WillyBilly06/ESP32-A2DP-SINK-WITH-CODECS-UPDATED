/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"

#include "codec_config.h"

#include "a2dp_sbc_constants.h"
#include "a2dp_vendor_aptx_constants.h"
#include "a2dp_vendor_aptx_ll_constants.h"
#include "a2dp_vendor_aptx_hd_constants.h"
#include "a2dp_vendor_ldac_constants.h"
#include "a2dp_vendor_opus_constants.h"
#include "a2dp_vendor_lc3plus_constants.h"
#include "a2dp_aac_constants.h"


bool get_codec_config(esp_a2d_cb_param_t *a2d, uint32_t* sr, uint8_t* bps,
                      uint8_t* ch)
{
    ESP_LOGI(CODEC_CONFIG_TAG, "a2dp audio_cfg_cb , codec type %d",
			 a2d->audio_cfg.mcc.type);

    uint32_t sample_rate = 0;
    uint8_t bits_per_sample = 0;
    uint8_t channels = 2;

    if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
        ESP_LOGI(CODEC_CONFIG_TAG, "%s: configure SBC codec", __func__);
        ESP_LOGI(CODEC_CONFIG_TAG, "%s: configure audio player %x-%x-%x-%x",
                __func__,
                a2d->audio_cfg.mcc.cie.sbc[0],      // sample rate | channel mode
                a2d->audio_cfg.mcc.cie.sbc[1],      // block len | sub bands | alloc method
                a2d->audio_cfg.mcc.cie.sbc[2],      // max bit pool
                a2d->audio_cfg.mcc.cie.sbc[3]);      // min bit pool
        bits_per_sample = 16;

        // determine sample rate
        uint8_t oct = a2d->audio_cfg.mcc.cie.sbc[0];
        if (oct & A2D_SBC_IE_SAMP_FREQ_16) {
            sample_rate = 16000;
        } else if (oct & A2D_SBC_IE_SAMP_FREQ_32) {
            sample_rate = 32000;
        } else if (oct & A2D_SBC_IE_SAMP_FREQ_44) {
            sample_rate = 44100;
        } else if (oct & A2D_SBC_IE_SAMP_FREQ_48) {
            sample_rate = 48000;
        } else {
            ESP_LOGE(CODEC_CONFIG_TAG, "Invalid SBC sample rate config");
        }
        if ((oct & A2D_SBC_IE_CH_MD_MSK) == A2D_SBC_IE_CH_MD_MONO) {
            channels = 1;
        } else {
            channels = 2;
        }
    } else if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_M24) {
#if defined(CONFIG_BT_A2DP_AAC_DECODER)
        ESP_LOGI(CODEC_CONFIG_TAG, "%s: configure AAC codec", __func__);
        ESP_LOGI(CODEC_CONFIG_TAG, "%s: configure audio player %x-%x-%x-%x-%x-%x-%x-%x",
                __func__,
                a2d->audio_cfg.mcc.cie.m24[0],      // objectType
                a2d->audio_cfg.mcc.cie.m24[1],      // sampleRate 
                a2d->audio_cfg.mcc.cie.m24[2],      //
                a2d->audio_cfg.mcc.cie.m24[3],      // channelMode
                a2d->audio_cfg.mcc.cie.m24[4],      // variableBitRateSupport
                a2d->audio_cfg.mcc.cie.m24[5],      // bitRate
                a2d->audio_cfg.mcc.cie.m24[6],
                a2d->audio_cfg.mcc.cie.m24[7]);

        bits_per_sample = 16;

        uint8_t* m24 = a2d->audio_cfg.mcc.cie.m24;
        uint8_t obj_type = m24[0];
        if (obj_type & A2DP_AAC_OBJECT_TYPE_MPEG2_LC) {
            ESP_LOGI(CODEC_CONFIG_TAG, "%s: AAC object type: MPEG-2 Low Complexity", __func__);
        } else if (obj_type & A2DP_AAC_OBJECT_TYPE_MPEG4_LC) {
            ESP_LOGI(CODEC_CONFIG_TAG, "%s: AAC object type: MPEG-4 Low Complexity", __func__);
        } else if (obj_type & A2DP_AAC_OBJECT_TYPE_MPEG4_LTP) {
            ESP_LOGI(CODEC_CONFIG_TAG, "%s: AAC object type: MPEG-4 Long Term Prediction", __func__);
        } else if (obj_type & A2DP_AAC_OBJECT_TYPE_MPEG4_SCALABLE) {
            ESP_LOGI(CODEC_CONFIG_TAG, "%s: AAC object type: MPEG-4 Scalable", __func__);
        }

        uint16_t sr = ((uint16_t)m24[1] & A2DP_AAC_SAMPLING_FREQ_MASK0) |
                        (((uint16_t)m24[2] << 8) & A2DP_AAC_SAMPLING_FREQ_MASK1);
        if (sr == A2DP_AAC_SAMPLING_FREQ_48000) {
            sample_rate = 48000;
        } else if (sr == A2DP_AAC_SAMPLING_FREQ_44100) {
            sample_rate = 44100;
        } else {
            ESP_LOGE(CODEC_CONFIG_TAG, "Invalid AAC sample rate config");
        }
        ESP_LOGI(CODEC_CONFIG_TAG, "%s: AAC sample rate = %lu", __func__, sample_rate);

        uint8_t ch_mode = m24[3] & A2DP_AAC_CHANNEL_MODE_MASK;
        if (ch_mode & A2DP_AAC_CHANNEL_MODE_MONO) {
            channels = 1;
        } else if (ch_mode & A2DP_AAC_CHANNEL_MODE_STEREO) {
            channels = 2;
        }
        ESP_LOGI(CODEC_CONFIG_TAG, "%s: AAC channels = %u", __func__, channels);

        uint32_t bitRate;
        bitRate = (((uint32_t)m24[3] << 16) & A2DP_AAC_BIT_RATE_MASK0) |
                  (((uint32_t)m24[4] << 8) & A2DP_AAC_BIT_RATE_MASK1) |
                  ((uint32_t)m24[5] & A2DP_AAC_BIT_RATE_MASK2);
        ESP_LOGI(CODEC_CONFIG_TAG, "%s: AAC bitrate = %lu", __func__, bitRate);

        uint8_t vbr = !!((m24[4] & A2DP_AAC_VARIABLE_BIT_RATE_MASK) &
                         A2DP_AAC_VARIABLE_BIT_RATE_ENABLED);
        ESP_LOGI(CODEC_CONFIG_TAG, "%s: AAC vbr support = %u", __func__, vbr);
#else
        ESP_LOGE(CODEC_CONFIG_TAG, "%s: AAC sink unsupported", __func__);
#endif /* CONFIG_BT_A2DP_AAC_DECODER */

    } else if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_NON_A2DP) {
        uint8_t *cie = (uint8_t*)&a2d->audio_cfg.mcc.cie;
        ESP_LOGI(CODEC_CONFIG_TAG, "%s: Configure audio player %x-%x-%x-%x-%x-%x",
            __func__,
            cie[0],     // vendor id
            cie[1],
            cie[2],
            cie[3],
            cie[4],     // codec id
            cie[5]
        );

        uint32_t vendor_id = *((uint32_t*) cie);
        uint16_t codec_id = *((uint16_t*) (cie + sizeof(uint32_t)));

        if ((vendor_id == A2DP_APTX_VENDOR_ID &&
             codec_id == A2DP_APTX_CODEC_ID_BLUETOOTH) ||
            (vendor_id == A2DP_APTX_LL_VENDOR_ID &&
              codec_id == A2DP_APTX_LL_CODEC_ID_BLUETOOTH))
            {
#if defined(CONFIG_BT_A2DP_APTX_DECODER)
            if (vendor_id == A2DP_APTX_VENDOR_ID &&
                codec_id == A2DP_APTX_CODEC_ID_BLUETOOTH)
            {
                ESP_LOGI(CODEC_CONFIG_TAG, "%s: configure aptX codec", __func__);
                bits_per_sample = 24;
            } else if (vendor_id == A2DP_APTX_LL_VENDOR_ID &&
                     codec_id == A2DP_APTX_LL_CODEC_ID_BLUETOOTH)
            {
                ESP_LOGI(CODEC_CONFIG_TAG, "%s: configure aptX-LL codec", __func__);
                bits_per_sample = 24;
            }

            uint8_t oct0 = a2d->audio_cfg.mcc.cie.aptx[6]; // sample rate | channel Mode
            if (oct0 & A2DP_APTX_SAMPLERATE_44100) {
                sample_rate = 44100;
            } else if (oct0 & A2DP_APTX_SAMPLERATE_48000) {
                sample_rate = 48000;
            } else {
                ESP_LOGE(CODEC_CONFIG_TAG, "%s: Invalid aptx sample rate config", __func__);
            }

            if (oct0 & A2DP_APTX_CHANNELS_STEREO) {
                channels = 2;
            } else if (oct0 & A2DP_APTX_CHANNELS_MONO) {
                channels = 1;
            } else {
                ESP_LOGE(CODEC_CONFIG_TAG, "%s: Invalid aptx channel config", __func__);
            }
#else
            ESP_LOGE(CODEC_CONFIG_TAG, "%s: aptX sink unsupported", __func__);
#endif /* CONFIG_BT_A2DP_APTX_DECODER */
        } else if (vendor_id == A2DP_APTX_HD_VENDOR_ID &&
                   codec_id == A2DP_APTX_HD_CODEC_ID_BLUETOOTH)
        {
#if defined(CONFIG_BT_A2DP_APTX_DECODER)
            ESP_LOGI(CODEC_CONFIG_TAG, "%s: configure aptX-HD codec", __func__);
            bits_per_sample = 24;

            sample_rate = 0;
            uint8_t oct0 = a2d->audio_cfg.mcc.cie.aptx_hd[6]; // sample rate | channel Mode
            if (oct0 & A2DP_APTX_SAMPLERATE_44100) {
                sample_rate = 44100;
            } else if (oct0 & A2DP_APTX_SAMPLERATE_48000) {
                sample_rate = 48000;
            } else {
                ESP_LOGE(CODEC_CONFIG_TAG, "%s: Invalid aptx-hd sample rate config", __func__);
            }

            if (oct0 & A2DP_APTX_CHANNELS_STEREO) {
                channels = 2;
            } else if (oct0 & A2DP_APTX_CHANNELS_MONO) {
                channels = 1;
            } else {
                ESP_LOGE(CODEC_CONFIG_TAG, "%s: Invalid aptx channel config", __func__);
            }
#else
            ESP_LOGE(CODEC_CONFIG_TAG, "%s: aptX-HD sink unsupported", __func__);
#endif
        } else if (vendor_id == A2DP_LDAC_VENDOR_ID &&
                   codec_id == A2DP_LDAC_CODEC_ID)
        {
#if defined(CONFIG_BT_A2DP_LDAC_DECODER)
            ESP_LOGI(CODEC_CONFIG_TAG, "%s: configure LDAC codec", __func__);
            bits_per_sample = 32;

            sample_rate = 0;
            uint8_t oct0 = a2d->audio_cfg.mcc.cie.ldac[6]; // sample rate
            if (oct0 & A2DP_LDAC_SAMPLING_FREQ_44100) {
                sample_rate = 44100;
            } else if (oct0 & A2DP_LDAC_SAMPLING_FREQ_48000) {
                sample_rate = 48000;
            } else if (oct0 & A2DP_LDAC_SAMPLING_FREQ_88200) {
                sample_rate = 88200;
            } else if (oct0 & A2DP_LDAC_SAMPLING_FREQ_96000) {
                sample_rate = 96000;
            } else {
                ESP_LOGE(CODEC_CONFIG_TAG, "%s: Invalid ldac sample rate config", __func__);
            }

            uint8_t oct7 = a2d->audio_cfg.mcc.cie.ldac[7]; // channel mode
            oct7 &= A2DP_LDAC_CHANNEL_MODE_MASK;
            if (oct7 == A2DP_LDAC_CHANNEL_MODE_DUAL ||
                oct7 == A2DP_LDAC_CHANNEL_MODE_STEREO)
            {
                channels = 2;
            } else if (oct7 == A2DP_LDAC_CHANNEL_MODE_MONO) {
                channels = 1;
            } else {
                ESP_LOGE(CODEC_CONFIG_TAG, "%s: Invalid ldac channel config", __func__);
            }
#else
            ESP_LOGE(CODEC_CONFIG_TAG, "LDAC sink unsupported");
#endif /* CONFIG_BT_A2DP_LDAC_DECODER */
        } else if (vendor_id == A2DP_OPUS_VENDOR_ID &&
                   codec_id == A2DP_OPUS_CODEC_ID)
        {
#if defined(CONFIG_BT_A2DP_OPUS_DECODER)
            ESP_LOGI(CODEC_CONFIG_TAG, "%s: configure Opus codec", __func__);
            bits_per_sample = 16;
            sample_rate = 48000;
#else
            ESP_LOGE(CODEC_CONFIG_TAG, "Opus sink unsupported");
#endif /* CONFIG_BT_A2DP_OPUS_DECODER */
        } else if (vendor_id == A2DP_LC3PLUS_VENDOR_ID &&
                   codec_id == A2DP_LC3PLUS_CODEC_ID)
        {
#if defined(CONFIG_BT_A2DP_LC3PLUS_DECODER)
            ESP_LOGI(CODEC_CONFIG_TAG, "%s: configure LC3 Plus codec", __func__);

            bits_per_sample = 24;

            // uint8_t dur = a2d->audio_cfg.mcc.cie.lc3plus[6];
            uint8_t ch = a2d->audio_cfg.mcc.cie.lc3plus[7];
            uint16_t freq = (uint16_t) ((a2d->audio_cfg.mcc.cie.lc3plus[8] << 8) |
                                        (a2d->audio_cfg.mcc.cie.lc3plus[9]));

            if (freq & A2DP_LC3PLUS_SAMPLING_FREQ_48000) {
                sample_rate = 48000;
            } else if (freq & A2DP_LC3PLUS_SAMPLING_FREQ_96000) {
                sample_rate = 96000;
            } else {
                ESP_LOGE(CODEC_CONFIG_TAG, "Invalid LC3 Plus sample rate: 0x%x", freq);
            }

            if (ch & A2DP_LC3PLUS_CHANNELS_2) {
                channels = 2;
            } else if (ch & A2DP_LC3PLUS_CHANNELS_1) {
                channels = 1;
            }
#else
            ESP_LOGE(CODEC_CONFIG_TAG, "LC3 Plus sink unsupported");
#endif /* CONFIG_BT_A2DP_LC3PLUS_DECODER */
        } else {
            ESP_LOGE(CODEC_CONFIG_TAG, "%s: Unsupported vendor_id 0x%lx, codec_id 0x%x",
                     __func__, vendor_id, codec_id);
        }
    } else {
        ESP_LOGE(CODEC_CONFIG_TAG, "%s: Unsupported codec type 0x%x",
                 __func__, a2d->audio_cfg.mcc.type);
    }

    if (sample_rate == 0) {
        ESP_LOGE(CODEC_CONFIG_TAG, "%s: sample_rate not set.", __func__);
        return false;
    }

    if (bits_per_sample == 0) {
        ESP_LOGE(CODEC_CONFIG_TAG, "%s: bits_per_sample not set.", __func__);
        return false;
    }

    *sr = sample_rate;
    *bps = bits_per_sample;
    *ch = channels;

    return true;
}
