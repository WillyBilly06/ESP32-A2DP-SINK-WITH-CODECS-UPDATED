/******************************************************************************
 *
 *  Copyright (C) 2004-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  This is the advanced audio/video call-out function implementation for
 *  BTC.
 *
 ******************************************************************************/
#include "string.h"
#include "common/bt_target.h"
#include "stack/a2d_api.h"
#include "stack/a2d_sbc.h"
#include "stack/a2dp_codec_api.h"
#include "bta/bta_sys.h"
#include "bta/bta_av_api.h"
#include "bta/bta_av_co.h"
#include "bta/bta_av_ci.h"
#include "bta/bta_av_sbc.h"
#include "btc_a2dp.h"
#include "btc_a2dp_source.h"
#include "btc_av_co.h"
#include "btc/btc_util.h"
#include "osi/mutex.h"

#if BTC_AV_INCLUDED

/*****************************************************************************
 **  Constants
 *****************************************************************************/

#define FUNC_TRACE()     APPL_TRACE_DEBUG("%s", __FUNCTION__);

/* Macro to retrieve the number of elements in a statically allocated array */
#define BTA_AV_CO_NUM_ELEMENTS(__a) (sizeof(__a)/sizeof((__a)[0]))

/* MIN and MAX macros */
#define BTA_AV_CO_MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define BTA_AV_CO_MAX(X,Y) ((X) > (Y) ? (X) : (Y))

/* Macro to convert audio handle to index and vice versa */
#define BTA_AV_CO_AUDIO_HNDL_TO_INDX(hndl) (((hndl) & (~BTA_AV_CHNL_MSK)) - 1)
#define BTA_AV_CO_AUDIO_INDX_TO_HNDL(indx) (((indx) + 1) | BTA_AV_CHNL_AUDIO)


/* Offsets to access codec information in SBC codec */
#define BTA_AV_CO_SBC_FREQ_CHAN_OFF    3
#define BTA_AV_CO_SBC_BLOCK_BAND_OFF   4
#define BTA_AV_CO_SBC_MIN_BITPOOL_OFF  5
#define BTA_AV_CO_SBC_MAX_BITPOOL_OFF  6

#define BTA_AV_CO_SBC_MAX_BITPOOL  53

/* SCMS-T protect info */
const UINT8 bta_av_co_cp_scmst[BTA_AV_CP_INFO_LEN] = "\x02\x02\x00";

#if !defined(BTC_AV_SBC_DEFAULT_SAMP_FREQ)
#define BTC_AV_SBC_DEFAULT_SAMP_FREQ A2D_SBC_IE_SAMP_FREQ_44
#endif

/* Default SBC codec configuration */
const tA2D_SBC_CIE btc_av_sbc_default_config = {
    BTC_AV_SBC_DEFAULT_SAMP_FREQ,   /* samp_freq */
    A2D_SBC_IE_CH_MD_JOINT,         /* ch_mode */
    A2D_SBC_IE_BLOCKS_16,           /* block_len */
    A2D_SBC_IE_SUBBAND_8,           /* num_subbands */
    A2D_SBC_IE_ALLOC_MD_L,          /* alloc_mthd */
    BTA_AV_CO_SBC_MAX_BITPOOL,      /* max_bitpool */
    A2D_SBC_IE_MIN_BITPOOL          /* min_bitpool */
};

/* Control block instance */
#if AVRC_DYNAMIC_MEMORY == FALSE
tBTA_AV_CO_CB bta_av_co_cb;
#else
tBTA_AV_CO_CB *bta_av_co_cb_ptr;
#endif

static BOOLEAN bta_av_co_audio_codec_build_config(const UINT8 *p_codec_caps, UINT8 *p_codec_cfg);
static void bta_av_co_audio_peer_reset_config(tBTA_AV_CO_PEER *p_peer);
static BOOLEAN bta_av_co_cp_is_scmst(const UINT8 *p_protectinfo);
static BOOLEAN bta_av_co_audio_sink_has_scmst(const tBTA_AV_CO_SINK *p_sink);
static tBTA_AV_CO_SINK* bta_av_co_audio_find_peer_sink(tBTA_AV_CO_PEER *p_peer);
static tBTA_AV_CO_SINK* bta_av_co_audio_find_peer_src(tBTA_AV_CO_PEER *p_peer);



/*******************************************************************************
 **
 ** Function         bta_av_co_cp_is_active
 **
 ** Description      Get the current configuration of content protection
 **
 ** Returns          TRUE if the current streaming has CP, FALSE otherwise
 **
 *******************************************************************************/
BOOLEAN bta_av_co_cp_is_active(void)
{
    FUNC_TRACE();
    return bta_av_co_cb.cp.active;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_cp_get_flag
 **
 ** Description      Get content protection flag
 **                  BTA_AV_CP_SCMS_COPY_NEVER
 **                  BTA_AV_CP_SCMS_COPY_ONCE
 **                  BTA_AV_CP_SCMS_COPY_FREE
 **
 ** Returns          The current flag value
 **
 *******************************************************************************/
UINT8 bta_av_co_cp_get_flag(void)
{
    FUNC_TRACE();
    return bta_av_co_cb.cp.flag;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_cp_set_flag
 **
 ** Description      Set content protection flag
 **                  BTA_AV_CP_SCMS_COPY_NEVER
 **                  BTA_AV_CP_SCMS_COPY_ONCE
 **                  BTA_AV_CP_SCMS_COPY_FREE
 **
 ** Returns          TRUE if setting the SCMS flag is supported else FALSE
 **
 *******************************************************************************/
BOOLEAN bta_av_co_cp_set_flag(UINT8 cp_flag)
{
    FUNC_TRACE();

#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
#else
    if (cp_flag != BTA_AV_CP_SCMS_COPY_FREE) {
        return FALSE;
    }
#endif
    bta_av_co_cb.cp.flag = cp_flag;
    return TRUE;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_get_peer
 **
 ** Description      find the peer entry for a given handle
 **
 ** Returns          the control block
 **
 *******************************************************************************/
static tBTA_AV_CO_PEER *bta_av_co_get_peer(tBTA_AV_HNDL hndl)
{
    UINT8 index;
    FUNC_TRACE();

    index = BTA_AV_CO_AUDIO_HNDL_TO_INDX(hndl);

    /* Sanity check */
    if (index >= BTA_AV_CO_NUM_ELEMENTS(bta_av_co_cb.peers)) {
        APPL_TRACE_ERROR("bta_av_co_get_peer peer index out of bounds:%d", index);
        return NULL;
    }

    return &bta_av_co_cb.peers[index];
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_init
 **
 ** Description      This callout function is executed by AV when it is
 **                  started by calling BTA_AvRegister().  This function can be
 **                  used by the phone to initialize audio paths or for other
 **                  initialization purposes.
 **
 **
 ** Returns          Stream codec and content protection capabilities info.
 **
 *******************************************************************************/
BOOLEAN bta_av_co_audio_init(btav_a2dp_codec_index_t codec_index,
                             UINT8 *p_codec_info, UINT8 *p_num_protect,
                             UINT8 *p_protect_info, UINT8 tsep)
{
    FUNC_TRACE();

    APPL_TRACE_DEBUG("bta_av_co_audio_init: %d", tsep);

    /* By default - no content protection info */
    *p_num_protect = 0;
    *p_protect_info = 0;

    /* reset remote preference through setconfig */
    memset(bta_av_co_cb.codec_cfg_setconfig.info, 0, AVDT_CODEC_SIZE);

    if (tsep == AVDT_TSEP_SRC) {
#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
        do {
            UINT8 *p = p_protect_info;

            /* Content protection info - support SCMS-T */
            *p_num_protect = 1;
            *p++ = BTA_AV_CP_LOSC;
            UINT16_TO_STREAM(p, BTA_AV_CP_SCMS_T_ID);
        } while (0);
#endif
        return A2DP_InitCodecConfig(codec_index, p_codec_info);
    } else if (tsep == AVDT_TSEP_SNK) {
        return A2DP_InitCodecConfig(codec_index, p_codec_info);
    } else {
        APPL_TRACE_WARNING("invalid SEP type %d", tsep);
        return FALSE;
    }
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_disc_res
 **
 ** Description      This callout function is executed by AV to report the
 **                  number of stream end points (SEP) were found during the
 **                  AVDT stream discovery process.
 **
 **
 ** Returns          void.
 **
 *******************************************************************************/
void bta_av_co_audio_disc_res(tBTA_AV_HNDL hndl, UINT8 num_seps, UINT8 num_snk,
                              UINT8 num_src, BD_ADDR addr, UINT16 uuid_local)
{
    tBTA_AV_CO_PEER *p_peer;

    FUNC_TRACE();

    APPL_TRACE_DEBUG("bta_av_co_audio_disc_res h:x%x num_seps:%d num_snk:%d num_src:%d",
                     hndl, num_seps, num_snk, num_src);

    /* Find the peer info */
    p_peer = bta_av_co_get_peer(hndl);
    if (p_peer == NULL) {
        APPL_TRACE_ERROR("bta_av_co_audio_disc_res could not find peer entry");
        return;
    }

    /* Sanity check : this should never happen */
    if (p_peer->opened) {
        APPL_TRACE_ERROR("bta_av_co_audio_disc_res peer already opened");
    }

    /* Copy the discovery results */
    bdcpy(p_peer->addr, addr);
    p_peer->num_snks = num_snk;
    p_peer->num_srcs = num_src;
    p_peer->num_seps = num_seps;
    p_peer->num_rx_snks = 0;
    p_peer->num_rx_srcs = 0;
    p_peer->num_sup_snks = 0;
    if (uuid_local == UUID_SERVCLASS_AUDIO_SINK) {
        p_peer->uuid_to_connect = UUID_SERVCLASS_AUDIO_SOURCE;
    } else if (uuid_local == UUID_SERVCLASS_AUDIO_SOURCE) {
        p_peer->uuid_to_connect = UUID_SERVCLASS_AUDIO_SINK;
    }
    p_peer->got_disc_res = TRUE;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_cfg_res
 **
 ** Description      This callout function is executed by AV to report the
 **                  number of stream end points (SEP) were found during the
 **                  incoming AVDT stream config request process.
 **
 **
 ** Returns          void.
 **
 *******************************************************************************/
void bta_av_co_audio_cfg_res(tBTA_AV_HNDL hndl, UINT8 num_seps, UINT8 num_snk,
                              UINT8 num_src, BD_ADDR addr, UINT16 uuid_local)
{
    tBTA_AV_CO_PEER *p_peer;

    FUNC_TRACE();

    APPL_TRACE_DEBUG("bta_av_co_audio_cfg_res h:x%x num_seps:%d num_snk:%d num_src:%d",
                     hndl, num_seps, num_snk, num_src);

    /* Find the peer info */
    p_peer = bta_av_co_get_peer(hndl);
    if (p_peer == NULL) {
        APPL_TRACE_ERROR("bta_av_co_audio_cfg_res could not find peer entry");
        return;
    }

    /* Sanity check : this should never happen */
    if (p_peer->opened) {
        APPL_TRACE_ERROR("bta_av_co_audio_cfg_res peer already opened");
    }

    /* Copy the discovery results */
    bdcpy(p_peer->addr, addr);
    if (!p_peer->got_disc_res) {
        p_peer->num_snks = num_snk;
        p_peer->num_srcs = num_src;
        p_peer->num_seps = num_seps;
        p_peer->num_rx_snks = 0;
        p_peer->num_rx_srcs = 0;
        p_peer->num_sup_snks = 0;
    }
    if (uuid_local == UUID_SERVCLASS_AUDIO_SINK) {
        p_peer->uuid_to_connect = UUID_SERVCLASS_AUDIO_SOURCE;
    } else if (uuid_local == UUID_SERVCLASS_AUDIO_SOURCE) {
        p_peer->uuid_to_connect = UUID_SERVCLASS_AUDIO_SINK;
    }
}

/*******************************************************************************
 **
 ** Function         bta_av_build_src_cfg
 **
 ** Description      This function will build preferred config from src capabilities
 **
 **
 ** Returns          Pass or Fail for current getconfig.
 **
 *******************************************************************************/
void bta_av_build_src_cfg (UINT8 *p_pref_cfg, UINT8 *p_src_cap)
{
    if (!A2DP_BuildCodecConfig(p_src_cap, p_pref_cfg)) {
        APPL_TRACE_ERROR("%s unsupported codec_index %s", __func__,
                         A2DP_CodecName(p_pref_cfg));
    }
}

/*******************************************************************************
 **
 ** Function         bta_av_audio_sink_getconfig
 **
 ** Description      This callout function is executed by AV to retrieve the
 **                  desired codec and content protection configuration for the
 **                  A2DP Sink audio stream in Initiator.
 **
 **
 ** Returns          Pass or Fail for current getconfig.
 **
 *******************************************************************************/
UINT8 bta_av_audio_sink_getconfig(tBTA_AV_HNDL hndl,
                                  UINT8 *p_codec_info, UINT8 *p_sep_info_idx, UINT8 seid, UINT8 *p_num_protect,
                                  UINT8 *p_protect_info)
{

    UINT8 result = A2D_FAIL;
    tBTA_AV_CO_PEER *p_peer;
    tBTA_AV_CO_SINK *p_src;
    UINT8 pref_cfg[AVDT_CODEC_SIZE];

    FUNC_TRACE();

    APPL_TRACE_DEBUG("bta_av_audio_sink_getconfig handle:0x%x seid:%d",
                     hndl, seid);
    APPL_TRACE_DEBUG("num_protect:0x%02x protect_info:0x%02x%02x%02x",
                     *p_num_protect, p_protect_info[0], p_protect_info[1], p_protect_info[2]);

    /* Retrieve the peer info */
    p_peer = bta_av_co_get_peer(hndl);
    if (p_peer == NULL) {
        APPL_TRACE_ERROR("bta_av_audio_sink_getconfig could not find peer entry");
        return A2D_FAIL;
    }

    APPL_TRACE_DEBUG("bta_av_audio_sink_getconfig peer(o=%d,n_snks=%d,n_rx_snks=%d,n_sup_snks=%d)",
                     p_peer->opened, p_peer->num_srcs, p_peer->num_rx_srcs, p_peer->num_sup_srcs);

    p_peer->num_rx_srcs++;

    /* Check if this is a supported configuration */
    if (A2DP_IsPeerSourceCodecSupported(p_codec_info) == A2D_SUCCESS) {
        /* If there is room for a new one */
        if (p_peer->num_sup_srcs < BTA_AV_CO_NUM_ELEMENTS(p_peer->srcs)) {
            p_src = &p_peer->srcs[p_peer->num_sup_srcs++];

            APPL_TRACE_DEBUG("bta_av_audio_sink_getconfig saved caps[%x:%x:%x:%x:%x:%x]",
                             p_codec_info[1], p_codec_info[2], p_codec_info[3],
                             p_codec_info[4], p_codec_info[5], p_codec_info[6]);

            memcpy(p_src->codec_caps, p_codec_info, AVDT_CODEC_SIZE);
            p_src->sep_info_idx = *p_sep_info_idx;
            p_src->seid = seid;
            p_src->num_protect = *p_num_protect;
            memcpy(p_src->protect_info, p_protect_info, BTA_AV_CP_INFO_LEN);
        } else {
            APPL_TRACE_ERROR("bta_av_audio_sink_getconfig no more room for SRC info");
        }
    }

    /* If last SNK get capabilities or all supported codec caps retrieved */
    if ((p_peer->num_rx_srcs == p_peer->num_srcs) ||
            (p_peer->num_sup_srcs == BTA_AV_CO_NUM_ELEMENTS(p_peer->srcs))) {
        APPL_TRACE_DEBUG("bta_av_audio_sink_getconfig last SRC reached");

        /* Protect access to bta_av_co_cb.codec_cfg */
        osi_mutex_global_lock();

        /* Find a src that matches the codec config */
        p_src = bta_av_co_audio_find_peer_src(p_peer);
        if (p_src != NULL) {
            APPL_TRACE_DEBUG(" Codec Supported ");

            /* Build the codec configuration for this sink */
            {
                /* Save the new configuration */
                p_peer->p_src = p_src;
                /* get preferred config from src_caps */
                bta_av_build_src_cfg(pref_cfg, p_src->codec_caps);
                memcpy(p_peer->codec_cfg, pref_cfg, AVDT_CODEC_SIZE);

                APPL_TRACE_DEBUG("bta_av_audio_sink_getconfig  p_codec_info[%x:%x:%x:%x:%x:%x]",
                                 p_peer->codec_cfg[1], p_peer->codec_cfg[2], p_peer->codec_cfg[3],
                                 p_peer->codec_cfg[4], p_peer->codec_cfg[5], p_peer->codec_cfg[6]);
                /* By default, no content protection */
                *p_num_protect = 0;

#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
                p_peer->cp_active = FALSE;
                bta_av_co_cb.cp.active = FALSE;
#endif

                *p_sep_info_idx = p_src->sep_info_idx;
                memcpy(p_codec_info, p_peer->codec_cfg, AVDT_CODEC_SIZE);
                result =  A2D_SUCCESS;
            }
        }
        /* Protect access to bta_av_co_cb.codec_cfg */
        osi_mutex_global_unlock();
    }
    return result;
}
/*******************************************************************************
 **
 ** Function         bta_av_co_audio_getconfig
 **
 ** Description      This callout function is executed by AV to retrieve the
 **                  desired codec and content protection configuration for the
 **                  audio stream.
 **
 **
 ** Returns          Stream codec and content protection configuration info.
 **
 *******************************************************************************/
UINT8 bta_av_co_audio_getconfig(tBTA_AV_HNDL hndl,
                                UINT8 *p_codec_info, UINT8 *p_sep_info_idx, UINT8 seid, UINT8 *p_num_protect,
                                UINT8 *p_protect_info)

{
    UINT8 result = A2D_FAIL;
    tBTA_AV_CO_PEER *p_peer;
    tBTA_AV_CO_SINK *p_sink;
    UINT8 codec_cfg[AVDT_CODEC_SIZE];

    FUNC_TRACE();

    /* Retrieve the peer info */
    p_peer = bta_av_co_get_peer(hndl);
    if (p_peer == NULL) {
        APPL_TRACE_ERROR("bta_av_co_audio_getconfig could not find peer entry");
        return A2D_FAIL;
    }

    if (p_peer->uuid_to_connect == UUID_SERVCLASS_AUDIO_SOURCE) {
        result = bta_av_audio_sink_getconfig(hndl, p_codec_info, p_sep_info_idx,
                                             seid, p_num_protect, p_protect_info);
        return result;
    }
    APPL_TRACE_DEBUG("bta_av_co_audio_getconfig handle:0x%x seid:%d",
                     hndl, seid);
    APPL_TRACE_DEBUG("num_protect:0x%02x protect_info:0x%02x%02x%02x",
                     *p_num_protect, p_protect_info[0], p_protect_info[1], p_protect_info[2]);

    APPL_TRACE_DEBUG("bta_av_co_audio_getconfig peer(o=%d,n_snks=%d,n_rx_snks=%d,n_sup_snks=%d)",
                     p_peer->opened, p_peer->num_snks, p_peer->num_rx_snks, p_peer->num_sup_snks);

    p_peer->num_rx_snks++;

    /* Check if this is a supported configuration */
    if (A2DP_IsPeerSinkCodecValid(p_codec_info)) {
        /* If there is room for a new one */
        if (p_peer->num_sup_snks < BTA_AV_CO_NUM_ELEMENTS(p_peer->snks)) {
            p_sink = &p_peer->snks[p_peer->num_sup_snks++];

            APPL_TRACE_DEBUG("bta_av_co_audio_getconfig saved caps[%x:%x:%x:%x:%x:%x]",
                             p_codec_info[1], p_codec_info[2], p_codec_info[3],
                             p_codec_info[4], p_codec_info[5], p_codec_info[6]);

            memcpy(p_sink->codec_caps, p_codec_info, AVDT_CODEC_SIZE);
            p_sink->sep_info_idx = *p_sep_info_idx;
            p_sink->seid = seid;
            p_sink->num_protect = *p_num_protect;
            memcpy(p_sink->protect_info, p_protect_info, BTA_AV_CP_INFO_LEN);
        } else {
            APPL_TRACE_ERROR("bta_av_co_audio_getconfig no more room for SNK info");
        }
    }

    /* If last SNK get capabilities or all supported codec capa retrieved */
    if ((p_peer->num_rx_snks == p_peer->num_snks) ||
            (p_peer->num_sup_snks == BTA_AV_CO_NUM_ELEMENTS(p_peer->snks))) {
        APPL_TRACE_DEBUG("bta_av_co_audio_getconfig last sink reached");

        /* Protect access to bta_av_co_cb.codec_cfg */
        osi_mutex_global_lock();

        /* Find a sink that matches the codec config */
        p_sink = bta_av_co_audio_find_peer_sink(p_peer);
        if (p_sink != NULL) {
            /* stop fetching caps once we retrieved a supported codec */
            if (p_peer->acp) {
                *p_sep_info_idx = p_peer->num_seps;
                APPL_TRACE_EVENT("no need to fetch more SEPs");
            }

            /* Build the codec configuration for this sink */
            if (bta_av_co_audio_codec_build_config(p_sink->codec_caps, codec_cfg)) {
                APPL_TRACE_DEBUG("bta_av_co_audio_getconfig reconfig p_codec_info[%x:%x:%x:%x:%x:%x]",
                                 codec_cfg[1], codec_cfg[2], codec_cfg[3],
                                 codec_cfg[4], codec_cfg[5], codec_cfg[6]);

                /* Save the new configuration */
                p_peer->p_snk = p_sink;
                memcpy(p_peer->codec_cfg, codec_cfg, AVDT_CODEC_SIZE);

                /* By default, no content protection */
                *p_num_protect = 0;

#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
                /* Check if this sink supports SCMS */
                if (bta_av_co_audio_sink_has_scmst(p_sink)) {
                    p_peer->cp_active = TRUE;
                    bta_av_co_cb.cp.active = TRUE;
                    *p_num_protect = BTA_AV_CP_INFO_LEN;
                    memcpy(p_protect_info, bta_av_co_cp_scmst, BTA_AV_CP_INFO_LEN);
                } else {
                    p_peer->cp_active = FALSE;
                    bta_av_co_cb.cp.active = FALSE;
                }
#endif

                /* If acceptor -> reconfig otherwise reply for configuration */
                if (p_peer->acp) {
                    if (p_peer->recfg_needed) {
                        APPL_TRACE_DEBUG("bta_av_co_audio_getconfig call BTA_AvReconfig(x%x)", hndl);
                        BTA_AvReconfig(hndl, TRUE, p_sink->sep_info_idx, p_peer->codec_cfg, *p_num_protect, (UINT8 *)bta_av_co_cp_scmst);
                    }
                } else {
                    *p_sep_info_idx = p_sink->sep_info_idx;
                    memcpy(p_codec_info, p_peer->codec_cfg, AVDT_CODEC_SIZE);
                }
                result =  A2D_SUCCESS;
            }
        }
        /* Protect access to bta_av_co_cb.codec_cfg */
        osi_mutex_global_unlock();
    }
    return result;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_setconfig
 **
 ** Description      This callout function is executed by AV to set the codec and
 **                  content protection configuration of the audio stream.
 **
 **
 ** Returns          void
 **
 *******************************************************************************/
void bta_av_co_audio_setconfig(tBTA_AV_HNDL hndl,
                               UINT8 *p_codec_info, UINT8 seid, BD_ADDR addr, UINT8 num_protect, UINT8 *p_protect_info,
                               UINT8 t_local_sep, UINT8 avdt_handle)
{
    tBTA_AV_CO_PEER *p_peer;
    UINT8 status = A2D_SUCCESS;
    UINT8 category = A2D_SUCCESS;
    BOOLEAN recfg_needed = FALSE;
    UINT8 codec_cfg_status = A2D_SUCCESS;
    UNUSED(seid);
    UNUSED(addr);

    FUNC_TRACE();

    APPL_TRACE_DEBUG("bta_av_co_audio_setconfig p_codec_info[%x:%x:%x:%x:%x:%x]",
                     p_codec_info[1], p_codec_info[2], p_codec_info[3],
                     p_codec_info[4], p_codec_info[5], p_codec_info[6]);
    APPL_TRACE_DEBUG("num_protect:0x%02x protect_info:0x%02x%02x%02x",
                     num_protect, p_protect_info[0], p_protect_info[1], p_protect_info[2]);

    /* Retrieve the peer info */
    p_peer = bta_av_co_get_peer(hndl);
    if (p_peer == NULL) {
        APPL_TRACE_ERROR("bta_av_co_audio_setconfig could not find peer entry");

        /* Call call-in rejecting the configuration */
        bta_av_ci_setconfig(hndl, A2D_BUSY, AVDT_ASC_CODEC, 0, NULL, FALSE, avdt_handle);
        return;
    }
    APPL_TRACE_DEBUG("bta_av_co_audio_setconfig peer(o=%d,n_snks=%d,n_rx_snks=%d,n_sup_snks=%d)",
                     p_peer->opened, p_peer->num_snks, p_peer->num_rx_snks, p_peer->num_sup_snks);

    /* Sanity check: should not be opened at this point */
    if (p_peer->opened) {
        APPL_TRACE_ERROR("bta_av_co_audio_setconfig peer already in use");
    }

#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
    if (num_protect != 0) {
        /* If CP is supported */
        if ((num_protect != 1) ||
                (bta_av_co_cp_is_scmst(p_protect_info) == FALSE)) {
            APPL_TRACE_ERROR("bta_av_co_audio_setconfig wrong CP configuration");
            status = A2D_BAD_CP_TYPE;
            category = AVDT_ASC_PROTECT;
        }
    }
#else
    /* Do not support content protection for the time being */
    if (num_protect != 0) {
        APPL_TRACE_ERROR("bta_av_co_audio_setconfig wrong CP configuration");
        status = A2D_BAD_CP_TYPE;
        category = AVDT_ASC_PROTECT;
    }
#endif
    if (status == A2D_SUCCESS) {
        if (AVDT_TSEP_SNK == t_local_sep) {
            codec_cfg_status = A2DP_IsPeerSourceCodecSupported(p_codec_info);
            APPL_TRACE_DEBUG(" Peer is  A2DP SRC ");
        }
        if (AVDT_TSEP_SRC == t_local_sep) {
            codec_cfg_status = A2DP_IsSinkCodecSupported(p_codec_info);
            APPL_TRACE_DEBUG(" Peer is A2DP SINK ");
        }
        /* Check if codec configuration is supported */
        if (codec_cfg_status == A2D_SUCCESS) {

            /* Protect access to bta_av_co_cb.codec_cfg */
            osi_mutex_global_lock();

            /* Check if the configuration matches the current codec config */
            if (!A2DP_CodecTypeEquals(bta_av_co_cb.codec_cfg_setconfig.info, p_codec_info)) {
                recfg_needed = TRUE;
            } else if ((num_protect == 1) && (!bta_av_co_cb.cp.active)) {
                recfg_needed = TRUE;
            }

            memcpy(bta_av_co_cb.codec_cfg_setconfig.info, p_codec_info, AVDT_CODEC_SIZE);
            if (AVDT_TSEP_SNK == t_local_sep) {
                /* If Peer is SRC, and our cfg subset matches with what is requested by peer, then
                                     just accept what peer wants */
                memcpy(bta_av_co_cb.codec_cfg.info, p_codec_info, AVDT_CODEC_SIZE);
                recfg_needed = FALSE;
            }

            /* Protect access to bta_av_co_cb.codec_cfg */
            osi_mutex_global_unlock();
        } else {
            category = AVDT_ASC_CODEC;
            status = A2D_FAIL;
        }
    }

    if (status != A2D_SUCCESS) {
        APPL_TRACE_DEBUG("bta_av_co_audio_setconfig reject s=%d c=%d", codec_cfg_status, category);

        /* Call call-in rejecting the configuration */
        bta_av_ci_setconfig(hndl, codec_cfg_status, category, 0, NULL, FALSE, avdt_handle);
    } else {
        /* Mark that this is an acceptor peer */
        p_peer->acp = TRUE;
        p_peer->recfg_needed = recfg_needed;

        APPL_TRACE_DEBUG("bta_av_co_audio_setconfig accept reconf=%d", recfg_needed);

        /* Call call-in accepting the configuration */
        bta_av_ci_setconfig(hndl, A2D_SUCCESS, A2D_SUCCESS, 0, NULL, recfg_needed, avdt_handle);
    }
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_open
 **
 ** Description      This function is called by AV when the audio stream connection
 **                  is opened.
 **
 **
 ** Returns          void
 **
 *******************************************************************************/
void bta_av_co_audio_open(tBTA_AV_HNDL hndl, UINT8 *p_codec_info,
                          UINT16 mtu)
{
    tBTA_AV_CO_PEER *p_peer;
    UNUSED(p_codec_info);

    FUNC_TRACE();

    APPL_TRACE_DEBUG("bta_av_co_audio_open mtu:%d codec:%s", mtu, A2DP_CodecName(p_codec_info));

    /* Retrieve the peer info */
    p_peer = bta_av_co_get_peer(hndl);
    if (p_peer == NULL) {
        APPL_TRACE_ERROR("bta_av_co_audio_setconfig could not find peer entry");
    } else {
        p_peer->opened = TRUE;
        p_peer->mtu = mtu;
    }
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_close
 **
 ** Description      This function is called by AV when the audio stream connection
 **                  is closed.
 **
 **
 ** Returns          void
 **
 *******************************************************************************/
void bta_av_co_audio_close(tBTA_AV_HNDL hndl, UINT16 mtu)

{
    tBTA_AV_CO_PEER *p_peer;
    UNUSED(mtu);

    FUNC_TRACE();

    APPL_TRACE_DEBUG("bta_av_co_audio_close");

    /* Retrieve the peer info */
    p_peer = bta_av_co_get_peer(hndl);
    if (p_peer) {
        /* Mark the peer closed and clean the peer info */
        memset(p_peer, 0, sizeof(*p_peer));
    } else {
        APPL_TRACE_ERROR("bta_av_co_audio_close could not find peer entry");
    }

    /* reset remote preference through setconfig */
    memset(bta_av_co_cb.codec_cfg_setconfig.info, 0, AVDT_CODEC_SIZE);
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_start
 **
 ** Description      This function is called by AV when the audio streaming data
 **                  transfer is started.
 **
 **
 ** Returns          void
 **
 *******************************************************************************/
void bta_av_co_audio_start(tBTA_AV_HNDL hndl,
                           UINT8 *p_codec_info, BOOLEAN *p_no_rtp_hdr)
{
    UNUSED(hndl);
    UNUSED(p_codec_info);
    UNUSED(p_no_rtp_hdr);

    FUNC_TRACE();

    APPL_TRACE_DEBUG("bta_av_co_audio_start");

}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_stop
 **
 ** Description      This function is called by AV when the audio streaming data
 **                  transfer is stopped.
 **
 **
 ** Returns          void
 **
 *******************************************************************************/
extern void bta_av_co_audio_stop(tBTA_AV_HNDL hndl)
{
    UNUSED(hndl);

    FUNC_TRACE();

    APPL_TRACE_DEBUG("bta_av_co_audio_stop");
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_src_data_path
 **
 ** Description      This function is called to manage data transfer from
 **                  the audio codec to AVDTP.
 **
 ** Returns          Pointer to the buffer to send, NULL if no buffer to send
 **
 *******************************************************************************/
void *bta_av_co_audio_src_data_path(const uint8_t* p_codec_info, UINT32 *p_len,
                                    UINT32 *p_timestamp)
{
#if BTC_AV_SRC_INCLUDED
    BT_HDR *p_buf;
    UNUSED(p_len);

    FUNC_TRACE();

    tBTA_AV_CODEC codec_type = A2DP_GetCodecType(p_codec_info);

    p_buf = btc_a2dp_source_audio_readbuf();
    if (p_buf != NULL) {
        switch (codec_type) {
        case BTA_AV_CODEC_SBC:
            /* In media packet SBC, the following information is available:
             * p_buf->layer_specific : number of SBC frames in the packet
             * p_buf->word[0] : timestamp
             */
            /* Retrieve the timestamp information from the media packet */
            *p_timestamp = *((UINT32 *) (p_buf + 1));

            /* Set up packet header */
            bta_av_sbc_bld_hdr(p_buf, p_buf->layer_specific);
            break;


        default:
            APPL_TRACE_ERROR("bta_av_co_audio_src_data_path Unsupported codec type (%d)", codec_type);
            break;
        }
#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
        {
            UINT8 *p;
            if (bta_av_co_cp_is_active()) {
                p_buf->len++;
                p_buf->offset--;
                p = (UINT8 *)(p_buf + 1) + p_buf->offset;
                *p = bta_av_co_cp_get_flag();
            }
        }
#endif
    }
    return p_buf;
#else /* BTC_AV_SRC_INCLUDED */
    return NULL;
#endif /* BTC_AV_SRC_INCLUDED */
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_drop
 **
 ** Description      An Audio packet is dropped. .
 **                  It's very likely that the connected headset with this handle
 **                  is moved far away. The implementation may want to reduce
 **                  the encoder bit rate setting to reduce the packet size.
 **
 ** Returns          void
 **
 *******************************************************************************/
void bta_av_co_audio_drop(tBTA_AV_HNDL hndl)
{
    FUNC_TRACE();

    APPL_TRACE_ERROR("bta_av_co_audio_drop dropped: x%x", hndl);
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_delay
 **
 ** Description      This function is called by AV when the audio stream connection
 **                  needs to send the initial delay report to the connected SRC.
 **
 **
 ** Returns          void
 **
 *******************************************************************************/
void bta_av_co_audio_delay(tBTA_AV_HNDL hndl, UINT16 delay)
{
    FUNC_TRACE();

    btc_source_report_delay_value(delay);

    APPL_TRACE_DEBUG("bta_av_co_audio_delay handle: x%x, delay:0x%x", hndl, delay);
}



/*******************************************************************************
 **
 ** Function         bta_av_co_audio_codec_build_config
 **
 ** Description      Build the codec configuration
 **
 ** Returns          TRUE if the codec was built successfully, FALSE otherwise
 **
 *******************************************************************************/
static BOOLEAN bta_av_co_audio_codec_build_config(const UINT8 *p_codec_caps, UINT8 *p_codec_cfg)
{
    FUNC_TRACE();

    memset(p_codec_cfg, 0, AVDT_CODEC_SIZE);
    return A2DP_BuildCodecConfig((UINT8 *)p_codec_caps, p_codec_cfg);
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_peer_reset_config
 **
 ** Description      Reset the peer codec configuration
 **
 ** Returns          Nothing
 **
 *******************************************************************************/
static void bta_av_co_audio_peer_reset_config(tBTA_AV_CO_PEER *p_peer)
{
    FUNC_TRACE();

    /* Indicate that there is no currently selected sink */
    p_peer->p_snk = NULL;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_cp_is_scmst
 **
 ** Description      Check if a content protection service is SCMS-T
 **
 ** Returns          TRUE if this CP is SCMS-T, FALSE otherwise
 **
 *******************************************************************************/
static BOOLEAN bta_av_co_cp_is_scmst(const UINT8 *p_protectinfo)
{
    UINT16 cp_id;
    FUNC_TRACE();

    if (*p_protectinfo >= BTA_AV_CP_LOSC) {
        p_protectinfo++;
        STREAM_TO_UINT16(cp_id, p_protectinfo);
        if (cp_id == BTA_AV_CP_SCMS_T_ID) {
            APPL_TRACE_DEBUG("bta_av_co_cp_is_scmst: SCMS-T found");
            return TRUE;
        }
    }

    return FALSE;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_sink_has_scmst
 **
 ** Description      Check if a sink supports SCMS-T
 **
 ** Returns          TRUE if the sink supports this CP, FALSE otherwise
 **
 *******************************************************************************/
static BOOLEAN bta_av_co_audio_sink_has_scmst(const tBTA_AV_CO_SINK *p_sink)
{
    UINT8 index;
    const UINT8 *p;
    FUNC_TRACE();

    /* Check if sink supports SCMS-T */
    index = p_sink->num_protect;
    p = &p_sink->protect_info[0];

    while (index) {
        if (bta_av_co_cp_is_scmst(p)) {
            return TRUE;
        }
        /* Move to the next SC */
        p += *p + 1;
        /* Decrement the SC counter */
        index--;
    }
    APPL_TRACE_DEBUG("bta_av_co_audio_sink_has_scmst: SCMS-T not found");
    return FALSE;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_sink_supports_cp
 **
 ** Description      Check if a sink supports the current content protection
 **
 ** Returns          TRUE if the sink supports this CP, FALSE otherwise
 **
 *******************************************************************************/
static BOOLEAN bta_av_co_audio_sink_supports_cp(const tBTA_AV_CO_SINK *p_sink)
{
    FUNC_TRACE();

    /* Check if content protection is enabled for this stream */
    if (bta_av_co_cp_get_flag() != BTA_AV_CP_SCMS_COPY_FREE) {
        return bta_av_co_audio_sink_has_scmst(p_sink);
    } else {
        APPL_TRACE_DEBUG("bta_av_co_audio_sink_supports_cp: not required");
        return TRUE;
    }
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_find_peer_sink
 **
 ** Description      Find a peer Sink SEP entry with a supported codec.
 **
 ** Returns          the peer Sink SEP for the codec index if found, otherwise NULL
 **
 *******************************************************************************/
static tBTA_AV_CO_SINK* bta_av_co_audio_find_peer_sink(tBTA_AV_CO_PEER *p_peer)
{
    btav_a2dp_codec_index_t codec_index;

    FUNC_TRACE();

    for (int index = p_peer->num_sup_snks - 1; index >= 0; index--) {
        tBTA_AV_CO_SINK* snk = &p_peer->snks[index];
        codec_index = A2DP_SinkCodecIndex(snk->codec_caps);
        if (codec_index != BTAV_A2DP_CODEC_INDEX_MAX)
            return snk;
    }
    return NULL;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_find_peer_src
 **
 ** Description      Find a peer Source SEP entry with a supported codec.
 **
 ** Returns          the peer Source SEP for the codec index if found, otherwise NULL
 **
 *******************************************************************************/
static tBTA_AV_CO_SINK* bta_av_co_audio_find_peer_src(tBTA_AV_CO_PEER *p_peer)
{
    btav_a2dp_codec_index_t codec_index;

    FUNC_TRACE();

    for (int index = p_peer->num_sup_srcs - 1; index >= 0; index--) {
        tBTA_AV_CO_SINK* src = &p_peer->srcs[index];
        codec_index = A2DP_SourceCodecIndex(src->codec_caps);
        if (codec_index != BTAV_A2DP_CODEC_INDEX_MAX)
            return src;
    }
    return NULL;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_codec_supported
 **
 ** Description      Check if all opened connections are compatible with a codec
 **                  configuration and content protection
 **
 ** Returns          TRUE if all opened devices support this codec, FALSE otherwise
 **
 *******************************************************************************/
BOOLEAN bta_av_co_audio_codec_supported(tBTC_AV_STATUS *p_status)
{
    UINT8 index;
    tBTA_AV_CO_PEER *p_peer;
    tBTA_AV_CO_SINK *p_sink;
    UINT8 codec_cfg[AVDT_CODEC_SIZE];
    UINT8 num_protect = 0;
#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
    BOOLEAN cp_active;
#endif

    FUNC_TRACE();

    APPL_TRACE_DEBUG("bta_av_co_audio_codec_supported");

    /* Check AV feeding is supported */
    *p_status = BTC_ERROR_SRV_AV_FEEDING_NOT_SUPPORTED;

    for (index = 0; index < BTA_AV_CO_NUM_ELEMENTS(bta_av_co_cb.peers); index++) {
        p_peer = &bta_av_co_cb.peers[index];
        if (p_peer->opened) {
            p_sink = bta_av_co_audio_find_peer_sink(p_peer);
            if (p_sink != NULL) {

                /* Check that this sink is compatible with the CP */
                if (!bta_av_co_audio_sink_supports_cp(p_sink)) {
                    APPL_TRACE_DEBUG("bta_av_co_audio_codec_supported sink codec of peer %d doesn't support cp",
                                     index);
                    *p_status = BTC_ERROR_SRV_AV_CP_NOT_SUPPORTED;
                    return FALSE;
                }

                /* Build the codec configuration for this sink */
                if (bta_av_co_audio_codec_build_config(p_sink->codec_caps, codec_cfg)) {
#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
                    /* Check if this sink supports SCMS */
                    cp_active = bta_av_co_audio_sink_has_scmst(p_sink);
#endif
                    /* Check if this is a new configuration (new sink or new config) */
                    if ((p_sink != p_peer->p_snk) ||
                            (memcmp(codec_cfg, p_peer->codec_cfg, AVDT_CODEC_SIZE))
#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
                            || (p_peer->cp_active != cp_active)
#endif
                       ) {
                        /* Save the new configuration */
                        p_peer->p_snk = p_sink;
                        memcpy(p_peer->codec_cfg, codec_cfg, AVDT_CODEC_SIZE);
#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
                        p_peer->cp_active = cp_active;
                        if (p_peer->cp_active) {
                            bta_av_co_cb.cp.active = TRUE;
                            num_protect = BTA_AV_CP_INFO_LEN;
                        } else {
                            bta_av_co_cb.cp.active = FALSE;
                        }
#endif
                        APPL_TRACE_DEBUG("bta_av_co_audio_codec_supported call BTA_AvReconfig(x%x)", BTA_AV_CO_AUDIO_INDX_TO_HNDL(index));
                        BTA_AvReconfig(BTA_AV_CO_AUDIO_INDX_TO_HNDL(index), TRUE, p_sink->sep_info_idx,
                                       p_peer->codec_cfg, num_protect, (UINT8 *)bta_av_co_cp_scmst);
                    }
                }
            } else {
                APPL_TRACE_DEBUG("bta_av_co_audio_codec_supported index %d doesn't support codec", index);
                return FALSE;
            }
        }
    }

    *p_status = BTC_AV_SUCCESS;
    return TRUE;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_codec_reset
 **
 ** Description      Reset the current codec configuration
 **
 ** Returns          void
 **
 *******************************************************************************/
void bta_av_co_audio_codec_reset(void)
{
    osi_mutex_global_lock();
    FUNC_TRACE();

    /* Reset the current configuration to SBC */
    if (A2D_BldSbcInfo(A2D_MEDIA_TYPE_AUDIO, (tA2D_SBC_CIE *)&btc_av_sbc_default_config, bta_av_co_cb.codec_cfg.info) != A2D_SUCCESS) {
        APPL_TRACE_ERROR("bta_av_co_audio_codec_reset A2D_BldSbcInfo failed");
    }

    osi_mutex_global_unlock();
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_set_codec
 **
 ** Description      Set the current codec configuration from the feeding type.
 **                  This function is starting to modify the configuration, it
 **                  should be protected.
 **
 ** Returns          TRUE if successful, FALSE otherwise
 **
 *******************************************************************************/
BOOLEAN bta_av_co_audio_set_codec(const tBTC_AV_MEDIA_FEEDINGS *p_feeding, tBTC_AV_STATUS *p_status)
{
    tA2D_SBC_CIE sbc_config;
    tBTC_AV_CODEC_INFO new_cfg;

    FUNC_TRACE();

    /* Check AV feeding is supported */
    *p_status = BTC_ERROR_SRV_AV_FEEDING_NOT_SUPPORTED;

    APPL_TRACE_DEBUG("bta_av_co_audio_set_codec cid=%d", p_feeding->format);

    /* Supported codecs */
    switch (p_feeding->format) {
    case BTC_AV_CODEC_PCM:
        sbc_config = btc_av_sbc_default_config;
        if ((p_feeding->cfg.pcm.num_channel != 1) &&
                (p_feeding->cfg.pcm.num_channel != 2)) {
            APPL_TRACE_ERROR("bta_av_co_audio_set_codec PCM channel number unsupported");
            return FALSE;
        }
        if ((p_feeding->cfg.pcm.bit_per_sample != 8) &&
                (p_feeding->cfg.pcm.bit_per_sample != 16)) {
            APPL_TRACE_ERROR("bta_av_co_audio_set_codec PCM sample size unsupported");
            return FALSE;
        }
        switch (p_feeding->cfg.pcm.sampling_freq) {
        case 8000:
        case 12000:
        case 16000:
        case 24000:
        case 32000:
        case 48000:
            sbc_config.samp_freq = A2D_SBC_IE_SAMP_FREQ_48;
            break;

        case 11025:
        case 22050:
        case 44100:
            sbc_config.samp_freq = A2D_SBC_IE_SAMP_FREQ_44;
            break;
        default:
            APPL_TRACE_ERROR("bta_av_co_audio_set_codec PCM sampling frequency unsupported");
            return FALSE;
            break;
        }
        /* Build the codec config */
        if (A2D_BldSbcInfo(A2D_MEDIA_TYPE_AUDIO, &sbc_config, new_cfg.info) != A2D_SUCCESS) {
            APPL_TRACE_ERROR("bta_av_co_audio_set_codec A2D_BldSbcInfo failed");
            return FALSE;
        }
        break;


    default:
        APPL_TRACE_ERROR("bta_av_co_audio_set_codec Feeding format unsupported");
        return FALSE;
        break;
    }

    /* The new config was correctly built */
    bta_av_co_cb.codec_cfg = new_cfg;


    /* Check all devices support it */
    *p_status = BTC_AV_SUCCESS;
    return bta_av_co_audio_codec_supported(p_status);
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_get_sbc_config
 **
 ** Description      Retrieves the SBC codec configuration.  If the codec in use
 **                  is not SBC, return the default SBC codec configuration.
 **
 ** Returns          TRUE if codec is SBC, FALSE otherwise
 **
 *******************************************************************************/
BOOLEAN bta_av_co_audio_get_sbc_config(tA2D_SBC_CIE *p_sbc_config, UINT16 *p_minmtu)
{
    BOOLEAN result = FALSE;
    UINT8 index, jndex;
    tBTA_AV_CO_PEER *p_peer;
    tBTA_AV_CO_SINK *p_sink;

    APPL_TRACE_EVENT("bta_av_co_cb.codec_cfg : codec %s",
                     A2DP_CodecName(bta_av_co_cb.codec_cfg.info));

    /* Minimum MTU is by default very large */
    *p_minmtu = 0xFFFF;

    osi_mutex_global_lock();
    if (A2DP_GetCodecType(bta_av_co_cb.codec_cfg.info) == BTC_AV_CODEC_SBC) {
        if (A2D_ParsSbcInfo(p_sbc_config, bta_av_co_cb.codec_cfg.info, FALSE) == A2D_SUCCESS) {
            for (index = 0; index < BTA_AV_CO_NUM_ELEMENTS(bta_av_co_cb.peers); index++) {
                p_peer = &bta_av_co_cb.peers[index];
                if (p_peer->opened) {
                    if (p_peer->mtu < *p_minmtu) {
                        *p_minmtu = p_peer->mtu;
                    }
                    for (jndex = 0; jndex < p_peer->num_sup_snks; jndex++) {
                        p_sink = &p_peer->snks[jndex];
                        tBTA_AV_CODEC codec_type = A2DP_GetCodecType(p_sink->codec_caps);
                        if (codec_type == A2D_MEDIA_CT_SBC) {
                            /* Update the bitpool boundaries of the current config */
                            p_sbc_config->min_bitpool =
                                BTA_AV_CO_MAX(p_sink->codec_caps[BTA_AV_CO_SBC_MIN_BITPOOL_OFF],
                                              p_sbc_config->min_bitpool);
                            p_sbc_config->max_bitpool =
                                BTA_AV_CO_MIN(p_sink->codec_caps[BTA_AV_CO_SBC_MAX_BITPOOL_OFF],
                                              p_sbc_config->max_bitpool);
                            APPL_TRACE_EVENT("bta_av_co_audio_get_sbc_config : sink bitpool min %d, max %d",
                                             p_sbc_config->min_bitpool, p_sbc_config->max_bitpool);
                            break;
                        }
                    }
                }
            }
            result = TRUE;
        }
    }

    if (!result) {
        /* Not SBC, still return the default values */
        *p_sbc_config = btc_av_sbc_default_config;
    }
    osi_mutex_global_unlock();

    return result;
}

/*******************************************************************************
 **
 ** Function         bta_av_co_audio_discard_config
 **
 ** Description      Discard the codec configuration of a connection
 **
 ** Returns          Nothing
 **
 *******************************************************************************/
void bta_av_co_audio_discard_config(tBTA_AV_HNDL hndl)
{
    tBTA_AV_CO_PEER *p_peer;

    FUNC_TRACE();

    /* Find the peer info */
    p_peer = bta_av_co_get_peer(hndl);
    if (p_peer == NULL) {
        APPL_TRACE_ERROR("bta_av_co_audio_discard_config could not find peer entry");
        return;
    }

    /* Reset the peer codec configuration */
    bta_av_co_audio_peer_reset_config(p_peer);
}

/*******************************************************************************
 **
 ** Function         bta_av_co_init
 **
 ** Description      Initialization
 **
 ** Returns          Nothing
 **
 *******************************************************************************/
void bta_av_co_init(void)
{
    FUNC_TRACE();

    /* Reset the control block */
    memset(&bta_av_co_cb, 0, sizeof(bta_av_co_cb));

    memset(bta_av_co_cb.codec_cfg_setconfig.info, 0, AVDT_CODEC_SIZE);

#if defined(BTA_AV_CO_CP_SCMS_T) && (BTA_AV_CO_CP_SCMS_T == TRUE)
    bta_av_co_cp_set_flag(BTA_AV_CP_SCMS_COPY_NEVER);
#else
    bta_av_co_cp_set_flag(BTA_AV_CP_SCMS_COPY_FREE);
#endif

    /* Reset the current config */
    bta_av_co_audio_codec_reset();
}


/*******************************************************************************
 **
 ** Function         bta_av_co_peer_cp_supported
 **
 ** Description      Checks if the peer supports CP
 **
 ** Returns          TRUE if the peer supports CP
 **
 *******************************************************************************/
BOOLEAN bta_av_co_peer_cp_supported(tBTA_AV_HNDL hndl)
{
    tBTA_AV_CO_PEER *p_peer;
    tBTA_AV_CO_SINK *p_sink;
    UINT8 index;

    FUNC_TRACE();

    /* Find the peer info */
    p_peer = bta_av_co_get_peer(hndl);
    if (p_peer == NULL) {
        APPL_TRACE_ERROR("bta_av_co_peer_cp_supported could not find peer entry");
        return FALSE;
    }

    for (index = 0; index < p_peer->num_sup_snks; index++) {
        p_sink = &p_peer->snks[index];
        if (A2DP_GetCodecType(p_sink->codec_caps) == A2D_MEDIA_CT_SBC) {
            return bta_av_co_audio_sink_has_scmst(p_sink);
        }
    }
    APPL_TRACE_ERROR("bta_av_co_peer_cp_supported did not find SBC sink");
    return FALSE;
}


/*******************************************************************************
 **
 ** Function         bta_av_co_get_remote_bitpool_pref
 **
 ** Description      Check if remote side did a setconfig within the limits
 **                  of our exported bitpool range. If set we will set the
 **                  remote preference.
 **
 ** Returns          TRUE if config set, FALSE otherwize
 **
 *******************************************************************************/

BOOLEAN bta_av_co_get_remote_bitpool_pref(UINT8 *min, UINT8 *max)
{
    /* check if remote peer did a set config */
    if (A2DP_GetCodecType(bta_av_co_cb.codec_cfg_setconfig.info)!= BTC_AV_CODEC_SBC){
        return FALSE;
    }

    *min = bta_av_co_cb.codec_cfg_setconfig.info[BTA_AV_CO_SBC_MIN_BITPOOL_OFF];
    *max = bta_av_co_cb.codec_cfg_setconfig.info[BTA_AV_CO_SBC_MAX_BITPOOL_OFF];

    return TRUE;
}

/* the call out functions for audio stream */
const tBTA_AV_CO_FUNCTS bta_av_a2d_cos = {
    bta_av_co_audio_init,
    bta_av_co_audio_disc_res,
    bta_av_co_audio_cfg_res,
    bta_av_co_audio_getconfig,
    bta_av_co_audio_setconfig,
    bta_av_co_audio_open,
    bta_av_co_audio_close,
    bta_av_co_audio_start,
    bta_av_co_audio_stop,
    bta_av_co_audio_src_data_path,
    bta_av_co_audio_delay
};

#endif /* #if BTC_AV_INCLUDED */
