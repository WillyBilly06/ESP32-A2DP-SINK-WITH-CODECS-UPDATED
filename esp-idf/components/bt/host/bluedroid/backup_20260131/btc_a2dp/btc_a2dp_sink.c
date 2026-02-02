/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/******************************************************************************
 **
 **  Name:          btc_a2dp_sink.c
 **
 ******************************************************************************/
#include "common/bt_target.h"
#include "common/bt_trace.h"
#include <string.h>
#include <stdint.h>
#include "common/bt_defs.h"
#include "osi/allocator.h"
#include "osi/mutex.h"
#include "osi/thread.h"
#include "osi/fixed_queue.h"
#include "stack/a2d_api.h"
#include "bta/bta_av_api.h"
#include "bta/bta_av_ci.h"
#include "btc_av_co.h"
#include "btc_a2dp.h"
#include "btc_a2dp_control.h"
#include "btc_a2dp_sink.h"
#include "btc/btc_manage.h"
#include "btc_av.h"
#include "btc/btc_util.h"
#include "esp_a2dp_api.h"
#include "oi_status.h"
#include "osi/future.h"
#include <assert.h>
#include "esp_heap_caps.h"  // For PSRAM allocation

#if (BTC_AV_SINK_INCLUDED == TRUE) && (BTC_AV_EXT_CODEC == FALSE)

#include "oi_codec_sbc.h"

/* IMPORTANT: Increased buffer size for LDAC/aptX hi-res codecs
 * LDAC at 96kHz stereo 32-bit needs ~64KB decode buffer
 * This buffer MUST be allocated from PSRAM for high bitrate codecs */
#if (defined(APTX_DEC_INCLUDED) && APTX_DEC_INCLUDED == TRUE) || (defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE)
#define BT_A2DP_SINK_BUF_SIZE   (64*1024)
#else
#define BT_A2DP_SINK_BUF_SIZE   (64*1024)
#endif

/*****************************************************************************
 **  Constants
 *****************************************************************************/

/* BTC media cmd event definition : BTC_MEDIA_TASK_CMD */
enum {
    BTC_MEDIA_TASK_SINK_INIT,
    BTC_MEDIA_TASK_SINK_CLEAN_UP,
    BTC_MEDIA_FLUSH_AA_RX,
    BTC_MEDIA_AUDIO_SINK_CFG_UPDATE,
    BTC_MEDIA_AUDIO_SINK_CLEAR_TRACK,
};

enum {
    BTC_A2DP_SINK_STATE_OFF = 0,
    BTC_A2DP_SINK_STATE_ON = 1,
    BTC_A2DP_SINK_STATE_SHUTTING_DOWN = 2
};

/*
 * CONGESTION COMPENSATION CTRL ::
 *
 * Thus setting controls how many buffers we will hold in media task
 * during temp link congestion. Together with the stack buffer queues
 * it controls much temporary a2dp link congestion we can
 * compensate for. It however also depends on the default run level of sinks
 * jitterbuffers. Depending on type of sink this would vary.
 * Ideally the (SRC) max tx buffer capacity should equal the sinks
 * jitterbuffer runlevel including any intermediate buffers on the way
 * towards the sinks codec.
 */

/* fixme -- define this in pcm time instead of buffer count */

/* The typical runlevel of the tx queue size is ~1 buffer
   but due to link flow control or thread preemption in lower
   layers we might need to temporarily buffer up data */

/* Increased from 25 to 150 to handle SPIFFS write/delete stalls during flash erase
   150 frames is equivalent to ~600ms of buffering at 44.1kHz
   This prevents packet drops when SPIFFS operations block the CPU */
#define MAX_OUTPUT_A2DP_SNK_FRAME_QUEUE_SZ     (150)

#define BTC_A2DP_SNK_DATA_QUEUE_IDX            (1)

#define A2DP_TASK_NAME                   "A2DP_DECODER"
#if CONFIG_SPIRAM
#define A2DP_TASK_STACK_SIZE             (50 * 1024)
#else
#define A2DP_TASK_STACK_SIZE             (BT_BTC_TASK_STACK_SIZE + BT_TASK_EXTRA_STACK_SIZE)
#endif
#define A2DP_TASK_PRIO                   (BT_TASK_MAX_PRIORITIES - 6)
#define A2DP_TASK_PINNED_TO_CORE         (1)
#define A2DP_TASK_WORKQUEUE_NUM          (2)
#define A2DP_TASK_WORKQUEUE0_LEN         (1)
#define A2DP_TASK_WORKQUEUE1_LEN         (5)

typedef struct {
    uint32_t sig;
    void *param;
} a2dp_sink_task_evt_t;

typedef struct {
    BOOLEAN rx_flush; /* discards any incoming data when true */
    struct osi_event *data_ready_event;
    fixed_queue_t *RxSbcQ;
} tBTC_A2DP_SINK_CB;

// typedef struct {
//     uint16_t expected_seq_num;
//     bool seq_num_recount;
// } a2dp_sink_media_pkt_seq_num_t;

typedef struct {
    tBTC_A2DP_SINK_CB   btc_aa_snk_cb;
    osi_thread_t        *btc_aa_snk_task_hdl;
    const tA2DP_DECODER_INTERFACE* decoder;
    unsigned char *decode_buf;  // Allocated from PSRAM for hi-res codecs
    // a2dp_sink_media_pkt_seq_num_t   media_pkt_seq_num;
} a2dp_sink_local_param_t;

static void btc_a2dp_sink_thread_init(UNUSED_ATTR void *context);
static void btc_a2dp_sink_thread_cleanup(UNUSED_ATTR void *context);
static void btc_a2dp_sink_flush_q(fixed_queue_t *p_q);
static void btc_a2dp_sink_rx_flush(void);
/* Handle incoming media packets A2DP SINK streaming*/
static void btc_a2dp_sink_handle_inc_media(BT_HDR *p_msg);
static void btc_a2dp_sink_handle_decoder_reset(tBTC_MEDIA_SINK_CFG_UPDATE *p_msg);
static void btc_a2dp_sink_handle_clear_track(void);
static BOOLEAN btc_a2dp_sink_clear_track(void);

static void btc_a2dp_sink_data_ready(void *context);

static int btc_a2dp_sink_state = BTC_A2DP_SINK_STATE_OFF;
static esp_a2d_sink_data_cb_t bt_aa_snk_data_cb = NULL;
#if A2D_DYNAMIC_MEMORY == FALSE
static a2dp_sink_local_param_t a2dp_sink_local_param;
#else
static a2dp_sink_local_param_t *a2dp_sink_local_param_ptr;
#define a2dp_sink_local_param (*a2dp_sink_local_param_ptr)
#endif ///A2D_DYNAMIC_MEMORY == FALSE

void btc_a2dp_sink_reg_data_cb(esp_a2d_sink_data_cb_t callback)
{
    // todo: critical section protection
    bt_aa_snk_data_cb = callback;
}

static inline void btc_a2d_data_cb_to_app(unsigned char *data, uint32_t len)
{
    // todo: critical section protection
    if (bt_aa_snk_data_cb) {
        bt_aa_snk_data_cb(data, len);
    }
}

/*****************************************************************************
 **  Misc helper functions
 *****************************************************************************/
static inline void btc_a2d_cb_to_app(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    esp_a2d_cb_t btc_aa_cb = (esp_a2d_cb_t)btc_profile_cb_get(BTC_PID_A2DP);
    if (btc_aa_cb) {
        btc_aa_cb(event, param);
    }
}

/*****************************************************************************
 **  BTC ADAPTATION
 *****************************************************************************/

static void btc_a2dp_sink_ctrl(void *param)
{
    BT_HDR* p_buf = (BT_HDR*)param;

    if (!p_buf) {
        APPL_TRACE_ERROR("%s: p_buf is null", __func__);
        return;
    }

    uint32_t sig = p_buf->event;

    switch (p_buf->event) {
    case BTC_MEDIA_TASK_SINK_INIT:
        btc_a2dp_sink_thread_init(NULL);
        break;
    case BTC_MEDIA_TASK_SINK_CLEAN_UP:
        btc_a2dp_sink_thread_cleanup(NULL);
        break;
    case BTC_MEDIA_AUDIO_SINK_CFG_UPDATE:
        btc_a2dp_sink_handle_decoder_reset(param);
        break;
    case BTC_MEDIA_AUDIO_SINK_CLEAR_TRACK:
        btc_a2dp_sink_handle_clear_track();
        break;
    case BTC_MEDIA_FLUSH_AA_RX:
        btc_a2dp_sink_rx_flush();
        break;
    default:
        APPL_TRACE_WARNING("media task unhandled evt: 0x%x\n", sig);
    }

    if (param != NULL) {
        osi_free(param);
    }
}

bool btc_a2dp_sink_startup(void)
{
    if (btc_a2dp_sink_state != BTC_A2DP_SINK_STATE_OFF) {
        APPL_TRACE_ERROR("warning : media task already running");
        return false;
    }

#if A2D_DYNAMIC_MEMORY == TRUE
    if ((a2dp_sink_local_param_ptr = (a2dp_sink_local_param_t *)osi_malloc(sizeof(a2dp_sink_local_param_t))) == NULL) {
        APPL_TRACE_ERROR("%s malloc failed!", __func__);
        return false;
    }
    memset((void *)a2dp_sink_local_param_ptr, 0, sizeof(a2dp_sink_local_param_t));
#endif

    APPL_TRACE_EVENT("## A2DP SINK START MEDIA THREAD ##");

    const size_t workqueue_len[] = {A2DP_TASK_WORKQUEUE0_LEN, A2DP_TASK_WORKQUEUE1_LEN};
#if CONFIG_SPIRAM
    a2dp_sink_local_param.btc_aa_snk_task_hdl = osi_thread_create_psram(
                                    A2DP_TASK_NAME,
                                    A2DP_TASK_STACK_SIZE,
                                    A2DP_TASK_PRIO,
                                    A2DP_TASK_PINNED_TO_CORE,
                                    A2DP_TASK_WORKQUEUE_NUM,
                                    workqueue_len);
#else
    a2dp_sink_local_param.btc_aa_snk_task_hdl = osi_thread_create(
                                    A2DP_TASK_NAME,
                                    A2DP_TASK_STACK_SIZE,
                                    A2DP_TASK_PRIO,
                                    A2DP_TASK_PINNED_TO_CORE,
                                    A2DP_TASK_WORKQUEUE_NUM,
                                    workqueue_len);
#endif

    if(!a2dp_sink_local_param.btc_aa_snk_task_hdl) {
        APPL_TRACE_ERROR("%s unable to create a2dp task\n", __func__);
        goto error_exit;
    }

    BT_HDR *p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));
    if (!p_buf) {
        APPL_TRACE_ERROR("%s: no memory", __func__);
        goto error_exit;
    }

    p_buf->event = BTC_MEDIA_TASK_SINK_INIT;
    osi_thread_post(a2dp_sink_local_param.btc_aa_snk_task_hdl,
                    btc_a2dp_sink_ctrl, p_buf, 0, OSI_THREAD_MAX_TIMEOUT);

    APPL_TRACE_EVENT("## A2DP SINK MEDIA THREAD STARTED ##\n");

    return true;

error_exit:;
    APPL_TRACE_ERROR("%s unable to start up media thread\n", __func__);

    if (a2dp_sink_local_param.btc_aa_snk_task_hdl) {
        osi_thread_free(a2dp_sink_local_param.btc_aa_snk_task_hdl);
    }
    a2dp_sink_local_param.btc_aa_snk_task_hdl = NULL;

#if A2D_DYNAMIC_MEMORY == TRUE
    osi_free(a2dp_sink_local_param_ptr);
    a2dp_sink_local_param_ptr = NULL;
#endif

    return false;
}

void btc_a2dp_sink_shutdown(void)
{
    APPL_TRACE_EVENT("## A2DP SINK STOP MEDIA THREAD ##\n");

    // Exit thread
    btc_a2dp_sink_state = BTC_A2DP_SINK_STATE_SHUTTING_DOWN;

    osi_thread_free(a2dp_sink_local_param.btc_aa_snk_task_hdl);

    BT_HDR *p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));
    if (p_buf) {
        p_buf->event = BTC_MEDIA_TASK_SINK_CLEAN_UP;
        osi_thread_post(a2dp_sink_local_param.btc_aa_snk_task_hdl,
                        btc_a2dp_sink_ctrl, p_buf, 0, OSI_THREAD_MAX_TIMEOUT);
    }

    APPL_TRACE_EVENT("## A2DP SINK MEDIA THREAD STARTED ##\n");

    a2dp_sink_local_param.btc_aa_snk_task_hdl = NULL;

#if A2D_DYNAMIC_MEMORY == TRUE
    osi_free(a2dp_sink_local_param_ptr);
    a2dp_sink_local_param_ptr = NULL;
#endif
}

/*****************************************************************************
**
** Function        btc_a2dp_sink_on_idle
**
*******************************************************************************/

void btc_a2dp_sink_on_idle(void)
{
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = TRUE;
    btc_a2dp_sink_rx_flush_req();
    btc_a2dp_sink_clear_track();

    APPL_TRACE_DEBUG("Stopped BT track");
}

/*****************************************************************************
**
** Function        btc_a2dp_sink_on_stopped
**
*******************************************************************************/

void btc_a2dp_sink_on_stopped(tBTA_AV_SUSPEND *p_av)
{
    APPL_TRACE_WARNING("%s: Flushing RX and resetting decoder for clean codec switch", __func__);
    
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = TRUE;
    btc_a2dp_sink_rx_flush_req();
    btc_a2dp_control_set_datachnl_stat(FALSE);
    
    /* Reset decoder state on stop to prepare for possible codec switch on resume.
     * This prevents chipmunk audio when phone stops/resumes with different codec. */
    if (a2dp_sink_local_param.decoder && a2dp_sink_local_param.decoder->decoder_reset) {
        APPL_TRACE_WARNING("%s: Resetting decoder", __func__);
        a2dp_sink_local_param.decoder->decoder_reset();
    }
}

/*****************************************************************************
**
** Function        btc_a2dp_on_suspended
**
*******************************************************************************/

void btc_a2dp_sink_on_suspended(tBTA_AV_SUSPEND *p_av)
{
    APPL_TRACE_WARNING("%s: Stream suspended, flushing RX and resetting decoder", __func__);
    
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = TRUE;
    btc_a2dp_sink_rx_flush_req();
    
    /* Reset decoder state on suspend to prepare for possible codec reconfiguration.
     * AVDTP allows reconfigure only while stream is opened but not started, or suspended.
     * This prevents audio glitches when resuming with potentially different codec settings. */
    if (a2dp_sink_local_param.decoder && a2dp_sink_local_param.decoder->decoder_reset) {
        APPL_TRACE_WARNING("%s: Resetting decoder on suspend", __func__);
        a2dp_sink_local_param.decoder->decoder_reset();
    }
    return;
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_clear_track
 **
 ** Description
 **
 ** Returns          TRUE is success
 **
 *******************************************************************************/
static BOOLEAN btc_a2dp_sink_clear_track(void)
{
    BT_HDR *p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));
    if (!p_buf) {
        APPL_TRACE_ERROR("%s: no memory", __func__);
        return false;
    }

    p_buf->event = BTC_MEDIA_AUDIO_SINK_CLEAR_TRACK;
    osi_thread_post(a2dp_sink_local_param.btc_aa_snk_task_hdl,
                    btc_a2dp_sink_ctrl, p_buf, 0, OSI_THREAD_MAX_TIMEOUT);
    return true;
}

/* when true media task discards any rx frames */
void btc_a2dp_sink_set_rx_flush(BOOLEAN enable)
{
    APPL_TRACE_EVENT("## DROP RX %d ##\n", enable);
    // if (enable == FALSE) {
    //     a2dp_sink_local_param.media_pkt_seq_num.expected_seq_num = 0x1;
    //     a2dp_sink_local_param.media_pkt_seq_num.seq_num_recount = true;
    // }
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = enable;
}

/*****************************************************************************
**
** Function        btc_a2dp_sink_reset_decoder
**
** Description
**
** Returns
**
*******************************************************************************/

void btc_a2dp_sink_reset_decoder(UINT8 *p_av)
{
    APPL_TRACE_EVENT("btc reset decoder");
    APPL_TRACE_DEBUG("btc reset decoder p_codec_info[%x:%x:%x:%x:%x:%x]\n",
                     p_av[1], p_av[2], p_av[3],
                     p_av[4], p_av[5], p_av[6]);

    tBTC_MEDIA_SINK_CFG_UPDATE *p_buf;
    p_buf = osi_malloc(sizeof(tBTC_MEDIA_SINK_CFG_UPDATE));
    if (!p_buf) {
        APPL_TRACE_ERROR("%s: no memory", __func__);
        return;
    }

    p_buf->hdr.event = BTC_MEDIA_AUDIO_SINK_CFG_UPDATE;
    memcpy(p_buf->codec_info, p_av, AVDT_CODEC_SIZE);
    osi_thread_post(a2dp_sink_local_param.btc_aa_snk_task_hdl,
                    btc_a2dp_sink_ctrl, p_buf, 0, OSI_THREAD_MAX_TIMEOUT);
}

static void btc_a2dp_sink_data_ready(UNUSED_ATTR void *context)
{
    BT_HDR *p_msg;
    int nb_of_msgs_to_process = 0;

    if (a2dp_sink_local_param.btc_aa_snk_cb.rx_flush == TRUE) {
        btc_a2dp_sink_flush_q(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
        return;
    }

    nb_of_msgs_to_process = fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
    APPL_TRACE_DEBUG("nb:%d", nb_of_msgs_to_process);
    while (nb_of_msgs_to_process > 0) {
        if (btc_a2dp_sink_state != BTC_A2DP_SINK_STATE_ON){
            return;
        }
        p_msg = (BT_HDR *)fixed_queue_dequeue(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ, 0);
        if ( p_msg == NULL ) {
            APPL_TRACE_DEBUG("Insufficient data in que ");
            break;
        }
        btc_a2dp_sink_handle_inc_media(p_msg);
        osi_free(p_msg);
        nb_of_msgs_to_process--;
    }
    APPL_TRACE_DEBUG(" Process Frames - ");

    if (!fixed_queue_is_empty(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ)) {
        osi_thread_post_event(a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event, OSI_THREAD_MAX_TIMEOUT);
    }
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_handle_decoder_reset
 **
 ** Description      Handle decoder reset on codec configuration change.
 **                  Properly flushes queued data before switching decoders.
 **
 ** Returns          void
 **
 *******************************************************************************/
static void btc_a2dp_sink_handle_decoder_reset(tBTC_MEDIA_SINK_CFG_UPDATE *p_msg)
{
    const tA2DP_DECODER_INTERFACE* decoder = A2DP_GetDecoderInterface(p_msg->codec_info);
    if (!decoder) {
        APPL_TRACE_ERROR("%s: Couldn't get decoder for codec %s", __func__,
                         A2DP_CodecName(p_msg->codec_info));
        return;
    }

    APPL_TRACE_WARNING("%s: Codec config update for %s", __func__, 
                       A2DP_CodecName(p_msg->codec_info));

    /* CRITICAL: Flush any queued encoded data before switching decoders
     * Old encoded data cannot be decoded by new decoder and will cause
     * audio artifacts (chipmunk sound, glitches) if not flushed */
    btc_a2dp_sink_flush_q(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);

    if (decoder != a2dp_sink_local_param.decoder) {
        APPL_TRACE_WARNING("%s: Switching decoder from %p to %p", __func__,
                          a2dp_sink_local_param.decoder, decoder);
        
        // De-initialize previous decoder
        if (a2dp_sink_local_param.decoder && a2dp_sink_local_param.decoder->decoder_cleanup) {
            a2dp_sink_local_param.decoder->decoder_cleanup();
        }

        // Initialize new decoder
        a2dp_sink_local_param.decoder = decoder;
        if (a2dp_sink_local_param.decoder->decoder_init &&
            !a2dp_sink_local_param.decoder->decoder_init(btc_a2d_data_cb_to_app)) {
            APPL_TRACE_ERROR("%s: Decoder failed to initialize", __func__);
            return;
        }
    } else {
        APPL_TRACE_WARNING("%s: Same decoder, resetting state", __func__);
        if (a2dp_sink_local_param.decoder->decoder_reset) {
            a2dp_sink_local_param.decoder->decoder_reset();
        }
    }

    if (a2dp_sink_local_param.decoder->decoder_configure){
        a2dp_sink_local_param.decoder->decoder_configure(p_msg->codec_info);
    }
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_handle_inc_media
 **
 ** Description
 **
 ** Returns          void
 **
 *******************************************************************************/
static void btc_a2dp_sink_handle_inc_media(BT_HDR *p_msg)
{

    /* XXX: Check if the below check is correct, we are checking for peer to be sink when we are sink */
    if (btc_av_get_peer_sep() == AVDT_TSEP_SNK || (a2dp_sink_local_param.btc_aa_snk_cb.rx_flush)) {
        APPL_TRACE_DEBUG(" State Changed happened in this tick ");
        osi_free(p_msg);
        return;
    }

    // ignore data if no one is listening
    if (!btc_a2dp_control_get_datachnl_stat()) {
        osi_free(p_msg);
        return;
    }

    // if (p_msg->layer_specific != a2dp_sink_local_param.media_pkt_seq_num.expected_seq_num) {
    //     /* Because the sequence number of some devices is not recounted */
    //     if (!a2dp_sink_local_param.media_pkt_seq_num.seq_num_recount ||
    //             a2dp_sink_local_param.media_pkt_seq_num.expected_seq_num != 0x1) {
    //         APPL_TRACE_WARNING("Sequence numbers error, recv:0x%x, expect:0x%x, recount:0x%x",
    //                             p_msg->layer_specific, a2dp_sink_local_param.media_pkt_seq_num.expected_seq_num,
    //                             a2dp_sink_local_param.media_pkt_seq_num.seq_num_recount);
    //     }
    // }
    // a2dp_sink_local_param.media_pkt_seq_num.expected_seq_num  = p_msg->layer_specific + 1;
    // a2dp_sink_local_param.media_pkt_seq_num.seq_num_recount = false;

    if (a2dp_sink_local_param.decoder->decode_packet_header) {
        ssize_t res = a2dp_sink_local_param.decoder->decode_packet_header(p_msg);
        if (res < 0) {
            return;
        }
    }

    if (a2dp_sink_local_param.decoder->decode_packet) {
        unsigned char* buf = a2dp_sink_local_param.decode_buf;
        size_t buf_len = BT_A2DP_SINK_BUF_SIZE;
        a2dp_sink_local_param.decoder->decode_packet(p_msg, buf, buf_len);
    }
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_rx_flush_req
 **
 ** Description
 **
 ** Returns          TRUE is success
 **
 *******************************************************************************/
BOOLEAN btc_a2dp_sink_rx_flush_req(void)
{
    if (fixed_queue_is_empty(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ) == TRUE) { /*  Que is already empty */
        return TRUE;
    }

    BT_HDR *p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));
    if (!p_buf) {
        APPL_TRACE_ERROR("%s: no memory", __func__);
        return false;
    }

    p_buf->event = BTC_MEDIA_FLUSH_AA_RX;
    osi_thread_post(a2dp_sink_local_param.btc_aa_snk_task_hdl,
                    btc_a2dp_sink_ctrl, p_buf, 0, OSI_THREAD_MAX_TIMEOUT);
    return true;
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_rx_flush
 **
 ** Description
 **
 ** Returns          void
 **
 *******************************************************************************/
static void btc_a2dp_sink_rx_flush(void)
{
    /* Flush all enqueued SBC  buffers (encoded) */
    APPL_TRACE_DEBUG("btc_a2dp_sink_rx_flush");

    btc_a2dp_sink_flush_q(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_enque_buf
 **
 ** Description      This function is called by the av_co to fill A2DP Sink Queue
 **
 **
 ** Returns          size of the queue
 *******************************************************************************/
UINT8 btc_a2dp_sink_enque_buf(BT_HDR *p_pkt)
{
    if (btc_a2dp_sink_state != BTC_A2DP_SINK_STATE_ON){
        osi_free(p_pkt);
        return 0;
    }

    if (a2dp_sink_local_param.btc_aa_snk_cb.rx_flush == TRUE) { /* Flush enabled, do not enqueue */
        osi_free(p_pkt);
        return fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
    }

    if (fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ) >= MAX_OUTPUT_A2DP_SNK_FRAME_QUEUE_SZ) {
        osi_free(p_pkt);
        APPL_TRACE_WARNING("Pkt dropped\n");
        return fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
    }

    APPL_TRACE_DEBUG("btc_a2dp_sink_enque_buf + ");

    // p_msg->num_frames_to_be_processed = (*((UINT8 *)(p_msg + 1) + p_msg->offset)) & 0x0f;
    fixed_queue_enqueue(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ, p_pkt, FIXED_QUEUE_MAX_TIMEOUT);
    osi_thread_post_event(a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event, OSI_THREAD_MAX_TIMEOUT);

    return fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
}

static void btc_a2dp_sink_handle_clear_track (void)
{
    APPL_TRACE_WARNING("%s: Clearing track and resetting decoder state", __FUNCTION__);
    
    /* Reset decoder state when track is cleared.
     * This ensures clean decoder state when reconnecting or switching codecs. */
    if (a2dp_sink_local_param.decoder && a2dp_sink_local_param.decoder->decoder_reset) {
        APPL_TRACE_WARNING("%s: Resetting decoder on track clear", __FUNCTION__);
        a2dp_sink_local_param.decoder->decoder_reset();
    }
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_flush_q
 **
 ** Description
 **
 ** Returns          void
 **
 *******************************************************************************/
static void btc_a2dp_sink_flush_q(fixed_queue_t *p_q)
{
    while (! fixed_queue_is_empty(p_q)) {
        osi_free(fixed_queue_dequeue(p_q, 0));
    }
}

static void btc_a2dp_sink_thread_init(UNUSED_ATTR void *context)
{
    APPL_TRACE_EVENT("%s\n", __func__);
    memset(&a2dp_sink_local_param.btc_aa_snk_cb, 0, sizeof(a2dp_sink_local_param.btc_aa_snk_cb));

    /* Allocate decode buffer from PSRAM for hi-res codecs (LDAC, aptX) */
#if CONFIG_SPIRAM
    a2dp_sink_local_param.decode_buf = (unsigned char *)heap_caps_malloc(BT_A2DP_SINK_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (a2dp_sink_local_param.decode_buf == NULL) {
        APPL_TRACE_ERROR("Failed to allocate decode buffer from PSRAM, trying internal RAM");
        a2dp_sink_local_param.decode_buf = (unsigned char *)osi_malloc(BT_A2DP_SINK_BUF_SIZE);
    }
#else
    a2dp_sink_local_param.decode_buf = (unsigned char *)osi_malloc(BT_A2DP_SINK_BUF_SIZE);
#endif
    if (a2dp_sink_local_param.decode_buf == NULL) {
        APPL_TRACE_ERROR("Failed to allocate decode buffer!");
        return;
    }

    btc_a2dp_sink_state = BTC_A2DP_SINK_STATE_ON;

    struct osi_event *data_event = osi_event_create(btc_a2dp_sink_data_ready, NULL);
    assert (data_event != NULL);
    osi_event_bind(data_event, a2dp_sink_local_param.btc_aa_snk_task_hdl, BTC_A2DP_SNK_DATA_QUEUE_IDX);
    a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event = data_event;

    a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ = fixed_queue_new(QUEUE_SIZE_MAX);

    btc_a2dp_control_init();
}

static void btc_a2dp_sink_thread_cleanup(UNUSED_ATTR void *context)
{

    if (a2dp_sink_local_param.decoder && a2dp_sink_local_param.decoder->decoder_cleanup) {
        a2dp_sink_local_param.decoder->decoder_cleanup();
        a2dp_sink_local_param.decoder = NULL;
    }

    /* Free decode buffer */
    if (a2dp_sink_local_param.decode_buf != NULL) {
#if CONFIG_SPIRAM
        heap_caps_free(a2dp_sink_local_param.decode_buf);
#else
        osi_free(a2dp_sink_local_param.decode_buf);
#endif
        a2dp_sink_local_param.decode_buf = NULL;
    }

    btc_a2dp_control_set_datachnl_stat(FALSE);
    /* Clear task flag */
    btc_a2dp_sink_state = BTC_A2DP_SINK_STATE_OFF;

    btc_a2dp_control_cleanup();

    fixed_queue_free(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ, osi_free_func);

    a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ = NULL;

    osi_event_delete(a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event);
    a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event = NULL;
}

#endif /* (BTC_AV_SINK_INCLUDED == TRUE) && (BTC_AV_EXT_CODEC == FALSE) */
