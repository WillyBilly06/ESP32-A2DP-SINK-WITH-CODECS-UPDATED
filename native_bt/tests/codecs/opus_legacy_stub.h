#ifndef DBT_TEST_OPUS_LEGACY_STUB_H
#define DBT_TEST_OPUS_LEGACY_STUB_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>
#define OPUS_DEC_INCLUDED 1
#define TRUE 1
#define A2D_SUCCESS 0
#define APPL_TRACE_ERROR(...) ((void)0)
#define LOG_INFO(...) ((void)0)
#define LOG_ERROR(...) ((void)0)
#include "stack/a2dp_vendor_opus_constants.h"
typedef struct {uint16_t event,len,offset,layer_specific;} BT_HDR;
typedef void (*decoded_data_callback_t)(uint8_t *,uint32_t);
typedef struct {
    uint32_t vendorId;uint16_t codecId;uint8_t variant,channelMode,frameSize,samplingFreq;
    uint8_t channels,coupled_streams;uint32_t audio_location;uint8_t frame_duration;
} tA2DP_OPUS_CIE;
struct __attribute__((packed)) media_packet_header {
    uint8_t cc:4,x:1,p:1,v:2;uint8_t pt:7,marker:1;uint16_t seq;uint32_t timestamp,ssrc;
};
struct __attribute__((packed)) media_payload_header {
    uint8_t frame_count:4,rfa0:1,is_last_fragment:1,is_first_fragment:1,is_fragmented:1;
};
int A2DP_ParseInfoOpus(tA2DP_OPUS_CIE *,const uint8_t *,bool);
void *osi_malloc(size_t);
void *osi_malloc_base(size_t);
void osi_free(void *);
void a2dp_opus_decoder_cleanup(void);
#endif
