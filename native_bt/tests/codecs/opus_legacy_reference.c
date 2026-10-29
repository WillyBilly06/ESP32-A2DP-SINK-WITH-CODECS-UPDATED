/* Test-only shim to exercise the existing SDK wrapper with the same libopus. */
#include <stdlib.h>
#include <string.h>
#include "opus_legacy_stub.h"
#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
void *osi_malloc(size_t n){return malloc(n);}
void *osi_malloc_base(size_t n){return malloc(n);}
void osi_free(void *p){free(p);}
int A2DP_ParseInfoOpus(tA2DP_OPUS_CIE *c,const uint8_t *p,bool capabilities){
    (void)capabilities;memset(c,0,sizeof(*c));
    if(!p || p[1] || p[2]!=255)return 1;
    if(p[0]==9 && p[3]==0xe0 && p[4]==0 && p[5]==0 && p[6]==0 && p[7]==1 && p[8]==0){
        c->variant=0;c->channelMode=p[9]&7;c->frameSize=p[9]&0x18;c->samplingFreq=p[9]&0x80;return 0;
    }
    if(p[0]==26 && p[3]==0xf1 && p[4]==5 && p[5]==0 && p[6]==0 && p[7]==5 && p[8]==0x10){
        c->variant=1;c->channels=p[9];c->coupled_streams=p[10];c->frame_duration=p[15];return 0;
    }
    return 1;
}
#include "a2dp_vendor_opus_decoder.c"
static uint8_t *output;
static unsigned written;
static void on_pcm(uint8_t *p,uint32_t n){if(output && n<=8192){memcpy(output,p,n);written=n;}}
EXPORT int reference_open(const uint8_t *cie,unsigned size){
    uint8_t codec[27];if(size>26)return -1;codec[0]=(uint8_t)size;memcpy(codec+1,cie,size);
    if(!a2dp_opus_decoder_init(on_pcm))return -2;a2dp_opus_decoder_configure(codec);
    return a2dp_opus_decoder_cb.decoder?0:-3;
}
EXPORT void reference_close(void){a2dp_opus_decoder_cleanup();}
EXPORT int reference_packet(const uint8_t *payload,unsigned size,uint8_t *pcm,unsigned *n){
    struct {BT_HDR h;uint8_t bytes[8192];} packet;
    if(size+12>sizeof(packet.bytes))return -1;
    packet.h=(BT_HDR){0,(uint16_t)(size+12),0,0};memset(packet.bytes,0,12);memcpy(packet.bytes+12,payload,size);
    output=pcm;written=0;uint8_t scratch[8192];
    int r=a2dp_opus_decoder_decode_packet_header(&packet.h)<0?-2:
          (a2dp_opus_decoder_decode_packet(&packet.h,scratch,sizeof(scratch))?0:-3);
    *n=written;output=0;return r;
}
