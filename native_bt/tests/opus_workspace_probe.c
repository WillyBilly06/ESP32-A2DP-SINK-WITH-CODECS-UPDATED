#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
static size_t fragment_live;
static void *fragment_allocation;
static int fail_next_fragment;
void *osi_malloc(size_t size) {
    if (fail_next_fragment) { fail_next_fragment = 0; return NULL; }
    void *pointer = malloc(size);
    if (pointer) { if (fragment_allocation) abort(); fragment_allocation=pointer; fragment_live=size; }
    return pointer;
}
void osi_free(void *pointer) {
    if (!pointer) return;
    if (pointer != fragment_allocation) abort();
    fragment_allocation=NULL; fragment_live=0; free(pointer);
}
#include "a2dp_vendor_opus_decoder.c"
static uint8_t cie[64], decoded[8000];
static uint32_t decoded_bytes;
static uint16_t active_samples;
static uint8_t active_channels;
static int canary_failed;
static union { uint64_t align; uint8_t bytes[8016]; } scratch;
static struct { BT_HDR header; uint8_t bytes[8192]; } packet;
static void on_pcm(uint8_t *data, uint32_t size) {
    if (size > sizeof(decoded)) abort();
    memcpy(decoded,data,size); decoded_bytes=size;
}
int probe_open(int variant, int channels, int samples) {
    a2dp_opus_decoder_cleanup();
    tA2DP_OPUS_CIE ie={0};
    ie.variant=(uint8_t)variant;
    if (variant == A2DP_OPUS_VARIANT_ANDROID) {
        ie.vendorId=A2DP_OPUS_ANDROID_VENDOR_ID; ie.codecId=A2DP_OPUS_ANDROID_CODEC_ID;
        ie.channelMode=channels==1?A2DP_OPUS_CHANNEL_MODE_MONO:A2DP_OPUS_CHANNEL_MODE_STEREO;
        ie.frameSize=samples==480?A2DP_OPUS_10MS_FRAMESIZE:A2DP_OPUS_20MS_FRAMESIZE;
        ie.samplingFreq=A2DP_OPUS_SAMPLING_FREQ_48000;
    } else {
        ie.vendorId=A2DP_OPUS_PIPEWIRE_VENDOR_ID; ie.codecId=A2DP_OPUS_PIPEWIRE_CODEC_ID;
        ie.channels=(uint8_t)channels; ie.coupled_streams=channels==2;
        ie.audio_location=channels==2?3:1;
        ie.frame_duration=samples==120?1:samples==240?2:samples==480?4:samples==960?8:16;
    }
    if (A2DP_BuildInfoOpus(0,&ie,cie)!=A2D_SUCCESS || !a2dp_opus_decoder_init(on_pcm)) return -1;
    a2dp_opus_decoder_configure(cie);
    if (!a2dp_opus_decoder_cb.decoder) return -2;
    active_samples=(uint16_t)samples; active_channels=(uint8_t)channels;
    return 0;
}
void probe_close(void) { a2dp_opus_decoder_cleanup(); }
size_t probe_scratch_bytes(void) {
#if OPUS_CANDIDATE
    return a2dp_opus_decoder_buffer_size(cie);
#else
    return 8000;
#endif
}
size_t probe_fragment_bytes(void) {
#if OPUS_CANDIDATE
    return a2dp_opus_decoder_cb.fragment?6144:0;
#else
    return sizeof(a2dp_opus_decoder_cb.fragment);
#endif
}
size_t probe_fragment_live(void) { return fragment_live; }
void probe_fail_next(void) { fail_next_fragment=1; }
int probe_reinit(void) { return a2dp_opus_decoder_init(on_pcm)?0:-1; }
int probe_packet(const uint8_t *data, uint16_t size, uint8_t payload_flags, int short_buffer) {
    if (size+13u>sizeof(packet.bytes)) return -9;
    packet.header.offset=0; packet.header.len=(uint16_t)(size+13u);
    memset(packet.bytes,0,13); packet.bytes[0]=0x80; packet.bytes[12]=payload_flags;
    memcpy(packet.bytes+13,data,size);
    decoded_bytes=0; canary_failed=0;
    if (a2dp_opus_decoder_decode_packet_header(&packet.header)<0) return -1;
    size_t capacity=probe_scratch_bytes();
    if (short_buffer && capacity) --capacity;
    memset(scratch.bytes,0xa5,sizeof(scratch.bytes));
    int result=a2dp_opus_decoder_decode_packet(&packet.header,scratch.bytes,capacity)?0:-2;
    for(size_t i=capacity;i<sizeof(scratch.bytes);++i) if(scratch.bytes[i]!=0xa5) canary_failed=1;
    return canary_failed?-3:result;
}
uint32_t probe_output(uint8_t *output) { memcpy(output,decoded,decoded_bytes); return decoded_bytes; }
#ifdef OPUS_SANITIZER_MAIN
int main(void) {
    static int16_t input[1920*2];
    static unsigned char encoded[4000];
    for (int channels=1;channels<=2;++channels) for(int samples=120;samples<=1920;samples*=2) {
        int error=0;
        OpusEncoder *encoder=opus_encoder_create(48000,channels,OPUS_APPLICATION_AUDIO,&error);
        if(!encoder || error || probe_open(1,channels,samples)) return 1;
        for(int i=0;i<samples*channels;++i) input[i]=(int16_t)((i*97)%20000-10000);
        int size=opus_encode(encoder,input,samples,encoded,sizeof(encoded));
        if(size<=0 || probe_packet(encoded,(uint16_t)size,1,0) || decoded_bytes!=(uint32_t)samples*channels*2) return 2;
        if(probe_packet(encoded,(uint16_t)(size/2),0xc2,0) || decoded_bytes) return 3;
        if(probe_packet(encoded+size/2,(uint16_t)(size-size/2),0xa1,0) || decoded_bytes!=(uint32_t)samples*channels*2) return 4;
        probe_close();
        if(fragment_live) return 5;
        opus_encoder_destroy(encoder);
    }
#if OPUS_CANDIDATE
    if(probe_fragment_bytes() || probe_open(1,2,960)) return 6;
    probe_fail_next();
    if(probe_packet(encoded,2,0xc2,0)!=-2 || fragment_live) return 7;
    if(probe_reinit() || fragment_live) return 8;
    probe_close();
#endif
    return 0;
}
#endif
