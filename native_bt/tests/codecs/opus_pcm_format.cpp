#define NATIVE_A2DP_PCM_FORMAT_ONLY
#include "a2dp_sink_native.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
static bool equal(const NativeOpusPcmFormat &p,unsigned ch) {
    return p.rate==48000u && p.bits==16u && p.channels==ch;
}
extern "C" EXPORT int opus_pcm_android_vectors(void) {
    uint8_t raw[]={0xe0,0,0,0,1,0,0x92}; NativeOpusPcmFormat out={0,0,0};
    CHECK(nativeOpusPcmFormat(raw,sizeof(raw),&out)==NativeOpusCieResult::Android && equal(out,2));
    raw[6]=0x91; CHECK(nativeOpusPcmFormat(raw,sizeof(raw),&out)==NativeOpusCieResult::Android && equal(out,1));
    raw[6]=0x8a; CHECK(nativeOpusPcmFormat(raw,sizeof(raw),&out)==NativeOpusCieResult::Android && equal(out,2));
    raw[6]=0x89; CHECK(nativeOpusPcmFormat(raw,sizeof(raw),&out)==NativeOpusCieResult::Android && equal(out,1));
    return 0;
}
extern "C" EXPORT int opus_pcm_opus05_vectors(void) {
    uint8_t raw[24]={0xf1,0x05,0,0,0x05,0x10,2,1,3,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0};
    NativeOpusPcmFormat out={0,0,0};
    CHECK(nativeOpusPcmFormat(raw,sizeof(raw),&out)==NativeOpusCieResult::Opus05 && equal(out,2));
    raw[6]=1;raw[7]=0;raw[8]=1;raw[12]=4;
    CHECK(nativeOpusPcmFormat(raw,sizeof(raw),&out)==NativeOpusCieResult::Opus05 && equal(out,1));
    for(unsigned i=0;i<5;i++) {raw[12]=(uint8_t)(1u<<i);CHECK(nativeOpusPcmFormat(raw,sizeof(raw),&out)==NativeOpusCieResult::Opus05 && equal(out,1));}
    return 0;
}
extern "C" EXPORT int opus_pcm_invalid_is_not_guessed(void) {
    uint8_t android[]={0xe0,0,0,0,1,0,0x92}; NativeOpusPcmFormat out={123,9,7};
    CHECK(nativeOpusPcmFormat(android,6,&out)==NativeOpusCieResult::Invalid && out.rate==123 && out.bits==9 && out.channels==7);
    CHECK(nativeOpusPcmFormat(nullptr,0,&out)==NativeOpusCieResult::Invalid);
    CHECK(nativeOpusPcmFormat(android,sizeof(android),nullptr)==NativeOpusCieResult::Invalid);
    const uint8_t invalid[]={0x12,0x80,0x94,0x93,0x9a,0xa2};
    for(unsigned i=0;i<sizeof(invalid);i++) {android[6]=invalid[i];CHECK(nativeOpusPcmFormat(android,sizeof(android),&out)==NativeOpusCieResult::Invalid);}
    uint8_t pw[24]={0xf1,5,0,0,5,0x10,2,1,3,0,0,0,8};
    CHECK(nativeOpusPcmFormat(pw,12,&out)==NativeOpusCieResult::Invalid);
    pw[7]=0;CHECK(nativeOpusPcmFormat(pw,sizeof(pw),&out)==NativeOpusCieResult::Invalid);
    pw[6]=1;pw[7]=1;CHECK(nativeOpusPcmFormat(pw,sizeof(pw),&out)==NativeOpusCieResult::Invalid);
    pw[7]=0;pw[12]=3;CHECK(nativeOpusPcmFormat(pw,sizeof(pw),&out)==NativeOpusCieResult::Invalid);
    pw[12]=0;CHECK(nativeOpusPcmFormat(pw,sizeof(pw),&out)==NativeOpusCieResult::Invalid);
    CHECK(out.rate==123 && out.bits==9 && out.channels==7);return 0;
}
extern "C" EXPORT int opus_pcm_other_vendor_unchanged(void) {
    uint8_t raw[]={0x4f,0,0,0,1,0,0x22,0};NativeOpusPcmFormat out={123,9,7};
    CHECK(nativeOpusPcmFormat(raw,sizeof(raw),&out)==NativeOpusCieResult::Other);
    CHECK(out.rate==123 && out.bits==9 && out.channels==7);
    raw[0]=0xe0;raw[4]=2;CHECK(nativeOpusPcmFormat(raw,sizeof(raw),&out)==NativeOpusCieResult::Other);
    return 0;
}
extern "C" EXPORT int opus_pcm_sample_stride(void) {
    uint8_t android[]={0xe0,0,0,0,1,0,0x92};NativeOpusPcmFormat f={0,0,0};
    CHECK(nativeOpusPcmFormat(android,sizeof(android),&f)==NativeOpusCieResult::Android);
    const uint8_t pcm[]={0x00,0x80,0xff,0x7f,0x01,0x00,0xff,0xff,0x34,0x12,0xcc,0xed,0x00,0x00,0x00,0x40};
    CHECK(sizeof(pcm)/(f.bits/8u*f.channels)==4);
    const int expected[]={-32768,32767,1,-1,4660,-4660,0,16384};
    for(unsigned i=0;i<8;i++) {
        unsigned u=(unsigned)pcm[i*2] | ((unsigned)pcm[i*2+1]<<8);
        int sample=(u&0x8000u)?(int)u-65536:(int)u;CHECK(sample==expected[i]);
    }
    return 0;
}
extern "C" EXPORT int opus_pcm_android_exhaustive(void) {
    uint8_t raw[]={0xe0,0,0,0,1,0,0};
    for(unsigned b=0;b<256;b++) {
        raw[6]=(uint8_t)b;NativeOpusPcmFormat f={123,9,7};
        unsigned mode=b&7u,frame=b&0x18u;
        bool expected=(b&0x80u)!=0 && (mode==1 || mode==2) && (frame==8 || frame==16);
        NativeOpusCieResult r=nativeOpusPcmFormat(raw,sizeof(raw),&f);
        CHECK((r==NativeOpusCieResult::Android)==expected);
        if(expected)CHECK(equal(f,mode));else CHECK(r==NativeOpusCieResult::Invalid && f.rate==123 && f.bits==9 && f.channels==7);
    }
    return 0;
}
