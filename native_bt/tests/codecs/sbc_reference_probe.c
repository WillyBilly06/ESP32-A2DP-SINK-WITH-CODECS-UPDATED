#include <string.h>
#include "common/bt_target.h"
#include "oi_codec_sbc.h"
#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
static OI_CODEC_SBC_DECODER_CONTEXT reference;
static OI_UINT32 data[CODEC_DATA_WORDS(2,SBC_CODEC_FAST_FILTER_BUFFERS)];
EXPORT int sbc_reference_open(unsigned channels){
    memset(data,0,sizeof(data));
    return (int)OI_CODEC_SBC_DecoderReset(&reference,data,sizeof(data),2,(OI_UINT8)channels,FALSE,FALSE);
}
EXPORT int sbc_reference(const uint8_t *frame,unsigned size,int16_t *pcm,unsigned *written){
    const OI_BYTE *cursor=frame;OI_UINT32 left=size,bytes=512;
    OI_STATUS r=OI_CODEC_SBC_DecodeFrame(&reference,&cursor,&left,pcm,&bytes);*written=bytes;
    return r==OI_OK && !left && cursor==frame+size?0:-1;
}
