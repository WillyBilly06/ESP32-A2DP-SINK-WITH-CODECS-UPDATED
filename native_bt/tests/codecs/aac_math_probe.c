#include "aacdec.h"
#include "aaccommon.h"
#include "coder.h"
#include "assembly.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API __attribute__((visibility("default")))
#endif
static HAACDecoder decoder;
static uint8_t *allocation;
API int math_open(int rate,int channels) {
    size_t size=sizeof(AACDecInfo)+sizeof(PSInfoBase)+64u;
    allocation=calloc(1,size); if(!allocation) return -1;
    decoder=AACInitDecoderPre(allocation,(int)size);
    if(!decoder) { free(allocation);allocation=NULL;return -2; }
    AACFrameInfo info={0}; info.nChans=channels; info.sampRateCore=rate; info.profile=AAC_PROFILE_LC;
    return AACSetRawBlockParams(decoder,0,&info);
}
API int math_decode(const uint8_t *input,int size,int16_t *output) {
    unsigned char *p=(unsigned char *)input; int remain=size;
    int rc=AACDecode(decoder,&p,&remain,(short *)output);
    return rc ? rc : remain ? -99 : 0;
}
API void math_close(void) { decoder=NULL;free(allocation);allocation=NULL; }
API int math_mul(int a,int b) { return MULSHIFT32(a,b); }
