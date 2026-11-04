#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "ldacBT.h"
#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
static HANDLE_LDAC_BT codec;
EXPORT int reference_open(unsigned rate,unsigned mode){
    codec=ldacBT_get_handle();if(!codec)return -1;
    return ldacBT_init_handle_decode(codec,(int)mode,(int)rate,0,0,0);
}
EXPORT void reference_close(void){if(codec){ldacBT_close_handle(codec);ldacBT_free_handle(codec);codec=0;}}
EXPORT int reference_decode(const uint8_t *data,unsigned size,uint8_t *pcm,unsigned *used,unsigned *written){
    unsigned char buffer[1030]={0};if(size>515)return -1;
    memcpy(buffer,data,size);int in=0,out=0;
    int r=ldacBT_decode(codec,buffer,pcm,LDACBT_SMPL_FMT_S32,(int)size,&in,&out);
    *used=(unsigned)in;*written=(unsigned)out;return r;
}
