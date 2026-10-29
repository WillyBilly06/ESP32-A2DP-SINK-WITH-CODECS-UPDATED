#include <stdlib.h>
#include <stdint.h>
#if defined(_WIN32)
#include <malloc.h>
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
void *heap_caps_aligned_alloc(size_t alignment,size_t size,uint32_t caps){
    (void)caps;
#if defined(_WIN32)
    return _aligned_malloc(size,alignment);
#else
    void *p=0;return posix_memalign(&p,alignment,size)?0:p;
#endif
}
void heap_caps_free(void *p){
#if defined(_WIN32)
    _aligned_free(p);
#else
    free(p);
#endif
}
#include "aac_decoder_helix.c"
static aac_decoder_t decoder;
EXPORT int baseline_open(int rate,int channels){int r=aac_decoder_init(&decoder);return r?r:aac_decoder_configure(&decoder,rate,channels);}
EXPORT int baseline_decode(const uint8_t *data,int size,int16_t *out,int *frames){return aac_decoder_decode(&decoder,data,size,out,frames);}
EXPORT void baseline_close(void){aac_decoder_deinit(&decoder);}
