/* Test-only raw reference using the unmodified sibling LHDC decoder. */
#include "lhdc_dec.h"
#include "lhdc_entropy_dec.h"
#include "lhdc_imdct.h"
#include <stdlib.h>
#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
int g_nzc_formula=0;
int g_pkframe=0;
static void *workspace;
static lhdc_decoder_t *decoder;
EXPORT int reference_open(unsigned rate,unsigned depth){
    size_t size=lhdc_dec_get_workspace_size(rate,5);
    workspace=calloc(1,size);if(!workspace)return -1;
    lhdc_dec_config_t config={(lhdc_dec_sample_rate_t)rate,(lhdc_dec_bitdepth_t)depth,LHDC_DEC_FRAME_5MS,2,1536,0};
    decoder=lhdc_dec_init(workspace,&config);return decoder?0:-1;
}
EXPORT int reference_frame(const uint8_t *frame,unsigned size,void *pcm,unsigned samples,unsigned *consumed,unsigned *generated){
    size_t used=0;uint32_t output=0;lhdc_dec_frame_info_t info={0};
    int result=lhdc_dec_decode_frame(decoder,frame,size,pcm,samples,&used,&output,&info);
    *consumed=(unsigned)used;*generated=output;
    return result;
}
EXPORT void reference_close(void){
    if(!workspace)return;
    lhdc_dec_deinit((lhdc_decoder_t *)workspace);
    lhdc_dec_free_slot_scratch();lhdc_entropy_free();lhdc_dec_free_window();
    lhdc_imdct_free_480();lhdc_imdct_free_960();lhdc_imdct_free_1920();lhdc_imdct_free_cos();
    free(workspace);workspace=0;decoder=0;
}
