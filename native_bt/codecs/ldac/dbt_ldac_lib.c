/* Standalone SDK codec units, with pinned storage/bit-reader ports. */
#include <stdlib.h>
#include <malloc.h>
#include "dbt_ldac_port.h"
#define malloc dbt_ldac_mem_alloc
#define calloc dbt_ldac_mem_calloc
#define free dbt_ldac_mem_free
#include "ldac.h"
#include "tables_ldac.o.c"
#include "tables_sigproc_ldac.o.c"
#include "setpcm_ldac.o.c"
#include "bitalloc_sub_ldac.o.c"
#include "memory_ldac.o.c"
#include "ldaclib_api.o.c"
#include "imdct_ldac.o.c"
#include "dequant_ldac.o.c"
#include "unpack_ldac.o.c"
#include "decode_ldac.o.c"

_Static_assert(sizeof(SCALAR)==4,"LDAC working storage requires the reviewed scalar width");
uint32_t dbt_ldac_work_bytes(unsigned samples,unsigned channel_mode){
    if(samples!=128u && samples!=256u)return 0;
    unsigned cci;
    if(channel_mode==LDACBT_CHANNEL_MODE_MONO)cci=0;
    else if(channel_mode==LDACBT_CHANNEL_MODE_DUAL_CHANNEL)cci=1;
    else if(channel_mode==LDACBT_CHANNEL_MODE_STEREO)cci=2;
    else return 0;
    unsigned channels=(unsigned)gaa_block_setting_ldac[cci][0];
    unsigned blocks=(unsigned)gaa_block_setting_ldac[cci][1];
    if(!channels || channels>2u || !blocks || blocks>2u)return 0;
    /* Exactly the generated init_decode_ldac allocation graph, including the
     * SDK caller-memory alignment rule for each block. No maximum-rate pool. */
    uint64_t ac=align_ldac(sizeof(AC)+2u*samples*sizeof(int16_t));
    uint64_t sub=align_ldac(sizeof(ACSUB)+3u*samples*sizeof(SCALAR));
    uint64_t bytes=channels*(ac+sub)+blocks*align_ldac(sizeof(AB));
    return bytes>UINT32_MAX?0:(uint32_t)bytes;
}
