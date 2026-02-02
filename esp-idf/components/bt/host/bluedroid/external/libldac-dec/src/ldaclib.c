#include "ldac.h"

#if CONFIG_USED_CSI_DSP
    #ifdef CONFIG_SCALAR_F64
        #undef CONFIG_SCALAR_F64
    #endif
    #define CONFIG_SCALAR_F64 0
#endif

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


