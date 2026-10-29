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
