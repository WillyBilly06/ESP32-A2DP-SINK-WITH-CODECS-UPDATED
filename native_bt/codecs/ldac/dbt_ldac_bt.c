/* This translation unit uses the original SDK implementation plus the pinned
 * negotiated-size handle unit. The standalone SDK ldacBT.c must not also be
 * linked into this component. */
#include <stdlib.h>
#include <malloc.h>
#include "dbt_ldac_port.h"
#define malloc dbt_ldac_mem_alloc
#define calloc dbt_ldac_mem_calloc
#define free dbt_ldac_mem_free
#include "ldac.h"
#include "ldacBT_internal.h"
#include "ldacBT_internal.o.c"
#include "ldacBT_api.o.c"

int dbt_ldac_bind_pcm(HANDLE_LDAC_BT handle,void *pcm,unsigned samples,unsigned channels){
    if(!handle || !pcm || (channels!=1 && channels!=2) || (samples!=128 && samples!=256))return 0;
    handle->ap_pcm[0]=(char *)pcm;
    handle->ap_pcm[1]=channels==2?(char *)pcm+samples*4u:NULL;
    handle->pp_pcm=handle->ap_pcm;return 1;
}
