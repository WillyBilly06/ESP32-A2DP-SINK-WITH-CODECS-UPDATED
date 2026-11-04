#include "lhdcv5_perf_helper.h"
#include "dbt_lhdcv5_codec.h"
#include <stdlib.h>
#include <time.h>
#define DBT_PROBE_LHDC_BUILD 1
#include "../../esp32_probe/main/probe_codec_vectors.h"
#ifdef _WIN32
#define EX __declspec(dllexport)
#else
#define EX __attribute__((visibility("default")))
#endif
int g_pkframe,g_nzc_formula;
static unsigned live;
static uint8_t *allocate(void *context,uint32_t n){(void)context;uint8_t *p=malloc(n);if(p)++live;return p;}
static void release(void *context,uint8_t *p){(void)context;if(p){--live;free(p);}}
static uint64_t micros(void *context){(void)context;struct timespec t;timespec_get(&t,TIME_UTC);return (uint64_t)t.tv_sec*1000000u+(uint64_t)t.tv_nsec/1000u;}
EX int lhdc_perf_fixture(unsigned which,unsigned warmup,unsigned measured,dbt_lhdc_perf_result_t *result){
    if(which>=4 || warmup>1000 || measured>1000)return DBT_INVALID;
    const probe_codec_vector_t *v=&probe_codec_vectors[which];dbt_lhdc_perf_packet_t packets[4];
    for(unsigned i=0;i<v->packet_count;i++)packets[i]=(dbt_lhdc_perf_packet_t){v->packet[i],v->bytes[i]};
    dbt_lhdc_perf_hooks_t hooks={0,allocate,release,micros};
    dbt_result_t r=dbt_lhdc_perf_run(hooks,v->cie,v->cie_size,packets,v->packet_count,(uint16_t)warmup,(uint16_t)measured,result);
    if(live)return DBT_INVALID;return r;
}
EX int lhdc_perf_fir(unsigned rate,unsigned samples,uint64_t *us,uint32_t *checksum){
    return dbt_lhdc_perf_resampler(micros,0,rate,samples,us,checksum);
}
