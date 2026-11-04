#include "probe_diagnostic_stats.h"
bool probe_diagnostic_stats(uint32_t *samples,unsigned count,uint32_t budget,probe_diagnostic_stats_t *result){
    if(!samples || !result || !count || count>PROBE_DIAGNOSTIC_SAMPLES_MAX || !budget)return false;
    *result=(probe_diagnostic_stats_t){0};
    for(unsigned i=0;i<count;++i){
        result->sum+=samples[i];if(samples[i]>=budget)++result->at_or_above_budget;
        if(i){uint32_t value=samples[i];unsigned j=i;
            while(j && samples[j-1]>value){samples[j]=samples[j-1];--j;}samples[j]=value;}
    }
    result->minimum=samples[0];result->maximum=samples[count-1];result->mean=(uint32_t)(result->sum/count);
    result->p50=samples[(count-1)*50u/100u];
    result->p95=samples[((count*95u+99u)/100u)-1u];
    result->p99=samples[((count*99u+99u)/100u)-1u];
    return true;
}
