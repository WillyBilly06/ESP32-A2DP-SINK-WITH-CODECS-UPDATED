#include <stddef.h>
#include <stdint.h>
#include "probe_diagnostic_stats.h"
#define REQUIRE(x) do {if(!(x))return __LINE__;}while(0)
#ifdef _WIN32
__declspec(dllexport)
#endif
int codec_diagnostic_stats_tests(void){
    probe_diagnostic_stats_t result;
    uint32_t values[]={5000,4999,0,5001,1,4998};
    REQUIRE(probe_diagnostic_stats(values,6,5000,&result));
    REQUIRE(result.sum==19999 && result.mean==3333 && result.minimum==0 && result.maximum==5001);
    REQUIRE(result.p50==4998 && result.p95==5001 && result.p99==5001 && result.at_or_above_budget==2);
    REQUIRE(!probe_diagnostic_stats(NULL,6,5000,&result));
    REQUIRE(!probe_diagnostic_stats(values,0,5000,&result));
    REQUIRE(!probe_diagnostic_stats(values,97,5000,&result));
    REQUIRE(!probe_diagnostic_stats(values,6,0,&result));
    REQUIRE(!probe_diagnostic_stats(values,6,5000,NULL));
    struct {uint32_t before,x[96],after;} guarded={.before=0x12345678,.after=0x87654321};
    for(unsigned i=0;i<96;++i)guarded.x[i]=96-i;
    REQUIRE(probe_diagnostic_stats(guarded.x,96,95,&result));
    REQUIRE(guarded.before==0x12345678 && guarded.after==0x87654321 && result.sum==4656);
    REQUIRE(result.p50==48 && result.p95==92 && result.p99==96 && result.at_or_above_budget==2);
    uint32_t large[]={UINT32_MAX,UINT32_MAX,3};
    REQUIRE(probe_diagnostic_stats(large,3,UINT32_MAX,&result));
    REQUIRE(result.sum==UINT64_C(8589934593) && result.at_or_above_budget==2);
    uint32_t single=77;
    REQUIRE(probe_diagnostic_stats(&single,1,78,&result) && result.mean==77 && result.p99==77 && !result.at_or_above_budget);
    return 0;
}
