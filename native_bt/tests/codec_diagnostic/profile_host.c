#include <stddef.h>
#include <stdint.h>
#include "probe_lhdc_profile.h"
#define REQUIRE(x) do {if(!(x))return __LINE__;}while(0)
#ifdef _WIN32
__declspec(dllexport)
#endif
int codec_diagnostic_profile_tests(void){
    probe_lhdc_profile_t result;
    probe_lhdc_profile_reset(false);
    dbt_lhdc_stage_record(1,2,3,4,5,6);
    REQUIRE(!probe_lhdc_profile_finish(&result) && !result.channel_calls && !result.sum_cycles[0]);
    probe_lhdc_profile_reset(true);
    REQUIRE(!probe_lhdc_profile_finish(NULL));
    dbt_lhdc_stage_record(UINT32_MAX,23,10,20,30,40);
    dbt_lhdc_stage_record(1,9,11,19,25,39);
    REQUIRE(probe_lhdc_profile_finish(&result));
    REQUIRE(result.channel_calls==2 && result.sum_cycles[0]==UINT64_C(4294967296) && result.max_cycles[0]==UINT32_MAX);
    REQUIRE(result.sum_cycles[1]==32 && result.max_cycles[1]==23);
    REQUIRE(result.sum_cycles[5]==79 && result.max_cycles[5]==40);
    dbt_lhdc_stage_record(10,10,10,10,10,10);
    REQUIRE(probe_lhdc_profile_finish(&result) && result.channel_calls==2 && result.sum_cycles[1]==32);
    probe_lhdc_profile_reset(false);
    REQUIRE(!probe_lhdc_profile_finish(&result) && !result.channel_calls);
    return 0;
}
