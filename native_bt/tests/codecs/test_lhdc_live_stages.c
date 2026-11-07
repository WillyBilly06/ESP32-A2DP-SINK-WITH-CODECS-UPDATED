#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#define PROBE_LHDC_PROFILE_TEST 1
#include "../../esp32_probe/main/probe_lhdc_profile.c"
static unsigned holds;
void probe_lhdc_profile_test_hold(void){probe_lhdc_live_t v,old;memset(&v,0x5a,sizeof(v));old=v;assert(!probe_lhdc_profile_snapshot(&v));assert(!memcmp(&v,&old,sizeof(v)));++holds;}
#define PROBE_SNAPSHOT_LOG(...) printf(__VA_ARGS__)
#include "live_stage_logger.h"
int main(void){
 probe_lhdc_live_t s;probe_lhdc_profile_t old;
 assert(!probe_lhdc_profile_snapshot(&s));probe_lhdc_profile_reset(false);assert(!probe_lhdc_profile_snapshot(&s));
 dbt_lhdc_stage_live_begin(192000,4);dbt_lhdc_stage_record(1,2,3,4,5,6);dbt_lhdc_stage_live_end();assert(!probe_lhdc_profile_snapshot(&s));
 probe_lhdc_profile_live_enable();
 dbt_lhdc_stage_live_begin(192000,4);assert(probe_lhdc_profile_snapshot(&s)&&s.channel_calls==0&&s.collecting==1);uint32_t gen=s.generation;
 for(unsigned i=0;i<31;++i)dbt_lhdc_stage_record(UINT32_MAX,2,3,4,5,6);
 assert(probe_lhdc_profile_snapshot(&s)&&s.channel_calls==0);
 dbt_lhdc_stage_record(UINT32_MAX,2,3,4,5,6);
 for(unsigned i=0;i<15;++i)dbt_lhdc_frame_stage_record(7,8,9,10);
 assert(probe_lhdc_profile_snapshot(&s)&&s.frame_calls==0&&s.channel_calls==0);
 dbt_lhdc_frame_stage_record(7,8,9,10);
 assert(probe_lhdc_profile_snapshot(&s));
 assert(s.channel_calls==32&&s.sum_cycles[0]==UINT32_MAX-31&&s.max_cycles[0]==UINT32_MAX&&s.sum_cycles[1]==64);
 assert(s.frame_calls==16&&s.frame_sum_cycles[0]==112&&s.frame_max_cycles[3]==10);
 log_lhdc_live(1234);assert(probe_lhdc_profile_finish(&old));assert(old.sum_cycles[0]==(uint64_t)32*UINT32_MAX);
 assert(probe_lhdc_profile_snapshot(&s)&&!s.collecting);dbt_lhdc_stage_record(99,99,99,99,99,99);assert(probe_lhdc_profile_snapshot(&s)&&s.channel_calls==32);
 dbt_lhdc_stage_live_begin(96000,4);dbt_lhdc_stage_record(1,2,3,4,5,6);dbt_lhdc_frame_stage_record(11,12,13,14);dbt_lhdc_stage_live_end();assert(probe_lhdc_profile_snapshot(&s)&&s.frame_calls==1&&s.frame_sum_cycles[0]==11&&s.generation!=gen&&s.channel_calls==1&&!s.collecting&&s.rate==96000);
 dbt_lhdc_stage_live_begin(44100,2);dbt_lhdc_frame_stage_record(UINT32_MAX,2,3,4);dbt_lhdc_frame_stage_record(1,2,3,4);dbt_lhdc_stage_live_end();
 assert(probe_lhdc_profile_snapshot(&s)&&s.frame_calls==2&&s.frame_sum_cycles[0]==0&&s.frame_max_cycles[0]==UINT32_MAX&&frame_sum[0]==UINT64_C(4294967296));
 atomic_store(&published.revision,UINT32_MAX-1u);dbt_lhdc_stage_live_end();assert(probe_lhdc_profile_snapshot(&s)&&s.revision==2);
 generation=UINT32_MAX;dbt_lhdc_stage_live_begin(48000,2);assert(probe_lhdc_profile_snapshot(&s)&&s.generation==1);
 probe_lhdc_profile_reset(true);assert(!probe_lhdc_profile_snapshot(&s));dbt_lhdc_stage_record(UINT32_MAX,0,0,0,0,0);dbt_lhdc_stage_record(1,0,0,0,0,0);assert(probe_lhdc_profile_finish(&old)&&old.sum_cycles[0]==UINT64_C(4294967296));
 assert(holds>0);puts("PASS live stage epochs/cadence/partial-publication/wrap/inactive and64-bit finite counters");return 0;
}
