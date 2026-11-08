/* Differential deadline test. Runtime/channel stand-ins are intentionally
 * limited to query observations; the separate runtime-pair suite tests loans. */
#include "dbt_avrcp.h"
#include "dbt_avrcp_bearer.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
static dbt_channel_t fixture_channel;
static uint32_t actual_transport_calls,reference_transport_calls;
static uint64_t comparisons;
const dbt_channel_t *dbt_channels_find(const dbt_channels_t *c,uint16_t h,uint16_t cid){
    (void)c;return h==fixture_channel.handle && cid==fixture_channel.local_cid?&fixture_channel:NULL;
}
const dbt_link_t *dbt_acl_find(const dbt_acl_t *c,uint16_t h){
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)if(c->links[i].used && c->links[i].handle==h)return &c->links[i];
    return NULL;
}
dbt_result_t dbt_runtime_send(dbt_runtime_t *r,uint16_t h,uint16_t cid,const uint8_t *p,uint16_t n,dbt_tx_done_t done,void *context){
    (void)r;(void)h;(void)cid;(void)p;(void)n;(void)done;(void)context;return DBT_BUSY;
}
uint32_t measured_avctp_wait_ms(const dbt_avctp_t *,uint32_t);
uint32_t dbt_avctp_wait_ms(const dbt_avctp_t *s,uint32_t now){++actual_transport_calls;return measured_avctp_wait_ms(s,now);}
#include "reference_deadlines.h"
#define REQUIRE(x) do{if(!(x)){fprintf(stderr,"FAIL line %u\n",(unsigned)__LINE__);return 1;}}while(0)
static uint32_t random_state=0x735a102du;
static uint32_t random32(void){random_state^=random_state<<13;random_state^=random_state>>17;random_state^=random_state<<5;return random_state;}
static int compare(dbt_avctp_t *t,dbt_avrcp_t *p,dbt_avctp_bearer_t *b,uint32_t now){
    dbt_avctp_t before_t;dbt_avrcp_t before_p;dbt_avctp_bearer_t before_b;dbt_channel_t before_channel;
    memcpy(&before_t,t,sizeof before_t);memcpy(&before_p,p,sizeof before_p);
    memcpy(&before_b,b,sizeof before_b);memcpy(&before_channel,&fixture_channel,sizeof before_channel);
    REQUIRE(dbt_avctp_wait_ms(t,now)==reference_avctp_wait_ms(t,now));
    REQUIRE(dbt_avrcp_wait_ms(p,now)==reference_avrcp_wait_ms(p,now));
    REQUIRE(dbt_avctp_bearer_wait_ms(b,now)==reference_avctp_bearer_wait_ms(b,now));
    REQUIRE(dbt_avrcp_bearer_wait_ms(b,p,now)==reference_avrcp_bearer_wait_ms(b,p,now));
    REQUIRE(!memcmp(t,&before_t,sizeof *t) && !memcmp(p,&before_p,sizeof *p) && !memcmp(b,&before_b,sizeof *b));
    REQUIRE(!memcmp(&fixture_channel,&before_channel,sizeof fixture_channel));
    comparisons+=4;return 0;
}
int main(void){
    static dbt_runtime_t runtime;dbt_avctp_t t={0};dbt_avrcp_t p={0};dbt_avctp_bearer_t b={0};
    runtime.ready=true;runtime.host.state=DBT_HOST_IDENTIFIED;
    runtime.acl.links[0].used=true;runtime.acl.links[0].handle=9;
    fixture_channel=(dbt_channel_t){.serial=606,.link_generation=10,.handle=9,.local_cid=0x40,.psm=0x17,.rx_mtu=64,.tx_mtu=64,.state=DBT_CH_OPEN};
    t.binding=606;t.rx_mtu=t.tx_mtu=64;p.transport=&t;
    b=(dbt_avctp_bearer_t){&runtime,&t,606,10,9,0x40,false,true};
    /* All 16-bit masks, including bit15, sparse/empty/full and disabled timers. */
    for(unsigned mode=0;mode<8;++mode)for(uint32_t mask=0;mask<=UINT16_MAX;++mask){
        uint32_t now=(mode&1u)?UINT32_MAX-3u:17u;
        t.local_sent=(uint16_t)mask;t.tx_used=(mode&2u)!=0;t.pinned=(mode&4u)!=0;
        t.tx_deadline=now+3u;t.rx_active=(mode&1u)!=0;t.rx_deadline=now-1u;
        p.remote_auto=(uint16_t)((mask<<5)|(mask>>11));p.remote_pending=(uint16_t)~mask;
        p.remote_interim=(uint16_t)(mask^0xaaaau);p.target_active=(mode&2u)!=0;
        p.target_deadline=now+1u;p.abort_needed=(mode==6);p.continuation_needed=(mode==7);
        for(unsigned i=0;i<16;++i){t.response_timeout[i]=(i+mode)%3u?100u:0u;
            t.response_deadline[i]=now+i-4u;p.remote_deadline[i]=now+15u-i;
            p.remote_auto_kind[i]=(uint8_t)((i+mode)%8u);p.remote_opcode[i]=i&1u?0x7c:0;}
        REQUIRE(!compare(&t,&p,&b,now));
    }
    for(unsigned trial=0;trial<200000;++trial){
        uint32_t now=random32();t.binding=random32()%7u?606:0;t.failed=(random32()%7u)==0;
        t.tx_used=(random32()&1u)!=0;t.pinned=(random32()&1u)!=0;t.rx_active=(random32()&1u)!=0;
        t.tx_deadline=random32();t.rx_deadline=random32();t.local_sent=(uint16_t)random32();
        p.failed=(random32()%9u)==0;p.remote_pending=(uint16_t)random32();p.remote_interim=(uint16_t)random32();
        p.remote_auto=(uint16_t)random32();p.target_active=(random32()&1u)!=0;p.target_deadline=random32();
        p.abort_needed=(random32()%5u)==0;p.continuation_needed=(random32()%5u)==0;
        b.attached=(random32()%5u)!=0;b.active=(random32()&1u)!=0;
        runtime.host.state=random32()%7u?DBT_HOST_IDENTIFIED:DBT_HOST_FAILED;
        runtime.tx[0].active=(random32()&1u)!=0;fixture_channel.serial=random32()%7u?606:607;
        for(unsigned i=0;i<16;++i){t.response_timeout[i]=random32()%3u?100:0;t.response_deadline[i]=random32();
            p.remote_deadline[i]=random32();p.remote_auto_kind[i]=(uint8_t)random32();p.remote_opcode[i]=(uint8_t)random32();}
        REQUIRE(!compare(&t,&p,&b,now));
    }
    /* A composite idle query now traverses AVCTP deadlines once, formerly twice. */
    t=(dbt_avctp_t){.binding=606,.rx_mtu=64,.tx_mtu=64};p=(dbt_avrcp_t){.transport=&t};
    b=(dbt_avctp_bearer_t){&runtime,&t,606,10,9,0x40,false,true};fixture_channel.serial=606;
    runtime.host.state=DBT_HOST_IDENTIFIED;actual_transport_calls=reference_transport_calls=0;
    REQUIRE(dbt_avrcp_bearer_wait_ms(&b,&p,0)==UINT32_MAX && actual_transport_calls==1);
    REQUIRE(reference_avrcp_bearer_wait_ms(&b,&p,0)==UINT32_MAX && reference_transport_calls==2);
    /* Malformed API state is checked without executing the old null dereference. */
    p.transport=NULL;b.signal=NULL;REQUIRE(dbt_avrcp_bearer_wait_ms(&b,&p,0)==0);
    REQUIRE(dbt_avctp_bearer_wait_ms(&b,0)==0);b.runtime=NULL;REQUIRE(dbt_avctp_bearer_local_wait_ms(&b)==0);
    REQUIRE(dbt_avctp_bearer_wait_ms(NULL,0)==UINT32_MAX && dbt_avrcp_bearer_wait_ms(NULL,NULL,0)==0);
    printf("PASS %" PRIu64 " actual/reference deadline comparisons; exhaustive masks, expiry/wrap, reply/continuation, failed/stale/busy bindings, unchanged state; AVCTP scans2->1; host only\n",comparisons);
    return 0;
}
