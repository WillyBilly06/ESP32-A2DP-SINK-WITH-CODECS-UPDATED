#include "probe_arrival.h"
#include "dbt_host.h"
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
static void yield_cpu(void){Sleep(0);}
#else
#include <pthread.h>
#include <sched.h>
static void yield_cpu(void){sched_yield();}
#endif
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL arrival line%d\n",__LINE__);return 1;}}while(0)
static unsigned cases;
static int unit_cases(void){
    const uint16_t sizes[]={5,6,7,8,9,37,587,1028,1029};
    uint8_t packet[DBT_MAX_H4_BYTES]={DBT_H4_ACL};
    probe_arrival_sample_t s,t;probe_arrival_stats_t st;
    for(unsigned offset=0;offset<DBT_QUEUE_BYTES;++offset)for(unsigned i=0;i<sizeof(sizes)/sizeof(sizes[0]);++i){
        probe_arrival_init();uint16_t size=sizes[i],flags=(uint16_t)(0x2001u|((offset&3u)<<14));
        uint32_t cursor=0xfffff000u+offset,at=offset?UINT32_MAX-offset:0u,end=cursor+size+2u;
        packet[1]=(uint8_t)flags;packet[2]=(uint8_t)(flags>>8);
        packet[3]=(uint8_t)(size-5);packet[4]=(uint8_t)((size-5)>>8);
        CHECK(probe_arrival_begin(cursor,at,packet,size));CHECK(probe_arrival_pending()==1);
        probe_arrival_end(cursor,size,DBT_OK,end);
        CHECK(!probe_arrival_peek(end+1,flags,size,&s));
        CHECK(!probe_arrival_peek(end,flags^1u,size,&s));
        CHECK(!probe_arrival_peek(end,flags,size==5?6:(uint16_t)(size-1),&s));
        CHECK(probe_arrival_peek(end,flags,size,&s));
        CHECK(s.queue_cursor==cursor && s.arrival_us==at && s.wire_bytes==size && s.acl_flags==flags);
        memset(packet+5,0xa5,size-5u);
        CHECK(probe_arrival_peek(end,flags,size,&t) && !memcmp(&s,&t,sizeof s));
        probe_arrival_stats(&st);CHECK(st.published==1 && !st.consumed && !st.status && !st.lost && st.bytes<=128);
        probe_arrival_retire(end,flags,size);probe_arrival_stats(&st);
        CHECK(st.consumed==1 && !st.status && !probe_arrival_peek(end,flags,size,&t) && !probe_arrival_pending());++cases;
    }
    uint8_t tiny[]={2,1,0x20,0,0},event[]={4,0xff,0};
    probe_arrival_init();CHECK(!probe_arrival_begin(0,1,event,sizeof event));
    CHECK(!probe_arrival_begin(0,1,NULL,0));probe_arrival_stats(&st);CHECK(!st.published && !st.status);
    for(unsigned i=0;i<PROBE_ARRIVAL_SLOTS;++i)CHECK(probe_arrival_begin(i*7u,i,tiny,sizeof tiny));
    CHECK(!probe_arrival_begin(56,8,tiny,sizeof tiny));probe_arrival_stats(&st);
    CHECK(probe_arrival_pending()==UINT32_MAX);
    CHECK(st.status==2 && st.lost==1 && st.published==8 && !probe_arrival_peek(7,0x2001,5,&s));
    probe_arrival_init();CHECK(!probe_arrival_begin(0,0,tiny,4));probe_arrival_stats(&st);CHECK(st.status==1);
    probe_arrival_init();CHECK(probe_arrival_begin(0,0,tiny,5));probe_arrival_end(0,5,DBT_NO_SPACE,0);
    probe_arrival_stats(&st);CHECK(st.status==3 && !probe_arrival_peek(7,0x2001,5,&s));
    probe_arrival_init();CHECK(probe_arrival_begin(0,0,tiny,5));probe_arrival_end(0,5,DBT_OK,0);
    probe_arrival_stats(&st);CHECK(st.status==3);
    probe_arrival_init();CHECK(probe_arrival_begin(0,0,tiny,5));probe_arrival_retire(8,0x2001,5);
    probe_arrival_stats(&st);CHECK(st.status==4 && !st.consumed);
    probe_arrival_init();probe_arrival_retire(7,0x2001,5);probe_arrival_stats(&st);CHECK(st.status==4);
    printf("PASS %u cursor/header identity cases; all4096 positions, timestamp0/wrap, nonmutating mismatch, overflow/rejection/missing explicit\n",cases);
    return 0;
}
static int media_cases(void){
    uint8_t h4[37]={2,1,0x20,32,0,28,0,0x41,0,0x80,0xe0,0x12,0x34,1,2,3,4,5,6,7,8};
    uint8_t retained[16]={0};
    dbt_avdtp_media_t m={h4+21,0x01020304,0x05060708,16,0x1234,96,true};
    probe_arrival_sample_t s;probe_arrival_stats_t st;
    probe_arrival_init();CHECK(probe_arrival_begin(100,0,h4,sizeof h4));
    probe_arrival_end(100,sizeof h4,DBT_OK,139);
    CHECK(probe_arrival_media(139,h4,sizeof h4,&m,&s) && s.queue_cursor==100 && !s.arrival_us);
    CHECK(!probe_arrival_media(140,h4,sizeof h4,&m,&s));
    ++m.sequence;CHECK(!probe_arrival_media(139,h4,sizeof h4,&m,&s));--m.sequence;
    ++m.timestamp;CHECK(!probe_arrival_media(139,h4,sizeof h4,&m,&s));--m.timestamp;
    ++m.ssrc;CHECK(!probe_arrival_media(139,h4,sizeof h4,&m,&s));--m.ssrc;
    m.payload_type=DBT_AVDTP_RAW_PAYLOAD_TYPE;CHECK(!probe_arrival_media(139,h4,sizeof h4,&m,&s));m.payload_type=96;
    m.payload=retained;CHECK(!probe_arrival_media(139,h4,sizeof h4,&m,&s));m.payload=h4+21;
    m.size=17;CHECK(!probe_arrival_media(139,h4,sizeof h4,&m,&s));m.size=16;
    m.payload=h4+20;CHECK(!probe_arrival_media(139,h4,sizeof h4,&m,&s));m.payload=h4+21;
    CHECK(!probe_arrival_media(139,h4,sizeof h4-1u,&m,&s));
    h4[5]=27;CHECK(!probe_arrival_media(139,h4,sizeof h4,&m,&s));h4[5]=28;
    CHECK(probe_arrival_media(139,h4,sizeof h4,&m,&s));probe_arrival_retire(139,0x2001,sizeof h4);
    for(unsigned pb=1;pb<4;pb+=2){
        h4[2]=(uint8_t)(pb<<4);probe_arrival_init();CHECK(probe_arrival_begin(100,50,h4,sizeof h4));
        probe_arrival_end(100,sizeof h4,DBT_OK,139);
        CHECK(!probe_arrival_media(139,h4,sizeof h4,&m,&s));
        probe_arrival_retire(139,(uint16_t)(1u|(pb<<12)),sizeof h4);probe_arrival_stats(&st);
        CHECK(!st.status && st.consumed==1);
    }
    puts("PASS complete current RTP identity; retained/deferred pointers, fragments, raw framing, truncation and tuple mismatches unavailable");
    return 0;
}

#define COUNT 20000u
static struct {dbt_host_t host;_Atomic unsigned done,error;unsigned acl,events;} f;
static bool unavailable(void *p){(void)p;return false;}
static bool no_send(void *p,uint8_t *data,uint16_t n){(void)p;(void)data;(void)n;return false;}
static void observe(void *p,const dbt_hci_packet_t *packet){
    (void)p;uint32_t end=atomic_load_explicit(&f.host.ingress.read,memory_order_relaxed);
    probe_arrival_sample_t s;
    if(packet->type!=DBT_H4_ACL){
        if(probe_arrival_peek(end,0,0,&s))atomic_store(&f.error,1);
        ++f.events;return;
    }
    if(packet->size!=32u){atomic_store(&f.error,8);return;}
    uint32_t ordinal=(uint32_t)packet->payload[0]|((uint32_t)packet->payload[1]<<8)|
        ((uint32_t)packet->payload[2]<<16)|((uint32_t)packet->payload[3]<<24);
    uint16_t flags=(uint16_t)(packet->handle|(packet->pb<<12)|(packet->bc<<14));
    uint16_t size=(uint16_t)(packet->size+5u);
    if(ordinal!=f.acl++ || !probe_arrival_peek(end,flags,size,&s) ||
       s.arrival_us!=(uint32_t)(UINT32_MAX-100u+ordinal*5000u) || s.queue_cursor+size+2u!=end)
        atomic_store(&f.error,2);
    probe_arrival_sample_t again;
    if(!probe_arrival_peek(end,flags,size,&again) || memcmp(&again,&s,sizeof s))atomic_store(&f.error,3);
    probe_arrival_retire(end,flags,size);
}
#ifdef _WIN32
static DWORD WINAPI produce(LPVOID p)
#else
static void *produce(void *p)
#endif
{
    (void)p;uint8_t acl[37]={2,1,0x20,32,0},event[]={4,0xff,0};
    for(uint32_t i=0;i<COUNT && !atomic_load(&f.error);++i){
        probe_arrival_stats_t st;
        for(;;){probe_arrival_stats(&st);if(st.status){atomic_store(&f.error,4);break;}
            if(st.published-st.consumed<3u || atomic_load(&f.error))break;yield_cpu();}
        if(atomic_load(&f.error))break;
        acl[5]=(uint8_t)i;acl[6]=(uint8_t)(i>>8);acl[7]=(uint8_t)(i>>16);acl[8]=(uint8_t)(i>>24);
        memset(acl+9,0xa5,28);
        uint32_t cursor=atomic_load_explicit(&f.host.ingress.written,memory_order_relaxed);
        if(!probe_arrival_begin(cursor,(uint32_t)(UINT32_MAX-100u+i*5000u),acl,sizeof acl)){atomic_store(&f.error,5);break;}
        dbt_result_t result=dbt_host_receive(&f.host,acl,sizeof acl);
        probe_arrival_end(cursor,sizeof acl,result,atomic_load_explicit(&f.host.ingress.written,memory_order_relaxed));
        if(result!=DBT_OK){atomic_store(&f.error,6);break;}
        if(i%19u==0u && dbt_host_receive(&f.host,event,sizeof event)!=DBT_OK){atomic_store(&f.error,7);break;}
    }
    atomic_store(&f.done,1);
#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}
int main(void){
    if(unit_cases() || media_cases())return 1;
    memset(&f,0,sizeof f);probe_arrival_init();
    dbt_transport_t t={NULL,unavailable,no_send};
    CHECK(dbt_host_init(&f.host,t,observe,NULL,2000)==DBT_OK);
    f.host.state=DBT_HOST_IDENTIFIED;
    atomic_store(&f.host.ingress.read,UINT32_MAX-21u);atomic_store(&f.host.ingress.written,UINT32_MAX-21u);
#ifdef _WIN32
    HANDLE worker=CreateThread(NULL,0,produce,NULL,0,NULL);CHECK(worker);
#else
    pthread_t worker;CHECK(!pthread_create(&worker,NULL,produce,NULL));
#endif
    while((!atomic_load(&f.done)||dbt_queue_used(&f.host.ingress))&&!atomic_load(&f.error)){
        (void)dbt_host_poll(&f.host,1,2);yield_cpu();
    }
#ifdef _WIN32
    WaitForSingleObject(worker,INFINITE);CloseHandle(worker);
#else
    pthread_join(worker,NULL);
#endif
    probe_arrival_stats_t st;probe_arrival_stats(&st);
    CHECK(!atomic_load(&f.error) && f.acl==COUNT && f.events==(COUNT-1u)/19u+1u);
    CHECK(st.published==COUNT && st.consumed==COUNT && !st.lost && !st.status);
    puts("PASS20000 actual HCI SPSC arrivals; intervening events,3credits, cursor/time wrap, repeated media peek before retirement; no hardware");
    return 0;
}
