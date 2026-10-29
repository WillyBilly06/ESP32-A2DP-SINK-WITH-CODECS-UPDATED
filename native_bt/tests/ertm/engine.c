#include "dbt_ertm_engine.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
typedef struct {
    dbt_ertm_engine_t engine;
    _Alignas(8) uint8_t storage[4][2056];
    uint8_t received[2048];uint16_t allocated[4],received_size;
    unsigned live,peak,allocations,releases,errors,deliveries;
    bool used[4],oom;
} peer_t;
static peer_t p[2];
static void zero(void *v,unsigned n){uint8_t *b=v;for(unsigned i=0;i<n;++i)b[i]=0;}
static bool equal(const uint8_t *a,const uint8_t *b,unsigned n){for(unsigned i=0;i<n;++i)if(a[i]!=b[i])return false;return true;}
static uint8_t *allocate(void *context,uint16_t bytes){
    peer_t *q=context;if(q->oom || !bytes || bytes>2048)return 0;
    for(unsigned i=0;i<4;++i)if(!q->used[i]){q->used[i]=true;q->allocated[i]=bytes;q->storage[i][bytes]=0xa5;
        q->live+=bytes;if(q->live>q->peak)q->peak=q->live;++q->allocations;return q->storage[i];}
    return 0;
}
static void release(void *context,uint8_t *bytes){
    peer_t *q=context;for(unsigned i=0;i<4;++i){if(q->storage[i]==bytes){
        if(!q->used[i] || q->storage[i][q->allocated[i]]!=0xa5){++q->errors;return;}
        q->used[i]=false;q->live-=q->allocated[i];++q->releases;return;
    }}++q->errors;
}
static void delivered(void *context,const uint8_t *bytes,uint16_t size){
    peer_t *q=context;++q->deliveries;q->received_size=size;
    for(unsigned i=0;i<size && i<sizeof(q->received);++i)q->received[i]=bytes[i];
    if(dbt_ertm_engine_busy(&q->engine,true)!=DBT_INVALID)++q->errors;
}
static bool setup(unsigned window,unsigned attempts,bool fcs){
    zero(p,sizeof(p));
    for(unsigned i=0;i<2;++i){dbt_ertm_hooks_t h={&p[i],allocate,release,delivered};
        dbt_ertm_parameters_t cfg={1024,48,2000,12000,(uint8_t)window,(uint8_t)attempts};
        if(dbt_ertm_engine_init(&p[i].engine,h,i?0x81:0x40,i?0x40:0x81,cfg,cfg,fcs)!=DBT_OK)return false;}
    return true;
}
static dbt_result_t control(peer_t *q,uint8_t function,uint8_t req,bool poll,bool final,uint32_t now){
    dbt_ertm_frame_t f;zero(&f,sizeof(f));f.supervisory=true;f.supervisor=function;f.req_seq=req;f.poll=poll;f.final=final;
    uint8_t wire[8];uint16_t size=0;
    dbt_result_t result=dbt_ertm_frame_encode(q->engine.local_cid,q->engine.fcs,&f,wire,sizeof(wire),&size);
    return result==DBT_OK?dbt_ertm_engine_receive(&q->engine,wire,size,now):result;
}
static int transfer(unsigned from,uint32_t now,int drop_seq,bool corrupt){
    uint8_t wire[1100];uint16_t size=0;dbt_ertm_frame_t f;
    dbt_result_t r=dbt_ertm_engine_view(&p[from].engine,&f);if(r==DBT_EMPTY)return 0;if(r!=DBT_OK)return -1;
    if(dbt_ertm_engine_peek(&p[from].engine,wire,sizeof(wire),&size)!=DBT_OK)return -2;
    if(dbt_ertm_engine_commit(&p[from].engine,now)!=DBT_OK)return -3;
    if(!f.supervisory && drop_seq>=0 && f.tx_seq==(unsigned)drop_seq)return 1;
    if(corrupt)wire[size-1u]^=1;
    if(dbt_ertm_engine_receive(&p[1u-from].engine,wire,size,now)!=DBT_OK)return -4;
    return 1;
}
static int pump(unsigned rounds,uint32_t now){
    for(unsigned n=0;n<rounds;++n){if(transfer(0,now+n,-1,false)<0 || transfer(1,now+n,-1,false)<0)return 1;}
    return 0;
}
static bool clean(void){
    for(unsigned i=0;i<2;++i){if(dbt_ertm_engine_close(&p[i].engine)==DBT_BUSY){if(dbt_ertm_engine_cancel_frame(&p[i].engine)!=DBT_OK)return false;}if(p[i].live || p[i].errors || p[i].allocations!=p[i].releases)return false;}
    return true;
}
EXPORT int ertm_engine_window_wrap_and_lazy_storage(void){
    CHECK(setup(4,4,true) && !p[0].allocations && !p[1].allocations);
    uint8_t bytes[1024];for(unsigned i=0;i<sizeof(bytes);++i)bytes[i]=(uint8_t)(i*31u+7u);
    const uint16_t sizes[]={0,1,47,48,49,200,1000,1024};
    unsigned frames=0;
    for(unsigned round=0;round<160;++round){uint16_t size=sizes[round%8u];unsigned before=p[1].deliveries;
        CHECK(dbt_ertm_engine_send(&p[0].engine,bytes,size)==DBT_OK && p[0].live<=32);
        CHECK(pump(100,round*100u)==0 && !p[0].engine.tx_active);
        CHECK(p[1].deliveries==before+1u && p[1].received_size==size && equal(p[1].received,bytes,size));
        CHECK(!p[0].live && !p[1].live && !p[0].errors && !p[1].errors);
        frames+=size?(size+47u)/48u:1u;
        CHECK(p[0].engine.next_tx==(frames&63u) && p[0].engine.expected_ack==p[0].engine.next_tx);
    }
    CHECK(p[0].engine.tx_sdus==160 && p[1].engine.rx_sdus==160);CHECK(clean());return 0;
}
EXPORT int ertm_engine_reject_recovers_lost_frame(void){
    CHECK(setup(4,4,true));uint8_t bytes[200];for(unsigned i=0;i<200;++i)bytes[i]=(uint8_t)i;
    CHECK(dbt_ertm_engine_send(&p[0].engine,bytes,sizeof(bytes))==DBT_OK);
    CHECK(transfer(0,0,-1,false)==1 && transfer(0,1,1,false)==1 && transfer(0,2,-1,false)==1 && transfer(0,3,-1,false)==1);
    CHECK(p[1].engine.expected_rx==1 && p[1].engine.rej_needed && !p[1].deliveries);
    dbt_ertm_frame_t reply;CHECK(dbt_ertm_engine_view(&p[1].engine,&reply)==DBT_OK && reply.supervisory && reply.supervisor==DBT_ERTM_REJ && reply.req_seq==1);
    CHECK(transfer(1,4,-1,false)==1 && p[0].engine.expected_ack==1);
    CHECK(pump(60,5)==0 && p[1].deliveries==1 && equal(bytes,p[1].received,200) && !p[0].engine.tx_active);
    CHECK(p[0].engine.tx_frames>5 && !p[1].engine.rej_sent);CHECK(clean());return 0;
}
EXPORT int ertm_engine_selective_ack_and_pinned_loan(void){
    CHECK(setup(4,5,true));uint8_t bytes[130];zero(bytes,sizeof(bytes));
    CHECK(dbt_ertm_engine_send(&p[0].engine,bytes,sizeof(bytes))==DBT_OK);
    dbt_ertm_frame_t f;for(unsigned i=0;i<3;++i){CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && !f.supervisory && f.tx_seq==i);CHECK(dbt_ertm_engine_commit(&p[0].engine,i)==DBT_OK);}
    CHECK(control(&p[0],DBT_ERTM_SREJ,1,false,false,4)==DBT_OK && p[0].engine.expected_ack==0 && p[0].engine.count==3);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.tx_seq==1 && !f.supervisory);
    CHECK(control(&p[0],DBT_ERTM_RR,3,false,false,5)==DBT_OK && !p[0].engine.count && p[0].engine.tx_active && p[0].live);
    dbt_ertm_frame_t pinned;CHECK(dbt_ertm_engine_view(&p[0].engine,&pinned)==DBT_OK && pinned.data==f.data && pinned.tx_seq==1);
    CHECK(dbt_ertm_engine_commit(&p[0].engine,6)==DBT_OK && !p[0].engine.tx_active && !p[0].live && p[0].engine.tx_sdus==1);
    CHECK(dbt_ertm_engine_send(&p[0].engine,bytes,sizeof(bytes))==DBT_OK);
    for(unsigned i=0;i<3;++i){CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK);CHECK(dbt_ertm_engine_commit(&p[0].engine,7+i)==DBT_OK);}
    CHECK(control(&p[0],DBT_ERTM_SREJ,4,true,false,11)==DBT_OK && p[0].engine.expected_ack==4 && p[0].engine.count==2);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.tx_seq==4 && f.final && !f.supervisory);
    CHECK(dbt_ertm_engine_commit(&p[0].engine,12)==DBT_OK);CHECK(clean());return 0;
}
EXPORT int ertm_engine_poll_monitor_and_retry_limits(void){
    CHECK(setup(2,2,true));uint8_t data=42;dbt_ertm_frame_t f;
    CHECK(dbt_ertm_engine_send(&p[0].engine,&data,1)==DBT_OK && dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK);
    CHECK(!p[0].engine.retrans_timer && dbt_ertm_engine_commit(&p[0].engine,100)==DBT_OK);
    CHECK(dbt_ertm_engine_wait_ms(&p[0].engine,2099)==1);dbt_ertm_engine_poll(&p[0].engine,2100);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.supervisory && f.poll && !p[0].engine.waiting_final);
    CHECK(dbt_ertm_engine_commit(&p[0].engine,2101)==DBT_OK && p[0].engine.waiting_final && p[0].engine.poll_attempts==1);
    dbt_ertm_engine_poll(&p[0].engine,14101);CHECK(!p[0].engine.failed && p[0].engine.poll_needed);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.poll && dbt_ertm_engine_commit(&p[0].engine,14102)==DBT_OK);
    dbt_ertm_engine_poll(&p[0].engine,26102);CHECK(p[0].engine.failed && !p[0].live);CHECK(clean());
    CHECK(setup(2,1,true));CHECK(dbt_ertm_engine_send(&p[0].engine,&data,1)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && dbt_ertm_engine_commit(&p[0].engine,0)==DBT_OK);
    CHECK(control(&p[0],DBT_ERTM_REJ,0,false,false,1)==DBT_INVALID);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_INVALID && p[0].engine.failed && !p[0].live);CHECK(clean());return 0;
}

EXPORT int ertm_engine_failure_retains_prepared_views(void){
    uint8_t bytes[130];zero(bytes,sizeof(bytes));dbt_ertm_frame_t f,pin;
    CHECK(setup(4,1,true) && dbt_ertm_engine_send(&p[0].engine,bytes,sizeof(bytes))==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && dbt_ertm_engine_commit(&p[0].engine,0)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&pin)==DBT_OK && pin.tx_seq==1);
    CHECK(control(&p[0],DBT_ERTM_SREJ,0,false,false,1)==DBT_INVALID && p[0].engine.failed);
    CHECK(p[0].engine.tx_active && p[0].live && dbt_ertm_engine_close(&p[0].engine)==DBT_BUSY);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.data==pin.data && f.tx_seq==pin.tx_seq);
    CHECK(dbt_ertm_engine_cancel_frame(&p[0].engine)==DBT_OK && !p[0].live && !p[0].engine.tx_active && !p[0].engine.tx_sdus);CHECK(clean());
    CHECK(setup(4,2,true) && dbt_ertm_engine_send(&p[0].engine,bytes,1)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && dbt_ertm_engine_commit(&p[0].engine,0)==DBT_OK);
    CHECK(control(&p[0],DBT_ERTM_REJ,0,false,false,1)==DBT_OK && dbt_ertm_engine_view(&p[0].engine,&pin)==DBT_OK);
    CHECK(control(&p[0],DBT_ERTM_RR,2,false,false,2)==DBT_INVALID && p[0].engine.tx_active && p[0].live);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.data==pin.data);
    CHECK(dbt_ertm_engine_commit(&p[0].engine,3)==DBT_OK && !p[0].live && !p[0].engine.tx_active && p[0].engine.failed);CHECK(clean());
    CHECK(setup(4,4,true) && dbt_ertm_engine_send(&p[0].engine,bytes,50)==DBT_OK && dbt_ertm_engine_view(&p[0].engine,&pin)==DBT_OK);
    CHECK(dbt_ertm_engine_close(&p[0].engine)==DBT_BUSY && p[0].engine.tx_active && p[0].live);
    CHECK(dbt_ertm_engine_cancel_frame(&p[0].engine)==DBT_OK && dbt_ertm_engine_close(&p[0].engine)==DBT_OK && !p[0].live);CHECK(clean());return 0;
}
EXPORT int ertm_engine_reject_poll_and_invalid_final_ack(void){
    uint8_t bytes[50];zero(bytes,sizeof(bytes));dbt_ertm_frame_t f;
    for(unsigned state=0;state<2;++state){
        CHECK(setup(4,4,true) && dbt_ertm_engine_send(&p[0].engine,bytes,sizeof(bytes))==DBT_OK);
        CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && dbt_ertm_engine_commit(&p[0].engine,0)==DBT_OK);
        if(!state)CHECK(dbt_ertm_engine_busy(&p[0].engine,true)==DBT_OK);else p[0].engine.rej_sent=true;
        CHECK(control(&p[0],DBT_ERTM_REJ,0,true,false,1)==DBT_OK);
        CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && !f.supervisory && f.tx_seq==0 && f.final);
        CHECK(dbt_ertm_engine_commit(&p[0].engine,2)==DBT_OK);CHECK(clean());
    }
    CHECK(setup(4,4,true) && control(&p[0],DBT_ERTM_RR,1,false,true,0)==DBT_INVALID && p[0].engine.failed);CHECK(clean());
    CHECK(setup(4,4,true) && control(&p[0],DBT_ERTM_SREJ,0,false,true,0)==DBT_INVALID && p[0].engine.failed);CHECK(clean());
    CHECK(setup(4,4,true) && control(&p[0],DBT_ERTM_RR,0,false,true,0)==DBT_OK && p[0].engine.ignored_frames==1 && !p[0].engine.failed);CHECK(clean());return 0;
}

static dbt_result_t information(peer_t *q,uint8_t seq,uint8_t req,uint8_t sar,const uint8_t *bytes,uint16_t size,uint16_t declared,uint32_t now){
    dbt_ertm_frame_t f;zero(&f,sizeof(f));f.data=bytes;f.size=size;f.sdu_length=declared;f.tx_seq=seq;f.req_seq=req;f.sar=sar;
    uint8_t wire[1100];uint16_t n=0;dbt_result_t result=dbt_ertm_frame_encode(q->engine.local_cid,q->engine.fcs,&f,wire,sizeof(wire),&n);
    return result==DBT_OK?dbt_ertm_engine_receive(&q->engine,wire,n,now):result;
}
static int prime_history(unsigned frames,bool final_ack){
    uint8_t data=9;dbt_ertm_frame_t f;
    for(unsigned n=0;n<frames;++n){
        CHECK(information(&p[0],(uint8_t)(n&63u),0,DBT_ERTM_UNSEGMENTED,&data,1,0,n)==DBT_OK);
        if(n+1u<frames || final_ack){CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.supervisory);CHECK(dbt_ertm_engine_commit(&p[0].engine,n)==DBT_OK);}
    }
    return 0;
}
EXPORT int ertm_engine_receive_window_epoch_and_rej_pin(void){
    uint8_t data=42;dbt_ertm_frame_t f;
    CHECK(setup(63,4,true));CHECK(information(&p[0],63,0,DBT_ERTM_UNSEGMENTED,&data,1,0,0)==DBT_INVALID && p[0].engine.failed);CHECK(clean());
    CHECK(setup(1,4,true));CHECK(information(&p[0],0,0,DBT_ERTM_UNSEGMENTED,&data,1,0,0)==DBT_OK);
    CHECK(information(&p[0],1,0,DBT_ERTM_UNSEGMENTED,&data,1,0,1)==DBT_OK && p[0].deliveries==1 && p[0].engine.rx_absolute==1);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && dbt_ertm_engine_commit(&p[0].engine,2)==DBT_OK && p[0].engine.ack_absolute==1);
    CHECK(information(&p[0],1,0,DBT_ERTM_UNSEGMENTED,&data,1,0,3)==DBT_OK && p[0].deliveries==2);CHECK(clean());
    CHECK(setup(4,4,true));CHECK(prime_history(129,false)==0 && p[0].engine.rx_absolute==129 && p[0].engine.ack_absolute==128);
    CHECK(information(&p[0],4,0,DBT_ERTM_UNSEGMENTED,&data,1,0,130)==DBT_OK && !p[0].engine.rej_needed && p[0].engine.rx_absolute==129);
    CHECK(information(&p[0],3,0,DBT_ERTM_UNSEGMENTED,&data,1,0,131)==DBT_OK && p[0].engine.rej_needed);CHECK(clean());
    CHECK(setup(63,4,true));CHECK(prime_history(64,true)==0);
    CHECK(information(&p[0],62,0,DBT_ERTM_UNSEGMENTED,&data,1,0,65)==DBT_OK && !p[0].engine.rej_needed && p[0].engine.ambiguous_frames==1 && p[0].deliveries==64);CHECK(clean());
    CHECK(setup(4,4,true));CHECK(information(&p[0],1,0,DBT_ERTM_UNSEGMENTED,&data,1,0,0)==DBT_OK && dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.supervisor==DBT_ERTM_REJ);
    CHECK(information(&p[0],0,0,DBT_ERTM_UNSEGMENTED,&data,1,0,1)==DBT_OK);
    CHECK(dbt_ertm_engine_commit(&p[0].engine,2)==DBT_OK && p[0].engine.rej_sent && p[0].engine.rej_absolute==0);
    CHECK(information(&p[0],2,0,DBT_ERTM_UNSEGMENTED,&data,1,0,3)==DBT_OK && !p[0].engine.rej_needed);
    CHECK(information(&p[0],0,0,DBT_ERTM_UNSEGMENTED,&data,1,0,4)==DBT_OK && !p[0].engine.rej_sent);
    CHECK(information(&p[0],2,0,DBT_ERTM_UNSEGMENTED,&data,1,0,5)==DBT_OK && p[0].engine.rej_needed);CHECK(clean());return 0;
}
EXPORT int ertm_engine_large_window_ambiguous_gap_recovers(void){
    CHECK(setup(63,0,true));uint8_t data[1024];for(unsigned n=0;n<sizeof(data);++n)data[n]=(uint8_t)(n*19u);
    for(unsigned n=0;n<64;++n){CHECK(dbt_ertm_engine_send(&p[0].engine,data,1)==DBT_OK && pump(4,n*4u)==0 && !p[0].engine.tx_active);}
    CHECK(p[1].engine.rx_absolute==64 && p[1].engine.ack_absolute==64 && p[0].engine.next_tx==0);
    CHECK(dbt_ertm_engine_send(&p[0].engine,data,sizeof(data))==DBT_OK);
    for(unsigned n=0;n<22;++n)CHECK(transfer(0,300u+n,0,false)==1);
    CHECK(p[1].deliveries==64 && !p[1].engine.rej_needed && p[1].engine.ambiguous_frames && p[0].engine.count==22);
    dbt_ertm_engine_poll(&p[0].engine,2321);CHECK(transfer(0,2321,-1,false)==1 && p[0].engine.waiting_final);
    CHECK(transfer(1,2322,-1,false)==1 && !p[0].engine.waiting_final);
    CHECK(pump(80,2323)==0 && !p[0].engine.tx_active && p[1].deliveries==65 && p[1].received_size==1024 && equal(data,p[1].received,1024));
    CHECK(clean());return 0;
}

EXPORT int ertm_engine_busy_recovery_and_timer_progress(void){
    CHECK(setup(4,4,true));uint8_t bytes[130];for(unsigned i=0;i<130;++i)bytes[i]=(uint8_t)i;dbt_ertm_frame_t f;
    CHECK(dbt_ertm_engine_send(&p[0].engine,bytes,100)==DBT_OK && dbt_ertm_engine_busy(&p[1].engine,true)==DBT_OK);
    CHECK(transfer(1,0,-1,false)==1 && p[0].engine.remote_busy && dbt_ertm_engine_view(&p[0].engine,&f)==DBT_EMPTY);
    CHECK(dbt_ertm_engine_wait_ms(&p[0].engine,1)==UINT32_MAX && dbt_ertm_engine_busy(&p[1].engine,false)==DBT_OK);
    CHECK(transfer(1,2,-1,false)==1 && p[1].engine.waiting_final);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && !f.supervisory && f.final);
    CHECK(transfer(0,3,-1,false)==1 && !p[1].engine.waiting_final && pump(40,4)==0);
    CHECK(p[1].deliveries==1 && equal(bytes,p[1].received,100) && !p[0].engine.tx_active);CHECK(clean());
    CHECK(setup(4,4,true) && dbt_ertm_engine_busy(&p[0].engine,true)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.supervisor==DBT_ERTM_RNR);
    CHECK(dbt_ertm_engine_busy(&p[0].engine,false)==DBT_OK && dbt_ertm_engine_commit(&p[0].engine,1)==DBT_OK && p[0].engine.poll_needed);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.poll && f.supervisor==DBT_ERTM_RR);CHECK(clean());
    CHECK(setup(4,4,true) && dbt_ertm_engine_send(&p[0].engine,bytes,sizeof(bytes))==DBT_OK);
    for(unsigned n=0;n<3;++n)CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && dbt_ertm_engine_commit(&p[0].engine,10u+n)==DBT_OK);
    CHECK(p[0].engine.retrans_at==2012 && control(&p[0],DBT_ERTM_RR,1,false,false,1000)==DBT_OK && p[0].engine.retrans_at==2012);
    CHECK(control(&p[0],DBT_ERTM_SREJ,1,false,false,1100)==DBT_OK && dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK);
    CHECK(dbt_ertm_engine_commit(&p[0].engine,1101)==DBT_OK && p[0].engine.retrans_at==2012);
    CHECK(control(&p[0],DBT_ERTM_RR,3,false,false,1200)==DBT_OK && !p[0].live);CHECK(clean());return 0;
}
EXPORT int ertm_engine_timer_wrap_and_infinite_retry(void){
    CHECK(setup(4,0,true));uint8_t data=42;dbt_ertm_frame_t f;
    CHECK(dbt_ertm_engine_send(&p[0].engine,&data,1)==DBT_OK && dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK);
    uint32_t now=UINT32_MAX-10u;CHECK(dbt_ertm_engine_commit(&p[0].engine,now)==DBT_OK);
    now+=1999u;CHECK(dbt_ertm_engine_wait_ms(&p[0].engine,now)==1);dbt_ertm_engine_poll(&p[0].engine,++now);
    for(unsigned n=0;n<8;++n){
        CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && f.poll && dbt_ertm_engine_commit(&p[0].engine,now)==DBT_OK);
        now+=12000;dbt_ertm_engine_poll(&p[0].engine,now);CHECK(!p[0].engine.failed && p[0].engine.poll_needed);
    }
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && dbt_ertm_engine_commit(&p[0].engine,now)==DBT_OK);
    CHECK(control(&p[0],DBT_ERTM_RR,0,false,true,now+1u)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].engine,&f)==DBT_OK && !f.supervisory && f.tx_seq==0 && dbt_ertm_engine_commit(&p[0].engine,now+2u)==DBT_OK);
    CHECK(control(&p[0],DBT_ERTM_RR,1,false,false,now+3u)==DBT_OK && !p[0].engine.tx_active);CHECK(clean());return 0;
}
EXPORT int ertm_engine_crc_sar_oom_and_ack_failure(void){
    uint8_t bytes[100];zero(bytes,sizeof(bytes));
    CHECK(setup(4,4,true));p[0].oom=true;CHECK(dbt_ertm_engine_send(&p[0].engine,bytes,100)==DBT_NO_SPACE && !p[0].engine.tx_active && !p[0].engine.failed);
    p[0].oom=false;CHECK(dbt_ertm_engine_send(&p[0].engine,bytes,100)==DBT_OK && transfer(0,0,-1,true)==1);
    CHECK(p[1].engine.bad_fcs==1 && !p[1].engine.expected_rx && !p[1].deliveries && !p[1].live);CHECK(clean());
    CHECK(setup(4,4,true) && information(&p[0],0,0,DBT_ERTM_START,bytes,6,10,0)==DBT_OK && p[0].live==10);
    CHECK(information(&p[0],1,0,DBT_ERTM_CONTINUE,bytes,4,0,1)==DBT_INVALID && p[0].engine.failed && !p[0].live);CHECK(clean());
    CHECK(setup(4,4,true));p[0].oom=true;CHECK(information(&p[0],0,0,DBT_ERTM_START,bytes,1,100,0)==DBT_NO_SPACE && p[0].engine.failed);CHECK(clean());
    CHECK(setup(4,4,true) && information(&p[0],0,0,DBT_ERTM_END,bytes,1,0,0)==DBT_INVALID && !p[0].deliveries);CHECK(clean());
    CHECK(setup(4,4,true) && information(&p[0],0,0,DBT_ERTM_UNSEGMENTED,bytes,49,0,0)==DBT_INVALID && p[0].engine.failed);CHECK(clean());
    CHECK(setup(4,4,true) && information(&p[0],20,1,DBT_ERTM_UNSEGMENTED,bytes,1,0,0)==DBT_INVALID && p[0].engine.failed);CHECK(clean());
    CHECK(setup(4,4,false));uint8_t invalid_flags[]={0x91,0};
    CHECK(dbt_ertm_engine_receive(&p[0].engine,invalid_flags,2,0)==DBT_OK && p[0].engine.ignored_frames==1 && !p[0].engine.failed);CHECK(clean());return 0;
}
