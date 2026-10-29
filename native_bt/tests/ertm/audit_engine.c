#include "dbt_ertm_engine.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
#define POOL_COUNT 8u
#define POOL_BYTES 2048u

typedef struct {
    dbt_ertm_engine_t e;
    _Alignas(8) uint8_t pool[POOL_COUNT][POOL_BYTES+1];
    uint8_t result[1200];
    uint16_t used_size[POOL_COUNT], result_size;
    bool used[POOL_COUNT];
    unsigned allocs, frees, errors, received;
} audit_peer;
static audit_peer p[2];
static unsigned detail_window, detail_step, detail_case, detail_a, detail_b;
static uint8_t left_data[600],right_data[600];
static void clear(void *ptr,unsigned n) { uint8_t *p0=ptr; for(unsigned i=0;i<n;i++)p0[i]=0; }
static bool equal(const uint8_t *a,const uint8_t *b,unsigned n) { for(unsigned i=0;i<n;i++)if(a[i]!=b[i])return false;return true; }
static uint8_t *take(void *context,uint16_t n) {
    audit_peer *q=context; if(!n || n>POOL_BYTES)return 0;
    for(unsigned i=0;i<POOL_COUNT;i++)if(!q->used[i]) {
        q->used[i]=true; q->used_size[i]=n; q->pool[i][n]=0xa5; q->allocs++; return q->pool[i];
    }
    return 0;
}
static void give(void *context,uint8_t *buf) {
    audit_peer *q=context; for(unsigned i=0;i<POOL_COUNT;i++)if(buf==q->pool[i]) {
        if(!q->used[i] || q->pool[i][q->used_size[i]]!=0xa5) {q->errors++;return;}
        q->used[i]=false; q->frees++; return;
    }
    q->errors++;
}
static void receive_sdu(void *context,const uint8_t *data,uint16_t n) {
    audit_peer *q=context; q->received++; q->result_size=n;
    if(n>sizeof(q->result)) {q->errors++;return;}
    for(unsigned i=0;i<n;i++) q->result[i]=data[i];
}
static bool setup(unsigned w,unsigned max,bool fcs) {
    clear(p,sizeof(p));
    for(unsigned i=0;i<2;i++) {
        dbt_ertm_hooks_t h={&p[i],take,give,receive_sdu};
        dbt_ertm_parameters_t c={1024,48,2000,12000,(uint8_t)w,(uint8_t)max};
        if(dbt_ertm_engine_init(&p[i].e,h,i ? 0x81 : 0x40,i ? 0x40 : 0x81,c,c,fcs)!=DBT_OK)return false;
    }
    return true;
}
static bool clean(void) {
    for(unsigned i=0;i<2;i++) {
        dbt_result_t r=dbt_ertm_engine_close(&p[i].e);
        if(r==DBT_BUSY && dbt_ertm_engine_cancel_frame(&p[i].e)!=DBT_OK)return false;
        if(r!=DBT_OK && r!=DBT_BUSY)return false;
        if(p[i].errors || p[i].allocs!=p[i].frees)return false;
    }
    return true;
}
static dbt_result_t info(audit_peer *q,uint8_t tx,uint8_t req,bool final,uint8_t sar,uint16_t declared,const uint8_t *data,uint16_t n,uint32_t now) {
    uint8_t wire[1100]; uint16_t ctrl=(uint16_t)((tx<<1)|(req<<8)|(sar<<14)|(final ? 0x80 : 0));
    unsigned at=0; wire[at++]=(uint8_t)ctrl; wire[at++]=(uint8_t)(ctrl>>8);
    if(sar==DBT_ERTM_START) {wire[at++]=(uint8_t)declared;wire[at++]=(uint8_t)(declared>>8);}
    for(unsigned i=0;i<n;i++)wire[at++]=data[i];
    return dbt_ertm_engine_receive(&q->e,wire,(uint16_t)at,now);
}
static dbt_result_t sf(audit_peer *q,uint8_t op,uint8_t req,bool poll,bool final,uint32_t now) {
    uint16_t ctrl=(uint16_t)(1u|((unsigned)op<<2)|(req<<8)|(poll?0x10:0)|(final?0x80:0));
    uint8_t wire[2]={(uint8_t)ctrl,(uint8_t)(ctrl>>8)};
    return dbt_ertm_engine_receive(&q->e,wire,2,now);
}
static int send_frame(unsigned from,uint32_t now,unsigned *losses,unsigned step) {
    dbt_ertm_frame_t f; uint8_t wire[1100]; uint16_t n=0;
    dbt_result_t r=dbt_ertm_engine_view(&p[from].e,&f);
    if(r==DBT_EMPTY)return 0;
    if(r!=DBT_OK) { detail_case=10;detail_a=(unsigned)r;detail_b=from;detail_step=step;return -1; }
    if(dbt_ertm_engine_peek(&p[from].e,wire,sizeof(wire),&n)!=DBT_OK) {detail_case=11;detail_b=from;detail_step=step;return -1;}
    if(dbt_ertm_engine_commit(&p[from].e,now)!=DBT_OK) {detail_case=12;detail_b=from;detail_step=step;return -1;}
    if(losses) {
        unsigned mask=f.final?4u:(!f.supervisory?1u:(!f.poll?2u:0u));
        if(mask && (losses[from]&mask)) {losses[from]&=~mask;return 1;}
    }
    r=dbt_ertm_engine_receive(&p[1u-from].e,wire,n,now);
    if(r!=DBT_OK) {detail_case=13;detail_a=(unsigned)r;detail_b=from;detail_step=step;return -1;}
    return 1;
}
EXPORT unsigned audit_window(void){return detail_window;}
EXPORT unsigned audit_case(void){return detail_case;}
EXPORT unsigned audit_step(void){return detail_step;}
EXPORT unsigned audit_a(void){return detail_a;}
EXPORT unsigned audit_b(void){return detail_b;}

EXPORT int audit_startup_window_matrix(void) {
    uint8_t byte=0x5a; detail_case=0;
    for(unsigned w=1;w<64;w++)for(unsigned seq=0;seq<64;seq++) {
        detail_window=w;detail_step=seq; CHECK(setup(w,4,false));
        dbt_result_t r=info(&p[0],(uint8_t)seq,0,false,DBT_ERTM_UNSEGMENTED,0,&byte,1,0);
        if(seq==0) CHECK(r==DBT_OK && p[0].received==1 && p[0].e.rx_absolute==1);
        else if(seq<w) CHECK(r==DBT_OK && !p[0].received && p[0].e.rej_needed);
        else if(w>32) CHECK(r==DBT_INVALID && p[0].e.failed);
        else CHECK(r==DBT_OK && !p[0].received && !p[0].e.rej_needed);
        CHECK(clean());
    }
    return 0;
}
EXPORT int audit_duplicate_multiround(void) {
    uint8_t value=0x6a; detail_case=0;
    for(unsigned w=1;w<64;w++) {
        detail_window=w; CHECK(setup(w,4,false));
        for(unsigned n=0;n<130;n++) {
            detail_step=n; CHECK(info(&p[0],(uint8_t)(n&63),0,false,DBT_ERTM_UNSEGMENTED,0,&value,1,n)==DBT_OK);
            dbt_ertm_frame_t f; CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.req_seq==((n+1)&63));
            CHECK(dbt_ertm_engine_commit(&p[0].e,n)==DBT_OK);
        }
        CHECK(p[0].e.rx_absolute==130 && p[0].e.ack_absolute==130 && p[0].received==130);
        for(unsigned back=1;back<=w;back++) {
            detail_step=back; uint8_t seq=(uint8_t)((130u-back)&63u);
            CHECK(info(&p[0],seq,0,false,DBT_ERTM_UNSEGMENTED,0,&value,1,200+back)==DBT_OK);
            CHECK(p[0].received==130 && p[0].e.rx_absolute==130 && !p[0].e.rej_needed);
            if(p[0].e.ack_needed) {dbt_ertm_frame_t f;CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK);CHECK(dbt_ertm_engine_commit(&p[0].e,200+back)==DBT_OK);}
        }
        CHECK(clean());
    }
    return 0;
}
EXPORT int audit_bidirectional_loss_all_windows(void) {
    detail_case=0;
    for(unsigned i=0;i<sizeof(left_data);i++){left_data[i]=(uint8_t)(i*7u+3u);right_data[i]=(uint8_t)(i*11u+5u);}
    for(unsigned w=1;w<64;w++) {
        detail_window=w; CHECK(setup(w,0,true)); uint32_t now=0;
        for(unsigned cycle=0;cycle<70;cycle++) {
            detail_step=cycle; uint8_t lv=(uint8_t)cycle,rv=(uint8_t)(cycle+1);
            CHECK(dbt_ertm_engine_send(&p[0].e,&lv,1)==DBT_OK && dbt_ertm_engine_send(&p[1].e,&rv,1)==DBT_OK);
            for(unsigned n=0;n<40 && (p[0].e.tx_active || p[1].e.tx_active);n++) {
                CHECK(send_frame(0,now,0,n)>=0 && send_frame(1,now,0,n)>=0); now++;
            }
            CHECK(!p[0].e.tx_active && !p[1].e.tx_active && p[0].received==cycle+1 && p[1].received==cycle+1);
        }
        CHECK(p[0].e.rx_absolute==70 && p[1].e.rx_absolute==70);
        CHECK(dbt_ertm_engine_send(&p[0].e,left_data,sizeof(left_data))==DBT_OK);
        CHECK(dbt_ertm_engine_send(&p[1].e,right_data,sizeof(right_data))==DBT_OK);
        unsigned losses[2]={7,7};
        for(unsigned step=0;step<30000 && (p[0].e.tx_active || p[1].e.tx_active);step++) {
            detail_step=step; dbt_ertm_engine_poll(&p[0].e,now);dbt_ertm_engine_poll(&p[1].e,now);
            CHECK(send_frame(0,now,losses,step)>=0 && send_frame(1,now,losses,step)>=0);now+=17;
        }
        detail_a=losses[0]; detail_b=losses[1]; detail_case=14;
        CHECK(!p[0].e.tx_active && !p[1].e.tx_active && !(losses[0]&5u) && !(losses[1]&5u));
        CHECK(p[0].received==71 && p[1].received==71 && p[0].result_size==600 && p[1].result_size==600);
        CHECK(equal(p[0].result,right_data,600) && equal(p[1].result,left_data,600));
        CHECK(clean());
    }
    return 0;
}
EXPORT int audit_busy_poll_final(void) {
    detail_window=4;detail_case=0; CHECK(setup(4,4,false)); uint8_t x=7; dbt_ertm_frame_t f;
    CHECK(dbt_ertm_engine_send(&p[0].e,&x,1)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && !f.supervisory && dbt_ertm_engine_commit(&p[0].e,0)==DBT_OK);
    CHECK(sf(&p[0],DBT_ERTM_RNR,0,false,false,1)==DBT_OK && p[0].e.remote_busy && !p[0].e.retrans_timer);
    CHECK(dbt_ertm_engine_busy(&p[0].e,true)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.supervisory && f.supervisor==DBT_ERTM_RNR && !f.poll && !f.final);
    CHECK(dbt_ertm_engine_commit(&p[0].e,2)==DBT_OK && p[0].e.rnr_sent);
    CHECK(sf(&p[0],DBT_ERTM_RR,0,true,false,3)==DBT_OK && !p[0].e.remote_busy);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.supervisory && f.supervisor==DBT_ERTM_RNR && f.final);
    CHECK(dbt_ertm_engine_commit(&p[0].e,4)==DBT_OK);
    CHECK(dbt_ertm_engine_busy(&p[0].e,false)==DBT_OK && p[0].e.poll_needed);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.supervisory && f.supervisor==DBT_ERTM_RR && f.poll);
    CHECK(dbt_ertm_engine_commit(&p[0].e,5)==DBT_OK && p[0].e.waiting_final && p[0].e.monitor_timer);
    CHECK(info(&p[0],0,0,true,DBT_ERTM_UNSEGMENTED,0,&x,1,6)==DBT_OK && p[0].received==1 && !p[0].e.waiting_final);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && !f.supervisory && f.tx_seq==0);
    CHECK(dbt_ertm_engine_commit(&p[0].e,7)==DBT_OK); CHECK(clean());return 0;
}
EXPORT int audit_repeated_poll_pins(void) {
    detail_window=4; detail_case=0; detail_step=0; detail_a=detail_b=0;
    CHECK(setup(4,4,false)); dbt_ertm_frame_t f;
    CHECK(sf(&p[0],DBT_ERTM_RR,0,true,false,0)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.supervisory && f.final && !f.poll);
    CHECK(sf(&p[0],DBT_ERTM_RR,0,true,false,12000)==DBT_OK);
    CHECK(sf(&p[0],DBT_ERTM_RR,0,true,false,24000)==DBT_OK);
    CHECK(dbt_ertm_engine_commit(&p[0].e,24001)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.final); CHECK(dbt_ertm_engine_commit(&p[0].e,24002)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.final); CHECK(dbt_ertm_engine_commit(&p[0].e,24003)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_EMPTY); CHECK(clean());return 0;
}
EXPORT int audit_retry_limit_during_other_pin(void) {
    detail_window=4;detail_case=0;CHECK(setup(4,2,false));uint8_t x=42;dbt_ertm_frame_t f;
    CHECK(dbt_ertm_engine_send(&p[0].e,&x,1)==DBT_OK);CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK);
    CHECK(dbt_ertm_engine_commit(&p[0].e,0)==DBT_OK);CHECK(sf(&p[0],DBT_ERTM_SREJ,0,false,false,1)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && !f.supervisory && dbt_ertm_engine_commit(&p[0].e,2)==DBT_OK);
    CHECK(info(&p[0],0,0,false,DBT_ERTM_UNSEGMENTED,0,&x,1,3)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.supervisory);
    CHECK(sf(&p[0],DBT_ERTM_SREJ,0,false,false,4)==DBT_INVALID && p[0].e.failed && p[0].e.tx_active);
    CHECK(dbt_ertm_engine_close(&p[0].e)==DBT_BUSY);CHECK(dbt_ertm_engine_cancel_frame(&p[0].e)==DBT_OK);
    CHECK(!p[0].e.tx_active && p[0].allocs==p[0].frees); CHECK(clean());return 0;
}
EXPORT int audit_timer_wrap_and_exhaustion(void) {
    detail_window=2;detail_case=0;CHECK(setup(2,2,false));uint8_t x=8;dbt_ertm_frame_t f;
    CHECK(dbt_ertm_engine_send(&p[0].e,&x,1)==DBT_OK && dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK);
    CHECK(dbt_ertm_engine_commit(&p[0].e,0xfffffff0u)==DBT_OK);
    CHECK(dbt_ertm_engine_wait_ms(&p[0].e,0x7be)==2);
    dbt_ertm_engine_poll(&p[0].e,0x7c0); CHECK(p[0].e.poll_needed);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.supervisory && f.poll);
    CHECK(dbt_ertm_engine_commit(&p[0].e,0x7c1)==DBT_OK && p[0].e.poll_attempts==1);
    dbt_ertm_engine_poll(&p[0].e,0x36a1); CHECK(p[0].e.poll_needed && !p[0].e.failed);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.poll && dbt_ertm_engine_commit(&p[0].e,0x36a2)==DBT_OK);
    dbt_ertm_engine_poll(&p[0].e,0x6582); CHECK(p[0].e.failed && !p[0].e.tx_active);
    CHECK(clean()); return 0;
}
EXPORT int audit_repeated_loss_finite_recovery(void) {
    detail_case=0;
    for(unsigned i=0;i<sizeof(left_data);i++) {left_data[i]=(uint8_t)(i*17u);right_data[i]=(uint8_t)(i*29u+1u);}
    const unsigned windows[]={1,2,4,16,32,33,63};
    for(unsigned wi=0;wi<sizeof(windows)/sizeof(windows[0]);wi++) {
        unsigned w=windows[wi]; detail_window=w; CHECK(setup(w,0,true));
        CHECK(dbt_ertm_engine_send(&p[0].e,left_data,sizeof(left_data))==DBT_OK);
        CHECK(dbt_ertm_engine_send(&p[1].e,right_data,sizeof(right_data))==DBT_OK);
        unsigned flags[]={5,5},data_drop[]={0,0},final_drop[]={0,0};
        unsigned data_goal=(w<13u?w:13u)+1u; uint32_t now=0;
        for(unsigned step=0;step<40000 && (p[0].e.tx_active || p[1].e.tx_active);step++) {
            detail_step=step; dbt_ertm_engine_poll(&p[0].e,now);dbt_ertm_engine_poll(&p[1].e,now);
            for(unsigned side=0;side<2;side++) {
                unsigned previous=flags[side]; CHECK(send_frame(side,now,flags,step)>=0);
                if((previous&1u) && !(flags[side]&1u)) {data_drop[side]++;if(data_drop[side]<data_goal)flags[side]|=1u;}
                if((previous&4u) && !(flags[side]&4u)) {final_drop[side]++;if(final_drop[side]<2)flags[side]|=4u;}
            }
            now+=17;
        }
        detail_a=flags[0];detail_b=flags[1];detail_case=15;
        CHECK(!p[0].e.tx_active && !p[1].e.tx_active && !flags[0] && !flags[1]);
        CHECK(data_drop[0]==data_goal && data_drop[1]==data_goal && final_drop[0]==2 && final_drop[1]==2);
        CHECK(p[0].received==1 && p[1].received==1 && p[0].result_size==600 && p[1].result_size==600);
        CHECK(equal(p[0].result,right_data,600) && equal(p[1].result,left_data,600)); CHECK(clean());
    }
    return 0;
}
EXPORT int audit_rej_poll_busy_window_matrix(void) {
    uint8_t data[70];for(unsigned i=0;i<sizeof(data);i++)data[i]=(uint8_t)i;
    detail_case=0;
    for(unsigned w=1;w<64;w++)for(unsigned state=0;state<4;state++) {
        if(w==1 && (state&2u)) continue;
        detail_window=w;detail_step=state;CHECK(setup(w,3,false));dbt_ertm_frame_t f;
        CHECK(dbt_ertm_engine_send(&p[0].e,data,sizeof(data))==DBT_OK);
        CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && !f.supervisory && f.tx_seq==0);
        CHECK(dbt_ertm_engine_commit(&p[0].e,0)==DBT_OK);
        if(state&2u) {
            CHECK(info(&p[0],1,0,false,DBT_ERTM_UNSEGMENTED,0,data,1,1)==DBT_OK && p[0].e.rej_needed);
            CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.supervisory && f.supervisor==DBT_ERTM_REJ);
            CHECK(dbt_ertm_engine_commit(&p[0].e,2)==DBT_OK && p[0].e.rej_sent);
        }
        if(state&1u) CHECK(dbt_ertm_engine_busy(&p[0].e,true)==DBT_OK);
        CHECK(sf(&p[0],DBT_ERTM_REJ,0,true,false,3)==DBT_OK);
        CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && !f.supervisory && f.tx_seq==0 && f.final);
        CHECK(dbt_ertm_engine_commit(&p[0].e,4)==DBT_OK); CHECK(clean());
    }
    return 0;
}
EXPORT int audit_pinned_teardown_matrix(void) {
    detail_window=4;detail_case=0; static uint8_t payload[100];
    for(unsigned i=0;i<sizeof(payload);i++)payload[i]=(uint8_t)(i*3u+1u);
    for(unsigned mode=0;mode<3;mode++) {
        detail_step=mode;CHECK(setup(4,2,false));dbt_ertm_frame_t pinned,again;
        CHECK(dbt_ertm_engine_send(&p[0].e,payload,sizeof(payload))==DBT_OK);
        CHECK(dbt_ertm_engine_view(&p[0].e,&pinned)==DBT_OK);
        if(mode==1) {
            CHECK(dbt_ertm_engine_commit(&p[0].e,0)==DBT_OK);
            CHECK(sf(&p[0],DBT_ERTM_SREJ,0,false,false,1)==DBT_OK);
            CHECK(dbt_ertm_engine_view(&p[0].e,&pinned)==DBT_OK && pinned.tx_seq==0);
        }
        const uint8_t *data=pinned.data;uint16_t size=pinned.size; CHECK(data && size==48 && equal(data,payload,size));
        if(mode==2) CHECK(dbt_ertm_engine_close(&p[0].e)==DBT_BUSY);
        else {uint8_t bad_req=mode==0 ? 1 : 2; CHECK(sf(&p[0],DBT_ERTM_RR,bad_req,false,false,2)==DBT_INVALID && p[0].e.failed);}
        CHECK(dbt_ertm_engine_close(&p[0].e)==DBT_BUSY && p[0].e.tx_active && p[0].allocs>p[0].frees);
        CHECK(dbt_ertm_engine_view(&p[0].e,&again)==DBT_OK && again.data==data && again.size==size && equal(again.data,payload,size));
        if(mode==1) CHECK(dbt_ertm_engine_commit(&p[0].e,3)==DBT_OK);
        else CHECK(dbt_ertm_engine_cancel_frame(&p[0].e)==DBT_OK);
        CHECK(!p[0].e.tx_active && !p[0].e.tx_sdus && p[0].allocs==p[0].frees); CHECK(clean());
    }
    CHECK(setup(4,2,false)); dbt_ertm_frame_t f; uint8_t data[48];for(unsigned i=0;i<48;i++)data[i]=(uint8_t)i;
    CHECK(info(&p[0],0,0,false,DBT_ERTM_START,96,data,48,0)==DBT_OK && p[0].e.partial);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.supervisory);
    CHECK(info(&p[0],1,0,false,DBT_ERTM_END,0,data,46,1)==DBT_INVALID && p[0].e.failed);
    CHECK(!p[0].e.partial && p[0].allocs==p[0].frees && dbt_ertm_engine_close(&p[0].e)==DBT_BUSY);
    CHECK(dbt_ertm_engine_cancel_frame(&p[0].e)==DBT_OK);CHECK(clean()); return 0;
}
static int random_frame(unsigned from,uint32_t now,unsigned *state,unsigned *budget,unsigned step) {
    dbt_ertm_frame_t f; uint8_t wire[1100]; uint16_t n=0;
    dbt_result_t r=dbt_ertm_engine_view(&p[from].e,&f);
    if(r==DBT_EMPTY)return 0;
    if(r!=DBT_OK){detail_case=20;detail_a=(unsigned)r;detail_b=from;detail_step=step;return -1;}
    if(dbt_ertm_engine_peek(&p[from].e,wire,sizeof(wire),&n)!=DBT_OK){detail_case=21;detail_b=from;detail_step=step;return -1;}
    if(dbt_ertm_engine_commit(&p[from].e,now)!=DBT_OK){detail_case=22;detail_b=from;detail_step=step;return -1;}
    *state=*state*1664525u+1013904223u;
    if(*budget && ((*state>>24)&3u)==0) {--*budget;return 1;}
    r=dbt_ertm_engine_receive(&p[1u-from].e,wire,n,now);
    if(r!=DBT_OK){detail_case=23;detail_a=(unsigned)r;detail_b=from;detail_step=step;return -1;}
    return 1;
}
EXPORT int audit_seeded_loss_and_busy(void) {
    const unsigned windows[]={1,2,3,4,16,32,33,63};
    for(unsigned i=0;i<sizeof(left_data);i++){left_data[i]=(uint8_t)(i*23u+2u);right_data[i]=(uint8_t)(i*31u+7u);}
    for(unsigned wi=0;wi<sizeof(windows)/sizeof(windows[0]);wi++)for(unsigned seed=1;seed<=32;seed++) {
        unsigned w=windows[wi]; detail_window=w;detail_case=seed;CHECK(setup(w,0,true));
        CHECK(dbt_ertm_engine_send(&p[0].e,left_data,200)==DBT_OK && dbt_ertm_engine_send(&p[1].e,right_data,200)==DBT_OK);
        unsigned state=seed*2654435761u, budgets[]={8,8};uint32_t now=0;
        for(unsigned step=0;step<7000 && (p[0].e.tx_active || p[1].e.tx_active);step++) {
            detail_step=step;
            if(step==3)CHECK(dbt_ertm_engine_busy(&p[0].e,true)==DBT_OK);
            if(step==7)CHECK(dbt_ertm_engine_busy(&p[1].e,true)==DBT_OK);
            if(step==15)CHECK(dbt_ertm_engine_busy(&p[0].e,false)==DBT_OK);
            if(step==20)CHECK(dbt_ertm_engine_busy(&p[1].e,false)==DBT_OK);
            if(step>500) budgets[0]=budgets[1]=0;
            dbt_ertm_engine_poll(&p[0].e,now);dbt_ertm_engine_poll(&p[1].e,now);
            CHECK(random_frame(0,now,&state,&budgets[0],step)>=0 && random_frame(1,now,&state,&budgets[1],step)>=0);
            now+=101;
        }
        CHECK(!p[0].e.tx_active && !p[1].e.tx_active && p[0].received==1 && p[1].received==1);
        CHECK(p[0].result_size==200 && p[1].result_size==200 && equal(p[0].result,right_data,200) && equal(p[1].result,left_data,200));
        CHECK(clean());
    }
    return 0;
}
EXPORT int audit_infinite_retransmit_counter(void) {
    const unsigned windows[]={1,32,63}; uint8_t value=9; detail_case=0;
    for(unsigned wi=0;wi<sizeof(windows)/sizeof(windows[0]);wi++) {
        detail_window=windows[wi]; CHECK(setup(windows[wi],0,false)); dbt_ertm_frame_t f;
        CHECK(dbt_ertm_engine_send(&p[0].e,&value,1)==DBT_OK && dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK);
        CHECK(dbt_ertm_engine_commit(&p[0].e,0)==DBT_OK);
        for(unsigned i=0;i<300;i++) {
            detail_step=i;CHECK(sf(&p[0],DBT_ERTM_SREJ,0,false,false,i+1)==DBT_OK);
            CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && !f.supervisory && f.tx_seq==0);
            CHECK(dbt_ertm_engine_commit(&p[0].e,i+1)==DBT_OK && !p[0].e.failed);
        }
        CHECK(sf(&p[0],DBT_ERTM_RR,1,false,false,301)==DBT_OK && !p[0].e.tx_active && p[0].e.tx_sdus==1);
        CHECK(p[0].e.tx_frames==301 && clean());
    }
    return 0;
}
EXPORT int audit_pinned_ack_receive_progress(void) {
    detail_window=4;detail_case=0;CHECK(setup(4,4,false));uint8_t x=8;dbt_ertm_frame_t f;
    CHECK(info(&p[0],0,0,false,DBT_ERTM_UNSEGMENTED,0,&x,1,0)==DBT_OK && p[0].e.rx_absolute==1);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.supervisory && f.req_seq==1);
    for(unsigned seq=1;seq<4;seq++) CHECK(info(&p[0],(uint8_t)seq,0,false,DBT_ERTM_UNSEGMENTED,0,&x,1,seq)==DBT_OK);
    CHECK(p[0].e.rx_absolute==4 && p[0].e.ack_absolute==0 && p[0].received==4);
    CHECK(info(&p[0],4,0,false,DBT_ERTM_UNSEGMENTED,0,&x,1,4)==DBT_OK && p[0].received==4);
    CHECK(dbt_ertm_engine_commit(&p[0].e,5)==DBT_OK && p[0].e.ack_absolute==1 && p[0].e.ack_needed);
    CHECK(info(&p[0],4,0,false,DBT_ERTM_UNSEGMENTED,0,&x,1,6)==DBT_OK && p[0].e.rx_absolute==5);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.req_seq==5);
    CHECK(dbt_ertm_engine_commit(&p[0].e,7)==DBT_OK && p[0].e.ack_absolute==5); CHECK(clean());return 0;
}
EXPORT int audit_bad_fcs_pinned_loan(void) {
    detail_window=4;detail_case=0;CHECK(setup(4,4,true));uint8_t x=42;dbt_ertm_frame_t held,again;
    CHECK(dbt_ertm_engine_send(&p[0].e,&x,1)==DBT_OK && dbt_ertm_engine_view(&p[0].e,&held)==DBT_OK);
    uint8_t sample[]={0x02,0x00,0,1,2,3,4,5,6,7,8,9,0x38,0x60};
    CHECK(dbt_ertm_engine_receive(&p[0].e,sample,sizeof(sample),0)==DBT_OK && p[0].e.bad_fcs==1);
    CHECK(dbt_ertm_engine_view(&p[0].e,&again)==DBT_OK && again.data==held.data && p[0].e.tx_active);
    sample[13]=0x61;CHECK(dbt_ertm_engine_receive(&p[0].e,sample,sizeof(sample),1)==DBT_OK && p[0].e.rej_needed);
    CHECK(dbt_ertm_engine_commit(&p[0].e,2)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].e,&again)==DBT_OK && again.supervisory && again.supervisor==DBT_ERTM_REJ);
    CHECK(dbt_ertm_engine_commit(&p[0].e,3)==DBT_OK); CHECK(clean());return 0;
}
EXPORT int audit_coalesced_final_causes_timeout(void) {
    detail_window=4;detail_case=0;detail_step=0;detail_a=detail_b=0; CHECK(setup(4,3,false));
    uint8_t x=3;dbt_ertm_frame_t f;
    CHECK(dbt_ertm_engine_send(&p[0].e,&x,1)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && !f.supervisory && dbt_ertm_engine_commit(&p[0].e,0)==DBT_OK);
    /* Drop initial data. Peer receives three Polls 12 seconds apart. Its first
     * Final remains pinned by the simulated transport for those retries. */
    dbt_ertm_engine_poll(&p[0].e,2000);CHECK(send_frame(0,2000,0,0)==1 && p[0].e.waiting_final);
    CHECK(dbt_ertm_engine_view(&p[1].e,&f)==DBT_OK && f.final);
    dbt_ertm_engine_poll(&p[0].e,14000);CHECK(send_frame(0,14000,0,1)==1 && p[0].e.poll_attempts==2);
    dbt_ertm_engine_poll(&p[0].e,26000);CHECK(send_frame(0,26000,0,2)==1 && p[0].e.poll_attempts==3);
    /* Lose the first two Final transmissions. The third Poll is unanswered. */
    CHECK(dbt_ertm_engine_commit(&p[1].e,26001)==DBT_OK);
    CHECK(dbt_ertm_engine_view(&p[1].e,&f)==DBT_OK && f.final && dbt_ertm_engine_commit(&p[1].e,26002)==DBT_OK);
    dbt_result_t r=dbt_ertm_engine_view(&p[1].e,&f);
    if(r!=DBT_OK || !f.final) {
        dbt_ertm_engine_poll(&p[0].e,38000);detail_case=30;detail_a=p[0].e.failed;detail_b=(unsigned)r;
        return __LINE__;
    }
    CHECK(send_frame(1,26003,0,3)==1 && !p[0].e.waiting_final && !p[0].e.failed);
    CHECK(clean());return 0;
}
EXPORT int audit_gap_across_sequence_wrap(void) {
    const unsigned windows[]={4,63}; static uint8_t message[100];
    for(unsigned i=0;i<sizeof(message);i++)message[i]=(uint8_t)(i*13u+5u);
    for(unsigned wi=0;wi<2;wi++) {
        unsigned w=windows[wi];detail_window=w;detail_case=0;CHECK(setup(w,0,true)); uint32_t now=0;
        for(unsigned round=0;round<62;round++) {
            CHECK(dbt_ertm_engine_send(&p[0].e,message,1)==DBT_OK);
            for(unsigned step=0;step<30 && p[0].e.tx_active;step++) {
                CHECK(send_frame(0,now,0,step)>=0 && send_frame(1,now,0,step)>=0);now++;
            }
            CHECK(!p[0].e.tx_active && p[1].received==round+1);
        }
        CHECK(p[0].e.next_tx==62 && p[1].e.expected_rx==62);
        CHECK(dbt_ertm_engine_send(&p[0].e,message,sizeof(message))==DBT_OK);
        CHECK(send_frame(0,now++,0,0)==1 && p[1].e.expected_rx==63);
        dbt_ertm_frame_t f;CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.tx_seq==63);
        CHECK(dbt_ertm_engine_commit(&p[0].e,now++)==DBT_OK); /* drop frame 63 */
        CHECK(dbt_ertm_engine_view(&p[0].e,&f)==DBT_OK && f.tx_seq==0);
        CHECK(send_frame(0,now++,0,1)==1);
        if(w==4)CHECK(p[1].e.rej_needed); else CHECK(p[1].e.ambiguous_frames && !p[1].e.rej_needed);
        for(unsigned step=0;step<4000 && p[0].e.tx_active;step++) {
            detail_step=step;dbt_ertm_engine_poll(&p[0].e,now);dbt_ertm_engine_poll(&p[1].e,now);
            CHECK(send_frame(0,now,0,step)>=0 && send_frame(1,now,0,step)>=0);now+=17;
        }
        CHECK(!p[0].e.tx_active && p[1].received==63 && p[1].result_size==100 && equal(p[1].result,message,100));
        CHECK(clean());
    }
    return 0;
}
