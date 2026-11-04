#include "dbt_rfcomm_bearer.h"
static void app_finish(dbt_rfcomm_bearer_t *b,dbt_tx_outcome_t outcome){
    dbt_tx_done_t done=b->app_done;void *context=b->app_context;
    b->app_pending=false;b->app_done=0;b->app_context=0;
    if(done){b->in_callback=true;done(context,outcome);b->in_callback=false;}
}
static void release_buffers(dbt_rfcomm_bearer_t *b){
    dbt_runtime_t *r=b->runtime;if(!r || !r->hooks.release)return;
    for(unsigned i=0;i<DBT_RFCOMM_BEARER_RX_SLOTS;++i)if(b->rx[i]){r->hooks.release(r->hooks.context,b->rx[i]);b->rx[i]=0;}
    if(b->tx){r->hooks.release(r->hooks.context,b->tx);b->tx=0;}
}
static void complete(void *context,dbt_tx_outcome_t outcome){
    dbt_rfcomm_bearer_t *b=context;b->tx_active=false;
    dbt_result_t r;
    if(b->tx_is_dlc){
        r=outcome==DBT_TX_ADMITTED?dbt_rfcomm_dlc_commit(&b->dlc,b->runtime->now_ms):dbt_rfcomm_dlc_cancel(&b->dlc);
        if(b->tx_is_app)app_finish(b,outcome);
    }else r=outcome==DBT_TX_ADMITTED?dbt_rfcomm_mux_commit(&b->mux,b->runtime->now_ms):dbt_rfcomm_mux_cancel(&b->mux);
    if(r!=DBT_OK || outcome!=DBT_TX_ADMITTED)b->faulted=true;
    b->tx_is_dlc=b->tx_is_app=false;
}
static bool legacy_stop(const dbt_rfcomm_bearer_t *b,const uint8_t *bytes,uint16_t size){
    if(b->dlc.credit_mode || !b->app_pending || !dbt_rfcomm_dlc_queued_data(&b->dlc))return false;
    dbt_rfcomm_frame_t f;dbt_rfcomm_mcc_t m;
    if(dbt_rfcomm_decode(bytes,size,false,&f)!=DBT_OK || f.dlci || f.type!=DBT_RFCOMM_UIH ||
       f.cr!=dbt_rfcomm_cr(!b->mux.initiator,true) || dbt_rfcomm_mcc_decode(f.data,f.size,&m)!=DBT_OK || !m.command)return false;
    if(m.type==0x18 && m.size==0)return true;
    return m.type==0x38 && (m.size==2 || m.size==3) && m.data &&
        m.data[0]==(uint8_t)((b->dlc.dlci<<2)|3u) && (m.data[1]&2u);
}
dbt_result_t dbt_rfcomm_bearer_bind(dbt_rfcomm_bearer_t *b,dbt_runtime_t *r,uint16_t handle,
                                    uint16_t cid,bool initiator,uint32_t t1){
    if(!b || !r || !r->ready || r->in_poll || !r->hooks.allocate || !r->hooks.release)return DBT_INVALID;
    if(b->runtime && !b->closed)return DBT_BUSY;
    const dbt_channel_t *c=dbt_channels_find(&r->channels,handle,cid);
    const dbt_link_t *link=dbt_acl_find(&r->acl,handle);
    if(!c || !link || c->link_generation!=link->generation || c->psm!=3 || c->le || c->ertm || c->state!=DBT_CH_OPEN)return DBT_INVALID;
    uint16_t cap=c->rx_mtu<c->tx_mtu?c->rx_mtu:c->tx_mtu;if(cap>DBT_RFCOMM_BEARER_FRAME_MAX)cap=DBT_RFCOMM_BEARER_FRAME_MAX;
    if(cap<28)return DBT_INVALID;
    *b=(dbt_rfcomm_bearer_t){0};b->runtime=r;b->serial=c->serial;b->frame_capacity=cap;
    for(unsigned i=0;i<DBT_RFCOMM_BEARER_RX_SLOTS;++i){b->rx[i]=r->hooks.allocate(r->hooks.context,cap);if(!b->rx[i]){release_buffers(b);*b=(dbt_rfcomm_bearer_t){0};return DBT_NO_SPACE;}}
    b->tx=r->hooks.allocate(r->hooks.context,cap);if(!b->tx){release_buffers(b);*b=(dbt_rfcomm_bearer_t){0};return DBT_NO_SPACE;}
    dbt_result_t result=dbt_rfcomm_mux_init(&b->mux,initiator,handle,link->generation,cid,t1);
    if(result==DBT_OK)result=dbt_rfcomm_dlc_init(&b->dlc,initiator,b->tx,cap,(dbt_rfcomm_dlc_hooks_t){0});
    if(result==DBT_OK && initiator)result=dbt_rfcomm_mux_start(&b->mux);
    if(result!=DBT_OK){release_buffers(b);*b=(dbt_rfcomm_bearer_t){0};}return result;
}
dbt_result_t dbt_rfcomm_bearer_enqueue(dbt_rfcomm_bearer_t *b,const dbt_channel_t *c,const uint8_t *data,uint16_t size){
    if(!b || !c || !data || b->closed || b->faulted || c->handle!=b->mux.handle || c->local_cid!=b->mux.local_cid ||
       c->serial!=b->serial || c->link_generation!=b->mux.generation)return DBT_INVALID;
    if(size>b->frame_capacity || size<4){b->faulted=true;return DBT_NO_SPACE;}
    if(b->count==DBT_RFCOMM_BEARER_RX_SLOTS){b->faulted=true;return DBT_NO_SPACE;}
    unsigned slot=(b->head+b->count)%DBT_RFCOMM_BEARER_RX_SLOTS;
    for(unsigned i=0;i<size;++i)b->rx[slot][i]=data[i];
    b->rx_size[slot]=size;++b->count;return DBT_OK;
}
void dbt_rfcomm_bearer_poll(dbt_rfcomm_bearer_t *b,uint32_t now){
    if(!b || !b->runtime || b->closed || b->runtime->in_poll || b->in_callback || b->dlc.in_callback)return;
    dbt_runtime_t *r=b->runtime;const dbt_channel_t *c=dbt_channels_find(&r->channels,b->mux.handle,b->mux.local_cid);
    const dbt_link_t *link=dbt_acl_find(&r->acl,b->mux.handle);
    if(!link || link->generation!=b->mux.generation || !c || c->serial!=b->serial || c->state!=DBT_CH_OPEN){
        if(b->tx_active)return;
        if(b->mux.tx_pinned)(void)dbt_rfcomm_mux_cancel(&b->mux);
        if(b->dlc.tx_pinned)(void)dbt_rfcomm_dlc_cancel(&b->dlc);
        if(b->app_pending)app_finish(b,(!link || link->generation!=b->mux.generation)?DBT_TX_LINK_LOST:DBT_TX_ABORTED);
        (void)dbt_rfcomm_dlc_shutdown(&b->dlc,!link || link->generation!=b->mux.generation);
        (void)dbt_rfcomm_mux_link_lost(&b->mux,b->mux.handle,b->mux.generation);release_buffers(b);b->closed=true;return;
    }
    if(!b->faulted && b->count){
        uint8_t *data=b->rx[b->head];uint16_t size=b->rx_size[b->head];
        dbt_result_t result;
        if(!(b->tx_active && b->tx_is_dlc) && legacy_stop(b,data,size)){
            (void)dbt_rfcomm_dlc_abort_queued_data(&b->dlc);app_finish(b,DBT_TX_ABORTED);
        }
        if((data[0]>>2)==0 && ((data[1]&0xefu)!=DBT_RFCOMM_UIH)) {
            if((data[1]&0xefu)==DBT_RFCOMM_DISC && b->dlc.tx_ready)result=DBT_BUSY;
            else result=dbt_rfcomm_mux_receive(&b->mux,c->handle,c->link_generation,c->local_cid,data,size);
        }
        else if(b->mux.state==DBT_RFCOMM_MUX_OPEN)result=dbt_rfcomm_dlc_receive(&b->dlc,data,size);
        else result=DBT_INVALID;
        if(result==DBT_OK){b->head=(uint8_t)((b->head+1u)%DBT_RFCOMM_BEARER_RX_SLOTS);--b->count;}
        else if(result!=DBT_BUSY)b->faulted=true;
    }
    dbt_rfcomm_mux_poll(&b->mux,now);dbt_rfcomm_dlc_poll(&b->dlc,now);
    if(b->mux.state==DBT_RFCOMM_MUX_DOWN && !b->tx_active && b->dlc.state!=DBT_RFCOMM_DLC_IDLE)
        (void)dbt_rfcomm_dlc_shutdown(&b->dlc,false);
    if(b->mux.timed_out || b->dlc.state==DBT_RFCOMM_DLC_FAILED)b->faulted=true;
    if(b->faulted){
        if(!b->tx_active && b->mux.tx_pinned)(void)dbt_rfcomm_mux_cancel(&b->mux);
        if(!b->tx_active && b->dlc.tx_pinned)(void)dbt_rfcomm_dlc_cancel(&b->dlc);
        if(!b->tx_active && b->app_pending)app_finish(b,DBT_TX_ABORTED);
        (void)dbt_channels_close(&r->channels,b->mux.handle,b->mux.local_cid,now);return;
    }
    if(!b->tx_active){
        const uint8_t *frame=0;uint16_t size=0;bool is_dlc=false;
        dbt_result_t result=dbt_rfcomm_mux_peek(&b->mux,&frame,&size);
        if(result==DBT_EMPTY && b->mux.state==DBT_RFCOMM_MUX_OPEN){result=dbt_rfcomm_dlc_peek(&b->dlc,&frame,&size);is_dlc=true;}
        if(result==DBT_OK){
            result=dbt_runtime_send(r,b->mux.handle,b->mux.local_cid,frame,size,complete,b);
            if(result==DBT_OK){b->tx_active=true;b->tx_is_dlc=is_dlc;b->tx_is_app=is_dlc && b->app_pending;}
            else if(result!=DBT_BUSY)b->faulted=true;
        }
    }
}
static bool usable(const dbt_rfcomm_bearer_t *b){return b && !b->closed && !b->faulted && !b->in_callback && !b->dlc.in_callback && b->mux.state==DBT_RFCOMM_MUX_OPEN;}
dbt_result_t dbt_rfcomm_bearer_listen(dbt_rfcomm_bearer_t *b,uint8_t scn,uint16_t n1,uint8_t k,dbt_rfcomm_dlc_hooks_t hooks){
    if(!usable(b))return DBT_BUSY;
    if(k>DBT_RFCOMM_BEARER_RX_CREDITS)return DBT_INVALID;
    dbt_result_t r=dbt_rfcomm_dlc_listen(&b->dlc,scn,n1,k);if(r==DBT_OK)b->dlc.hooks=hooks;return r;
}
dbt_result_t dbt_rfcomm_bearer_open(dbt_rfcomm_bearer_t *b,uint8_t scn,uint16_t n1,uint8_t k,dbt_rfcomm_dlc_hooks_t hooks){
    if(!usable(b))return DBT_BUSY;
    if(k>DBT_RFCOMM_BEARER_RX_CREDITS)return DBT_INVALID;
    dbt_result_t r=dbt_rfcomm_dlc_open(&b->dlc,scn,n1,k);if(r==DBT_OK)b->dlc.hooks=hooks;return r;
}
dbt_result_t dbt_rfcomm_bearer_send(dbt_rfcomm_bearer_t *b,const uint8_t *data,uint16_t size,dbt_tx_done_t done,void *ctx){
    if(!usable(b))return DBT_BUSY;
    if(b->app_pending)return DBT_BUSY;
    dbt_result_t r=dbt_rfcomm_dlc_send(&b->dlc,data,size);if(r==DBT_OK){b->app_pending=true;b->app_done=done;b->app_context=ctx;}return r;
}
dbt_result_t dbt_rfcomm_bearer_grant(dbt_rfcomm_bearer_t *b,uint8_t n){
    if(!usable(b))return DBT_BUSY;
    if((uint32_t)b->dlc.credits.rx_outstanding+n>DBT_RFCOMM_BEARER_RX_CREDITS)return DBT_NO_SPACE;
    return dbt_rfcomm_dlc_grant(&b->dlc,n);
}
dbt_result_t dbt_rfcomm_bearer_close_dlc(dbt_rfcomm_bearer_t *b){return usable(b)?dbt_rfcomm_dlc_close(&b->dlc):DBT_BUSY;}
