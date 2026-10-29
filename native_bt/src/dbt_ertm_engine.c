#include "dbt_ertm_engine.h"
enum {PREP_NONE,PREP_NEW,PREP_RETRY,PREP_ACK,PREP_REJ,PREP_POLL};
struct dbt_ertm_slot {uint16_t offset,size,attempts;uint8_t sar,seq;};
static void zero(void *p,unsigned n){uint8_t *v=p;for(unsigned i=0;i<n;++i)v[i]=0;}
static void copy(void *a,const void *b,unsigned n){uint8_t *x=a;const uint8_t *y=b;for(unsigned i=0;i<n;++i)x[i]=y[i];}
static bool live(const dbt_ertm_engine_t *c){return c && c->active && !c->failed && !c->closing && !c->in_callback;}
static unsigned distance(uint8_t a,uint8_t b){return (a-b)&63u;}
static bool due(uint32_t now,uint32_t at){return (uint32_t)(now-at)<0x80000000u;}
static uint16_t read16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static bool parameters(dbt_ertm_parameters_t p){return p.mtu>=48 && p.mps>=48 && p.mps<=DBT_ERTM_MAX_MPS && p.window && p.window<64;}
static struct dbt_ertm_slot *slot(dbt_ertm_engine_t *c,uint8_t seq){
    unsigned at=distance(seq,c->expected_ack);
    return at<c->count?&c->slots[(c->head+at)%c->capacity]:0;
}
static void partial_clear(dbt_ertm_engine_t *c){
    if(c->partial)c->hooks.release(c->hooks.context,c->partial);
    c->partial=0;c->rx_size=c->rx_cursor=0;
}
static void transmit_clear(dbt_ertm_engine_t *c){
    if(c->slots)c->hooks.release(c->hooks.context,(uint8_t *)c->slots);
    c->slots=0;c->transmit=0;c->tx_active=false;c->capacity=c->head=c->count=0;c->retransmit=0;
}
dbt_result_t dbt_ertm_engine_close(dbt_ertm_engine_t *c){
    if(!c || c->in_callback)return DBT_INVALID;
    partial_clear(c);c->active=false;c->closing=true;
    c->retrans_timer=c->monitor_timer=c->poll_needed=c->final_needed=false;
    if(c->prepared_kind)return DBT_BUSY;
    transmit_clear(c);return DBT_OK;
}
static dbt_result_t fail(dbt_ertm_engine_t *c,dbt_result_t result){c->failed=true;(void)dbt_ertm_engine_close(c);return result;}

static void completed(dbt_ertm_engine_t *c){
    if(c->failed || c->closing)return;
    if(c->tx_active && c->generated && !c->count && c->prepared_kind!=PREP_NEW && c->prepared_kind!=PREP_RETRY){
        transmit_clear(c);++c->tx_sdus;
    }
}
static void start_retrans(dbt_ertm_engine_t *c,uint32_t now){
    c->retrans_timer=c->count && !c->remote_busy && !c->waiting_final;
    if(c->retrans_timer)c->retrans_at=now+c->local.retransmission_ms;
}
dbt_result_t dbt_ertm_engine_init(dbt_ertm_engine_t *c,dbt_ertm_hooks_t hooks,uint16_t local_cid,uint16_t remote_cid,
    dbt_ertm_parameters_t local,dbt_ertm_parameters_t peer,bool fcs){
    if(!c || !hooks.allocate || !hooks.release || !hooks.sdu || local_cid<0x40 || remote_cid<0x40 ||
       !parameters(local) || !parameters(peer) || local.retransmission_ms<2000 || local.monitor_ms<12000)return DBT_INVALID;
    zero(c,sizeof(*c));c->hooks=hooks;c->local=local;c->peer=peer;c->local_cid=local_cid;c->remote_cid=remote_cid;c->fcs=fcs;c->active=true;
    return DBT_OK;
}
dbt_result_t dbt_ertm_engine_send(dbt_ertm_engine_t *c,const uint8_t *bytes,uint16_t size){
    if(!live(c) || (size && !bytes))return DBT_INVALID;
    if(c->tx_active)return DBT_BUSY;
    if(size>c->peer.mtu)return DBT_NO_SPACE;
    uint32_t frames=size?((uint32_t)size+c->peer.mps-1u)/c->peer.mps:1u;
    uint8_t count=(uint8_t)(frames<c->peer.window?frames:c->peer.window);
    uint8_t *storage=c->hooks.allocate(c->hooks.context,(uint16_t)(count*sizeof(struct dbt_ertm_slot)));
    if(!storage)return DBT_NO_SPACE;
    if((uintptr_t)storage%_Alignof(struct dbt_ertm_slot)){c->hooks.release(c->hooks.context,storage);return DBT_NO_SPACE;}
    c->slots=(struct dbt_ertm_slot *)storage;c->capacity=count;c->head=c->count=0;
    c->transmit=bytes;c->tx_size=size;c->tx_cursor=0;c->tx_active=true;c->generated=false;return DBT_OK;
}
static void mark_all(dbt_ertm_engine_t *c){
    for(unsigned i=0;i<c->count;++i)c->retransmit|=UINT64_C(1)<<c->slots[(c->head+i)%c->capacity].seq;
}
static dbt_result_t acknowledge(dbt_ertm_engine_t *c,uint8_t req,uint32_t now){
    unsigned count=distance(req,c->expected_ack);if(count>c->count)return fail(c,DBT_INVALID);
    for(unsigned i=0;i<count;++i){c->retransmit&=~(UINT64_C(1)<<c->slots[c->head].seq);c->head=(uint8_t)((c->head+1u)%c->capacity);--c->count;}
    c->expected_ack=req;
    if(!c->count)c->retrans_timer=false;
    (void)now;
    return DBT_OK;
}
static void consume_final(dbt_ertm_engine_t *c,bool final,uint32_t now){
    if(!final || !c->waiting_final)return;
    c->waiting_final=c->monitor_timer=false;c->poll_attempts=0;start_retrans(c,now);
    if(c->clear_busy_pending){c->clear_busy_pending=c->rnr_sent=false;c->poll_needed=true;c->retrans_timer=false;}
}
static void poll_response(dbt_ertm_engine_t *c,bool selective,uint8_t seq){
    if(c->final_epoch==UINT32_MAX){(void)fail(c,DBT_INVALID);return;}
    ++c->final_epoch;c->final_needed=true;c->final_selective=selective;c->final_seq=seq;
}

static dbt_result_t payload(dbt_ertm_engine_t *c,const dbt_ertm_frame_t *f,const uint8_t **ready,uint16_t *size){
    *ready=0;*size=0;
    if(f->sar==DBT_ERTM_UNSEGMENTED){
        if(c->partial || f->size>c->local.mtu)return fail(c,DBT_INVALID);
        *ready=f->data;*size=f->size;return DBT_OK;
    }
    if(f->sar==DBT_ERTM_START){
        if(c->partial || !f->sdu_length || f->sdu_length>c->local.mtu || f->size>f->sdu_length)return fail(c,DBT_INVALID);
        c->partial=c->hooks.allocate(c->hooks.context,f->sdu_length);if(!c->partial)return fail(c,DBT_NO_SPACE);
        c->rx_size=f->sdu_length;c->rx_cursor=f->size;copy(c->partial,f->data,f->size);return DBT_OK;
    }
    if(!c->partial || (uint32_t)c->rx_cursor+f->size>c->rx_size)return fail(c,DBT_INVALID);
    uint16_t next=(uint16_t)(c->rx_cursor+f->size);
    if((f->sar==DBT_ERTM_END && next!=c->rx_size) || (f->sar==DBT_ERTM_CONTINUE && next==c->rx_size))return fail(c,DBT_INVALID);
    copy(c->partial+c->rx_cursor,f->data,f->size);c->rx_cursor=next;
    if(f->sar==DBT_ERTM_END){*ready=c->partial;*size=c->rx_size;}return DBT_OK;
}
dbt_result_t dbt_ertm_engine_receive(dbt_ertm_engine_t *c,const uint8_t *bytes,uint16_t size,uint32_t now){
    if(!live(c))return DBT_INVALID;
    dbt_ertm_frame_t f;dbt_ertm_wire_result_t wire=dbt_ertm_frame_decode(c->local_cid,c->fcs,bytes,size,&f);
    if(wire==DBT_ERTM_WIRE_BAD_FCS){++c->bad_fcs;return DBT_OK;}
    if(wire!=DBT_ERTM_WIRE_OK){
        if(bytes && size>=2 && (read16(bytes)&0x91u)==0x91u){++c->ignored_frames;return DBT_OK;}
        return fail(c,DBT_INVALID);
    }
    bool retry_request=f.supervisory && (f.supervisor==DBT_ERTM_REJ || f.supervisor==DBT_ERTM_SREJ);
    unsigned ack_count=distance(f.req_seq,c->expected_ack);
    if((retry_request && ack_count>=c->count) || (!retry_request && ack_count>c->count) ||
       (!f.supervisory && f.size>c->local.mps))return fail(c,DBT_INVALID);
    if(retry_request && c->peer.max_transmit){
        unsigned end=f.supervisor==DBT_ERTM_SREJ?ack_count+1u:c->count;
        for(unsigned i=ack_count;i<end;++i)if(c->slots[(c->head+i)%c->capacity].attempts>=c->peer.max_transmit)return fail(c,DBT_INVALID);
    }
    if(f.final && !c->waiting_final){++c->ignored_frames;return DBT_OK;}
    ++c->rx_frames;
    if(!f.supervisory){
        if(f.size>c->local.mps || distance(f.req_seq,c->expected_ack)>c->count)return fail(c,DBT_INVALID);
        if(c->rx_absolute>UINT32_MAX-64u || c->ack_absolute>UINT32_MAX-64u)return fail(c,DBT_INVALID);
        unsigned delta=distance(f.tx_seq,c->expected_rx),back=distance(c->expected_rx,f.tx_seq);
        unsigned history=c->rx_absolute<c->local.window?c->rx_absolute:c->local.window;
        bool future=c->rx_absolute+delta<c->ack_absolute+c->local.window;
        bool duplicate=back && back<=history;
        bool expected=!delta && future,unexpected=delta && future && !duplicate;
        if(duplicate && future){++c->ambiguous_frames;c->ack_needed=true;}
        if(c->rej_sent && ((expected && c->rx_absolute==c->rej_absolute) ||
           (duplicate && c->rx_absolute-back==c->rej_absolute)))c->rej_sent=false;
        if(!expected && !unexpected && !duplicate){if(c->local.window>32)return fail(c,DBT_INVALID);++c->ignored_frames;return DBT_OK;}
        if(acknowledge(c,f.req_seq,now)!=DBT_OK)return DBT_INVALID;
        consume_final(c,f.final,now);
        if(!c->local_busy && expected){
            const uint8_t *ready;uint16_t count;dbt_result_t result=payload(c,&f,&ready,&count);if(result!=DBT_OK)return result;
            ++c->rx_absolute;c->expected_rx=(uint8_t)(c->rx_absolute&63u);c->rej_needed=false;c->ack_needed=true;
            if(f.final){if(!c->rej_actioned)mark_all(c);else c->rej_actioned=false;}
            if(ready){++c->rx_sdus;c->in_callback=true;c->hooks.sdu(c->hooks.context,ready,count);c->in_callback=false;if(ready==c->partial)partial_clear(c);}
        }else if(!c->local_busy && unexpected && !c->rej_sent)c->rej_needed=true;
        completed(c);return DBT_OK;
    }
    bool retry=f.supervisor==DBT_ERTM_REJ || f.supervisor==DBT_ERTM_SREJ;
    if(retry && distance(f.req_seq,c->expected_ack)>=c->count)return fail(c,DBT_INVALID);
    bool was_waiting=c->waiting_final,was_busy=c->remote_busy;
    if(f.supervisor!=DBT_ERTM_SREJ || f.poll){if(acknowledge(c,f.req_seq,now)!=DBT_OK)return DBT_INVALID;}
    consume_final(c,f.final,now);
    if(f.supervisor==DBT_ERTM_RNR){
        c->remote_busy=true;c->retrans_timer=false;
        if(f.poll)poll_response(c,false,0);
        else if(c->rej_sent){c->ack_needed=true;c->force_ready_ack=true;}
    }
    else{
        c->remote_busy=false;
        if(f.supervisor==DBT_ERTM_RR){
            if(f.final){if(!c->rej_actioned)mark_all(c);else c->rej_actioned=false;}
            if(f.poll)poll_response(c,false,0);
            if(was_busy || f.final)start_retrans(c,now);
            if(c->rej_sent && !f.poll && !f.final)c->ack_needed=true;
        }else if(f.supervisor==DBT_ERTM_REJ){
            if(!f.final || !c->rej_actioned)mark_all(c);else c->rej_actioned=false;
            if(was_waiting && !f.final)c->rej_actioned=true;
            if(f.poll)poll_response(c,true,f.req_seq);
        }else{
            if(f.final && c->srej_actioned && c->selective_seq==f.req_seq)c->srej_actioned=false;
            else c->retransmit|=UINT64_C(1)<<f.req_seq;
            if(was_waiting && !f.final){c->srej_actioned=true;c->selective_seq=f.req_seq;}
            if(f.poll)poll_response(c,true,f.req_seq);
        }
    }
    if(c->failed)return DBT_INVALID;
    completed(c);return DBT_OK;
}
dbt_result_t dbt_ertm_engine_busy(dbt_ertm_engine_t *c,bool busy){
    if(!live(c))return DBT_INVALID;
    if(c->local_busy==busy)return DBT_OK;
    c->local_busy=busy;
    if(busy)c->ack_needed=true;
    else if(c->rnr_sent){if(c->waiting_final)c->clear_busy_pending=true;else{c->rnr_sent=false;c->poll_needed=true;c->retrans_timer=false;}}
    return DBT_OK;
}
void dbt_ertm_engine_poll(dbt_ertm_engine_t *c,uint32_t now){
    if(!live(c))return;
    if(c->monitor_timer && due(now,c->monitor_at)){
        c->monitor_timer=false;
        if(c->peer.max_transmit && c->poll_attempts>=c->peer.max_transmit){(void)fail(c,DBT_INVALID);return;}
        c->poll_needed=true;
    }
    if(c->retrans_timer && due(now,c->retrans_at)){c->retrans_timer=false;c->poll_needed=true;}
}

static bool retry_frame(dbt_ertm_engine_t *c,dbt_ertm_frame_t *f,uint8_t seq){
    struct dbt_ertm_slot *item=slot(c,seq);if(!item || c->remote_busy)return false;
    if(c->peer.max_transmit && item->attempts>=c->peer.max_transmit){(void)fail(c,DBT_INVALID);return false;}
    f->data=item->size?c->transmit+item->offset:0;f->size=item->size;f->sdu_length=c->tx_size;f->sar=item->sar;f->tx_seq=item->seq;
    c->prepared_kind=PREP_RETRY;c->prepared_seq=seq;return true;
}
static bool any_retry(dbt_ertm_engine_t *c,dbt_ertm_frame_t *f){
    if(c->remote_busy)return false;
    for(unsigned i=0;i<c->count;++i){uint8_t seq=c->slots[(c->head+i)%c->capacity].seq;
        if(c->retransmit&(UINT64_C(1)<<seq))return retry_frame(c,f,seq);
    }
    return false;
}
static bool new_frame(dbt_ertm_engine_t *c,dbt_ertm_frame_t *f){
    if(!c->tx_active || c->generated || c->remote_busy || c->waiting_final || c->count>=c->capacity)return false;
    uint16_t left=(uint16_t)(c->tx_size-c->tx_cursor);f->size=left>c->peer.mps?c->peer.mps:left;
    f->data=f->size?c->transmit+c->tx_cursor:0;f->sdu_length=c->tx_size;f->tx_seq=c->next_tx;
    f->sar=c->tx_size<=c->peer.mps?DBT_ERTM_UNSEGMENTED:!c->tx_cursor?DBT_ERTM_START:left<=c->peer.mps?DBT_ERTM_END:DBT_ERTM_CONTINUE;
    c->prepared_kind=PREP_NEW;c->prepared_seq=f->tx_seq;return true;
}
static void supervisor(dbt_ertm_engine_t *c,dbt_ertm_frame_t *f,uint8_t kind,uint8_t function){
    f->supervisory=true;f->supervisor=function;c->prepared_kind=kind;
}
dbt_result_t dbt_ertm_engine_view(dbt_ertm_engine_t *c,dbt_ertm_frame_t *out){
    if(!c || !out || c->in_callback)return DBT_INVALID;
    if(c->prepared_kind){copy(out,&c->prepared,sizeof(*out));return DBT_OK;}
    if(!live(c))return DBT_INVALID;
    dbt_ertm_frame_t f;zero(&f,sizeof(f));f.req_seq=c->expected_rx;
    if(c->final_needed){
        bool data=false;
        if(c->final_selective)data=retry_frame(c,&f,c->final_seq);
        else if(!c->local_busy && !c->rej_sent)data=new_frame(c,&f);
        if(c->failed)return DBT_INVALID;
        if(!data)supervisor(c,&f,PREP_ACK,c->local_busy && !c->rej_sent?DBT_ERTM_RNR:DBT_ERTM_RR);
        f.final=true;c->prepared_final_epoch=c->final_served_epoch+1u;
    }else if(c->rej_needed && !c->rej_sent){supervisor(c,&f,PREP_REJ,DBT_ERTM_REJ);}
    else if(c->poll_needed){supervisor(c,&f,PREP_POLL,c->local_busy?DBT_ERTM_RNR:DBT_ERTM_RR);f.poll=true;}
    else{
        bool data=false;
        if(!(c->ack_needed && c->local_busy)){data=any_retry(c,&f);if(!data && !c->failed)data=new_frame(c,&f);}
        if(c->failed)return DBT_INVALID;
        if(!data){if(!c->ack_needed)return DBT_EMPTY;supervisor(c,&f,PREP_ACK,c->force_ready_ack?DBT_ERTM_RR:c->local_busy?DBT_ERTM_RNR:DBT_ERTM_RR);}
    }
    c->prepared_busy=c->local_busy;c->prepared_ready_ack=c->force_ready_ack;c->prepared_ack_absolute=c->rx_absolute;
    copy(&c->prepared,&f,sizeof(f));copy(out,&f,sizeof(f));return DBT_OK;
}
dbt_result_t dbt_ertm_engine_peek(dbt_ertm_engine_t *c,uint8_t *output,uint16_t capacity,uint16_t *written){
    dbt_ertm_frame_t frame;dbt_result_t result=dbt_ertm_engine_view(c,&frame);if(result!=DBT_OK)return result;
    return dbt_ertm_frame_encode(c->remote_cid,c->fcs,&frame,output,capacity,written);
}
dbt_result_t dbt_ertm_engine_commit(dbt_ertm_engine_t *c,uint32_t now){
    if(!c || c->in_callback || !c->prepared_kind)return DBT_INVALID;
    if(c->closing || c->failed){c->prepared_kind=PREP_NONE;c->prepared.data=0;transmit_clear(c);return DBT_OK;}
    if(!live(c))return DBT_INVALID;
    dbt_ertm_frame_t *f=&c->prepared;uint8_t kind=c->prepared_kind;
    if(kind==PREP_NEW){
        struct dbt_ertm_slot *item=&c->slots[(c->head+c->count)%c->capacity];
        item->offset=c->tx_cursor;item->size=f->size;item->attempts=1;item->sar=f->sar;item->seq=f->tx_seq;
        ++c->count;c->next_tx=(uint8_t)((c->next_tx+1u)&63u);c->tx_cursor=(uint16_t)(c->tx_cursor+f->size);
        c->generated=c->tx_cursor==c->tx_size;start_retrans(c,now);
    }else if(kind==PREP_RETRY){
        struct dbt_ertm_slot *item=slot(c,c->prepared_seq);
        if(item && item->attempts<UINT16_MAX)++item->attempts;
        c->retransmit&=~(UINT64_C(1)<<c->prepared_seq);if(!c->retrans_timer)start_retrans(c,now);
    }else if(kind==PREP_POLL){
        c->poll_needed=false;c->retrans_timer=false;
        if(!c->waiting_final)c->poll_attempts=0;
        if(c->poll_attempts<UINT16_MAX)++c->poll_attempts;
        c->waiting_final=true;c->monitor_timer=true;c->monitor_at=now+c->local.monitor_ms;
    }else if(kind==PREP_REJ){c->rej_sent=true;c->rej_needed=false;c->rej_absolute=c->prepared_ack_absolute;}
    if(c->prepared_ack_absolute>c->ack_absolute)c->ack_absolute=c->prepared_ack_absolute;
    c->last_ack=(uint8_t)(c->ack_absolute&63u);if(c->expected_rx==f->req_seq)c->ack_needed=false;
    if(f->supervisory && f->supervisor==DBT_ERTM_RR && c->prepared_ready_ack)c->force_ready_ack=false;
    if(f->supervisory && f->supervisor==DBT_ERTM_RNR){
        c->rnr_sent=true;
        if(!c->local_busy){if(c->waiting_final)c->clear_busy_pending=true;else{c->rnr_sent=false;c->poll_needed=true;}}
    }
    if(c->local_busy!=c->prepared_busy)c->ack_needed=true;
    if(f->final){
        c->final_served_epoch=c->prepared_final_epoch;c->final_needed=c->final_served_epoch<c->final_epoch;
        if(!c->final_needed)c->final_selective=false;
    }
    ++c->tx_frames;c->prepared_kind=PREP_NONE;c->prepared.data=0;completed(c);return DBT_OK;
}
dbt_result_t dbt_ertm_engine_cancel_frame(dbt_ertm_engine_t *c){
    if(!c || c->in_callback || !c->prepared_kind)return DBT_INVALID;
    c->prepared_kind=PREP_NONE;c->prepared.data=0;
    if(c->closing || c->failed)transmit_clear(c);else completed(c);
    return DBT_OK;
}
uint32_t dbt_ertm_engine_wait_ms(const dbt_ertm_engine_t *c,uint32_t now){
    if(!live(c))return UINT32_MAX;
    bool data=c->tx_active && !c->remote_busy &&
        (c->retransmit || (!c->generated && !c->waiting_final && c->count<c->capacity));
    if(c->prepared_kind || c->final_needed || c->poll_needed || c->rej_needed || c->ack_needed || data)return 0;
    uint32_t wait=UINT32_MAX;
    if(c->retrans_timer){uint32_t n=due(now,c->retrans_at)?0:c->retrans_at-now;if(n<wait)wait=n;}
    if(c->monitor_timer){uint32_t n=due(now,c->monitor_at)?0:c->monitor_at-now;if(n<wait)wait=n;}
    return wait;
}
