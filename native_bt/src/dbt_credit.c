#include "dbt_credit.h"
static void copy(uint8_t *to,const uint8_t *from,unsigned n){for(unsigned i=0;i<n;++i)to[i]=from[i];}
static bool valid(dbt_credit_parameters_t p){return p.mtu>=23 && p.mps>=23 && p.mps<=65533u;}
static bool live(const dbt_credit_t *c){return c && c->active && !c->failed && !c->in_callback;}
static void partial_clear(dbt_credit_t *c){
    if(c->partial)c->hooks.release(c->hooks.context,c->partial);
    c->partial=0;c->expected=c->received=0;
}
static dbt_result_t fail(dbt_credit_t *c,dbt_result_t reason){
    partial_clear(c);c->failed=true;c->pending_grant=0;c->transmit=0;c->tx_active=c->pending_frame=false;return reason;
}
dbt_result_t dbt_credit_init(dbt_credit_t *c,dbt_credit_hooks_t hooks,dbt_credit_parameters_t local,dbt_credit_parameters_t peer){
    if(!c || !hooks.allocate || !hooks.release || !hooks.sdu || !valid(local) || !valid(peer))return DBT_INVALID;
    uint8_t *p=(uint8_t *)c;for(unsigned i=0;i<sizeof(*c);++i)p[i]=0;
    c->hooks=hooks;c->local=local;c->peer=peer;c->active=true;return DBT_OK;
}
void dbt_credit_close(dbt_credit_t *c){
    if(!c || c->in_callback)return;
    partial_clear(c);c->active=false;c->transmit=0;c->tx_active=c->pending_frame=false;c->pending_grant=0;
}
static void deliver(dbt_credit_t *c,const uint8_t *bytes,uint16_t size){
    ++c->sdus_received;c->in_callback=true;c->hooks.sdu(c->hooks.context,bytes,size);c->in_callback=false;
}
dbt_result_t dbt_credit_receive(dbt_credit_t *c,const uint8_t *bytes,uint16_t size){
    if(!live(c) || (size && !bytes))return DBT_INVALID;
    if(!c->local.credits || size>c->local.mps)return fail(c,DBT_INVALID);
    --c->local.credits;++c->frames_received;
    if(!c->partial){
        if(size<2)return fail(c,DBT_INVALID);
        uint16_t expected=dbt_read_le16(bytes),received=(uint16_t)(size-2u);
        if(expected>c->local.mtu || received>expected)return fail(c,DBT_INVALID);
        if(received==expected){deliver(c,bytes+2,expected);return DBT_OK;}
        uint8_t *storage=c->hooks.allocate(c->hooks.context,expected);
        if(!storage)return fail(c,DBT_NO_SPACE);
        copy(storage,bytes+2,received);c->partial=storage;c->received=received;c->expected=expected;
        return DBT_OK;
    }
    if((uint32_t)c->received+size>c->expected)return fail(c,DBT_INVALID);
    copy(c->partial+c->received,bytes,size);c->received=(uint16_t)(c->received+size);
    if(c->received==c->expected){deliver(c,c->partial,c->expected);partial_clear(c);}
    return DBT_OK;
}
dbt_result_t dbt_credit_add_peer_credits(dbt_credit_t *c,uint16_t credits){
    if(!live(c))return DBT_INVALID;
    if((uint32_t)c->peer.credits+credits>UINT16_MAX)return fail(c,DBT_INVALID);
    c->peer.credits=(uint16_t)(c->peer.credits+credits);return DBT_OK;
}
dbt_result_t dbt_credit_grant_prepare(dbt_credit_t *c,uint16_t credits){
    if(!live(c) || !credits)return DBT_INVALID;
    if(c->pending_grant)return DBT_BUSY;
    if((uint32_t)c->local.credits+credits>UINT16_MAX)return DBT_NO_SPACE;
    c->pending_grant=credits;return DBT_OK;
}
dbt_result_t dbt_credit_grant_commit(dbt_credit_t *c){
    if(!live(c) || !c->pending_grant)return DBT_INVALID;
    c->local.credits=(uint16_t)(c->local.credits+c->pending_grant);c->pending_grant=0;return DBT_OK;
}
void dbt_credit_grant_cancel(dbt_credit_t *c){if(c && !c->in_callback)c->pending_grant=0;}
dbt_result_t dbt_credit_tx_begin(dbt_credit_t *c,const uint8_t *bytes,uint16_t size){
    if(!live(c) || (size && !bytes))return DBT_INVALID;
    if(c->tx_active)return DBT_BUSY;
    if(size>c->peer.mtu)return DBT_NO_SPACE;
    c->transmit=bytes;c->tx_size=size;c->tx_cursor=0;c->tx_active=c->tx_first=true;c->pending_frame=false;return DBT_OK;
}
static dbt_result_t frame_view(dbt_credit_t *c,dbt_credit_frame_t *frame){
    if(!live(c) || !frame)return DBT_INVALID;
    if(!c->tx_active)return DBT_EMPTY;
    if(!c->peer.credits)return DBT_BUSY;
    uint16_t header=c->tx_first?2u:0u;
    uint16_t bytes=(uint16_t)(c->tx_size-c->tx_cursor);
    if(bytes>c->peer.mps-header)bytes=(uint16_t)(c->peer.mps-header);
    frame->data=bytes?c->transmit+c->tx_cursor:0;frame->size=bytes;
    frame->sdu_length=c->tx_size;frame->first=c->tx_first;return DBT_OK;
}
dbt_result_t dbt_credit_tx_view(dbt_credit_t *c,dbt_credit_frame_t *frame){
    dbt_result_t result=frame_view(c,frame);if(result!=DBT_OK)return result;
    c->pending_bytes=frame->size;c->pending_frame=true;return DBT_OK;
}
dbt_result_t dbt_credit_tx_peek(dbt_credit_t *c,uint8_t *output,uint16_t capacity,uint16_t *size){
    if(!output || !size)return DBT_INVALID;
    dbt_credit_frame_t frame;dbt_result_t result=frame_view(c,&frame);if(result!=DBT_OK)return result;
    uint16_t header=frame.first?2u:0u,bytes=frame.size;
    if((uint32_t)header+bytes>capacity)return DBT_NO_SPACE;
    if(header){output[0]=(uint8_t)frame.sdu_length;output[1]=(uint8_t)(frame.sdu_length>>8);}
    if(bytes)copy(output+header,frame.data,bytes);
    *size=(uint16_t)(header+bytes);c->pending_bytes=bytes;c->pending_frame=true;return DBT_OK;
}
dbt_result_t dbt_credit_tx_commit(dbt_credit_t *c){
    if(!live(c) || !c->tx_active || !c->pending_frame || !c->peer.credits)return DBT_INVALID;
    c->tx_cursor=(uint16_t)(c->tx_cursor+c->pending_bytes);c->pending_frame=false;c->tx_first=false;
    --c->peer.credits;++c->frames_sent;
    if(c->tx_cursor==c->tx_size){c->tx_active=false;c->transmit=0;++c->sdus_sent;}
    return DBT_OK;
}
