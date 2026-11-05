#include "dbt_sdp_client.h"
#include <string.h>

static uint16_t be16(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
static void put16(uint8_t *p,uint16_t value){p[0]=(uint8_t)(value>>8);p[1]=(uint8_t)value;}
static bool overlap(const void *a,uint32_t an,const void *b,uint32_t bn){
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
    return an && bn && (x<=y?y-x<an:x-y<bn);
}
static bool due(uint32_t now,uint32_t deadline){return (int32_t)(now-deadline)>=0;}
static uint32_t until(uint32_t now,uint32_t deadline){return due(now,deadline)?0:deadline-now;}
static void finish(dbt_sdp_client_t *c,dbt_sdp_client_outcome_t outcome,bool close){
    c->result.outcome=outcome;c->result.must_close|=close;
    c->ready=c->waiting=c->more=false;c->active=c->pinned;c->result.pinned=c->pinned;
}
static bool packet(dbt_sdp_client_t *c,uint32_t now){
    if(c->next_tid==UINT16_MAX){finish(c,DBT_SDP_CLIENT_NO_SPACE,true);return false;}
    c->tid=++c->next_tid;
    uint8_t *out=c->config.request;out[0]=c->kind;put16(out+1,c->tid);
    uint16_t parameters=(uint16_t)(c->prefix_size+1u+c->continuation_size);
    put16(out+3,parameters);out[5u+c->prefix_size]=c->continuation_size;
    memcpy(out+6u+c->prefix_size,c->continuation,c->continuation_size);
    c->tx_size=(uint16_t)(5u+parameters);c->response_seen=c->waiting=false;c->ready=true;
    c->phase_deadline=now+c->config.phase_ms;return true;
}
static void advance(dbt_sdp_client_t *c,uint32_t now){
    if(c->pinned || !c->response_seen)return;
    if(c->result.outcome!=DBT_SDP_CLIENT_NONE){c->active=false;return;}
    if(c->more){(void)packet(c,now);return;}
    if(c->kind!=2 && !dbt_sdp_attribute_lists_valid(c->config.response,c->result.bytes,c->kind==6))
        finish(c,DBT_SDP_CLIENT_PROTOCOL_ERROR,true);
    else finish(c,DBT_SDP_CLIENT_SUCCESS,false);
}
dbt_result_t dbt_sdp_client_init(dbt_sdp_client_t *c,dbt_sdp_client_config_t config){
    if(!c || !config.binding || !config.request || !config.response ||
       config.rx_mtu<48 || config.rx_mtu>DBT_SDP_RECORD_BYTES ||
       config.tx_mtu<48 || config.tx_mtu>DBT_SDP_RECORD_BYTES ||
       config.request_capacity<22 || config.request_capacity>DBT_SDP_RECORD_BYTES ||
       config.response_capacity<2 || config.response_capacity>DBT_SDP_RECORD_BYTES ||
       !config.phase_ms || config.phase_ms>60000u || !config.total_ms || config.total_ms>60000u ||
       overlap(c,sizeof(*c),config.request,config.request_capacity) ||
       overlap(c,sizeof(*c),config.response,config.response_capacity) ||
       overlap(config.request,config.request_capacity,config.response,config.response_capacity))return DBT_INVALID;
    *c=(dbt_sdp_client_t){0};c->config=config;return DBT_OK;
}
dbt_result_t dbt_sdp_client_begin(dbt_sdp_client_t *c,uint8_t kind,const uint8_t *prefix,uint16_t size,
                                 uint32_t now,dbt_sdp_client_id_t *id){
    if(!c || !c->config.binding || !id || !prefix)return DBT_INVALID;
    if(c->active || c->pinned || c->result.must_close)return DBT_BUSY;
    uint16_t maximum;
    if(!dbt_sdp_query_prefix_valid(kind,prefix,size,&maximum))return DBT_INVALID;
    if((uint32_t)size+22u>c->config.request_capacity || (uint32_t)size+22u>c->config.tx_mtu)return DBT_NO_SPACE;
    if(c->next_tid==UINT16_MAX){c->result.must_close=true;return DBT_NO_SPACE;}
    memmove(c->config.request+5,prefix,size);
    c->result=(dbt_sdp_client_result_t){0};c->result.id.binding=c->config.binding;
    c->kind=kind;c->prefix_size=size;c->maximum=maximum;c->continuation_size=0;c->total_records=0;
    c->active=true;c->pinned=c->ready=c->waiting=c->response_seen=c->more=c->total_known=c->ever_admitted=false;
    c->total_deadline=now+c->config.total_ms;
    (void)packet(c,now);c->result.id.query=c->tid;*id=c->result.id;return DBT_OK;
}
void dbt_sdp_client_poll(dbt_sdp_client_t *c,uint32_t now){
    if(!c || (!c->active && !c->pinned) || c->result.retirement_timed_out)return;
    if(c->result.outcome!=DBT_SDP_CLIENT_NONE){
        if(c->pinned && due(now,c->phase_deadline)){
            c->result.retirement_timed_out=true;c->result.must_close=true;
        }
        return;
    }
    if(due(now,c->total_deadline) || due(now,c->phase_deadline)){
        if(c->pinned)c->result.retirement_timed_out=true;
        finish(c,DBT_SDP_CLIENT_TIMEOUT,c->pinned || c->waiting || c->ever_admitted);
    }
}
bool dbt_sdp_client_tx_ready(const dbt_sdp_client_t *c){
    return c && c->active && c->result.outcome==DBT_SDP_CLIENT_NONE && c->ready && !c->pinned;
}
dbt_result_t dbt_sdp_client_tx_peek(dbt_sdp_client_t *c,uint32_t now,const uint8_t **data,
                                   uint16_t *size,uint16_t *transaction){
    if(!c || !data || !size || !transaction)return DBT_INVALID;
    dbt_sdp_client_poll(c,now);
    if(c->pinned)return DBT_BUSY;
    if(!dbt_sdp_client_tx_ready(c))return DBT_EMPTY;
    c->pinned=c->result.pinned=true;c->ready=false;
    *data=c->config.request;*size=c->tx_size;*transaction=c->tid;return DBT_OK;
}
dbt_result_t dbt_sdp_client_tx_done(dbt_sdp_client_t *c,uint32_t binding,uint16_t transaction,
                                   dbt_sdp_client_tx_outcome_t outcome,uint32_t now){
    if(!c || (unsigned)outcome>(unsigned)DBT_SDP_TX_ABORTED)return DBT_INVALID;
    if(binding!=c->config.binding || transaction!=c->tid || !c->pinned)return DBT_EMPTY;
    dbt_sdp_client_poll(c,now);c->pinned=c->result.pinned=false;
    if(outcome==DBT_SDP_TX_LINK_LOST || outcome==DBT_SDP_TX_ABORTED){
        if(c->result.outcome==DBT_SDP_CLIENT_NONE || c->result.outcome==DBT_SDP_CLIENT_SUCCESS)
            finish(c,DBT_SDP_CLIENT_TRANSPORT_ERROR,true);
        else{c->result.must_close=true;c->active=false;}
        return DBT_OK;
    }
    if(outcome==DBT_SDP_TX_UNSENT && c->response_seen){finish(c,DBT_SDP_CLIENT_PROTOCOL_ERROR,true);return DBT_OK;}
    if(outcome==DBT_SDP_TX_ADMITTED){
        c->ever_admitted=true;if(c->result.admissions!=UINT16_MAX)++c->result.admissions;
    }
    if(c->result.outcome!=DBT_SDP_CLIENT_NONE){c->active=false;return DBT_OK;}
    if(outcome==DBT_SDP_TX_UNSENT){c->ready=true;return DBT_OK;}
    c->phase_deadline=now+c->config.phase_ms;
    if(c->response_seen)advance(c,now);else c->waiting=true;
    return DBT_OK;
}
static dbt_result_t invalid_response(dbt_sdp_client_t *c){finish(c,DBT_SDP_CLIENT_PROTOCOL_ERROR,true);return DBT_INVALID;}
dbt_result_t dbt_sdp_client_receive(dbt_sdp_client_t *c,uint32_t binding,const uint8_t *data,uint16_t size,uint32_t now){
    if(!c || !data)return DBT_INVALID;
    if(binding!=c->config.binding)return DBT_EMPTY;
    if(overlap(data,size,c,sizeof(*c)) || overlap(data,size,c->config.request,c->config.request_capacity) ||
       overlap(data,size,c->config.response,c->config.response_capacity))return DBT_INVALID;
    dbt_sdp_client_poll(c,now);
    if(!c->active || c->result.outcome!=DBT_SDP_CLIENT_NONE)return DBT_EMPTY;
    if(size<5 || size>c->config.rx_mtu || be16(data+3)!=size-5u)return invalid_response(c);
    if(be16(data+1)!=c->tid || c->response_seen || (!c->pinned && !c->waiting)){
        if(c->result.stale!=UINT16_MAX)++c->result.stale;
        return DBT_EMPTY;
    }
    if(c->result.responses!=UINT16_MAX)++c->result.responses;
    const uint8_t *p=data+5;uint16_t parameters=(uint16_t)(size-5u);
    if(data[0]==1){
        if(parameters<2)return invalid_response(c);
        c->result.peer_error=be16(p);c->result.error_info_bytes=(uint16_t)(parameters-2u);
        c->response_seen=true;finish(c,DBT_SDP_CLIENT_REMOTE_ERROR,false);return DBT_OK;
    }
    if(data[0]!=c->kind+1u)return invalid_response(c);
    uint16_t start=c->kind==2?4u:2u;
    if(parameters<start+1u)return invalid_response(c);
    uint16_t total=0,count=0;uint32_t bytes;
    if(c->kind==2){total=be16(p);count=be16(p+2);bytes=(uint32_t)count*4u;}
    else{bytes=be16(p);if(bytes<2u || bytes>c->maximum)return invalid_response(c);}
    uint32_t tail=start+bytes;
    if(tail>=parameters)return invalid_response(c);
    uint8_t continuation=p[tail];
    if(continuation>16u || tail+1u+continuation!=parameters || (continuation && !bytes))return invalid_response(c);
    if(c->kind==2){
        uint32_t received=(uint32_t)c->result.records+count;
        if(total>c->maximum || count>total || (c->total_known && total!=c->total_records) || received>total ||
           (continuation?received>=total:received!=total))return invalid_response(c);
    }
    if(bytes>(uint32_t)c->config.response_capacity-c->result.bytes){finish(c,DBT_SDP_CLIENT_NO_SPACE,true);return DBT_NO_SPACE;}
    memcpy(c->continuation,p+tail+1u,continuation);c->continuation_size=continuation;
    memcpy(c->config.response+c->result.bytes,p+start,bytes);c->result.bytes=(uint16_t)(c->result.bytes+bytes);
    if(c->kind==2){c->total_records=total;c->total_known=true;c->result.records=(uint16_t)(c->result.records+count);}
    c->response_seen=true;c->more=continuation!=0;c->waiting=false;
    advance(c,now);return DBT_OK;
}
dbt_result_t dbt_sdp_client_cancel(dbt_sdp_client_t *c,dbt_sdp_client_id_t id){
    if(!c || !id.binding || !id.query)return DBT_INVALID;
    if(id.binding!=c->result.id.binding || id.query!=c->result.id.query)return DBT_EMPTY;
    c->result.cancel_requested=true;
    if(c->result.outcome==DBT_SDP_CLIENT_NONE)finish(c,DBT_SDP_CLIENT_CANCELLED,c->pinned || c->waiting || c->ever_admitted);
    return c->pinned?DBT_PENDING:DBT_OK;
}
void dbt_sdp_client_abort(dbt_sdp_client_t *c,uint32_t binding){
    if(c && binding==c->config.binding){
        c->result.must_close=true;
        if((c->active || c->pinned) && c->result.outcome==DBT_SDP_CLIENT_NONE)
            finish(c,DBT_SDP_CLIENT_TRANSPORT_ERROR,true);
    }
}
uint32_t dbt_sdp_client_wait_ms(const dbt_sdp_client_t *c,uint32_t now){
    if(!c || (!c->active && !c->pinned) || c->result.retirement_timed_out)return UINT32_MAX;
    uint32_t wait=until(now,c->phase_deadline);
    if(c->result.outcome==DBT_SDP_CLIENT_NONE){uint32_t total=until(now,c->total_deadline);if(total<wait)wait=total;}
    return wait;
}
dbt_result_t dbt_sdp_client_result(const dbt_sdp_client_t *c,dbt_sdp_client_id_t id,
                                  dbt_sdp_client_result_t *result,const uint8_t **data){
    if(!c || !result || !data || !id.binding || !id.query)return DBT_INVALID;
    if(id.binding!=c->result.id.binding || id.query!=c->result.id.query)return DBT_EMPTY;
    *result=c->result;*data=0;
    if(c->active || c->pinned)return DBT_PENDING;
    if(c->result.outcome==DBT_SDP_CLIENT_SUCCESS)*data=c->config.response;
    return DBT_OK;
}
