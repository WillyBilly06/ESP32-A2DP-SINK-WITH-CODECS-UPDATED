#include "dbt_a2dp_rank.h"

static bool overlap(const void *a,unsigned an,const void *b,unsigned bn){uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;return an && bn && (x<=y?y-x<an:x-y<bn);}
static bool owned(const dbt_avdtp_t *s,const void *p,unsigned n){return overlap(s,sizeof(*s),p,n) || overlap(s->rx,s->rx_capacity,p,n) ||
    overlap(s->packet,s->packet_size,p,n) || overlap(s->tx[0].data,s->tx[0].size,p,n) || overlap(s->tx[1].data,s->tx[1].size,p,n);}
static bool due(uint32_t now,uint32_t deadline){return (int32_t)(now-deadline)>=0;}
static uint32_t until(uint32_t now,uint32_t deadline){return due(now,deadline)?0:deadline-now;}
static void cause(dbt_a2dp_rank_t *c,dbt_a2dp_rank_outcome_t why){if(c->result.outcome==DBT_A2DP_RANK_NONE)c->result.outcome=why;}
static void finish(dbt_a2dp_rank_t *c,dbt_a2dp_rank_outcome_t why){
    if(c->has_query)return;
    cause(c,why);c->active=false;c->result.phase=DBT_A2DP_RANK_DONE;
}
static void fault(dbt_a2dp_rank_t *c){
    cause(c,DBT_A2DP_RANK_INTERNAL_ERROR);
    if(!c->has_query && !c->query.active && c->signal->local_completion!=&c->query.completion){finish(c,DBT_A2DP_RANK_INTERNAL_ERROR);return;}
    c->has_query=true;c->result.needs_signal_retirement=true;c->result.phase=DBT_A2DP_RANK_FAULT_RETAIN;
}
static bool success(const dbt_avdtp_query_result_t *q,uint32_t binding){return q->binding==binding && q->outcome==DBT_AVDTP_QUERY_SUCCESS &&
    q->transport_outcome==DBT_AVDTP_RESPONSE && q->response_seen && q->response_type==DBT_AVDTP_ACCEPT &&
    q->payload_status==DBT_OK && !q->cancel_requested && q->bytes==q->required;}
static void snapshot(dbt_a2dp_rank_t *c,const dbt_avdtp_query_result_t *q){
    if(q->command==DBT_AVDTP_DISCOVER)c->result.discovery=*q;else c->result.last_capabilities=*q;
}
static dbt_result_t submit(dbt_a2dp_rank_t *c,uint8_t command,uint8_t seid,uint8_t *buffer,uint16_t capacity,uint32_t now){
    if(c->query.active){fault(c);return DBT_INVALID;}
    uint32_t token=0;dbt_result_t rc=dbt_avdtp_query_begin(&c->query,c->signal,command,seid,buffer,capacity,now,&token);
    c->submit_attempted=true;c->result.submit_status=rc;
    if(rc==DBT_PENDING){c->query_token=token;c->has_query=true;c->cancel_attempted=false;}
    else if(rc==DBT_BUSY && c->result.submit_busy<UINT16_MAX)++c->result.submit_busy;
    return rc;
}
void dbt_a2dp_rank_init(dbt_a2dp_rank_t *c){if(c)*c=(dbt_a2dp_rank_t){0};}
static bool writes_alias(const dbt_a2dp_rank_t *c,const dbt_a2dp_rank_config_t *config,const uint32_t *token,const void *p,unsigned bytes){
    return overlap(c,sizeof(*c),p,bytes) || overlap(config->discovery,config->discovery_capacity,p,bytes) ||
        overlap(config->capabilities,config->capability_capacity,p,bytes) || overlap(token,sizeof(*token),p,bytes);
}
dbt_result_t dbt_a2dp_rank_begin(dbt_a2dp_rank_t *c,dbt_avdtp_t *signal,const dbt_avdtp_endpoint_t *endpoints,uint8_t count,
    dbt_a2dp_rank_config_t config,uint32_t now,uint32_t *token){
    if(!c || !signal || !endpoints || !count || count>DBT_AVDTP_SINK_ENDPOINT_LIMIT || !token ||
       !config.requests || !config.request_count || config.request_count>DBT_A2DP_RANK_MAX_REQUESTS ||
       !config.discovery || !config.discovery_capacity || config.discovery_capacity>124 || config.discovery_capacity>signal->maximum_message ||
       !config.capabilities || !config.capability_capacity || config.capability_capacity>DBT_A2DP_SELECT_MAX_CAPABILITIES || config.capability_capacity>signal->maximum_message ||
       !config.timeout_ms || config.timeout_ms>INT32_MAX || signal->in_callback || !signal->binding || signal->failed ||
       owned(signal,c,sizeof(*c)) || owned(signal,config.discovery,config.discovery_capacity) || owned(signal,config.capabilities,config.capability_capacity) ||
       owned(signal,token,sizeof(*token)) || owned(signal,endpoints,(unsigned)count*sizeof(*endpoints)) ||
       owned(signal,config.requests,(unsigned)config.request_count*sizeof(*config.requests)) || overlap(c,sizeof(*c),token,sizeof(*token)) ||
       overlap(c,sizeof(*c),config.discovery,config.discovery_capacity) || overlap(c,sizeof(*c),config.capabilities,config.capability_capacity) ||
       overlap(config.discovery,config.discovery_capacity,config.capabilities,config.capability_capacity) ||
       overlap(token,sizeof(*token),config.discovery,config.discovery_capacity) || overlap(token,sizeof(*token),config.capabilities,config.capability_capacity) ||
       writes_alias(c,&config,token,endpoints,(unsigned)count*sizeof(*endpoints)) ||
       writes_alias(c,&config,token,config.requests,(unsigned)config.request_count*sizeof(*config.requests)))return DBT_INVALID;
    for(unsigned i=0;i<count;++i)if(writes_alias(c,&config,token,endpoints[i].capabilities,endpoints[i].size) ||
        owned(signal,endpoints[i].capabilities,endpoints[i].size))return DBT_INVALID;
    for(unsigned i=0;i<config.request_count;++i){const dbt_a2dp_generate_request_t *r=&config.requests[i];
        if(r->family<DBT_A2DP_FAMILY_SBC || r->family>DBT_A2DP_FAMILY_LHDCV5 || !r->rate || r->channels<1 || r->channels>2 ||
           r->delay<DBT_A2DP_DELAY_DISABLED || r->delay>DBT_A2DP_DELAY_REQUIRED || r->avdtp_version!=config.requests[0].avdtp_version)return DBT_INVALID;
    }
    if(c->active || c->query.active)return DBT_BUSY;
    if(c->serial==UINT32_MAX)return DBT_NO_SPACE;
    uint32_t query_token;dbt_result_t rc=dbt_avdtp_query_begin(&c->query,signal,DBT_AVDTP_DISCOVER,0,config.discovery,config.discovery_capacity,now,&query_token);
    if(rc!=DBT_PENDING)return rc;
    c->signal=signal;c->endpoints=endpoints;c->endpoint_count=count;c->config=config;
    c->binding=signal->binding;c->query_token=query_token;c->deadline=now+config.timeout_ms;
    c->discovery_bytes=c->capability_bytes=c->cursor=0;c->request_index=0;
    c->active=true;c->has_query=true;c->cancel_attempted=false;c->submit_attempted=true;
    c->result=(dbt_a2dp_rank_result_t){0};c->result.token=++c->serial;c->result.binding=c->binding;
    c->result.phase=DBT_A2DP_RANK_DISCOVER_WAIT;c->result.submit_status=DBT_PENDING;c->result.best_request=UINT8_MAX;
    *token=c->serial;return DBT_PENDING;
}
dbt_result_t dbt_a2dp_rank_interrupt(dbt_a2dp_rank_t *c,uint32_t token,dbt_a2dp_rank_outcome_t why){
    if(!c || !token || (why!=DBT_A2DP_RANK_CANCELLED && why!=DBT_A2DP_RANK_SIGNAL_CHANGED && why!=DBT_A2DP_RANK_CONFLICT && why!=DBT_A2DP_RANK_TIMEOUT))return DBT_INVALID;
    if(c->serial!=token || !c->signal)return DBT_EMPTY;
    if(!c->active)return DBT_OK;
    cause(c,why);c->result.cancel_requested=true;
    if(c->result.phase!=DBT_A2DP_RANK_FAULT_RETAIN)c->result.phase=DBT_A2DP_RANK_CANCELLING;
    return DBT_PENDING;
}
dbt_result_t dbt_a2dp_rank_cancel(dbt_a2dp_rank_t *c,uint32_t token){return dbt_a2dp_rank_interrupt(c,token,DBT_A2DP_RANK_CANCELLED);}
void dbt_a2dp_rank_poll(dbt_a2dp_rank_t *c,uint32_t now){
    if(!c || !c->active || !c->signal || c->signal->in_callback)return;
    if(due(now,c->deadline)) (void)dbt_a2dp_rank_interrupt(c,c->serial,DBT_A2DP_RANK_TIMEOUT);
    if(c->signal->failed || c->signal->binding!=c->binding)(void)dbt_a2dp_rank_interrupt(c,c->serial,DBT_A2DP_RANK_SIGNAL_CHANGED);
    if(c->result.cancel_requested || c->result.phase==DBT_A2DP_RANK_FAULT_RETAIN){
        if(!c->has_query){finish(c,DBT_A2DP_RANK_CANCELLED);return;}
        dbt_avdtp_query_result_t q;const uint8_t *body;
        dbt_result_t rc=dbt_avdtp_query_result(&c->query,c->query_token,&q,&body);
        if(rc==DBT_OK){snapshot(c,&q);c->has_query=false;finish(c,DBT_A2DP_RANK_CANCELLED);return;}
        if(rc!=DBT_PENDING){fault(c);return;}
        if(!c->cancel_attempted){
            rc=dbt_avdtp_query_cancel(&c->query,c->query_token,now);c->result.cancel_status=rc;
            if(rc==DBT_OK || rc==DBT_PENDING)c->cancel_attempted=true;else fault(c);
        }
        return;
    }
    if(c->result.phase==DBT_A2DP_RANK_DISCOVER_WAIT || c->result.phase==DBT_A2DP_RANK_CAPS_WAIT){
        bool discover=c->result.phase==DBT_A2DP_RANK_DISCOVER_WAIT;dbt_avdtp_query_result_t q;const uint8_t *body;
        dbt_result_t rc=dbt_avdtp_query_result(&c->query,c->query_token,&q,&body);
        if(rc==DBT_PENDING)return;
        if(rc!=DBT_OK){fault(c);return;}
        snapshot(c,&q);c->has_query=false;
        if(!success(&q,c->binding) || q.command!=(discover?DBT_AVDTP_DISCOVER:(c->config.requests[0].avdtp_version>=0x0103?DBT_AVDTP_GET_ALL_CAPABILITIES:DBT_AVDTP_GET_CAPABILITIES)) ||
           (!discover && q.seid!=c->current.seid)){
            finish(c,discover?DBT_A2DP_RANK_DISCOVERY_ERROR:DBT_A2DP_RANK_CAPABILITIES_ERROR);return;
        }
        if(discover){c->discovery_bytes=q.bytes;c->result.records=(uint8_t)(q.bytes/2u);c->cursor=0;c->result.phase=DBT_A2DP_RANK_NEXT;}
        else {c->capability_bytes=q.bytes;++c->result.assessed;c->request_index=0;c->result.phase=DBT_A2DP_RANK_EVALUATE;}
        return;
    }
    if(c->result.phase==DBT_A2DP_RANK_NEXT){
        for(unsigned skip=0;skip<4;++skip){
            dbt_result_t rc=dbt_avdtp_sep_next(c->config.discovery,c->discovery_bytes,&c->cursor,&c->current);
            if(rc==DBT_EMPTY){c->result.assessment_complete=true;finish(c,c->result.best_found?DBT_A2DP_RANK_SUCCESS:DBT_A2DP_RANK_NO_MATCH);return;}
            if(rc!=DBT_OK){finish(c,DBT_A2DP_RANK_DISCOVERY_ERROR);return;}
            ++c->result.visited;
            if(c->current.in_use || c->current.sink || c->current.media_type){++c->result.skipped;continue;}
            ++c->result.eligible;c->result.current_seid=c->current.seid;c->submit_attempted=false;c->result.phase=DBT_A2DP_RANK_CAPS_SUBMIT;return;
        }
        return;
    }
    if(c->result.phase==DBT_A2DP_RANK_CAPS_SUBMIT){
        dbt_result_t rc=submit(c,c->config.requests[0].avdtp_version>=0x0103?DBT_AVDTP_GET_ALL_CAPABILITIES:DBT_AVDTP_GET_CAPABILITIES,
            c->current.seid,c->config.capabilities,c->config.capability_capacity,now);
        if(rc==DBT_PENDING)c->result.phase=DBT_A2DP_RANK_CAPS_WAIT;
        else if(rc!=DBT_BUSY && c->result.phase!=DBT_A2DP_RANK_FAULT_RETAIN)finish(c,DBT_A2DP_RANK_CAPABILITIES_ERROR);
        return;
    }
    if(c->result.phase==DBT_A2DP_RANK_EVALUATE){
        uint8_t candidate[DBT_A2DP_RANK_CONFIGURATION_BYTES];c->result.current_request=c->request_index;
        dbt_result_t rc=dbt_a2dp_generate(c->endpoints,c->endpoint_count,&c->current,c->config.capabilities,c->capability_bytes,
            &c->config.requests[c->request_index],candidate,sizeof(candidate),&c->result.last_generated);
        ++c->result.evaluations;
        if(c->result.last_generated.outcome==DBT_A2DP_GENERATE_PEER_CAPS_INVALID ||
           c->result.last_generated.outcome==DBT_A2DP_GENERATE_LAYOUT_UNSUPPORTED){
            finish(c,DBT_A2DP_RANK_CAPABILITIES_ERROR);return;
        }
        if(rc==DBT_OK){
            uint16_t bytes=c->result.last_generated.selection.configuration_size;
            if(bytes>sizeof(c->best)){finish(c,DBT_A2DP_RANK_GENERATOR_ERROR);return;}
            if(!c->result.best_found || c->request_index<c->result.best_request){
                for(unsigned i=0;i<bytes;++i)c->best[i]=candidate[i];
                c->result.best_found=true;c->result.best_request=c->request_index;c->result.configuration_size=bytes;
                c->result.best_generated=c->result.last_generated;c->result.best_capabilities=c->result.last_capabilities;
            }
        }else if(rc!=DBT_UNSUPPORTED){finish(c,DBT_A2DP_RANK_GENERATOR_ERROR);return;}
        ++c->request_index;
        if(c->request_index>=c->config.request_count)c->result.phase=DBT_A2DP_RANK_NEXT;
        return;
    }
    fault(c);
}
uint32_t dbt_a2dp_rank_wait_ms(const dbt_a2dp_rank_t *c,uint32_t now){
    if(!c || !c->active || !c->signal || c->signal->in_callback)return UINT32_MAX;
    uint32_t wait=until(now,c->deadline);
    if(c->result.cancel_requested || c->result.phase==DBT_A2DP_RANK_FAULT_RETAIN){
        if(c->result.phase==DBT_A2DP_RANK_FAULT_RETAIN)return c->has_query && c->query.done &&
            c->query.serial==c->query_token && c->query.signal==c->signal?0:UINT32_MAX;
        if(!c->has_query || c->query.done)return 0;
        if(!c->cancel_attempted)return 0;
        return dbt_avdtp_wait_ms(c->signal,now);
    }
    if(c->has_query){if(c->query.done)return 0;uint32_t q=dbt_avdtp_wait_ms(c->signal,now);return q<wait?q:wait;}
    if(c->result.phase==DBT_A2DP_RANK_CAPS_SUBMIT && c->submit_attempted && c->signal->tx[0].used){
        uint32_t q=dbt_avdtp_wait_ms(c->signal,now);return q<wait?q:wait;
    }
    return 0;
}
dbt_result_t dbt_a2dp_rank_result(const dbt_a2dp_rank_t *c,uint32_t token,dbt_a2dp_rank_result_t *result,const uint8_t **proposal){
    if(!c || !token || !result || !proposal || overlap(c,sizeof(*c),result,sizeof(*result)) || overlap(c,sizeof(*c),proposal,sizeof(*proposal)) ||
       overlap(result,sizeof(*result),proposal,sizeof(*proposal)))return DBT_INVALID;
    if(c->serial!=token || !c->signal)return DBT_EMPTY;
    if(c->active && (overlap(result,sizeof(*result),c->config.discovery,c->config.discovery_capacity) ||
       overlap(result,sizeof(*result),c->config.capabilities,c->config.capability_capacity) ||
       overlap(proposal,sizeof(*proposal),c->config.discovery,c->config.discovery_capacity) ||
       overlap(proposal,sizeof(*proposal),c->config.capabilities,c->config.capability_capacity) ||
       overlap(result,sizeof(*result),c->config.requests,(unsigned)c->config.request_count*sizeof(*c->config.requests)) ||
       overlap(proposal,sizeof(*proposal),c->config.requests,(unsigned)c->config.request_count*sizeof(*c->config.requests))))return DBT_INVALID;
    if(c->active){
        if(owned(c->signal,result,sizeof(*result)) || owned(c->signal,proposal,sizeof(*proposal)) ||
           overlap(result,sizeof(*result),c->endpoints,(unsigned)c->endpoint_count*sizeof(*c->endpoints)) ||
           overlap(proposal,sizeof(*proposal),c->endpoints,(unsigned)c->endpoint_count*sizeof(*c->endpoints)))return DBT_INVALID;
        for(unsigned i=0;i<c->endpoint_count;++i)if(overlap(result,sizeof(*result),c->endpoints[i].capabilities,c->endpoints[i].size) ||
            overlap(proposal,sizeof(*proposal),c->endpoints[i].capabilities,c->endpoints[i].size))return DBT_INVALID;
    }
    *result=c->result;*proposal=!c->active && c->result.outcome==DBT_A2DP_RANK_SUCCESS && c->result.assessment_complete?c->best:0;
    return c->active?DBT_PENDING:DBT_OK;
}
bool dbt_a2dp_rank_done(const dbt_a2dp_rank_t *c){return c && !c->active && c->signal;}
