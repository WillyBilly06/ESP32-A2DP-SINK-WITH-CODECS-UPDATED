#include "dbt_avdtp_query.h"
#include <string.h>

static bool overlap(const void *a,unsigned an,const void *b,unsigned bn){
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;return an && bn && (x<=y?y-x<an:x-y<bn);
}
static bool owned(const dbt_avdtp_t *s,const void *p,unsigned n){
    return overlap(s,sizeof(*s),p,n) || overlap(s->rx,s->rx_capacity,p,n) ||
        overlap(s->packet,s->packet_size,p,n) || overlap(s->tx[0].data,s->tx[0].size,p,n) ||
        overlap(s->tx[1].data,s->tx[1].size,p,n);
}
dbt_result_t dbt_avdtp_sep_next(const uint8_t *data,uint16_t size,uint16_t *cursor,dbt_avdtp_sep_t *sep){
    if((!data && size) || !cursor || !sep || *cursor>size || (*cursor&1u) || (size&1u) || size>124)return DBT_INVALID;
    if(*cursor==size)return DBT_EMPTY;
    uint8_t seid=data[*cursor]>>2;if(!seid || seid>62)return DBT_INVALID;
    dbt_avdtp_sep_t value={seid,(uint8_t)(data[*cursor+1u]>>4),
        (data[*cursor+1u]&8u)!=0,(data[*cursor]&2u)!=0};
    *sep=value;*cursor=(uint16_t)(*cursor+2u);return DBT_OK;
}
static dbt_result_t validate(const dbt_avdtp_message_t *m,dbt_avdtp_cap_error_t *error){
    if(m->signal==DBT_AVDTP_DISCOVER){
        uint16_t cursor=0;dbt_avdtp_sep_t sep;uint64_t seen=0;dbt_result_t r;
        while((r=dbt_avdtp_sep_next(m->data,m->size,&cursor,&sep))==DBT_OK){
            uint64_t bit=UINT64_C(1)<<sep.seid;if(seen&bit)return DBT_INVALID;seen|=bit;
        }
        return r==DBT_EMPTY && seen?DBT_OK:DBT_INVALID;
    }
    dbt_avdtp_caps_t caps;
    return dbt_avdtp_caps_validate(m->data,m->size,
        m->signal==DBT_AVDTP_GET_CONFIGURATION?DBT_AVDTP_CONFIGURATION:DBT_AVDTP_CAPABILITIES,&caps,error);
}
static void completed(void *context,dbt_avdtp_outcome_t outcome,const dbt_avdtp_message_t *m){
    dbt_avdtp_query_t *q=context;if(!q || !q->active)return;
    dbt_avdtp_query_result_t *r=&q->result;r->transport_outcome=outcome;
    if(outcome==DBT_AVDTP_TIMEOUT)r->outcome=DBT_AVDTP_QUERY_TIMEOUT;
    else if(outcome==DBT_AVDTP_CANCELLED)r->outcome=DBT_AVDTP_QUERY_CANCELLED;
    else if(!m || m->signal!=r->command || m->label!=r->label)r->outcome=DBT_AVDTP_QUERY_PROTOCOL_ERROR;
    else {
        r->response_seen=true;r->response_type=m->type;r->required=m->size;
        if(m->size>q->capacity){r->payload_status=DBT_NO_SPACE;r->outcome=DBT_AVDTP_QUERY_NO_SPACE;}
        else {
            if(m->size){memmove(q->response,m->data,m->size);}
            r->bytes=m->size;
            dbt_avdtp_message_t copied=*m;copied.data=q->response;
            if(m->type==DBT_AVDTP_ACCEPT){
                r->payload_status=validate(&copied,&r->capability_error);
                r->outcome=r->payload_status==DBT_OK?DBT_AVDTP_QUERY_SUCCESS:DBT_AVDTP_QUERY_PROTOCOL_ERROR;
            }else if(m->type==DBT_AVDTP_REJECT){
                r->peer_error=m->size?q->response[m->size-1u]:0;r->outcome=DBT_AVDTP_QUERY_REJECTED;
            }else if(m->type==DBT_AVDTP_GENERAL_REJECT)r->outcome=DBT_AVDTP_QUERY_GENERAL_REJECT;
            else r->outcome=DBT_AVDTP_QUERY_PROTOCOL_ERROR;
        }
    }
    if(r->cancel_requested)r->outcome=DBT_AVDTP_QUERY_CANCELLED;
    q->active=false;q->done=true;
}
void dbt_avdtp_query_init(dbt_avdtp_query_t *q){if(q)*q=(dbt_avdtp_query_t){0};}
dbt_result_t dbt_avdtp_query_begin(dbt_avdtp_query_t *q,dbt_avdtp_t *s,uint8_t command,uint8_t seid,
    uint8_t *response,uint16_t capacity,uint32_t now,uint32_t *token){
    bool discover=command==DBT_AVDTP_DISCOVER;
    if(!q || !s || !response || !token || !capacity || capacity>s->maximum_message ||
       (discover?seid!=0:(!seid || seid>62)) ||
       (!discover && command!=DBT_AVDTP_GET_CAPABILITIES && command!=DBT_AVDTP_GET_ALL_CAPABILITIES && command!=DBT_AVDTP_GET_CONFIGURATION) ||
       overlap(q,sizeof(*q),response,capacity) || overlap(q,sizeof(*q),token,sizeof(*token)) ||
       overlap(token,sizeof(*token),response,capacity) || owned(s,q,sizeof(*q)) ||
       owned(s,response,capacity) || owned(s,token,sizeof(*token)))return DBT_INVALID;
    if(q->active)return DBT_BUSY;
    if(q->serial==UINT32_MAX)return DBT_NO_SPACE;
    q->completion=(dbt_avdtp_completion_t){q,completed};
    uint8_t body=(uint8_t)(seid<<2),label=0;
    dbt_result_t rc=dbt_avdtp_command_with_completion(s,command,discover?0:&body,discover?0:1,now,&label,&q->completion);
    if(rc!=DBT_OK)return rc;
    uint32_t serial=q->serial+1u;
    *q=(dbt_avdtp_query_t){0};q->completion=(dbt_avdtp_completion_t){q,completed};
    q->signal=s;q->response=response;q->serial=serial;q->binding=s->binding;q->capacity=capacity;q->active=true;
    q->result.token=serial;q->result.binding=s->binding;q->result.command=command;q->result.seid=seid;q->result.label=label;
    *token=serial;return DBT_PENDING;
}
dbt_result_t dbt_avdtp_query_cancel(dbt_avdtp_query_t *q,uint32_t token,uint32_t now){
    if(!q || !token)return DBT_INVALID;
    if(q->serial!=token || !q->signal)return DBT_EMPTY;
    q->result.cancel_requested=true;
    if(q->done)return DBT_OK;
    dbt_result_t rc=dbt_avdtp_command_cancel(q->signal,q->binding,q->result.label,&q->completion,now);
    return rc==DBT_OK || rc==DBT_PENDING?rc:DBT_INVALID;
}
dbt_result_t dbt_avdtp_query_result(const dbt_avdtp_query_t *q,uint32_t token,
    dbt_avdtp_query_result_t *result,const uint8_t **body){
    if(!q || !token || !result || !body)return DBT_INVALID;
    if(q->serial!=token || !q->signal)return DBT_EMPTY;
    if(overlap(q,sizeof(*q),result,sizeof(*result)) || overlap(q,sizeof(*q),body,sizeof(*body)) ||
       overlap(result,sizeof(*result),body,sizeof(*body)) || overlap(result,sizeof(*result),q->response,q->capacity) ||
       overlap(body,sizeof(*body),q->response,q->capacity))return DBT_INVALID;
    *result=q->result;*body=q->done && q->result.bytes?q->response:0;
    return q->done?DBT_OK:DBT_PENDING;
}
