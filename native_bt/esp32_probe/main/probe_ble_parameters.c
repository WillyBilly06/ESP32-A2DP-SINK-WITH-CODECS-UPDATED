#include "probe_ble_parameters.h"
#include <string.h>

enum { QUERY_NEW,QUERY_COMMAND,QUERY_EVENT,UPDATE_NEW,UPDATE_COMMAND,UPDATE_EVENT,DONE,LE_NEW,LE_EVENT };
enum { CREATED,OBSERVED,QUEUED,ADMITTED,UPDATED,FAILED,EXPIRED,RETIRED };
enum { USED=1,QUEUED_COMMAND=2,REMOVED=4,TIMED_OUT=8,APPLIED=16 };
#define READ_FEATURES 0x2016u
#define UPDATE_PARAMETERS 0x2013u
#define PROCEDURE_MS 45000u /* Beyond the Link Layer response timeout. */
typedef struct {
    uint32_t generation,token,deadline,next_attempt;
    uint16_t handle,interval,latency,timeout,opcode;
    uint8_t flags,phase,role,peer_feature,retries;
} parameter_request_t;
static dbt_runtime_t *owner_runtime;
static parameter_request_t requests[DBT_MAX_LINKS];
static bool due(uint32_t now,uint32_t when){return (int32_t)(now-when)>=0;}
static bool bind_runtime(dbt_runtime_t *r){if(!owner_runtime)owner_runtime=r;return r && owner_runtime==r;}
static bool live(const parameter_request_t *p){
    const dbt_link_t *l=dbt_acl_find(&owner_runtime->acl,p->handle);
    return l && l->generation==p->generation && l->kind==DBT_LINK_LE && !(p->flags&(REMOVED|TIMED_OUT));
}
static void emit_wire(const parameter_request_t *p,uint8_t stage,uint8_t status,uint8_t outcome,uint8_t result,uint16_t wire){
#if defined(DBT_PROBE_CLASSIC_SECURITY_TEST) || defined(DBT_BLE_PARAMETERS_TEST)
    probe_ble_parameters_record_t r={p->generation,owner_runtime->now_ms,p->handle,p->interval,p->latency,p->timeout,p->opcode,wire,
        stage,status,outcome,p->role,(uint8_t)((owner_runtime->host.controller.le_features[0]>>1)&1u),p->peer_feature,p->retries,result};
    probe_ble_parameters_log(&r);
#else
    (void)p;(void)stage;(void)status;(void)outcome;(void)result;(void)wire;
#endif
}
#define emit(p,s,h,o,r) emit_wire(p,s,h,o,r,0xffff)
#define note(p,stage,status,outcome) emit(p,stage,status,outcome,0xff)
static bool matches(const parameter_request_t *p){return p->interval>=PROBE_BLE_INTERVAL_MIN&&p->interval<=PROBE_BLE_INTERVAL_MAX&&!p->latency&&p->timeout==PROBE_BLE_SUPERVISION;}
static void failed(parameter_request_t *p,uint8_t status,uint8_t outcome){
    note(p,FAILED,status,outcome);
    if(p->role && (status==0x11u||status==0x1au)){
        p->phase=LE_NEW;p->next_attempt=owner_runtime->now_ms;p->opcode=0x12;
    }else if(live(p) && !p->retries && (status==0x0cu || status==0x23u)){
        ++p->retries;p->phase=p->opcode==0x12?LE_NEW:UPDATE_NEW;p->next_attempt=owner_runtime->now_ms+500u;
    }else p->phase=DONE;
}
static void command_done(void *context,uint16_t opcode,dbt_command_outcome_t outcome,uint8_t status,const uint8_t *data,uint8_t size){
    (void)data;(void)size;parameter_request_t *p=context;
    p->flags&=(uint8_t)~QUEUED_COMMAND;p->token=0;
    if(!live(p)){note(p,RETIRED,status,(uint8_t)outcome);p->flags=0;return;}
    if(outcome==DBT_COMMAND_SUCCESS && !status){
        p->phase=opcode==READ_FEATURES?QUERY_EVENT:UPDATE_EVENT;note(p,ADMITTED,0,(uint8_t)outcome);
    }else if(opcode==READ_FEATURES){
        note(p,FAILED,status,(uint8_t)outcome);
        p->phase=outcome==DBT_COMMAND_REJECTED?LE_NEW:DONE;p->opcode=0x12;
    }
    else failed(p,status,(uint8_t)outcome);
}
void probe_ble_parameters_link(dbt_runtime_t *r,const dbt_link_change_t *change){
    if(!bind_runtime(r) || !change)return;
    if(change->kind==DBT_LINK_REMOVED){
        for(unsigned i=0;i<DBT_MAX_LINKS;++i){parameter_request_t *p=&requests[i];
            if(!(p->flags&USED)||p->handle!=change->handle||p->generation!=change->generation)continue;
            p->flags|=REMOVED;note(p,RETIRED,0xff,0xff);
            if(p->flags&QUEUED_COMMAND){(void)dbt_host_cancel_command(&r->host,p->token);return;}
            if(p->phase!=QUERY_EVENT && p->phase!=UPDATE_EVENT)p->flags=0;
            return;
        }
        return;
    }
    if(change->kind!=DBT_LINK_ADDED)return;
    const dbt_link_t *l=dbt_acl_find(&r->acl,change->handle);
    if(!l || l->generation!=change->generation || l->kind!=DBT_LINK_LE)return;
    parameter_request_t *free_slot=0;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){parameter_request_t *p=&requests[i];
        if((p->flags&USED)&&p->handle==l->handle&&p->generation==l->generation)return;
        if(!(p->flags&USED)&&!free_slot)free_slot=p;
    }
    if(!free_slot){
        parameter_request_t full={.handle=l->handle,.generation=l->generation,.role=l->role,.peer_feature=0xff};
        emit(&full,FAILED,0xff,0xff,DBT_NO_SPACE);return;
    }
    *free_slot=(parameter_request_t){.generation=l->generation,.deadline=r->now_ms+PROCEDURE_MS,
        .next_attempt=r->now_ms,.handle=l->handle,.flags=USED,.role=l->role,
        .peer_feature=0xff,.opcode=l->role?READ_FEATURES:UPDATE_PARAMETERS,.phase=l->role?QUERY_NEW:UPDATE_NEW};
    note(free_slot,CREATED,0xff,0xff);
    if(l->role && !(r->host.controller.le_features[0]&2u)){
        free_slot->phase=LE_NEW;free_slot->opcode=0x12;
    }
}
static parameter_request_t *event_request(uint16_t handle,bool retired_first){
    /* A retired procedure keeps its cell until event/deadline. A replacement
     * using the same handle cannot submit while that old procedure remains. */
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){parameter_request_t *p=&requests[i];
        if(retired_first&&(p->flags&USED)&&p->handle==handle&&(p->flags&REMOVED))return p;
    }
    const dbt_link_t *l=dbt_acl_find(&owner_runtime->acl,handle);
    if(!l || l->kind!=DBT_LINK_LE)return 0;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)if((requests[i].flags&USED)&&requests[i].handle==handle&&requests[i].generation==l->generation)return &requests[i];
    return 0;
}
void probe_ble_parameters_event(dbt_runtime_t *r,const dbt_hci_packet_t *packet){
    if(!bind_runtime(r)||!packet||packet->type!=DBT_H4_EVENT||packet->event!=0x3e||!packet->payload||packet->size<4)return;
    const uint8_t *v=packet->payload;uint8_t event=v[0],status=v[1];
    if(!((event==1&&packet->size==19)||(event==0x0a&&packet->size==31)||
         (event==3&&packet->size==10)||(event==4&&packet->size==12)))return;
    parameter_request_t *p=event_request(dbt_read_le16(v+2),event==3||event==4);if(!p)return;
    if(event==4){
        if(p->phase!=QUERY_EVENT)return;
        if(!live(p)){p->flags=0;return;}
        p->peer_feature=(uint8_t)((v[4]>>1)&1u);
        if(!status&&p->peer_feature){p->phase=UPDATE_NEW;p->next_attempt=r->now_ms;note(p,OBSERVED,0,0xff);}
        else {note(p,OBSERVED,status,0xff);p->phase=LE_NEW;p->next_attempt=r->now_ms;p->opcode=0x12;}
        return;
    }
    unsigned offset=event==1?12u:event==0x0a?24u:4u;
    if(!status){p->interval=dbt_read_le16(v+offset);p->latency=dbt_read_le16(v+offset+2);p->timeout=dbt_read_le16(v+offset+4);}
    if(event==3&&!status&&matches(p))p->flags|=APPLIED;
    note(p,event==3?UPDATED:OBSERVED,status,0xff);
    if(event==3&&p->phase==UPDATE_EVENT){
        if(!live(p))p->flags=0;
        else if(status)failed(p,status,0xff);
        else p->phase=DONE;
    }else if(event!=3&&!status&&(p->phase==QUERY_NEW||p->phase==LE_NEW)&&matches(p))p->phase=DONE;
}
static void put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static bool old_handle_pending(const parameter_request_t *p){
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)if(&requests[i]!=p&&(requests[i].flags&USED)&&
        requests[i].handle==p->handle&&requests[i].generation!=p->generation)return true;
    return false;
}
void probe_ble_parameters_poll(dbt_runtime_t *r,uint32_t now,bool hold){
    if(!bind_runtime(r))return;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){parameter_request_t *p=&requests[i];if(!(p->flags&USED))continue;
        if(hold && !(p->flags&REMOVED)){
            p->flags|=REMOVED;
            if(p->flags&QUEUED_COMMAND){(void)dbt_host_cancel_command(&r->host,p->token);continue;}
            if(p->phase==LE_EVENT)(void)dbt_channels_cancel_le_parameters(&r->channels,p->handle);
            p->flags=0;continue;
        }
        if(p->phase==DONE && !(p->flags&REMOVED))continue;
        if(due(now,p->deadline) && !(p->flags&TIMED_OUT)){
            p->flags|=TIMED_OUT;note(p,EXPIRED,0xff,0xff);
            if(p->flags&QUEUED_COMMAND)(void)dbt_host_cancel_command(&r->host,p->token);
            else {if(p->phase==LE_EVENT)(void)dbt_channels_cancel_le_parameters(&r->channels,p->handle);p->flags=0;}
            continue;
        }
        if(p->flags&(QUEUED_COMMAND|REMOVED|TIMED_OUT))continue;
        if((p->phase==QUERY_NEW||p->phase==UPDATE_NEW||p->phase==LE_NEW)&&live(p)&&matches(p)){
            p->phase=DONE;note(p,OBSERVED,0,0xff);continue;
        }
        if(p->phase==LE_EVENT){
            uint16_t wire=0xffff;dbt_result_t result=dbt_channels_le_parameters_result(&r->channels,p->handle,&wire);
            if(result==DBT_BUSY)continue;
            emit_wire(p,result==DBT_OK&&!wire?ADMITTED:FAILED,0xff,0xff,(uint8_t)result,wire);
            p->phase=result==DBT_OK&&!wire?((p->flags&APPLIED)?DONE:UPDATE_EVENT):DONE;
            continue;
        }
        if(p->phase==LE_NEW&&due(now,p->next_attempt)&&live(p)&&!old_handle_pending(p)){
            p->opcode=0x12;
            dbt_result_t result=dbt_channels_request_le_parameters(&r->channels,p->handle,PROBE_BLE_INTERVAL_MIN,PROBE_BLE_INTERVAL_MAX,0,PROBE_BLE_SUPERVISION,now);
            if(result==DBT_OK){p->phase=LE_EVENT;p->flags&=(uint8_t)~APPLIED;note(p,QUEUED,0xff,0xff);}
            else if(result!=DBT_BUSY&&result!=DBT_NO_SPACE){p->phase=DONE;emit(p,FAILED,0xff,0xff,(uint8_t)result);}
            continue;
        }
        if((p->phase!=QUERY_NEW&&p->phase!=UPDATE_NEW)||!due(now,p->next_attempt)||!live(p))continue;
        if(old_handle_pending(p))continue;
        uint8_t data[14]={0};put16(data,p->handle);uint16_t opcode=READ_FEATURES;uint8_t bytes=2,previous=p->phase;
        if(p->phase==UPDATE_NEW){opcode=UPDATE_PARAMETERS;bytes=14;put16(data+2,PROBE_BLE_INTERVAL_MIN);put16(data+4,PROBE_BLE_INTERVAL_MAX);put16(data+8,PROBE_BLE_SUPERVISION);}
        p->phase=previous==QUERY_NEW?QUERY_COMMAND:UPDATE_COMMAND;p->opcode=opcode;
        dbt_result_t result=dbt_host_command_tracked(&r->host,opcode,data,bytes,DBT_COMMAND_STATUS,command_done,p,now,&p->token);
        if(result==DBT_OK){p->flags|=QUEUED_COMMAND;note(p,QUEUED,0xff,0xff);}
        else if(result==DBT_BUSY)p->phase=previous;
        else {p->phase=DONE;emit(p,FAILED,0xff,0xff,(uint8_t)result);}
    }
}
uint32_t probe_ble_parameters_wait_ms(uint32_t now){
    uint32_t wait=UINT32_MAX;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){const parameter_request_t *p=&requests[i];
        if(!(p->flags&USED)||(p->flags&TIMED_OUT)||(p->phase==DONE&&!(p->flags&REMOVED)))continue;
        if(p->phase==LE_NEW&&live(p)&&!old_handle_pending(p)&&due(now,p->next_attempt)&&owner_runtime->channels.tx_count<DBT_SIGNAL_TX_SLOTS){
            uint16_t wire;
            if(dbt_channels_le_parameters_result(&owner_runtime->channels,p->handle,&wire)!=DBT_BUSY)return 0;
        }
        if(p->phase==LE_EVENT&&live(p)){
            uint16_t wire;dbt_result_t result=dbt_channels_le_parameters_result(&owner_runtime->channels,p->handle,&wire);
            if(result!=DBT_BUSY)return 0;
        }
        if((p->phase==QUERY_NEW||p->phase==UPDATE_NEW)&&!(p->flags&(REMOVED|QUEUED_COMMAND))&&
            due(now,p->next_attempt)&&live(p)&&!old_handle_pending(p)&&
            owner_runtime->host.state==DBT_HOST_IDENTIFIED&&owner_runtime->host.command_count<DBT_COMMAND_SLOTS)return 0;
        uint32_t next=due(now,p->deadline)?0:p->deadline-now;
        if((p->phase==QUERY_NEW||p->phase==UPDATE_NEW||p->phase==LE_NEW)&&!due(now,p->next_attempt)&&p->next_attempt-now<next)next=p->next_attempt-now;
        if(next<wait)wait=next;
    }
    return wait;
}
