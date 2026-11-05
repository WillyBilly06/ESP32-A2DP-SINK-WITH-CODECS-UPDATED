#include "dbt_page.h"

static bool same(const uint8_t a[6],const uint8_t b[6]){
    uint8_t difference=0;
    for(unsigned i=0;i<6;++i)difference|=(uint8_t)(a[i]^b[i]);
    return difference==0;
}
static bool due(uint32_t now,uint32_t when){return (int32_t)(now-when)>=0;}
static uint32_t until(uint32_t now,uint32_t when){return due(now,when)?0:when-now;}
static void settle(dbt_page_t *p){
    if(p->host->state==DBT_HOST_FAILED){p->result.outcome=DBT_PAGE_FAILED;p->final_seen=true;}
    if(p->final_seen && !p->create_pending && !p->cancel_pending)p->active=false;
}
static void broken(dbt_page_t *p,dbt_fault_t fault){
    p->result.outcome=DBT_PAGE_FAILED;p->final_seen=true;
    /* An unretired controller procedure cannot be aliased by another page.
     * Abort host ownership, including queued callbacks; do not reset hardware. */
    dbt_host_abort(p->host,fault);settle(p);
}
static void created(void *context,uint16_t opcode,dbt_command_outcome_t outcome,
                    uint8_t status,const uint8_t *returns,uint8_t size){
    dbt_page_t *p=context;p->create_pending=false;
    if(opcode!=0x0405){broken(p,DBT_FAULT_BAD_RESPONSE);return;}
    if(outcome==DBT_COMMAND_SUCCESS || outcome==DBT_COMMAND_REJECTED){
        if(size!=1 || !returns || returns[0]!=status){broken(p,DBT_FAULT_BAD_RESPONSE);return;}
        p->result.command_status=status;p->result.status_mask|=DBT_PAGE_STATUS_CREATE;
    }
    if(p->final_seen){settle(p);return;}
    if(outcome==DBT_COMMAND_SUCCESS){p->admitted=true;return;}
    p->final_seen=true;
    if(outcome==DBT_COMMAND_REJECTED)p->result.outcome=DBT_PAGE_REJECTED;
    else if(outcome==DBT_COMMAND_ABORTED && p->result.cancel_requested && p->host->state==DBT_HOST_IDENTIFIED)
        p->result.outcome=DBT_PAGE_CANCELLED; /* Tracked command was never sent. */
    else p->result.outcome=DBT_PAGE_FAILED;
    settle(p);
}
static void cancelled(void *context,uint16_t opcode,dbt_command_outcome_t outcome,
                      uint8_t status,const uint8_t *returns,uint8_t size){
    dbt_page_t *p=context;p->cancel_pending=false;p->cancel_token=0;
    if(opcode!=0x0408){broken(p,DBT_FAULT_BAD_RESPONSE);return;}
    if(outcome==DBT_COMMAND_SUCCESS || outcome==DBT_COMMAND_REJECTED){
        if(size!=7 || !returns || returns[0]!=status || !same(returns+1,p->address_le)){
            broken(p,DBT_FAULT_BAD_RESPONSE);return;
        }
        p->result.cancel_status=status;p->result.status_mask|=DBT_PAGE_STATUS_CANCEL;
    }
    /* Even successful HCI cancellation has a later Connection Complete. */
    settle(p);
}
dbt_result_t dbt_page_init(dbt_page_t *p,dbt_host_t *host){
    if(!p || !host)return DBT_INVALID;
    *p=(dbt_page_t){0};p->host=host;return DBT_OK;
}
uint16_t dbt_page_packet_types(const dbt_controller_info_t *c){
    if(!c)return 0;
    uint16_t types=0x0018u; /* Always-allowed DM1 and DH1. */
    if(c->features[0]&1u)types|=0x0c00u;
    if(c->features[0]&2u)types|=0xc000u;
    if(!(c->features[3]&2u))types|=0x1102u; /* Exclude unsupported 2-Mbit EDR. */
    if(!(c->features[3]&4u))types|=0x2204u; /* Exclude unsupported 3-Mbit EDR. */
    if(!(c->features[4]&0x80u))types|=0x0300u;
    if(!(c->features[5]&1u))types|=0x3000u;
    return types;
}
dbt_result_t dbt_page_begin(dbt_page_t *p,const dbt_acl_t *acl,const dbt_page_config_t *config,
                          uint32_t now,uint32_t timeout,uint32_t retire_ms,uint32_t *token){
    if(!p || !p->host || !acl || !config || !token || !timeout || timeout>60000u ||
       !retire_ms || retire_ms>60000u || config->page_repetition>2u)return DBT_INVALID;
    if(p->active || p->host->state!=DBT_HOST_IDENTIFIED)return DBT_BUSY;
    if(same(config->address_le,p->host->controller.address_le))return DBT_INVALID;
    if((p->host->controller.supported_commands[0]&0x90u)!=0x90u)return DBT_UNSUPPORTED;
    bool capacity=false;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){
        const dbt_link_t *link=&acl->links[i];
        if(!link->used)capacity=true;
        else if(link->kind==DBT_LINK_CLASSIC && same(link->peer,config->address_le))return DBT_BUSY;
    }
    if(!capacity)return DBT_NO_SPACE;
    uint8_t body[13]={0};
    for(unsigned i=0;i<6;++i)body[i]=config->address_le[i];
    uint16_t types=dbt_page_packet_types(&p->host->controller);
    body[6]=(uint8_t)types;body[7]=(uint8_t)(types>>8);body[8]=config->page_repetition;
    body[10]=(uint8_t)config->clock_offset;body[11]=(uint8_t)(config->clock_offset>>8);
    body[12]=config->allow_role_switch?1u:0u;
    uint32_t id;
    dbt_result_t result=dbt_host_command_tracked(p->host,0x0405,body,sizeof(body),
        DBT_COMMAND_STATUS,created,p,now,&id);
    if(result!=DBT_OK)return result;
    /* Host enqueue copies parameters and cannot call back before owner polling.
     * Refusal above preserves the previous result and caller's token. */
    dbt_host_t *host=p->host;
    *p=(dbt_page_t){0};p->host=host;p->result.token=id;
    p->deadline=now+timeout;p->retire_ms=retire_ms;p->active=p->create_pending=true;
    for(unsigned i=0;i<6;++i)p->address_le[i]=body[i];
    *token=id;return DBT_OK;
}
static void request_cancel(dbt_page_t *p,uint32_t now,bool timeout){
    if(!p->result.cancel_requested){p->result.cancel_requested=true;p->retire_deadline=now+p->retire_ms;}
    if(timeout)p->result.timed_out=true;
    if(p->create_pending){
        dbt_command_cancel_result_t result=dbt_host_cancel_command(p->host,p->result.token);
        if(result==DBT_COMMAND_NOT_FOUND){p->create_pending=false;broken(p,DBT_FAULT_BAD_RESPONSE);}
    }
}
dbt_result_t dbt_page_cancel(dbt_page_t *p,uint32_t token,uint32_t now){
    if(!p || !p->host || !token)return DBT_INVALID;
    if(p->result.token!=token)return DBT_EMPTY;
    if(!p->active)return DBT_OK;
    request_cancel(p,now,false);return p->active?DBT_PENDING:DBT_OK;
}
dbt_result_t dbt_page_result(const dbt_page_t *p,uint32_t token,dbt_page_result_t *result){
    if(!p || !result || !token)return DBT_INVALID;
    if(p->result.token!=token)return DBT_EMPTY;
    *result=p->result;return p->active?DBT_PENDING:DBT_OK;
}
static void remove_unsent_after_final(dbt_page_t *p){
    /* A connection may win before a queued cancel is sent. Never let that
     * leftover cancel reach a later procedure for the same address. Submitted
     * commands instead keep the context until their actual callbacks retire. */
    if(p->create_pending && dbt_host_cancel_command(p->host,p->result.token)==DBT_COMMAND_NOT_FOUND){
        p->create_pending=false;broken(p,DBT_FAULT_BAD_RESPONSE);return;
    }
    if(p->cancel_pending && dbt_host_cancel_command(p->host,p->cancel_token)==DBT_COMMAND_NOT_FOUND){
        p->cancel_pending=false;broken(p,DBT_FAULT_BAD_RESPONSE);
    }
    settle(p);
}
dbt_result_t dbt_page_event(dbt_page_t *p,const dbt_hci_packet_t *packet){
    if(!p || !p->host || !packet)return DBT_INVALID;
    if(packet->type!=DBT_H4_EVENT || packet->event!=3)return DBT_UNSUPPORTED;
    if(packet->size!=11 || !packet->payload)return DBT_INVALID;
    const uint8_t *data=packet->payload;
    if(!p->active || p->final_seen || !same(data+3,p->address_le))return DBT_EMPTY;
    if(!data[0] && data[9]!=1)return DBT_EMPTY; /* A synchronous link has another owner. */
    uint16_t handle=dbt_read_le16(data+1);
    if(!data[0] && (handle>0x0effu || data[10]>1u))return DBT_INVALID;
    /* The connection manager has already arbitrated this peer's establishment.
     * Runtime owns ACL allocation/security; this object records procedure state. */
    p->result.connection_status=data[0];p->result.status_mask|=DBT_PAGE_STATUS_CONNECTION;
    p->final_seen=true;
    if(!data[0]){p->result.handle=handle;p->result.outcome=DBT_PAGE_CONNECTED;}
    else if(data[0]==2u && p->result.cancel_requested &&
            (p->result.status_mask&DBT_PAGE_STATUS_CANCEL) && !p->result.cancel_status)
        p->result.outcome=DBT_PAGE_CANCELLED;
    else p->result.outcome=DBT_PAGE_REJECTED;
    remove_unsent_after_final(p);return DBT_OK;
}
void dbt_page_poll(dbt_page_t *p,uint32_t now){
    if(!p || !p->host || !p->active)return;
    if(p->host->state==DBT_HOST_FAILED){settle(p);return;}
    if(p->host->state!=DBT_HOST_IDENTIFIED){broken(p,DBT_FAULT_BAD_RESPONSE);return;}
    if(p->final_seen){settle(p);return;}
    if(!p->result.cancel_requested && due(now,p->deadline))request_cancel(p,now,true);
    if(!p->active)return;
    if(p->result.cancel_requested && due(now,p->retire_deadline)){broken(p,DBT_FAULT_TIMEOUT);return;}
    if(!p->result.cancel_requested || p->create_pending || !p->admitted || p->cancel_attempted)return;
    uint32_t id;
    dbt_result_t result=dbt_host_command_tracked(p->host,0x0408,p->address_le,6,
        DBT_COMMAND_COMPLETE,cancelled,p,now,&id);
    if(result==DBT_OK){p->cancel_token=id;p->cancel_pending=p->cancel_attempted=true;}
    else if(result!=DBT_NO_SPACE || p->host->command_count!=DBT_COMMAND_SLOTS)
        broken(p,DBT_FAULT_CONTROLLER); /* Includes exhausted unique-token space. */
}
uint32_t dbt_page_wait_ms(const dbt_page_t *p,uint32_t now){
    if(!p || !p->host || !p->active)return UINT32_MAX;
    if(p->host->state!=DBT_HOST_IDENTIFIED)return 0;
    if(p->final_seen)return UINT32_MAX; /* Submitted command completion owns the wakeup. */
    if(!p->result.cancel_requested)return until(now,p->deadline);
    if(!p->create_pending && p->admitted && !p->cancel_attempted &&
       p->host->command_count<DBT_COMMAND_SLOTS)return 0;
    return until(now,p->retire_deadline);
}
