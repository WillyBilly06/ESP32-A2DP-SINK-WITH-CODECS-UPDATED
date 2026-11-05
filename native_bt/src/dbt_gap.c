#include "dbt_gap.h"
#include "dbt_utf8.h"

static void copy(uint8_t *out, const uint8_t *in, unsigned size) {
    for (unsigned i = 0; i < size; ++i) out[i] = in[i];
}
static void put16(uint8_t *out, uint16_t value) { out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8); }
enum { IN_USED=1,IN_QUEUED=2,IN_COMMAND=4,IN_FINAL=8,IN_ACCEPT=16,IN_ADMITTED=32 };
static bool same(const uint8_t a[6],const uint8_t b[6]){
    uint8_t difference=0;for(unsigned i=0;i<6;++i)difference|=(uint8_t)(a[i]^b[i]);
    return difference==0;
}
static void incoming_trace(dbt_gap_incoming_t *entry,dbt_gap_incoming_phase_t phase,dbt_command_outcome_t outcome){
    dbt_gap_t *gap=entry->gap;
    if(gap->incoming_observer)gap->incoming_observer(gap->incoming_context,phase,entry,
        (entry->flags&IN_ACCEPT)?0x0409:0x040a,outcome);
}
static void release_incoming(dbt_gap_incoming_t *entry){
    if((entry->flags&IN_FINAL) && !(entry->flags&(IN_QUEUED|IN_COMMAND))){
        incoming_trace(entry,DBT_GAP_INCOMING_RETIRED,DBT_COMMAND_SUCCESS);entry->flags=0;
    }
}
/* HCI Write Extended Inquiry Response: FEC octet followed by 240 octets of
 * length/type/value data. Keep the advertised name and installed 16-bit
 * service classes together; controller command storage owns its copied bytes. */
static void inquiry_response(uint8_t data[248],const char name[33],const dbt_gap_config_t *config){
    unsigned length=0;while(length<32u && name[length])++length;
    data[0]=0;data[1]=(uint8_t)(length+1u);data[2]=9;
    copy(data+3,(const uint8_t *)name,length);
    if(config->classic_service_count){
        unsigned next=length+3u;
        data[next]=(uint8_t)(2u*config->classic_service_count+1u);data[next+1u]=3;
        for(unsigned i=0;i<config->classic_service_count;++i)put16(data+next+2u+2u*i,config->classic_services[i]);
    }
}
static void mask_event(uint8_t mask[8], uint8_t event) {
    mask[(event - 1u) / 8u] |= (uint8_t)(1u << ((event - 1u) % 8u));
}
static void configured(void *context, uint16_t opcode, dbt_command_outcome_t outcome,
                         uint8_t status, const uint8_t *returns, uint8_t size) {
    dbt_gap_t *gap = context;
    (void)opcode; (void)returns;
    gap->command_pending = false; gap->last_status = status;
    if (outcome != DBT_COMMAND_SUCCESS || size != 1) { gap->state = DBT_GAP_FAILED; return; }
    if (++gap->step == 10) gap->state = DBT_GAP_READY;
}
static void replied(void *context, uint16_t opcode, dbt_command_outcome_t outcome,
                      uint8_t status, const uint8_t *returns, uint8_t size) {
    dbt_gap_t *gap = context;
    (void)opcode; (void)returns; (void)size;
    if (outcome != DBT_COMMAND_SUCCESS) { ++gap->rejected_commands; gap->last_status = status; }
    if (outcome == DBT_COMMAND_ABORTED || outcome == DBT_COMMAND_TIMEOUT) gap->state = DBT_GAP_FAILED;
}
static void queued_replied(void *context,uint16_t opcode,dbt_command_outcome_t outcome,
                           uint8_t status,const uint8_t *returns,uint8_t size){
    dbt_gap_t *gap=context;
    if(!gap->reply_inflight){dbt_host_abort(gap->host,DBT_FAULT_BAD_RESPONSE);return;}
    --gap->reply_inflight;replied(gap,opcode,outcome,status,returns,size);
}
static void incoming_replied(void *context,uint16_t opcode,dbt_command_outcome_t outcome,
                             uint8_t status,const uint8_t *returns,uint8_t size){
    dbt_gap_incoming_t *entry=context;dbt_gap_t *gap=entry->gap;
    uint16_t expected=(entry->flags&IN_ACCEPT)?0x0409:0x040a;
    if(!(entry->flags&IN_COMMAND) || opcode!=expected){dbt_host_abort(gap->host,DBT_FAULT_BAD_RESPONSE);return;}
    entry->flags&=(uint8_t)~IN_COMMAND;entry->status=status;
    queued_replied(gap,opcode,outcome,status,returns,size);
    incoming_trace(entry,DBT_GAP_INCOMING_ACK,outcome);
    if(outcome==DBT_COMMAND_SUCCESS || outcome==DBT_COMMAND_REJECTED){
        if(size!=1 || !returns || returns[0]!=status){dbt_host_abort(gap->host,DBT_FAULT_BAD_RESPONSE);return;}
    }
    if(outcome==DBT_COMMAND_SUCCESS)entry->flags|=IN_ADMITTED;
    else entry->flags|=IN_FINAL; /* Controller did not admit this decision. */
    release_incoming(entry);
}
static void visibility(void *context, uint16_t opcode, dbt_command_outcome_t outcome,
                        uint8_t status, const uint8_t *returns, uint8_t size) {
    dbt_gap_t *gap = context;
    (void)returns;
    bool success = outcome == DBT_COMMAND_SUCCESS && size == 1;
    gap->last_status = status;
    if (opcode == 0x0c1a) {
        gap->classic_pending = false;
        if (success) gap->classic_scan = gap->wanted_scan;
    } else {
        gap->advertising_pending = false;
        if (success) gap->advertising = gap->wanted_advertising;
    }
    if (!success) ++gap->rejected_commands;
    if (outcome == DBT_COMMAND_ABORTED || outcome == DBT_COMMAND_TIMEOUT) gap->state = DBT_GAP_FAILED;
}
dbt_result_t dbt_gap_init(dbt_gap_t *gap, dbt_host_t *host, dbt_gap_config_t config, dbt_gap_hooks_t hooks) {
    if (!gap || !host || config.advertising_size > 31 || config.scan_response_size > 31 ||
        config.classic_service_count>DBT_GAP_CLASSIC_SERVICES ||
        config.advertising_minimum < 0x20 || config.advertising_maximum > 0x4000 ||
        config.advertising_minimum > config.advertising_maximum) return DBT_INVALID;
    bool terminated = false;
    for (unsigned i = 0; i < sizeof(config.name); ++i) if (!config.name[i]) terminated = true;
    if (!terminated) return DBT_INVALID;
    uint8_t *bytes = (uint8_t *)gap;
    for (unsigned i = 0; i < sizeof(*gap); ++i) bytes[i] = 0;
    gap->host = host; gap->config = config; gap->hooks = hooks;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)gap->incoming[i].gap=gap;
    return DBT_OK;
}
dbt_result_t dbt_gap_start(dbt_gap_t *gap) {
    if (!gap || gap->state != DBT_GAP_IDLE || gap->host->state != DBT_HOST_IDENTIFIED) return DBT_BUSY;
    gap->state = DBT_GAP_CONFIGURING;
    return DBT_OK;
}
static dbt_result_t configure_next(dbt_gap_t *gap) {
    uint8_t data[248] = {0}; uint8_t size = 0; uint16_t opcode = 0;
    switch (gap->step) {
    case 0: {
        opcode = 0x0c01; size = 8;
        static const uint8_t events[] = {3,4,5,6,7,8,0x0b,0x0c,0x0e,0x0f,0x10,0x12,0x13,
            0x16,0x17,0x18,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x3b,0x3e};
        for (unsigned i = 0; i < sizeof(events); ++i) mask_event(data, events[i]);
        break;
    }
    case 1: opcode = 0x2001; size = 8; data[0] = 0x1f; break; /* LE v4.0 events 1..5. */
    case 2: opcode = 0x0c6d; size = 2; data[0] = 1; break; /* LE Supported Host. */
    case 3:
        opcode = 0x0c13; size = 248;
        for (unsigned i = 0; i < sizeof(gap->config.name) && gap->config.name[i]; ++i) data[i] = (uint8_t)gap->config.name[i];
        break;
    case 4: opcode = 0x0c24; size = 3; copy(data, gap->config.class_of_device, 3); break;
    case 5: opcode = 0x0c56; size = 1; data[0] = 1; break; /* SSP enabled; security owner still required. */
    case 6:
        opcode = 0x2006; size = 15;
        put16(data, gap->config.advertising_minimum); put16(data + 2, gap->config.advertising_maximum);
        data[13] = 7; break; /* ADV_IND, public address, all primary channels, no filter. */
    case 7:
        opcode = 0x2008; size = 32; data[0] = gap->config.advertising_size;
        copy(data + 1, gap->config.advertising, data[0]); break;
    case 8:
        opcode = 0x2009; size = 32; data[0] = gap->config.scan_response_size;
        copy(data + 1, gap->config.scan_response, data[0]); break;
    case 9:
        opcode=0x0c52;size=241;inquiry_response(data,gap->config.name,&gap->config);break;
    default: return DBT_INVALID;
    }
    return dbt_host_command(gap->host, opcode, data, size, DBT_COMMAND_COMPLETE, configured, gap, gap->now_ms);
}
static void renamed(void *context,uint16_t opcode,dbt_command_outcome_t outcome,uint8_t status,const uint8_t *returns,uint8_t size){
    dbt_gap_t *g=context;(void)returns;
    g->rename_pending=false;g->rename_status=status;
    uint16_t expected=g->rename_step==0?0x0c13:g->rename_step==1?0x2009:0x0c52;
    if(!g->rename_active || opcode!=expected || outcome!=DBT_COMMAND_SUCCESS || size!=1){
        g->rename_done=g->rename_serial;g->rename_applied=false;g->rename_active=false;
        if(outcome==DBT_COMMAND_ABORTED || outcome==DBT_COMMAND_TIMEOUT)g->state=DBT_GAP_FAILED;
        return;
    }
    if(g->rename_step<2){++g->rename_step;g->rename_partial=true;return;}
    copy((uint8_t *)g->config.name,(const uint8_t *)g->rename_name,sizeof(g->config.name));
    copy(g->config.scan_response,g->rename_scan,g->rename_scan_size);g->config.scan_response_size=g->rename_scan_size;
    g->rename_done=g->rename_serial;g->rename_applied=true;g->rename_partial=false;g->rename_active=false;
}
static dbt_result_t rename_next(dbt_gap_t *g){
    uint8_t data[248]={0};uint16_t opcode;uint8_t size;
    if(!g->rename_step){opcode=0x0c13;size=248;copy(data,(const uint8_t *)g->rename_name,sizeof(g->rename_name));}
    else if(g->rename_step==1){opcode=0x2009;size=32;data[0]=g->rename_scan_size;copy(data+1,g->rename_scan,data[0]);}
    else{opcode=0x0c52;size=241;inquiry_response(data,g->rename_name,&g->config);}
    return dbt_host_command(g->host,opcode,data,size,DBT_COMMAND_COMPLETE,renamed,g,g->now_ms);
}
dbt_result_t dbt_gap_rename(dbt_gap_t *g,const uint8_t *name,uint8_t size,uint32_t *token){
    if(!g || !name || !token || !size || size>32 || !dbt_utf8_valid(name,size))return DBT_INVALID;
    if(g->state!=DBT_GAP_READY || g->rename_active || g->rename_pending)return DBT_BUSY;
    if(g->rename_serial==UINT32_MAX)return DBT_NO_SPACE;
    for(unsigned i=0;i<sizeof(g->rename_name);i++)g->rename_name[i]=i<size?(char)name[i]:0;
    unsigned n=size>29?29:size;if(n<size)while(n && (name[n]&0xc0u)==0x80u)--n;
    g->rename_scan[0]=(uint8_t)(n+1);g->rename_scan[1]=n<size?8:9;copy(g->rename_scan+2,name,n);g->rename_scan_size=(uint8_t)(n+2);
    ++g->rename_serial;g->rename_step=0;g->rename_status=0;g->rename_applied=false;g->rename_active=true;*token=g->rename_serial;return DBT_OK;
}
dbt_result_t dbt_gap_rename_result(const dbt_gap_t *g,uint32_t token,bool *applied,uint8_t *status){
    if(!g || !token || !applied || !status)return DBT_INVALID;
    if(token!=g->rename_serial)return DBT_EMPTY;
    if(g->rename_active)return DBT_BUSY;
    if(token!=g->rename_done)return DBT_EMPTY;
    *applied=g->rename_applied;*status=g->rename_status;return DBT_OK;
}
static dbt_result_t incoming_final(dbt_gap_t *gap,const uint8_t *data){
    if(data[9]!=1)return DBT_UNSUPPORTED;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){
        dbt_gap_incoming_t *entry=&gap->incoming[i];
        if(!(entry->flags&IN_USED) || (entry->flags&IN_FINAL) || !same(entry->address_le,data+3))continue;
        entry->flags|=IN_FINAL;entry->status=data[0];
        incoming_trace(entry,DBT_GAP_INCOMING_FINAL,DBT_COMMAND_SUCCESS);
        if(entry->flags&IN_COMMAND){
            dbt_command_cancel_result_t result=dbt_host_cancel_command(gap->host,entry->token);
            if(result==DBT_COMMAND_NOT_FOUND){
                entry->flags&=(uint8_t)~IN_COMMAND;
                dbt_host_abort(gap->host,DBT_FAULT_BAD_RESPONSE);return DBT_INVALID;
            }
        }
        release_incoming(entry);return DBT_OK;
    }
    return DBT_UNSUPPORTED;
}
void dbt_gap_poll(dbt_gap_t *gap, uint32_t now_ms) {
    if (!gap) return;
    gap->now_ms = now_ms;
    if (gap->host->state == DBT_HOST_FAILED) { gap->state = DBT_GAP_FAILED;if(gap->rename_active){gap->rename_done=gap->rename_serial;gap->rename_applied=false;gap->rename_active=false;}return; }
    if (gap->state == DBT_GAP_FAILED || gap->state == DBT_GAP_IDLE) return;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){
        const dbt_gap_incoming_t *entry=&gap->incoming[i];
        if((entry->flags&IN_USED) && !(entry->flags&IN_FINAL) && (int32_t)(now_ms-entry->deadline)>=0){
            incoming_trace(&gap->incoming[i],DBT_GAP_INCOMING_EXPIRED,DBT_COMMAND_TIMEOUT);
            gap->state=DBT_GAP_FAILED;dbt_host_abort(gap->host,DBT_FAULT_TIMEOUT);return;
        }
    }
    /* Security and connection decisions precede nonurgent setup commands. */
    if (gap->reply_count) {
        dbt_gap_reply_t *reply = &gap->replies[gap->reply_head];
        dbt_gap_incoming_t *entry=reply->incoming<DBT_MAX_LINKS?&gap->incoming[reply->incoming]:0;
        if(entry && (!(entry->flags&IN_USED) || !(entry->flags&IN_QUEUED))){
            gap->state=DBT_GAP_FAILED;dbt_host_abort(gap->host,DBT_FAULT_BAD_RESPONSE);return;
        }
        dbt_result_t result;
        if(entry && (entry->flags&IN_FINAL)){
            /* Controller finished before this decision reached HCI. Drop the
             * queued reply; it must not act on a later request for that peer. */
            entry->flags&=(uint8_t)~IN_QUEUED;release_incoming(entry);result=DBT_OK;
        }else if(entry){
            uint32_t token;
            result=dbt_host_command_tracked(gap->host,reply->opcode,reply->bytes,reply->size,
                reply->response,incoming_replied,entry,now_ms,&token);
            if(result==DBT_OK){
                entry->token=token;entry->flags=(uint8_t)((entry->flags&~IN_QUEUED)|IN_COMMAND);
                ++gap->reply_inflight;
                incoming_trace(entry,DBT_GAP_INCOMING_SUBMITTED,DBT_COMMAND_SUCCESS);
            }else if(result==DBT_NO_SPACE && gap->host->command_count<DBT_COMMAND_SLOTS){
                gap->state=DBT_GAP_FAILED;dbt_host_abort(gap->host,DBT_FAULT_CONTROLLER);return;
            }
        }else{
            result=dbt_host_command(gap->host,reply->opcode,reply->bytes,reply->size,
                reply->response,queued_replied,gap,now_ms);
            if(result==DBT_OK)++gap->reply_inflight;
        }
        if (result == DBT_OK) {
            gap->reply_head = (uint8_t)((gap->reply_head + 1u) % DBT_GAP_REPLY_SLOTS); --gap->reply_count;
        }
    }
    if(gap->state==DBT_GAP_READY && gap->rename_active && !gap->rename_pending && !gap->classic_pending && !gap->advertising_pending && !gap->reply_count && !gap->reply_inflight){
        dbt_result_t result=rename_next(gap);
        if(result==DBT_OK)gap->rename_pending=true;
        else if(result!=DBT_BUSY && result!=DBT_NO_SPACE){gap->rename_done=gap->rename_serial;gap->rename_applied=false;gap->rename_active=false;}
    }
    if (gap->state == DBT_GAP_CONFIGURING && !gap->command_pending && !gap->reply_count && !gap->reply_inflight) {
        dbt_result_t result = configure_next(gap);
        if (result == DBT_OK) gap->command_pending = true;
        else if (result != DBT_NO_SPACE && result != DBT_BUSY) gap->state = DBT_GAP_FAILED;
    }
}
static dbt_result_t reply_owned(dbt_gap_t *gap,uint16_t opcode,const uint8_t *data,uint8_t size,
                                dbt_command_response_t response,uint8_t incoming){
    if (gap->reply_count == DBT_GAP_REPLY_SLOTS) return DBT_NO_SPACE;
    dbt_gap_reply_t *slot = &gap->replies[(gap->reply_head + gap->reply_count) % DBT_GAP_REPLY_SLOTS];
    slot->opcode = opcode; slot->response = response; slot->size = size;slot->incoming=incoming;copy(slot->bytes, data, size);
    ++gap->reply_count;
    return DBT_OK;
}
static dbt_result_t reply(dbt_gap_t *gap,uint16_t opcode,const uint8_t *data,uint8_t size,
                         dbt_command_response_t response){return reply_owned(gap,opcode,data,size,response,UINT8_MAX);}
dbt_result_t dbt_gap_event_at(dbt_gap_t *gap, const dbt_hci_packet_t *packet,uint32_t now_ms) {
    if (!gap || !packet || packet->type != DBT_H4_EVENT) return DBT_INVALID;
    gap->now_ms=now_ms;
    if (gap->state == DBT_GAP_IDLE || gap->state == DBT_GAP_FAILED) return DBT_BUSY;
    const uint8_t *data = packet->payload;
    if (packet->size && !data) return DBT_INVALID;
    uint8_t bytes[7] = {0}; uint16_t opcode = 0; uint16_t expected = 0;
    switch (packet->event) {
    case 0x03:
        if(packet->size!=11)return DBT_INVALID;
        return incoming_final(gap,data);
    case 0x04: { /* Connection Request. Status is admission, Connection Complete is final. */
        if (packet->size != 10) return DBT_INVALID;
        copy(bytes, data, 6);
        if(data[9]>2u)return DBT_INVALID;
        if(data[9]!=1u){
            bytes[6]=0x0f;
            /* eSCO requires the synchronous rejection command. No synchronous
             * profile is installed, and no ACL reservation belongs to it. */
            return reply(gap,data[9]==2u?0x042a:0x040a,bytes,7,DBT_COMMAND_STATUS);
        }
        unsigned free_slot=DBT_MAX_LINKS;
        for(unsigned i=0;i<DBT_MAX_LINKS;++i){
            if(!(gap->incoming[i].flags&IN_USED))free_slot=i;
            else if(!(gap->incoming[i].flags&IN_FINAL) && same(gap->incoming[i].address_le,data))
                return DBT_OK; /* Already owns one decision; do not grant a second lease. */
        }
        if(gap->reply_count==DBT_GAP_REPLY_SLOTS)return DBT_NO_SPACE;
        if(free_slot==DBT_MAX_LINKS){bytes[6]=0x0d;return reply(gap,0x040a,bytes,7,DBT_COMMAND_STATUS);}
        /* A peer cannot legitimately use this controller's own public BD_ADDR.
         * Reject before the policy hook can grant a pairing/admission lease. */
        uint8_t remote=0;
        for(unsigned i=0;i<6;++i)remote|=(uint8_t)(data[i]^gap->host->controller.address_le[i]);
        bool accepted = remote && data[9] == 1 && gap->state == DBT_GAP_READY && gap->hooks.accept_classic &&
            gap->hooks.accept_classic(gap->hooks.context, data, data + 6);
        bytes[6] = accepted ? 1 : 0x0f;
        dbt_gap_incoming_t *entry=&gap->incoming[free_slot];
        copy(entry->address_le,data,6);entry->deadline=now_ms+DBT_GAP_CLASSIC_SETUP_MS;
        entry->token=0;entry->status=0;entry->flags=(uint8_t)(IN_USED|IN_QUEUED|(accepted?IN_ACCEPT:0));
        dbt_result_t result=reply_owned(gap,accepted?0x0409:0x040a,bytes,7,DBT_COMMAND_STATUS,(uint8_t)free_slot);
        if(result!=DBT_OK)entry->flags=0;
        else incoming_trace(entry,DBT_GAP_INCOMING_QUEUED,DBT_COMMAND_SUCCESS);
        return result;
    }
    case 0x16: opcode = 0x040e; expected = 6; break; /* PIN negative. */
    case 0x17: opcode = 0x040c; expected = 6; break; /* Link key negative. */
    case 0x31: opcode = 0x0434; expected = 6; break; /* IO capability negative. */
    case 0x33: opcode = 0x042d; expected = 10; break; /* User confirmation negative. */
    case 0x34: opcode = 0x042f; expected = 6; break; /* Passkey negative. */
    case 0x35: opcode = 0x0433; expected = 6; break; /* OOB negative. */
    case 0x3e:
        if (!packet->size) return DBT_INVALID;
        if (data[0] == 1 || data[0] == 0x0a) {
            if (packet->size != (data[0] == 1 ? 19 : 31)) return DBT_INVALID;
            /* Advertising stops when a peripheral connection is established. */
            if (data[1] == 0 && data[4] == 1) gap->advertising = false;
            return DBT_OK;
        }
        if (data[0] != 5) return DBT_UNSUPPORTED;
        if (packet->size != 13) return DBT_INVALID;
        if (gap->hooks.security && gap->hooks.security(gap->hooks.context, packet)) return DBT_OK;
        return reply(gap, 0x201b, data + 1, 2, DBT_COMMAND_COMPLETE);
    default: return DBT_UNSUPPORTED;
    }
    if (packet->size != expected) return DBT_INVALID;
    if (gap->hooks.security && gap->hooks.security(gap->hooks.context, packet)) return DBT_OK;
    copy(bytes, data, 6); bytes[6] = 0x18; /* Pairing Not Allowed. */
    return reply(gap, opcode, bytes, opcode == 0x0434 ? 7 : 6, DBT_COMMAND_COMPLETE);
}
dbt_result_t dbt_gap_event(dbt_gap_t *gap,const dbt_hci_packet_t *packet){
    return dbt_gap_event_at(gap,packet,gap?gap->now_ms:0);
}
void dbt_gap_set_incoming_observer(dbt_gap_t *gap,dbt_gap_incoming_observer_t observer,void *context){
    if(gap){gap->incoming_observer=observer;gap->incoming_context=context;}
}
bool dbt_gap_classic_pending(const dbt_gap_t *gap,const uint8_t address[6]){
    if(!gap || !address)return false;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)
        if((gap->incoming[i].flags&IN_USED) && same(gap->incoming[i].address_le,address))return true;
    return false;
}
uint8_t dbt_gap_classic_reservations(const dbt_gap_t *gap,const dbt_acl_t *acl){
    if(!gap || !acl)return DBT_MAX_LINKS;
    uint8_t reserved=0;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){
        const dbt_gap_incoming_t *entry=&gap->incoming[i];
        if((entry->flags&(IN_USED|IN_ACCEPT|IN_FINAL))!=(IN_USED|IN_ACCEPT))continue;
        bool published=false;
        for(unsigned j=0;j<DBT_MAX_LINKS;++j){
            const dbt_link_t *link=&acl->links[j];
            if(link->used && link->kind==DBT_LINK_CLASSIC && same(link->peer,entry->address_le)){published=true;break;}
        }
        if(!published)++reserved;
    }
    return reserved;
}
uint32_t dbt_gap_wait_ms(const dbt_gap_t *gap) {
    if (!gap || gap->state == DBT_GAP_FAILED || gap->state == DBT_GAP_IDLE) return UINT32_MAX;
    uint32_t wait=UINT32_MAX;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){
        const dbt_gap_incoming_t *entry=&gap->incoming[i];
        if((entry->flags&IN_USED) && !(entry->flags&IN_FINAL)){
            uint32_t next=(int32_t)(gap->now_ms-entry->deadline)>=0?0:entry->deadline-gap->now_ms;
            if(next<wait)wait=next;
        }
    }
    if(gap->reply_count){
        unsigned owner=gap->replies[gap->reply_head].incoming;
        if(owner<DBT_MAX_LINKS && (gap->incoming[owner].flags&IN_FINAL))return 0;
    }
    if(gap->host->command_count==DBT_COMMAND_SLOTS)return wait;
    if(gap->reply_count ||
       (gap->state==DBT_GAP_CONFIGURING && !gap->command_pending && !gap->reply_inflight) ||
       (gap->state==DBT_GAP_READY && gap->rename_active && !gap->rename_pending &&
        !gap->classic_pending && !gap->advertising_pending && !gap->reply_inflight))return 0;
    return wait;
}
dbt_result_t dbt_gap_set_classic_scan(dbt_gap_t *gap, bool discoverable, bool connectable) {
    if (!gap || gap->state != DBT_GAP_READY || gap->classic_pending || gap->rename_active) return DBT_BUSY;
    uint8_t scan = (uint8_t)((discoverable ? 1 : 0) | (connectable ? 2 : 0));
    dbt_result_t result = dbt_host_command(gap->host, 0x0c1a, &scan, 1, DBT_COMMAND_COMPLETE, visibility, gap, gap->now_ms);
    if (result == DBT_OK) { gap->wanted_scan = scan; gap->classic_pending = true; }
    return result;
}
dbt_result_t dbt_gap_set_advertising(dbt_gap_t *gap, bool enabled) {
    if (!gap || gap->state != DBT_GAP_READY || gap->advertising_pending || gap->rename_active) return DBT_BUSY;
    uint8_t flag = enabled ? 1 : 0;
    dbt_result_t result = dbt_host_command(gap->host, 0x200a, &flag, 1, DBT_COMMAND_COMPLETE, visibility, gap, gap->now_ms);
    if (result == DBT_OK) { gap->wanted_advertising = enabled; gap->advertising_pending = true; }
    return result;
}
dbt_result_t dbt_gap_disconnect(dbt_gap_t *gap, const dbt_link_t *link, uint8_t reason) {
    if (!gap || !link || !link->used || gap->state != DBT_GAP_READY ||
        (reason != 0x05 && reason != 0x13 && reason != 0x14 && reason != 0x15 && reason != 0x1a && reason != 0x29 && reason != 0x3b)) return DBT_INVALID;
    uint8_t bytes[3]; put16(bytes, link->handle); bytes[2] = reason;
    return dbt_host_command(gap->host, 0x0406, bytes, 3, DBT_COMMAND_STATUS, replied, gap, gap->now_ms);
}
