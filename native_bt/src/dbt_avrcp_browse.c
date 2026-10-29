#include "dbt_avrcp_browse.h"
#include <string.h>
enum { GENERAL_REJECT=0xa0, AUTO_PID=0xff };
static uint16_t u16(const uint8_t *p) { return (uint16_t)((p[0]<<8)|p[1]); }
static uint32_t u32(const uint8_t *p) { return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]; }
static bool due(uint32_t n,uint32_t d) { return (int32_t)(n-d)>=0; }
static bool supported(uint8_t id) { return id==0x70||id==0x71||id==0x72||id==0x73||id==0x75||id==0x80; }
static bool changing(uint8_t id) { return id==0x70 || id==0x72 || id==0x80; }
static bool independent(uint8_t id,uint8_t scope) { return id==0x71 && scope==0; }
static bool command(uint8_t id,const uint8_t *p,uint16_t n) {
    switch(id) {
    case 0x70: return n==2;
    case 0x71: return n>=10 && p[0]<=3 && u32(p+1)<=u32(p+5) && n==10u+4u*(p[9]==0xff ? 0u : p[9]);
    case 0x72: return n==11 && p[2]<=1;
    case 0x73: { if (n<12 || p[0]>3 || n!=12u+4u*p[11]) return false;
        uint8_t any=0; for(unsigned i=1;i<9;i++) any|=p[i]; return any!=0; }
    case 0x75: return n==1 && p[0]<=3;
    case 0x80: return n>=4 && u16(p+2)==n-4 && n>4;
    default: return false;
    }
}
static bool attributes(const uint8_t *p,uint16_t n) {
    if (!n) return false;
    uint32_t at=1;
    for(unsigned i=0;i<p[0];i++) { if(at+8>n) return false; uint16_t len=u16(p+at+6); at+=8u+len; if(at>n) return false; }
    return at==n;
}
static bool item(uint8_t type,const uint8_t *p,uint16_t n) {
    if(type==1) return n>=28 && (p[7]<=4 || p[7]==0xff) && n==28u+u16(p+26);
    if(type==2) return n>=14 && p[8]<=6 && p[9]<=1 && n==14u+u16(p+12);
    if(type==3) { if(n<14) return false; uint16_t name=u16(p+11); uint32_t at=13u+name;
        return at<n && attributes(p+at,(uint16_t)(n-at)); }
    return false;
}
static bool reply(uint8_t id,const uint8_t *p,uint16_t n,uint8_t scope) {
    if (!n) return false;
    if (p[0]!=4) return n==1 && p[0]<=0x16 && p[0]!=6;
    switch(id) {
    case 0x70: { if(n<10) return false; uint32_t at=10;
        for(unsigned i=0;i<p[9];i++) { if(at+2>n) return false; uint16_t len=u16(p+at); at+=2u+len; if(at>n) return false; }
        return at==n; }
    case 0x71: { if(n<5) return false; uint32_t at=5; uint16_t count=u16(p+3);
        for(unsigned i=0;i<count;i++) { if(at+3>n) return false; uint8_t type=p[at]; uint16_t len=u16(p+at+1); at+=3;
            if(at+len>n || (scope==0 && type!=1) || (scope==1 && type==1) ||
               ((scope==2 || scope==3) && type!=3) || !item(type,p+at,len)) return false;
            at+=len; }
        return at==n; }
    case 0x72: return n==5;
    case 0x73: return n>=2 && attributes(p+1,(uint16_t)(n-1));
    case 0x75: case 0x80: return n==7;
    default: return false;
    }
}
static bool bump(dbt_avrcp_browse_t *s,uint32_t *epoch) {
    if (++*epoch==0) { s->failed=true; return false; } return true;
}
static bool auto_reply(dbt_avrcp_browse_t *s,uint8_t label,uint8_t pdu,uint8_t status) {
    if(s->remote_pending&(1u<<label)) { s->status=DBT_BUSY; return false; }
    s->remote_pending|=(uint16_t)(1u<<label); s->remote_auto|=(uint16_t)(1u<<label);
    s->auto_pdu[label]=pdu; s->auto_status[label]=status; s->token[label]=0;
    s->deadline[label]=s->now+s->timeout; return true;
}
static uint8_t scope_of(uint8_t id,const uint8_t *p) { return id==0x71 || id==0x73 || id==0x75 ? p[0] : 0xff; }
static uint16_t arg_of(uint8_t id,const uint8_t *p) { return id==0x70 ? u16(p) : id==0x72 ? p[2] : 0; }
static uint16_t request_uid(uint8_t id,const uint8_t *p) { return id==0x72 ? u16(p) : u16(p+9); }
static void apply_state(dbt_avrcp_browse_t *s,bool target,uint8_t id,uint8_t scope,uint16_t arg,const uint8_t *p) {
    bool *selected=target ? &s->target_selected : &s->local_selected;
    bool *known=target ? &s->target_uid_valid : &s->local_uid_valid;
    bool *search=target ? &s->target_search_valid : &s->local_search_valid;
    uint16_t *player=target ? &s->target_player : &s->local_player;
    uint16_t *uid=target ? &s->target_uid : &s->local_uid;
    uint8_t *depth=target ? &s->target_depth : &s->local_depth;
    uint32_t *epoch=target ? &s->target_epoch : &s->local_epoch;
    if(id==0x70) {
        *selected=true; *known=true; *search=false; *player=arg; *uid=u16(p+1); *depth=p[9]; (void)bump(s,epoch);
    } else if(id==0x72) {
        if(arg==0) { if(*depth) --*depth; } else if(*depth<255) ++*depth;
        (void)bump(s,epoch);
    } else if(id==0x80) {
        *search=true; *uid=u16(p+1); *known=true; (void)bump(s,epoch);
    } else if((id==0x71 || id==0x75) && (scope==1 || scope==2) && *selected) {
        uint16_t fresh=u16(p+1);
        if(*known && *uid!=fresh) { *search=false; (void)bump(s,epoch); }
        *uid=fresh; *known=true;
    } else if((id==0x71 || id==0x75) && scope==3) {
        uint16_t *addressed=target ? &s->target_addressed_uid : &s->local_addressed_uid;
        bool *valid=target ? &s->target_addressed_valid : &s->local_addressed_valid;
        uint16_t fresh=u16(p+1); if(*valid && *addressed!=fresh) (void)bump(s,epoch);
        *addressed=fresh; *valid=true;
    }
}
static bool incoming(void *ctx,const dbt_avctp_message_t *m) {
    dbt_avrcp_browse_t *s=ctx; dbt_avrcp_message_t v={0}; v.response=m->response; v.label=m->label;
    if (m->pid!=0x110e) {
        if(m->response) {s->status=DBT_UNSUPPORTED; return true;}
        if(auto_reply(s,m->label,AUTO_PID,0)) s->auto_pid[m->label]=m->pid;
        return false;
    }
    if(m->invalid_pid) { v.invalid_pid=true; s->local_mutating&=(uint16_t)~(1u<<m->label); s->in_callback=true; s->message(s->context,&v); s->in_callback=false; return false; }
    if(m->size<3) {
        if(!m->response) { (void)auto_reply(s,m->label,GENERAL_REJECT,2); return false; }
        goto invalid;
    }
    v.pdu=m->data[0]; v.parameters=m->data+3; v.size=(uint16_t)(m->size-3);
    if(u16(m->data+1)!=m->size-3) {
        if(!m->response) { (void)auto_reply(s,m->label,supported(v.pdu)?v.pdu:GENERAL_REJECT,2); return false; }
        goto invalid;
    }
    if(m->response) {
        if(v.pdu==GENERAL_REJECT) { if(v.size!=1 || v.parameters[0]>0x16) goto invalid; }
        else if(v.pdu!=s->local_pdu[v.label] || !reply(v.pdu,v.parameters,v.size,s->local_scope[v.label])) goto invalid;
        v.stale=s->local_snapshot[v.label]!=s->local_epoch && !independent(s->local_pdu[v.label],s->local_scope[v.label]);
        if(!v.stale && v.pdu==0x72 && v.parameters[0]==4 &&
            ((s->local_arg[v.label]==0 && s->local_depth==0) || (s->local_arg[v.label]==1 && s->local_depth==255))) goto invalid;
        if(!v.stale && v.pdu!=GENERAL_REJECT && v.parameters[0]==5) {
            if(s->local_scope[v.label]==3) s->local_addressed_valid=false;
            else { s->local_uid_valid=false; s->local_search_valid=false; }
            (void)bump(s,&s->local_epoch);
        }
        if(!v.stale && v.pdu!=GENERAL_REJECT && v.parameters[0]==4)
            apply_state(s,false,v.pdu,s->local_scope[v.label],s->local_arg[v.label],v.parameters);
        s->local_mutating&=(uint16_t)~(1u<<v.label);
    } else {
        if(!supported(v.pdu)) { (void)auto_reply(s,v.label,GENERAL_REJECT,0); return false; }
        if(!command(v.pdu,v.parameters,v.size)) { (void)auto_reply(s,v.label,v.pdu,2); return false; }
        if(s->remote_pending&(1u<<v.label)) {s->status=DBT_BUSY; return false;}
        uint8_t scope=scope_of(v.pdu,v.parameters);
        bool independent_cmd=independent(v.pdu,scope), mutating=changing(v.pdu);
        if(!independent_cmd && v.pdu!=0x70 && !s->target_selected) { (void)auto_reply(s,v.label,v.pdu,3); return false; }
        if(!independent_cmd && (s->remote_mutating || (mutating && (s->remote_pending & (uint16_t)~s->remote_auto)))) {
            (void)auto_reply(s,v.label,v.pdu,3); return false;
        }
        if(scope==2 && !s->target_search_valid) { (void)auto_reply(s,v.label,v.pdu,0x14); return false; }
        if(((v.pdu==0x72 || (v.pdu==0x73 && (scope==1 || scope==2))) && s->target_uid_valid && request_uid(v.pdu,v.parameters)!=s->target_uid) ||
            (v.pdu==0x73 && scope==3 && s->target_addressed_valid && request_uid(v.pdu,v.parameters)!=s->target_addressed_uid)) {
            (void)auto_reply(s,v.label,v.pdu,5); return false;
        }
        if(++s->next_token==0) {s->failed=true; goto invalid;}
        v.token=((uint64_t)s->transport->binding<<32)|s->next_token;
        s->token[v.label]=v.token; s->remote_pdu[v.label]=v.pdu; s->remote_scope[v.label]=scope;
        s->remote_arg[v.label]=arg_of(v.pdu,v.parameters); s->remote_snapshot[v.label]=s->target_epoch;
        s->remote_pending|=(uint16_t)(1u<<v.label); if(mutating) s->remote_mutating|=(uint16_t)(1u<<v.label);
        s->deadline[v.label]=s->now+s->timeout;
    }
    s->in_callback=true; s->message(s->context,&v); s->in_callback=false; return false;
invalid:
    s->status=DBT_INVALID; return m->response;
}
dbt_result_t dbt_avrcp_browse_init(dbt_avrcp_browse_t *s,dbt_avctp_t *t,dbt_avrcp_message_fn fn,void *ctx,uint32_t timeout) {
    if(!s || !t || !fn || !timeout || timeout>INT32_MAX) return DBT_INVALID;
    *s=(dbt_avrcp_browse_t){0}; s->transport=t; s->message=fn; s->context=ctx; s->timeout=timeout; s->local_epoch=1; s->target_epoch=1; return DBT_OK;
}
static void transmitted(void *context,uint8_t label,bool response,uint32_t now) {
    (void)now; dbt_avrcp_browse_t *s=context;
    if(response && (s->state_change_admission & (1u<<label))) {
        s->state_change_admission&=(uint16_t)~(1u<<label);
        dbt_avrcp_message_t event={0}; event.pdu=0x70; event.state_change=true;
        s->in_callback=true; s->message(s->context,&event); s->in_callback=false;
    }
}
dbt_avctp_hooks_t dbt_avrcp_browse_hooks(dbt_avrcp_browse_t *s) {
    return (dbt_avctp_hooks_t){.context=s,.message=incoming,.transmitted=transmitted};
}
dbt_result_t dbt_avrcp_browse_uids_changed(dbt_avrcp_browse_t *s,bool target,uint16_t uid,bool unavailable) {
    if(!s || s->failed || s->in_callback) return DBT_INVALID;
    if(target) { if(!bump(s,&s->target_epoch)) return DBT_INVALID; s->target_uid=uid; s->target_uid_valid=!unavailable;
        s->target_search_valid=false; if(unavailable) s->target_selected=false; }
    else { if(!bump(s,&s->local_epoch)) return DBT_INVALID; s->local_uid=uid; s->local_uid_valid=!unavailable;
        s->local_search_valid=false; if(unavailable) s->local_selected=false; }
    return DBT_OK;
}
dbt_result_t dbt_avrcp_browse_addressed_uids_changed(dbt_avrcp_browse_t *s,bool target,uint16_t uid,bool valid) {
    if(!s || s->failed || s->in_callback) return DBT_INVALID;
    if(target) { if(!bump(s,&s->target_epoch)) return DBT_INVALID; s->target_addressed_uid=uid; s->target_addressed_valid=valid; }
    else { if(!bump(s,&s->local_epoch)) return DBT_INVALID; s->local_addressed_uid=uid; s->local_addressed_valid=valid; }
    return DBT_OK;
}
dbt_result_t dbt_avrcp_browse_poll(dbt_avrcp_browse_t *s,uint32_t now) {
    if(!s || !s->transport || !s->transport->browsing || s->failed || s->in_callback) return DBT_INVALID;
    for(unsigned i=0;i<16;i++) if((s->remote_pending&(1u<<i)) && due(now,s->deadline[i])) {s->failed=true; return DBT_INVALID;}
    dbt_result_t r=dbt_avctp_poll(s->transport,now); if(r!=DBT_OK) return r;
    for(unsigned i=0;i<16;i++) if(s->remote_auto&(1u<<i)) {
        if(s->auto_pdu[i]==AUTO_PID) r=dbt_avctp_reject_pid(s->transport,(uint8_t)i,s->auto_pid[i],now);
        else { uint8_t f[]={s->auto_pdu[i],0,1,s->auto_status[i]}; r=dbt_avctp_respond(s->transport,(uint8_t)i,false,f,sizeof(f),now); }
        if(r==DBT_BUSY) return DBT_OK;
        if(r!=DBT_OK) return r;
        s->remote_auto&=(uint16_t)~(1u<<i); s->remote_pending&=(uint16_t)~(1u<<i); s->remote_mutating&=(uint16_t)~(1u<<i); break;
    }
    return DBT_OK;
}
dbt_result_t dbt_avrcp_browse_receive(dbt_avrcp_browse_t *s,uint32_t binding,const uint8_t *p,uint16_t n,uint32_t now) {
    if(!s || !s->transport || binding!=s->transport->binding) return DBT_EMPTY;
    if(dbt_avrcp_browse_poll(s,now)!=DBT_OK) return DBT_INVALID;
    s->now=now; s->status=DBT_OK; dbt_result_t r=dbt_avctp_receive(s->transport,binding,p,n,now);
    return r==DBT_OK ? s->status : r;
}
static dbt_result_t queue(dbt_avrcp_browse_t *s,uint8_t id,const uint8_t *p,uint16_t n,uint32_t now,uint8_t *label,bool response) {
    if(!s || !s->transport || !s->transport->browsing || s->failed || (n && !p) || !label) return DBT_INVALID;
    dbt_avctp_t *t=s->transport;
    if(!t->binding || t->failed || !t->tx) return DBT_INVALID;
    if(t->tx_used || t->pinned) return DBT_BUSY;
    if((uint32_t)n+3u>t->tx_capacity || (uint32_t)n+6u>t->tx_mtu || (uint32_t)n+3u>UINT16_MAX) return DBT_NO_SPACE;
    uint8_t *frame=t->tx; if(n) memmove(frame+3,p,n);
    frame[0]=id; frame[1]=(uint8_t)(n>>8); frame[2]=(uint8_t)n;
    return response ? dbt_avctp_respond(t,*label,false,frame,(uint16_t)(n+3),now) :
                      dbt_avctp_command(t,frame,(uint16_t)(n+3),now,label);
}
dbt_result_t dbt_avrcp_browse_command(dbt_avrcp_browse_t *s,uint8_t id,const uint8_t *p,uint16_t n,uint32_t now,uint8_t *label) {
    if(!s || !s->transport || s->in_callback || (n && !p) || !command(id,p,n)) return DBT_INVALID;
    uint8_t scope=scope_of(id,p); bool indep=independent(id,scope);
    if(!indep && id!=0x70 && !s->local_selected) return DBT_INVALID;
    if(s->local_mutating || (changing(id) && s->transport->local_pending)) return DBT_BUSY;
    if(scope==2 && !s->local_search_valid) return DBT_INVALID;
    if((id==0x72 || (id==0x73 && (scope==1 || scope==2))) &&
        (!s->local_uid_valid || request_uid(id,p)!=s->local_uid)) return DBT_INVALID;
    if(id==0x73 && scope==3 && (!s->local_addressed_valid || request_uid(id,p)!=s->local_addressed_uid)) return DBT_INVALID;
    dbt_result_t r=queue(s,id,p,n,now,label,false);
    if(r==DBT_OK) {s->local_pdu[*label]=id; s->local_scope[*label]=scope; s->local_arg[*label]=arg_of(id,p);
        s->local_snapshot[*label]=s->local_epoch; if(changing(id)) s->local_mutating|=(uint16_t)(1u<<*label);}
    return r;
}
dbt_result_t dbt_avrcp_browse_reply(dbt_avrcp_browse_t *s,uint64_t token,const uint8_t *p,uint16_t n,uint32_t now) {
    if(!s || !token || (n && !p)) return DBT_INVALID;
    unsigned l;
    for(l=0;l<16;l++) if((s->remote_pending&(1u<<l)) && s->token[l]==token) break;
    if(l==16) return DBT_EMPTY;
    if(due(now,s->deadline[l])) { s->failed=true; return DBT_INVALID; }
    uint8_t id=s->remote_pdu[l], scope=s->remote_scope[l];
    if(s->remote_snapshot[l]!=s->target_epoch && !independent(id,scope)) {
        s->remote_auto|=(uint16_t)(1u<<l); s->auto_pdu[l]=id; s->auto_status[l]=5; s->token[l]=0;
        s->remote_mutating&=(uint16_t)~(1u<<l); return DBT_EMPTY;
    }
    if(!reply(id,p,n,scope)) return DBT_INVALID;
    if(id==0x72 && p[0]==4 && ((s->remote_arg[l]==0 && s->target_depth==0) ||
        (s->remote_arg[l]==1 && s->target_depth==255))) return DBT_INVALID;
    uint8_t label=(uint8_t)l; dbt_result_t r=queue(s,id,p,n,now,&label,true);
    if(r==DBT_OK) {
        if(p[0]==4) apply_state(s,true,id,scope,s->remote_arg[l],p);
        s->remote_pending&=(uint16_t)~(1u<<l); s->remote_mutating&=(uint16_t)~(1u<<l);
        if(id==0x70 && p[0]==4) s->state_change_admission|=(uint16_t)(1u<<l);
    }
    return r;
}

uint32_t dbt_avrcp_browse_wait_ms(const dbt_avrcp_browse_t *s,uint32_t now) {
    if(!s || !s->transport || !s->transport->binding || s->failed) return 0;
    if(s->remote_auto && !s->transport->tx_used && !s->transport->pinned) return 0;
    uint32_t wait=dbt_avctp_wait_ms(s->transport,now);
    for(unsigned i=0;i<16;i++) if(s->remote_pending&(1u<<i)) {
        uint32_t d=due(now,s->deadline[i]) ? 0 : s->deadline[i]-now;
        if(d<wait) wait=d;
    }
    return wait;
}
