#include "dbt_avrcp.h"
#include <string.h>

enum { VENDOR=0, PASS=0x7c, PID=0x110e, MAX_PARAMS=502 };
static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0]<<8)|p[1]); }
static bool same(const uint8_t *a,const uint8_t *b,uint16_t n) {
    for(uint16_t i=0;i<n;i++) if(a[i]!=b[i]) return false;
    return true;
}
static bool due(uint32_t n,uint32_t d) { return (int32_t)(n-d)>=0; }
static bool command_params(uint8_t pdu,const uint8_t *p,uint16_t n) {
    switch (pdu) {
    case 0x10: return n==1 && (p[0]==2 || p[0]==3);
    case 0x20:
        if (n<9) return false;
        for (unsigned i=0;i<8;i++) if (p[i]) return false;
        return n==(uint16_t)(9u+4u*p[8]);
    case 0x30: return n==0;
    case 0x40: case 0x41: return n==1 && p[0]==0x20;
    case 0x31: return n==5 && p[0]>=1 && p[0]<=13;
    case 0x50: return n==1 && !(p[0]&0x80);
    case 0x60: return n==2;
    case 0x74: case 0x90: {
        if(n!=11 || p[0]<1 || p[0]>3) return false;
        uint8_t any=0; for(unsigned i=1;i<9;i++) any|=p[i]; return any!=0;
    }
    default: return false;
    }
}
static uint8_t ctype(uint8_t pdu) { return pdu==0x31 ? 3 : (pdu==0x50 || pdu==0x40 || pdu==0x41 || pdu==0x60 || pdu==0x74 || pdu==0x90) ? 0 : 1; }
static bool metadata(const uint8_t *p,uint16_t n) {
    if (!n || !p[0]) return false;
    unsigned count=p[0]; uint32_t pos=1;
    for (unsigned i=0;i<count;i++) {
        if (pos+8>n) return false;
        uint16_t len=be16(p+pos+6); pos+=8;
        if (pos+len>n) return false;
        pos+=len;
    }
    return pos==n;
}
static bool event_value(uint8_t event,const uint8_t *p,uint16_t n) {
    if (!n || p[0]!=event) return false;
    switch (event) {
    case 1: return n==2 && (p[1]<=4 || p[1]==0xff);
    case 2: return n==9;
    case 3: case 4: case 9: case 10: return n==1;
    case 5: return n==5;
    case 6: return n==2 && p[1]<=4;
    case 7: return n==2 && p[1]<=2;
    case 8: return n>=2 && n==2u+2u*p[1];
    case 11: return n==5;
    case 12: return n==3;
    case 13: return n==2 && !(p[1]&0x80);
    default: return false;
    }
}
static bool reply_params(uint8_t pdu,uint8_t code,uint8_t event,const uint8_t *p,uint16_t n) {
    if (code==0x0a) return n==1;
    if (code==8) return command_params(pdu,p,n);
    if (pdu==0x31) return (code==0x0f || code==0x0d) && event_value(event,p,n);
    if (pdu==0x41) return code==9 && n==0;
    if (pdu==0x60 || pdu==0x74 || pdu==0x90) return code==9 && n==1 && p[0]==4;
    if (pdu==0x50) return code==9 && n==1 && !(p[0]&0x80);
    if (code!=0x0c) return false;
    switch (pdu) {
    case 0x10:
        if (n<2 || (p[0]!=2 && p[0]!=3)) return false;
        return n==(uint16_t)(2u+p[1]*(p[0]==2 ? 3u : 1u));
    case 0x20: return metadata(p,n);
    case 0x30: return n==9 && (p[8]<=4 || p[8]==0xff);
    default: return false;
    }
}
enum { AUTO_UNIT=1, AUTO_CONT=2, AUTO_REJECT=3, AUTO_PASS_NOT_IMPL=4, AUTO_PID=5, AUTO_ECHO=6 };
static dbt_result_t send(dbt_avrcp_t *,uint8_t,uint8_t,const uint8_t *,uint16_t,uint32_t,uint8_t *);
static bool auto_queue(dbt_avrcp_t *s,uint8_t l,uint8_t kind,uint8_t opcode,uint8_t pdu,uint8_t value) {
    if (s->remote_pending & (1u<<l)) { s->input_status=DBT_BUSY; return false; }
    s->remote_pending|=(uint16_t)(1u<<l); s->remote_auto|=(uint16_t)(1u<<l);
    s->remote_auto_kind[l]=kind; s->remote_opcode[l]=opcode; s->remote_pdu[l]=pdu;
    s->remote_event[l]=value; s->remote_token[l]=0; s->remote_deadline[l]=s->now+s->remote_timeout;
    return true;
}
static void abandon(dbt_avrcp_t *s,bool send_abort) {
    s->local_abandoned|=s->local_continuation; s->local_continuation=0;
    s->continuation_active=false; s->continuation_needed=false;
    s->continuation_size=0; s->continuation_fragments=0;
    if(send_abort) s->abort_needed=true;
    dbt_avrcp_message_t v={0}; v.response=true; v.aborted=true; v.pdu=0x20; v.label=s->continuation_label;
    s->in_callback=true; s->message(s->context,&v); s->in_callback=false;
}
static bool incoming(void *context,const dbt_avctp_message_t *m) {
    dbt_avrcp_t *s=context; dbt_avrcp_message_t v={0};
    v.label=m->label; v.response=m->response; v.invalid_pid=m->invalid_pid;
    if (m->pid!=PID) {
        if (m->response) { s->input_status=DBT_UNSUPPORTED; return true; }
        if (auto_queue(s,m->label,AUTO_PID,0,0,0)) s->remote_pid[m->label]=m->pid;
        return false;
    }
    if (m->response && (s->local_abandoned & (1u<<m->label)) && m->invalid_pid) {
        s->local_abandoned&=(uint16_t)~(1u<<m->label); return false;
    }
    if (m->invalid_pid) {
        if (!m->response) { s->input_status=DBT_INVALID; return false; }
        s->in_callback=true; s->message(s->context,&v); s->in_callback=false;
        return false;
    }
    const uint8_t *p=m->data; uint16_t n=m->size;
    if (m->response && (s->local_abandoned & (1u<<m->label))) {
        if (n>=10 && p[1]==0x48 && p[2]==VENDOR && !p[3] && p[4]==0x19 && p[5]==0x58 &&
            (p[6]==0x20 || p[6]==0x40) && !(p[7]&0xfc) && be16(p+8)==n-10) {
            s->local_abandoned&=(uint16_t)~(1u<<m->label); return false;
        }
        s->input_status=DBT_INVALID; return true;
    }
    if (!m->response && n==8 && p[0]==1 && p[1]==0xff && (p[2]==0x30 || p[2]==0x31)) {
        if (s->remote_pending & (1u<<m->label)) { s->input_status=DBT_BUSY; return false; }
        if (p[2]==0x30 && (p[3]!=0xff || p[4]!=0xff || p[5]!=0xff || p[6]!=0xff || p[7]!=0xff)) goto invalid;
        if (p[2]==0x31 && ((p[3]&0x8f)!=7 || p[4]!=0xff || p[5]!=0xff || p[6]!=0xff || p[7]!=0xff)) goto invalid;
        (void)auto_queue(s,m->label,AUTO_UNIT,p[2],0,p[3]);
        return false;
    }
    if (n<3 || (p[0]&0xf0) || p[1]!=0x48 || (p[2]!=VENDOR && p[2]!=PASS)) goto invalid;
    v.code=p[0]; v.opcode=p[2]; uint8_t fragment=0;
    if (v.opcode==PASS) {
        if (n!=5 || p[4] || (p[3]&0x7f)>=0x7e || (!m->response && v.code!=0)) goto invalid;
        v.parameters=p+3; v.size=2;
    } else {
        if (n<10 || p[3] || p[4]!=0x19 || p[5]!=0x58 || (p[7]&0xfc) || be16(p+8)!=n-10) goto invalid;
        v.pdu=p[6]; fragment=p[7]; v.parameters=p+10; v.size=(uint16_t)(n-10);
        if (!m->response && fragment) goto invalid;
        if (!m->response && (v.pdu==0x40 || v.pdu==0x41)) {
            if (v.code!=0 || v.size!=1 || v.parameters[0]!=0x20) goto invalid;
            if (s->remote_pending & (1u<<v.label)) { s->input_status=DBT_BUSY; return false; }
            (void)auto_queue(s,v.label,AUTO_CONT,VENDOR,v.pdu,0);
            return false;
        }
        if (!m->response && (v.code!=ctype(v.pdu) || !command_params(v.pdu,v.parameters,v.size))) goto invalid;
    }
    if (m->response) {
        uint8_t l=m->label;
        if ((s->local_continuation & (1u<<l)) && v.opcode==VENDOR &&
            v.pdu==0x40 && !fragment && v.size==1 && (v.code==0x0a || (v.code==8 && v.parameters[0]==0x20))) {
            s->local_continuation&=(uint16_t)~(1u<<l); abandon(s,false);
            s->in_callback=true; s->message(s->context,&v); s->in_callback=false;
            return false;
        }
        if (v.opcode!=s->local_opcode[l] || (v.opcode==VENDOR && v.pdu!=s->local_pdu[l])) goto invalid;
        if (fragment) {
            bool continuing=(s->local_continuation & (1u<<l))!=0;
            if (v.opcode!=VENDOR || v.pdu!=0x20 || v.code!=0x0c ||
                (fragment==1 && (s->continuation_active || continuing)) ||
                (fragment!=1 && (!s->continuation_active || !continuing)) ||
                !v.size || s->continuation_fragments==255) {
                s->failed=true; s->input_status=DBT_INVALID; return false;
            }
            if (fragment==1) { s->continuation_active=true; s->continuation_size=0; s->continuation_fragments=0; s->continuation_label=l; }
            if (!s->continuation_buffer || (uint32_t)s->continuation_size+v.size>s->continuation_capacity) {
                s->local_continuation&=(uint16_t)~(1u<<l);
                abandon(s,true); s->input_status=DBT_NO_SPACE; return false;
            }
            memcpy(s->continuation_buffer+s->continuation_size,v.parameters,v.size);
            s->continuation_size=(uint16_t)(s->continuation_size+v.size); s->continuation_fragments++;
            s->local_continuation&=(uint16_t)~(1u<<l);
            if (fragment!=3) { s->continuation_needed=true; return false; }
            if (!metadata(s->continuation_buffer,s->continuation_size)) {
                s->failed=true; s->input_status=DBT_INVALID; return false;
            }
            v.parameters=s->continuation_buffer; v.size=s->continuation_size;
            s->continuation_active=false; s->continuation_needed=false;
            s->in_callback=true; s->message(s->context,&v); s->in_callback=false;
            s->continuation_size=0; s->continuation_fragments=0;
            return false;
        }
        if (s->local_continuation & (1u<<l)) { s->failed=true; s->input_status=DBT_INVALID; return false; }
        if (v.opcode==PASS) {
            if (v.parameters[0]!=s->local_operation[l] || (v.code!=9 && v.code!=0x0a && v.code!=8)) goto invalid;
        } else {
            if (!reply_params(v.pdu,v.code,s->local_event[l],v.parameters,v.size)) goto invalid;
            if (v.pdu==0x31 && ((v.code==0x0f && (s->local_interim & (1u<<l))) ||
                (v.code==0x0d && !(s->local_interim & (1u<<l))) ||
                (v.code==8 && (s->local_interim & (1u<<l))))) goto invalid;
            if ((v.pdu==0x10 || v.pdu==0x31) && v.code==8 && v.parameters[0]!=s->local_event[l]) goto invalid;
            if (v.pdu==0x10 && v.code==0x0c && v.parameters[0]!=s->local_event[l]) goto invalid;
            if (v.code==8 && s->local_control_size[l]) {
                if (v.size!=s->local_control_size[l] || !same(v.parameters,s->local_control[l],v.size)) goto invalid;
            }
        }
        if (v.opcode==VENDOR && v.pdu!=0x20 && v.pdu!=0x41 && s->continuation_active) abandon(s,false);
        s->local_abort&=(uint16_t)~(1u<<l);
        s->in_callback=true; s->message(s->context,&v); s->in_callback=false;
        if (v.opcode==VENDOR && v.pdu==0x31 && v.code==0x0f) {
            s->local_interim|=(uint16_t)(1u<<l);
            (void)dbt_avctp_transaction_timeout(s->transport,l,0,s->now); return true;
        }
        s->local_interim&=(uint16_t)~(1u<<l);
        return false;
    }
    if (s->remote_pending & (1u<<v.label)) { s->input_status=DBT_BUSY; return false; }
    if (++s->next_token==0) { s->failed=true; s->input_status=DBT_INVALID; return false; }
    v.token=((uint64_t)s->transport->binding<<32)|s->next_token; s->remote_token[v.label]=v.token;
    s->remote_pending|=(uint16_t)(1u<<v.label); s->remote_deadline[v.label]=s->now+s->remote_timeout;
    s->remote_opcode[v.label]=v.opcode; s->remote_pdu[v.label]=v.pdu;
    s->remote_event[v.label]=v.opcode==PASS ? v.parameters[0] : (v.pdu==0x31 || v.pdu==0x10) ? v.parameters[0] : 0;
    s->remote_control_size[v.label]=0;
    if(v.opcode==VENDOR && (v.pdu==0x50 || v.pdu==0x60 || v.pdu==0x74 || v.pdu==0x90)) {
        memcpy(s->remote_control[v.label],v.parameters,v.size); s->remote_control_size[v.label]=(uint8_t)v.size;
    }
    s->in_callback=true; s->message(s->context,&v); s->in_callback=false;
    return false;
invalid:
    if (!m->response && n>=7 && p[1]==0x48 && p[2]==VENDOR &&
        p[3]==0 && p[4]==0x19 && p[5]==0x58) {
        uint8_t id=p[6], error=2;
        if (id!=0x10 && id!=0x20 && id!=0x30 && id!=0x31 && id!=0x40 && id!=0x41 && id!=0x50 && id!=0x60 && id!=0x74 && id!=0x90) error=0;
        else if (n>=11 && (id==0x10 || id==0x31) &&
            ((id==0x10 && p[10]!=2 && p[10]!=3) || (id==0x31 && (p[10]<1 || p[10]>13)))) error=1;
        (void)auto_queue(s,m->label,AUTO_REJECT,VENDOR,id,error); return false;
    }
    if (!m->response && n==5 && p[1]==0x48 && p[2]==PASS && p[4]==0) {
        (void)auto_queue(s,m->label,AUTO_PASS_NOT_IMPL,PASS,0,p[3]); return false;
    }
    if (!m->response && n>=3 && n<=512 && !(p[0]&0xf0) && p[0]<=4) {
        if (s->echo_active) { s->input_status=DBT_BUSY; return false; }
        if (!s->echo_buffer || s->echo_capacity<n) {
            if (!s->transport->tx_used && !s->transport->pinned) {
                uint8_t frame[512]; memcpy(frame,p,n); frame[0]=8;
                s->input_status=dbt_avctp_respond(s->transport,m->label,false,frame,n,s->now);
            } else s->input_status=DBT_NO_SPACE;
            return false;
        }
        if (auto_queue(s,m->label,AUTO_ECHO,p[2],0,0)) {
            memcpy(s->echo_buffer,p,n); s->echo_buffer[0]=8; s->echo_size=n; s->echo_label=m->label; s->echo_active=true;
        }
        return false;
    }
    s->input_status=DBT_INVALID; return m->response;
}
dbt_result_t dbt_avrcp_init(dbt_avrcp_t *s, dbt_avctp_t *t,dbt_avrcp_message_fn fn,void *ctx,uint32_t timeout) {
    if (!s || !t || !fn || !timeout || timeout>INT32_MAX) return DBT_INVALID;
    *s=(dbt_avrcp_t){0}; s->transport=t; s->message=fn; s->context=ctx; s->remote_timeout=timeout;
    return DBT_OK;
}
dbt_result_t dbt_avrcp_set_echo_buffer(dbt_avrcp_t *s,uint8_t *buffer,uint16_t capacity) {
    if(!s || !buffer || capacity<3 || s->echo_active || s->in_callback) return DBT_INVALID;
    s->echo_buffer=buffer; s->echo_capacity=capacity; return DBT_OK;
}
dbt_result_t dbt_avrcp_set_continuation_buffer(dbt_avrcp_t *s,uint8_t *buffer,uint16_t capacity) {
    if (!s || !buffer || !capacity || s->continuation_active || s->in_callback) return DBT_INVALID;
    s->continuation_buffer=buffer; s->continuation_capacity=capacity; return DBT_OK;
}
dbt_result_t dbt_avrcp_set_target_buffer(dbt_avrcp_t *s,uint8_t *buffer,uint16_t capacity,uint32_t hold_ms) {
    if (!s || !buffer || capacity<=MAX_PARAMS || !hold_ms || hold_ms>INT32_MAX || s->target_active || s->in_callback) return DBT_INVALID;
    s->target_buffer=buffer; s->target_capacity=capacity; s->target_hold_ms=hold_ms; return DBT_OK;
}
dbt_avctp_hooks_t dbt_avrcp_hooks(dbt_avrcp_t *s) { return (dbt_avctp_hooks_t){.context=s,.message=incoming,.transmitted=0}; }
dbt_result_t dbt_avrcp_receive(dbt_avrcp_t *s,uint32_t binding,const uint8_t *p,uint16_t n,uint32_t now) {
    if (!s || !s->transport || s->failed || s->in_callback || s->transport->browsing) return DBT_INVALID;
    if (binding!=s->transport->binding) return DBT_EMPTY;
    if (dbt_avrcp_poll(s,now)!=DBT_OK) return DBT_INVALID;
    s->now=now; s->input_status=DBT_OK;
    dbt_result_t r=dbt_avctp_receive(s->transport,binding,p,n,now);
    return r==DBT_OK ? s->input_status : r;
}
static dbt_result_t send(dbt_avrcp_t *s,uint8_t opcode,uint8_t pdu,const uint8_t *p,uint16_t n,uint32_t now,uint8_t *label) {
    if (!s || !s->transport || !label || s->failed || s->in_callback || (n && !p) || n>MAX_PARAMS) return DBT_INVALID;
    uint8_t frame[10+MAX_PARAMS]; uint16_t size;
    if (opcode==PASS) { frame[0]=0; frame[1]=0x48; frame[2]=PASS; frame[3]=p[0]; frame[4]=0; size=5; }
    else {
        frame[0]=ctype(pdu); frame[1]=0x48; frame[2]=VENDOR; frame[3]=0; frame[4]=0x19; frame[5]=0x58;
        frame[6]=pdu; frame[7]=0; frame[8]=(uint8_t)(n>>8); frame[9]=(uint8_t)n;
        if (n) memcpy(frame+10,p,n);
        size=(uint16_t)(10+n);
    }
    dbt_result_t r=dbt_avctp_command(s->transport,frame,size,now,label);
    if (r==DBT_OK) { uint8_t l=*label; s->local_opcode[l]=opcode; s->local_pdu[l]=pdu==0x40 ? 0x20 : pdu;
        if (pdu==0x40) s->local_continuation|=(uint16_t)(1u<<l);
        if (pdu==0x41) s->local_abort|=(uint16_t)(1u<<l);
        s->local_event[l]=(opcode==VENDOR && (pdu==0x31 || pdu==0x10)) ? p[0] : 0;
        s->local_control_size[l]=0;
        if(opcode==VENDOR && (pdu==0x50 || pdu==0x60 || pdu==0x74 || pdu==0x90)) {
            memcpy(s->local_control[l],p,n); s->local_control_size[l]=(uint8_t)n;
        }
        s->local_operation[l]=opcode==PASS ? p[0] : 0; }
    return r;
}
dbt_result_t dbt_avrcp_passthrough(dbt_avrcp_t *s,uint8_t op,bool release,uint32_t now,uint8_t *label) {
    if (op>=0x7e) return DBT_INVALID;
    uint8_t p=(uint8_t)(op|(release?0x80:0)); return send(s,PASS,0,&p,1,now,label);
}
dbt_result_t dbt_avrcp_command(dbt_avrcp_t *s,uint8_t pdu,const uint8_t *p,uint16_t n,uint32_t now,uint8_t *label) {
    if (!s || pdu==0x40 || pdu==0x41 || (pdu==0x20 && (s->continuation_active || s->abort_needed || s->local_abort)) || (n && !p) || !command_params(pdu,p,n)) return DBT_INVALID;
    if (pdu==0x31 && s->transport) {
        unsigned count=0;
        for (unsigned i=0;i<16;i++) if ((s->transport->local_pending & (1u<<i)) &&
            s->local_opcode[i]==VENDOR && s->local_pdu[i]==0x31) count++;
        if (count>=15) return DBT_BUSY;
    }
    return send(s,VENDOR,pdu,p,n,now,label);
}
dbt_result_t dbt_avrcp_reply(dbt_avrcp_t *s,uint64_t token,uint8_t code,const uint8_t *p,uint16_t n,uint32_t now) {
    if (!s || !s->transport || s->failed || !token || (n && !p)) return DBT_INVALID;
    unsigned l; for (l=0;l<16;l++) if ((s->remote_pending & (1u<<l)) && s->remote_token[l]==token) break;
    if (l==16) return DBT_EMPTY;
    if (!(s->remote_interim & (1u<<l)) && due(now,s->remote_deadline[l])) {
        s->failed=true; return DBT_INVALID;
    }
    uint8_t opcode=s->remote_opcode[l], pdu=s->remote_pdu[l];
    if (s->target_active && due(now,s->target_deadline)) {
        s->target_active=false; s->target_size=0; s->target_offset=0;
    }
    if (s->target_active && opcode==VENDOR) return DBT_BUSY;
    if (n>MAX_PARAMS) {
        if (opcode!=VENDOR || pdu!=0x20 || code!=0x0c || !s->target_buffer ||
            n>s->target_capacity || !metadata(p,n)) return DBT_NO_SPACE;
        uint8_t start[10+MAX_PARAMS]={0x0c,0x48,0,0,0x19,0x58,0x20,1,
                                      (uint8_t)(MAX_PARAMS>>8),(uint8_t)MAX_PARAMS};
        memmove(s->target_buffer,p,n); memcpy(start+10,s->target_buffer,MAX_PARAMS);
        dbt_result_t first=dbt_avctp_respond(s->transport,(uint8_t)l,false,start,sizeof(start),now);
        if (first==DBT_OK) { s->target_size=n; s->target_offset=MAX_PARAMS;
            s->target_deadline=now+s->target_hold_ms; s->target_active=true;
            s->remote_pending&=(uint16_t)~(1u<<l); }
        return first;
    }
    uint8_t frame[10+MAX_PARAMS]; uint16_t size;
    if (opcode==PASS) {
        if ((code!=9 && code!=0x0a && code!=8) || n!=2 || p[0]!=s->remote_event[l] || p[1]) return DBT_INVALID;
        frame[0]=code; frame[1]=0x48; frame[2]=PASS; frame[3]=p[0]; frame[4]=0; size=5;
    } else {
        if (!reply_params(pdu,code,s->remote_event[l],p,n)) return DBT_INVALID;
        if (pdu==0x10 && (code==0x0c || code==8) && p[0]!=s->remote_event[l]) return DBT_INVALID;
        if (pdu==0x31 && code==8 && p[0]!=s->remote_event[l]) return DBT_INVALID;
        if (code==8 && s->remote_control_size[l] &&
            (n!=s->remote_control_size[l] || !same(p,s->remote_control[l],n))) return DBT_INVALID;
        if (pdu==0x31 && ((code==0x0f && (s->remote_interim & (1u<<l))) ||
            (code==0x0d && !(s->remote_interim & (1u<<l))) || (code==8 && (s->remote_interim & (1u<<l))))) return DBT_INVALID;
        frame[0]=code; frame[1]=0x48; frame[2]=VENDOR; frame[3]=0; frame[4]=0x19; frame[5]=0x58;
        frame[6]=pdu; frame[7]=0; frame[8]=(uint8_t)(n>>8); frame[9]=(uint8_t)n;
        if (n) memcpy(frame+10,p,n);
        size=(uint16_t)(10+n);
    }
    dbt_result_t r=dbt_avctp_respond(s->transport,(uint8_t)l,false,frame,size,now);
    if (r==DBT_OK) {
        if (opcode==VENDOR && pdu==0x31 && code==0x0f) s->remote_interim|=(uint16_t)(1u<<l);
        else { s->remote_pending&=(uint16_t)~(1u<<l); s->remote_interim&=(uint16_t)~(1u<<l); }
    }
    return r;
}
dbt_result_t dbt_avrcp_cancel_notification(dbt_avrcp_t *s,uint8_t event,uint8_t error,uint32_t now) {
    if(!s || !s->transport || !s->transport->binding || s->failed || s->in_callback ||
       event<1 || event>13 || error>0x16 || error==4 || error==6) return DBT_INVALID;
    bool found=false;
    for(unsigned i=0;i<16;i++) if((s->remote_pending & (1u<<i)) && !(s->remote_auto & (1u<<i)) &&
        s->remote_opcode[i]==VENDOR && s->remote_pdu[i]==0x31 && s->remote_event[i]==event) {
        s->remote_auto|=(uint16_t)(1u<<i); s->remote_interim&=(uint16_t)~(1u<<i);
        s->remote_auto_kind[i]=AUTO_REJECT; s->remote_pdu[i]=0x31; s->remote_event[i]=error;
        s->remote_token[i]=0; s->remote_deadline[i]=now+s->remote_timeout; found=true;
    }
    return found ? DBT_OK : DBT_EMPTY;
}
dbt_result_t dbt_avrcp_addressed_player_changed(dbt_avrcp_t *s,uint32_t now) {
    static const uint8_t events[]={1,2,3,4,5,8,9}; bool found=false;
    if(!s || !s->transport || !s->transport->binding || s->failed || s->in_callback) return DBT_INVALID;
    for(unsigned i=0;i<sizeof(events);i++) {
        dbt_result_t r=dbt_avrcp_cancel_notification(s,events[i],0x16,now);
        if(r==DBT_OK) found=true; else if(r!=DBT_EMPTY) return r;
    }
    return found ? DBT_OK : DBT_EMPTY;
}
dbt_result_t dbt_avrcp_abort_continuation(dbt_avrcp_t *s) {
    if(!s || !s->transport || s->failed || s->in_callback || !s->continuation_active) return DBT_INVALID;
    abandon(s,true); return DBT_OK;
}
dbt_result_t dbt_avrcp_poll(dbt_avrcp_t *s,uint32_t now) {
    if (!s || !s->transport || s->failed || s->in_callback) return DBT_INVALID;
    for (unsigned i=0;i<16;i++) if ((s->remote_pending & (1u<<i)) &&
        !(s->remote_interim & (1u<<i)) && due(now,s->remote_deadline[i])) {
        s->failed=true; return DBT_INVALID;
    }
    dbt_result_t result=dbt_avctp_poll(s->transport,now);
    if (result!=DBT_OK) return result;
    if (s->target_active && due(now,s->target_deadline)) {
        s->target_active=false; s->target_size=0; s->target_offset=0;
    }
    if (s->abort_needed) {
        uint8_t label, pdu=0x20;
        result=send(s,VENDOR,0x41,&pdu,1,now,&label);
        if (result==DBT_OK) s->abort_needed=false;
        else if (result!=DBT_BUSY) { s->failed=true; return result; }
    }
    if (s->continuation_needed && !s->abort_needed) {
        uint8_t label, pdu=0x20;
        result=send(s,VENDOR,0x40,&pdu,1,now,&label);
        if (result==DBT_OK) s->continuation_needed=false;
        else if (result!=DBT_BUSY) { s->failed=true; return result; }
    }
    for (unsigned i=0;i<16;i++) if (s->remote_auto & (1u<<i)) {
        uint8_t kind=s->remote_auto_kind[i];
        if (s->target_active && kind!=AUTO_CONT && kind!=AUTO_PID && kind!=AUTO_PASS_NOT_IMPL &&
            !(kind==AUTO_ECHO && s->remote_opcode[i]==PASS)) continue;
        if (kind==AUTO_ECHO) {
            if (!s->echo_active || s->echo_label!=i) { s->failed=true; return DBT_INVALID; }
            result=dbt_avctp_respond(s->transport,(uint8_t)i,false,s->echo_buffer,s->echo_size,now);
            if (result==DBT_BUSY) return DBT_OK;
            if (result!=DBT_OK) return result;
            s->echo_active=false; s->remote_auto&=(uint16_t)~(1u<<i); s->remote_pending&=(uint16_t)~(1u<<i);
            break;
        }
        if (kind==AUTO_PID) {
            result=dbt_avctp_reject_pid(s->transport,(uint8_t)i,s->remote_pid[i],now);
            if (result==DBT_BUSY) return DBT_OK;
            if (result!=DBT_OK) return result;
            s->remote_auto&=(uint16_t)~(1u<<i); s->remote_pending&=(uint16_t)~(1u<<i);
            break;
        }
        if (kind==AUTO_REJECT || kind==AUTO_PASS_NOT_IMPL) {
            uint8_t frame[11]={0x0a,0x48,0,0,0x19,0x58,0,0,0,1,0}; uint16_t size=11;
            if (kind==AUTO_REJECT) { frame[6]=s->remote_pdu[i]; frame[10]=s->remote_event[i]; }
            else { frame[0]=8; frame[2]=PASS; frame[3]=s->remote_event[i]; frame[4]=0; size=5; }
            result=dbt_avctp_respond(s->transport,(uint8_t)i,false,frame,size,now);
            if (result==DBT_BUSY) return DBT_OK;
            if (result!=DBT_OK) return result;
            s->remote_auto&=(uint16_t)~(1u<<i); s->remote_pending&=(uint16_t)~(1u<<i);
            break;
        }
        if (kind==AUTO_CONT) {
            uint8_t frame[10+MAX_PARAMS]={0}; uint8_t pdu=s->remote_pdu[i]; uint16_t size=10, body=0;
            frame[0]=s->target_active ? (pdu==0x40 ? 0x0c : 9) : 0x0a;
            frame[1]=0x48; frame[3]=0; frame[4]=0x19; frame[5]=0x58;
            frame[6]=s->target_active && pdu==0x40 ? 0x20 : pdu;
            if (!s->target_active) { frame[9]=1; frame[10]=1; size=11; }
            else if (pdu==0x40) {
                uint16_t remaining=(uint16_t)(s->target_size-s->target_offset);
                body=remaining>MAX_PARAMS ? MAX_PARAMS : remaining;
                frame[7]=remaining>MAX_PARAMS ? 2 : 3; frame[8]=(uint8_t)(body>>8); frame[9]=(uint8_t)body;
                memcpy(frame+10,s->target_buffer+s->target_offset,body); size=(uint16_t)(10+body);
            }
            result=dbt_avctp_respond(s->transport,(uint8_t)i,false,frame,size,now);
            if (result==DBT_BUSY) return DBT_OK;
            if (result!=DBT_OK) return result;
            if (s->target_active && pdu==0x40) {
                s->target_offset=(uint16_t)(s->target_offset+body); s->target_deadline=now+s->target_hold_ms;
                if (s->target_offset==s->target_size) s->target_active=false;
            } else if (s->target_active && pdu==0x41) s->target_active=false;
            s->remote_auto&=(uint16_t)~(1u<<i); s->remote_pending&=(uint16_t)~(1u<<i);
            break;
        }
        uint8_t frame[8]={0x0c,0xff,s->remote_opcode[i],0x07,0x48,0xff,0xff,0xff};
        if (frame[2]==0x31) {
            frame[3]=s->remote_event[i];
            if ((frame[3]&0x70)!=0) frame[4]=0xff;
        }
        result=dbt_avctp_respond(s->transport,(uint8_t)i,false,frame,sizeof(frame),now);
        if (result==DBT_BUSY) return DBT_OK;
        if (result!=DBT_OK) return result;
        s->remote_auto&=(uint16_t)~(1u<<i); s->remote_pending&=(uint16_t)~(1u<<i);
        break;
    }
    return DBT_OK;
}

uint32_t dbt_avrcp_wait_ms(const dbt_avrcp_t *s,uint32_t now) {
    if(!s || !s->transport || !s->transport->binding || s->failed) return 0;
    const dbt_avctp_t *t=s->transport;
    if(!t->tx_used && !t->pinned) {
        if(s->continuation_needed || s->abort_needed) return 0;
        for(unsigned i=0;i<16;i++) if(s->remote_auto & (1u<<i)) {
            uint8_t kind=s->remote_auto_kind[i];
            if(!s->target_active || kind==AUTO_CONT || kind==AUTO_PID || kind==AUTO_PASS_NOT_IMPL ||
               (kind==AUTO_ECHO && s->remote_opcode[i]==PASS)) return 0;
        }
    }
    uint32_t wait=dbt_avctp_wait_ms(t,now);
    if(s->target_active) { uint32_t d=due(now,s->target_deadline)?0:s->target_deadline-now; if(d<wait) wait=d; }
    for(unsigned i=0;i<16;i++) if((s->remote_pending & (1u<<i)) && !(s->remote_interim & (1u<<i))) {
        uint32_t d=due(now,s->remote_deadline[i])?0:s->remote_deadline[i]-now; if(d<wait) wait=d;
    }
    return wait;
}
