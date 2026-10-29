#include "dbt_avctp.h"
#include <string.h>

enum { PID = 0x110e };
static bool expired(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}
dbt_result_t dbt_avctp_init(dbt_avctp_t *s, dbt_avctp_hooks_t h, uint32_t binding,
    uint16_t rx_mtu, uint16_t tx_mtu, bool browsing, uint8_t *rx, uint16_t rx_cap,
    uint8_t *tx, uint16_t tx_cap, uint8_t *packet, uint32_t tx_ms, uint32_t rsp_ms) {
    if (!s || !h.message || !binding || rx_mtu < 5 || tx_mtu < 5 || !rx || !tx ||
        !packet || !rx_cap || !tx_cap || !tx_ms || !rsp_ms || tx_ms > INT32_MAX ||
        rsp_ms > INT32_MAX || rx == tx || packet == rx || packet == tx) return DBT_INVALID;
    *s = (dbt_avctp_t){0};
    s->hooks=h; s->binding=binding; s->rx_mtu=rx_mtu; s->tx_mtu=tx_mtu;
    s->browsing=browsing; s->rx=rx; s->tx=tx; s->packet=packet;
    s->rx_capacity=rx_cap; s->tx_capacity=tx_cap; s->transmit_ms=tx_ms;
    s->response_ms=rsp_ms;
    return DBT_OK;
}
static dbt_result_t queue(dbt_avctp_t *s, uint8_t label, bool response, bool ipid, uint16_t pid,
                          const uint8_t *data, uint16_t size, uint32_t now) {
    if (!s || !s->binding || s->failed || label > 15 || (size && !data) ||
        (ipid && (!response || size))) return DBT_INVALID;
    if (s->tx_used || s->pinned) return DBT_BUSY;
    if (size > s->tx_capacity) return DBT_NO_SPACE;
    uint32_t count=1;
    if ((uint32_t)size + 3 > s->tx_mtu) {
        if (s->browsing || ipid) return DBT_NO_SPACE;
        uint32_t left = size - (s->tx_mtu - 4u);
        count = 1u + (left + s->tx_mtu - 2u) / (s->tx_mtu - 1u);
        if (count > 255) return DBT_NO_SPACE;
    }
    if (size) memmove(s->tx,data,size);
    s->tx_size=size; s->tx_offset=0; s->tx_count=(uint8_t)count; s->tx_index=0;
    s->tx_label=label; s->tx_pid=pid; s->tx_response=response; s->tx_ipid=ipid; s->tx_used=true;
    s->tx_deadline=now+s->transmit_ms;
    return DBT_OK;
}
dbt_result_t dbt_avctp_command(dbt_avctp_t *s,const uint8_t *data,uint16_t size,
    uint32_t now,uint8_t *label) {
    if (!s || !label) return DBT_INVALID;
    uint8_t l=s->next_label;
    unsigned i;
    for (i=0; i<16 && (s->local_pending & (1u<<l)); ++i) l=(uint8_t)((l+1u)&15u);
    if (i==16) return DBT_BUSY;
    dbt_result_t r=queue(s,l,false,false,PID,data,size,now);
    if (r==DBT_OK) { s->local_pending |= (uint16_t)(1u<<l);
        s->response_timeout[l]=s->response_ms;
        s->next_label=(uint8_t)((l+1u)&15u); *label=l; }
    return r;
}
dbt_result_t dbt_avctp_respond(dbt_avctp_t *s,uint8_t label,bool ipid,
    const uint8_t *data,uint16_t size,uint32_t now) {
    return queue(s,label,true,ipid,PID,data,size,now);
}
dbt_result_t dbt_avctp_reject_pid(dbt_avctp_t *s, uint8_t label, uint16_t pid, uint32_t now) {
    return queue(s,label,true,true,pid,0,0,now);
}
dbt_result_t dbt_avctp_transaction_timeout(dbt_avctp_t *s, uint8_t label, uint32_t ms, uint32_t now) {
    if (!s || !s->binding || s->failed || label>15 || ms>INT32_MAX ||
        !(s->local_pending & (1u<<label))) return DBT_INVALID;
    s->response_timeout[label]=ms;
    if (s->local_sent & (1u<<label)) s->response_deadline[label]=now+ms;
    return DBT_OK;
}
dbt_result_t dbt_avctp_tx_peek(dbt_avctp_t *s,const uint8_t **out,uint16_t *size) {
    if (!s || !out || !size || !s->binding || s->failed) return DBT_INVALID;
    if (!s->tx_used) return DBT_EMPTY;
    if (!s->pinned) {
        uint8_t type=s->tx_count==1 ? 0 : s->tx_index==0 ? 1 :
            s->tx_index+1==s->tx_count ? 3 : 2;
        uint16_t h=(uint16_t)(type==0 ? 3 : type==1 ? 4 : 1);
        s->packet[0]=(uint8_t)((s->tx_label<<4)|(type<<2)|(s->tx_response?2:0)|(s->tx_ipid?1:0));
        if (type==0) { s->packet[1]=(uint8_t)(s->tx_pid>>8); s->packet[2]=(uint8_t)s->tx_pid; }
        if (type==1) { s->packet[1]=s->tx_count; s->packet[2]=(uint8_t)(s->tx_pid>>8); s->packet[3]=(uint8_t)s->tx_pid; }
        uint16_t remaining=(uint16_t)(s->tx_size-s->tx_offset);
        s->packet_body=remaining < s->tx_mtu-h ? remaining : (uint16_t)(s->tx_mtu-h);
        if (s->packet_body) memcpy(s->packet+h,s->tx+s->tx_offset,s->packet_body);
        s->packet_size=(uint16_t)(h+s->packet_body); s->pinned=true;
    }
    *out=s->packet; *size=s->packet_size; return DBT_OK;
}
dbt_result_t dbt_avctp_tx_commit(dbt_avctp_t *s,uint32_t now) {
    if (!s || !s->binding || !s->pinned) return DBT_INVALID;
    if (s->in_callback) return DBT_BUSY;
    s->pinned=false; s->tx_offset=(uint16_t)(s->tx_offset+s->packet_body); s->tx_index++;
    if (s->tx_index==s->tx_count) {
        s->tx_used=false;
        if (!s->tx_response) {
            s->local_sent |= (uint16_t)(1u<<s->tx_label);
            s->response_deadline[s->tx_label]=now+s->response_timeout[s->tx_label];
        }
        if (s->hooks.transmitted) {
            uint8_t label=s->tx_label; bool response=s->tx_response;
            s->in_callback=true; s->hooks.transmitted(s->hooks.context,label,response,now); s->in_callback=false;
        }
    } else s->tx_deadline=now+s->transmit_ms;
    return DBT_OK;
}
dbt_result_t dbt_avctp_tx_cancel(dbt_avctp_t *s) {
    if (!s || !s->pinned) return DBT_INVALID;
    if (s->in_callback) return DBT_BUSY;
    s->pinned=false; return DBT_OK;
}
static dbt_result_t deliver(dbt_avctp_t *s,uint8_t label,bool response,bool ipid,
                            uint16_t pid,const uint8_t *data,uint16_t size) {
    if (response && (!(s->local_pending & (1u<<label)) ||
        !(s->local_sent & (1u<<label)))) return DBT_EMPTY;
    dbt_avctp_message_t m={data,size,pid,label,response,ipid};
    s->in_callback=true; bool keep=s->hooks.message(s->hooks.context,&m); s->in_callback=false;
    if (response && !keep) {
        s->local_pending &= (uint16_t)~(1u<<label);
        s->local_sent &= (uint16_t)~(1u<<label);
    }
    return DBT_OK;
}
dbt_result_t dbt_avctp_receive(dbt_avctp_t *s,uint32_t binding,const uint8_t *p,
                                uint16_t n,uint32_t now) {
    if (!s || !s->binding || s->failed || s->in_callback || !p || !n || n>s->rx_mtu) return DBT_INVALID;
    if (binding!=s->binding) return DBT_EMPTY;
    if (dbt_avctp_poll(s,now)!=DBT_OK) return DBT_INVALID;
    uint8_t type=(uint8_t)((p[0]>>2)&3), label=(uint8_t)(p[0]>>4);
    bool response=(p[0]&2)!=0, ipid=(p[0]&1)!=0;
    if ((ipid && (!response || type!=0)) || (s->browsing && type)) return DBT_INVALID;
    if (type==0) {
        if (n<3 || (ipid && n!=3)) return DBT_INVALID;
        if (s->rx_active) return DBT_BUSY;
        uint16_t pid=(uint16_t)((p[1]<<8)|p[2]);
        return deliver(s,label,response,ipid,pid,p+3,(uint16_t)(n-3));
    }
    if (type==1) {
        if (n<4 || p[1]<2) return DBT_INVALID;
        if (s->rx_active) return DBT_BUSY;
        if (n-4u>s->rx_capacity) return DBT_NO_SPACE;
        s->rx_label=label; s->rx_response=response; s->rx_pid=(uint16_t)((p[2]<<8)|p[3]);
        s->rx_remaining=(uint8_t)(p[1]-1u); s->rx_size=(uint16_t)(n-4u);
        if (s->rx_size) memcpy(s->rx,p+4,s->rx_size);
        s->rx_active=true; s->rx_deadline=now+s->transmit_ms; return DBT_OK;
    }
    if (!s->rx_active || label!=s->rx_label || response!=s->rx_response ||
        (type==3)!=(s->rx_remaining==1) || n==1) return DBT_INVALID;
    if ((uint32_t)s->rx_size+n-1>s->rx_capacity) { s->rx_active=false; return DBT_NO_SPACE; }
    memcpy(s->rx+s->rx_size,p+1,n-1); s->rx_size=(uint16_t)(s->rx_size+n-1);
    s->rx_remaining--; s->rx_deadline=now+s->transmit_ms;
    if (type==3) { s->rx_active=false; return deliver(s,label,response,false,s->rx_pid,s->rx,s->rx_size); }
    return DBT_OK;
}
dbt_result_t dbt_avctp_poll(dbt_avctp_t *s,uint32_t now) {
    if (!s || !s->binding || s->failed || s->in_callback) return DBT_INVALID;
    if ((s->tx_used && expired(now,s->tx_deadline)) ||
        (s->rx_active && expired(now,s->rx_deadline))) { s->failed=true; return DBT_INVALID; }
    for (unsigned i=0;i<16;i++) if ((s->local_sent & (1u<<i)) &&
        s->response_timeout[i] && expired(now,s->response_deadline[i])) {
        s->failed=true; return DBT_INVALID;
    }
    return DBT_OK;
}
uint32_t dbt_avctp_wait_ms(const dbt_avctp_t *s, uint32_t now) {
    if (!s || !s->binding || s->failed) return 0;
    uint32_t best=UINT32_MAX;
    if (s->tx_used) best=expired(now,s->tx_deadline)?0:s->tx_deadline-now;
    for (unsigned i=0;i<16;i++) if ((s->local_sent & (1u<<i)) && s->response_timeout[i]) {
        uint32_t d=expired(now,s->response_deadline[i])?0:s->response_deadline[i]-now;
        if (d<best) best=d;
    }
    if (s->rx_active) {
        uint32_t d=expired(now,s->rx_deadline)?0:s->rx_deadline-now;
        if (d<best) best=d;
    }
    return best;
}
dbt_result_t dbt_avctp_clear(dbt_avctp_t *s) {
    if (!s || !s->binding) return DBT_INVALID;
    if (s->pinned || s->in_callback) return DBT_BUSY;
    *s=(dbt_avctp_t){0}; return DBT_OK;
}
