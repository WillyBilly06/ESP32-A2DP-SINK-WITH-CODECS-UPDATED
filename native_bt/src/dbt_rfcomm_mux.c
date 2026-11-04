#include "dbt_rfcomm_mux.h"
enum { SENT_SABM=1,SENT_ACCEPT,SENT_DISC,SENT_CLOSE,SENT_DUPLICATE };
static bool due(uint32_t now,uint32_t end){return (uint32_t)(now-end)<0x80000000u;}
static dbt_result_t queue(dbt_rfcomm_mux_t *m,dbt_rfcomm_type_t type,bool command,uint8_t effect){
    if(m->tx_ready)return DBT_BUSY;
    dbt_rfcomm_frame_t f={0};f.type=type;f.dlci=0;f.pf=true;f.cr=dbt_rfcomm_cr(m->initiator,command);
    uint16_t n=0;dbt_result_t r=dbt_rfcomm_encode(&f,false,m->tx,sizeof(m->tx),&n);
    if(r!=DBT_OK || n!=4)return DBT_INVALID;
    m->tx_effect=effect;m->tx_ready=true;return DBT_OK;
}
dbt_result_t dbt_rfcomm_mux_init(dbt_rfcomm_mux_t *m,bool initiator,uint16_t handle,uint32_t generation,uint16_t cid,uint32_t t1){
    if(!m || !generation || cid<0x40 || t1<10000 || t1>60000)return DBT_INVALID;
    *m=(dbt_rfcomm_mux_t){0};m->initiator=initiator;m->handle=handle;m->generation=generation;m->local_cid=cid;m->t1_ms=t1;return DBT_OK;
}
dbt_result_t dbt_rfcomm_mux_start(dbt_rfcomm_mux_t *m){
    if(!m || !m->generation || !m->initiator || m->state!=DBT_RFCOMM_MUX_DOWN)return DBT_INVALID;
    if(m->disc_crossed)return DBT_BUSY;
    dbt_result_t r=queue(m,DBT_RFCOMM_SABM,true,SENT_SABM);if(r==DBT_OK)m->state=DBT_RFCOMM_MUX_START_QUEUED;return r;
}
dbt_result_t dbt_rfcomm_mux_close(dbt_rfcomm_mux_t *m){
    if(!m || m->state!=DBT_RFCOMM_MUX_OPEN)return DBT_INVALID;
    dbt_result_t r=queue(m,DBT_RFCOMM_DISC,true,SENT_DISC);if(r==DBT_OK)m->state=DBT_RFCOMM_MUX_DISC_QUEUED;return r;
}
dbt_result_t dbt_rfcomm_mux_receive(dbt_rfcomm_mux_t *m,uint16_t handle,uint32_t generation,uint16_t cid,const uint8_t *data,uint16_t size){
    if(!m || m->handle!=handle || m->generation!=generation || m->local_cid!=cid)return DBT_INVALID;
    dbt_rfcomm_frame_t f;dbt_result_t r=dbt_rfcomm_decode(data,size,false,&f);if(r!=DBT_OK)return r;
    if(f.dlci || f.type==DBT_RFCOMM_UIH)return DBT_UNSUPPORTED;
    bool command=f.type==DBT_RFCOMM_SABM || f.type==DBT_RFCOMM_DISC;
    if(f.cr!=dbt_rfcomm_cr(!m->initiator,command))return DBT_INVALID;
    if(f.type==DBT_RFCOMM_SABM){
        if(!m->initiator && m->state==DBT_RFCOMM_MUX_ACCEPT_QUEUED)return DBT_BUSY;
        if(m->initiator || (m->state!=DBT_RFCOMM_MUX_DOWN && m->state!=DBT_RFCOMM_MUX_OPEN))return DBT_INVALID;
        bool duplicate=m->state==DBT_RFCOMM_MUX_OPEN;r=queue(m,DBT_RFCOMM_UA,false,duplicate?SENT_DUPLICATE:SENT_ACCEPT);
        if(r==DBT_OK && !duplicate)m->state=DBT_RFCOMM_MUX_ACCEPT_QUEUED;
        return r;
    }
    if(f.type==DBT_RFCOMM_DISC){
        if(m->state==DBT_RFCOMM_MUX_DISC_QUEUED)return DBT_BUSY;
        if(m->state!=DBT_RFCOMM_MUX_OPEN && m->state!=DBT_RFCOMM_MUX_WAIT_DISC)return DBT_INVALID;
        bool crossed=m->state==DBT_RFCOMM_MUX_WAIT_DISC;
        r=queue(m,DBT_RFCOMM_UA,false,SENT_CLOSE);
        if(r==DBT_OK){m->disc_crossed=crossed;m->state=DBT_RFCOMM_MUX_REPLY_QUEUED;}return r;
    }
    if(f.type==DBT_RFCOMM_UA || f.type==DBT_RFCOMM_DM){
        if(!f.pf)return DBT_INVALID;
        if(m->disc_crossed && (m->state==DBT_RFCOMM_MUX_REPLY_QUEUED || m->state==DBT_RFCOMM_MUX_DOWN)){
            m->disc_crossed=false;return DBT_OK;
        }
        if(m->state==DBT_RFCOMM_MUX_WAIT_UA){m->state=f.type==DBT_RFCOMM_UA?DBT_RFCOMM_MUX_OPEN:DBT_RFCOMM_MUX_DOWN;return DBT_OK;}
        if(m->state==DBT_RFCOMM_MUX_WAIT_DISC){m->state=DBT_RFCOMM_MUX_DOWN;return DBT_OK;}
    }
    return DBT_INVALID;
}
dbt_result_t dbt_rfcomm_mux_peek(dbt_rfcomm_mux_t *m,const uint8_t **data,uint16_t *size){
    if(!m || !data || !size)return DBT_INVALID;
    if(!m->tx_ready)return DBT_EMPTY;
    m->tx_pinned=true;*data=m->tx;*size=4;return DBT_OK;
}
dbt_result_t dbt_rfcomm_mux_commit(dbt_rfcomm_mux_t *m,uint32_t now){
    if(!m || !m->tx_ready || !m->tx_pinned)return DBT_INVALID;
    uint8_t effect=m->tx_effect;m->tx_pinned=m->tx_ready=false;m->tx_effect=0;
    if(effect==SENT_SABM){m->state=DBT_RFCOMM_MUX_WAIT_UA;m->deadline=now+m->t1_ms;}
    else if(effect==SENT_ACCEPT)m->state=DBT_RFCOMM_MUX_OPEN;
    else if(effect==SENT_DISC){m->state=DBT_RFCOMM_MUX_WAIT_DISC;m->deadline=now+m->t1_ms;}
    else if(effect==SENT_CLOSE)m->state=DBT_RFCOMM_MUX_DOWN;
    return DBT_OK;
}
dbt_result_t dbt_rfcomm_mux_cancel(dbt_rfcomm_mux_t *m){
    if(!m || !m->tx_ready || !m->tx_pinned)return DBT_INVALID;
    m->tx_pinned=m->tx_ready=false;m->tx_effect=0;m->state=DBT_RFCOMM_MUX_DOWN;return DBT_OK;
}
void dbt_rfcomm_mux_poll(dbt_rfcomm_mux_t *m,uint32_t now){
    if(m && (m->state==DBT_RFCOMM_MUX_WAIT_UA || m->state==DBT_RFCOMM_MUX_WAIT_DISC ||
       (m->disc_crossed && m->state==DBT_RFCOMM_MUX_DOWN)) && due(now,m->deadline)){
        m->state=DBT_RFCOMM_MUX_DOWN;m->timed_out=true;
    }
}
dbt_result_t dbt_rfcomm_mux_link_lost(dbt_rfcomm_mux_t *m,uint16_t handle,uint32_t generation){
    if(!m || m->handle!=handle || m->generation!=generation)return DBT_INVALID;
    if(m->tx_pinned)return DBT_BUSY;
    m->tx_ready=false;m->state=DBT_RFCOMM_MUX_DOWN;m->generation=0;m->disc_crossed=false;return DBT_OK;
}
