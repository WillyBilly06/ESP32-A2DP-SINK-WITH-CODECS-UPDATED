#include "dbt_rfcomm_dlc.h"
enum { OUT_PN=1,OUT_PN_REPLY,OUT_SABM,OUT_UA,OUT_MSC,OUT_MSC_REPLY,OUT_DATA,OUT_GRANT,OUT_DISC,OUT_DISC_REPLY,OUT_OTHER };
enum { MCC_PN=0x20,MCC_MSC=0x38,MCC_RPN=0x24,MCC_RLS=0x14,MCC_TEST=0x08,MCC_FCON=0x28,MCC_FCOFF=0x18,MCC_NSC=0x04 };
static bool due(uint32_t now,uint32_t end){return (uint32_t)(now-end)<0x80000000u;}
static void signal_event(dbt_rfcomm_dlc_t *d,dbt_rfcomm_dlc_event_t event){
    if(d->hooks.event){d->in_callback=true;d->hooks.event(d->hooks.context,event,d->dlci);d->in_callback=false;}
}
static void fail(dbt_rfcomm_dlc_t *d){if(d->state!=DBT_RFCOMM_DLC_FAILED){d->state=DBT_RFCOMM_DLC_FAILED;signal_event(d,DBT_RFCOMM_DLC_EVENT_FAILED);}}
static dbt_result_t frame(dbt_rfcomm_dlc_t *d,uint8_t dlci,dbt_rfcomm_type_t type,bool command,
                          const uint8_t *data,uint16_t size,bool with_credit,uint8_t credits,uint8_t effect){
    if(d->tx_ready)return DBT_BUSY;
    dbt_rfcomm_frame_t f={data,size,dlci,credits,type,dbt_rfcomm_cr(d->mux_initiator,command),
                          type==DBT_RFCOMM_UIH?with_credit:true,with_credit};uint16_t written=0;
    dbt_result_t r=dbt_rfcomm_encode(&f,d->credit_mode,d->tx,d->capacity,&written);
    if(r==DBT_OK){d->tx_size=written;d->tx_ready=true;d->tx_effect=effect;}return r;
}
static dbt_result_t mcc(dbt_rfcomm_dlc_t *d,uint8_t type,bool command,const uint8_t *data,uint8_t size,uint8_t effect){
    uint8_t bytes[130];uint16_t written=0;dbt_rfcomm_mcc_t m={data,size,type,command};
    dbt_result_t r=dbt_rfcomm_mcc_encode(&m,bytes,sizeof(bytes),&written);if(r!=DBT_OK)return r;
    return frame(d,0,DBT_RFCOMM_UIH,true,bytes,written,false,0,effect);
}
static dbt_result_t send_msc(dbt_rfcomm_dlc_t *d){
    uint8_t data[2]={(uint8_t)((d->dlci<<2)|3u),0x8d};
    return mcc(d,MCC_MSC,true,data,2,OUT_MSC);
}
static void maybe_open(dbt_rfcomm_dlc_t *d){
    if(d->state==DBT_RFCOMM_DLC_MSC && d->msc_sent && d->msc_received && d->msc_ack && d->msc_reply){
        d->state=DBT_RFCOMM_DLC_OPEN;signal_event(d,DBT_RFCOMM_DLC_EVENT_OPEN);
    }
}
static void reset_activation(dbt_rfcomm_dlc_t *d){
    d->dlci=0;d->msc_sent=d->msc_received=d->msc_ack=d->msc_reply=false;
    d->remote_flow_off=d->aggregate_flow_off=d->credit_mode=false;
    d->credits=(dbt_rfcomm_credit_t){0};d->deadline=0;
    if(d->server)d->max_frame=d->listener_max;
}
dbt_result_t dbt_rfcomm_dlc_init(dbt_rfcomm_dlc_t *d,bool initiator,uint8_t *tx,uint16_t cap,dbt_rfcomm_dlc_hooks_t hooks){
    if(!d || !tx || cap<28)return DBT_INVALID;
    *d=(dbt_rfcomm_dlc_t){0};d->mux_initiator=initiator;d->tx=tx;d->capacity=cap;
    d->hooks=hooks;d->t1_ms=60000;d->t2_ms=20000;return DBT_OK;
}
static bool policy(const dbt_rfcomm_dlc_t *d,uint8_t scn,uint16_t n1,uint8_t k){
    return d && !d->in_callback && d->state==DBT_RFCOMM_DLC_IDLE && !d->tx_ready &&
        scn>=1 && scn<=30 && n1>=23 && (uint32_t)n1+5u<=d->capacity && k<=7;
}
dbt_result_t dbt_rfcomm_dlc_listen(dbt_rfcomm_dlc_t *d,uint8_t scn,uint16_t n1,uint8_t k){
    if(!policy(d,scn,n1,k))return DBT_INVALID;
    reset_activation(d);d->server=true;d->server_channel=scn;d->max_frame=d->listener_max=n1;d->initial_credits=k;return DBT_OK;
}
dbt_result_t dbt_rfcomm_dlc_open(dbt_rfcomm_dlc_t *d,uint8_t scn,uint16_t n1,uint8_t k){
    if(!policy(d,scn,n1,k))return DBT_INVALID;
    reset_activation(d);
    uint8_t dlci=(uint8_t)((scn<<1)|(d->mux_initiator?0u:1u));
    dbt_rfcomm_pn_t p={n1,dlci,0,k,true};uint8_t data[8];dbt_rfcomm_mcc_t msg;
    if(dbt_rfcomm_pn_encode(&p,data,&msg)!=DBT_OK)return DBT_INVALID;
    dbt_result_t r=mcc(d,MCC_PN,true,data,8,OUT_PN);
    if(r==DBT_OK){d->server=false;d->dlci=dlci;d->server_channel=scn;d->proposed=n1;d->initial_credits=k;d->state=DBT_RFCOMM_DLC_PN_QUEUED;}return r;
}
static dbt_result_t pn_command(dbt_rfcomm_dlc_t *d,const dbt_rfcomm_mcc_t *m){
    if(d->server && d->state==DBT_RFCOMM_DLC_PN_REPLY && m->size==8 && m->data){
        bool same=true;for(unsigned i=0;i<8;++i)if(m->data[i]!=d->last_pn[i])same=false;
        if(same)return DBT_BUSY;
    }
    if(d->server && d->state==DBT_RFCOMM_DLC_WAIT_SABM && m->size==8 && m->data){
        bool same=true;for(unsigned i=0;i<8;++i)if(m->data[i]!=d->last_pn[i])same=false;
        if(same)return mcc(d,MCC_PN,false,d->last_pn_reply,8,OUT_OTHER);
    }
    if(d->state==DBT_RFCOMM_DLC_OPEN && m->size==8 && m->data && m->data[0]==d->dlci && (m->data[1]>>4)==0){
        uint8_t reply[8]={0};reply[0]=d->dlci;reply[2]=(uint8_t)(m->data[2]&63u);
        reply[4]=(uint8_t)d->max_frame;reply[5]=(uint8_t)(d->max_frame>>8);
        return mcc(d,MCC_PN,false,reply,8,OUT_OTHER);
    }
    if(!d->server || d->state!=DBT_RFCOMM_DLC_IDLE || m->size!=8 || !m->data)return DBT_INVALID;
    const uint8_t *p=m->data;uint8_t expected=(uint8_t)((d->server_channel<<1)|(d->mux_initiator?1u:0u));
    if(p[0]!=expected || p[7]>7)return DBT_INVALID;
    unsigned offered=(unsigned)p[4]|((unsigned)p[5]<<8);if(offered<23 || offered>32767)return DBT_INVALID;
    uint8_t cl=p[1]>>4;if(cl!=15 && cl!=0)return DBT_UNSUPPORTED;
    if(cl==0 && p[7])return DBT_INVALID;
    uint16_t chosen=(uint16_t)(offered<d->max_frame?offered:d->max_frame);uint8_t reply[8];
    reply[0]=expected;reply[1]=cl==15?0xe0:0;reply[2]=(uint8_t)(p[2]&63u);reply[3]=0;
    reply[4]=(uint8_t)chosen;reply[5]=(uint8_t)(chosen>>8);reply[6]=0;reply[7]=cl==15?d->initial_credits:0;
    dbt_result_t r=mcc(d,MCC_PN,false,reply,8,OUT_PN_REPLY);if(r!=DBT_OK)return r;
    for(unsigned i=0;i<8;++i){d->last_pn[i]=p[i];d->last_pn_reply[i]=reply[i];}
    d->dlci=expected;d->credit_mode=cl==15;d->max_frame=chosen;
    if(d->credit_mode)(void)dbt_rfcomm_credit_init(&d->credits,p[7],d->initial_credits);
    d->state=DBT_RFCOMM_DLC_PN_REPLY;return DBT_OK;
}
static dbt_result_t pn_response(dbt_rfcomm_dlc_t *d,const dbt_rfcomm_mcc_t *m){
    if(d->state!=DBT_RFCOMM_DLC_WAIT_PN || m->size!=8 || !m->data)return DBT_INVALID;
    const uint8_t *p=m->data;unsigned n1=(unsigned)p[4]|((unsigned)p[5]<<8);uint8_t cl=p[1]>>4;
    if(p[0]!=d->dlci || n1<23 || n1>d->proposed || p[7]>7 || (cl!=14 && cl!=0) || (cl==0 && p[7]))return DBT_INVALID;
    dbt_result_t r=frame(d,d->dlci,DBT_RFCOMM_SABM,true,0,0,false,0,OUT_SABM);if(r!=DBT_OK)return r;
    d->credit_mode=cl==14;d->max_frame=(uint16_t)n1;
    if(d->credit_mode)(void)dbt_rfcomm_credit_init(&d->credits,p[7],d->initial_credits);
    d->state=DBT_RFCOMM_DLC_SABM_QUEUED;return DBT_OK;
}
static dbt_result_t command(dbt_rfcomm_dlc_t *d,const dbt_rfcomm_mcc_t *m){
    if(m->type==MCC_PN)return m->command?pn_command(d,m):pn_response(d,m);
    if(m->type==MCC_MSC){
        if((d->state!=DBT_RFCOMM_DLC_MSC && d->state!=DBT_RFCOMM_DLC_OPEN) || (m->size!=2 && m->size!=3) ||
           m->data[0]!=(uint8_t)((d->dlci<<2)|3u) ||
           (m->size==2 && !(m->data[1]&1u)) ||
           (m->size==3 && ((m->data[1]&1u) || !(m->data[2]&1u))))return DBT_INVALID;
        if(m->command){
            dbt_result_t r=mcc(d,MCC_MSC,false,m->data,(uint8_t)m->size,OUT_MSC_REPLY);if(r!=DBT_OK)return r;
            d->msc_received=true;d->remote_flow_off=!d->credit_mode && (m->data[1]&2u);return DBT_OK;
        }
        if(!d->msc_sent || m->data[1]!=0x8d || m->size!=2)return DBT_INVALID;
        d->msc_ack=true;maybe_open(d);return DBT_OK;
    }
    if(m->type==MCC_RPN && m->command){
        if((m->size!=1 && m->size!=8) || !m->data)return DBT_INVALID;
        uint8_t expected=d->dlci?d->dlci:(uint8_t)((d->server_channel<<1)|(d->mux_initiator?1u:0u));
        if(!expected || m->data[0]!=(uint8_t)((expected<<2)|3u))return DBT_INVALID;
        /* Report default 9600/8N1/no flow control. Parameter changes are not
         * implemented; return zero mask instead of claiming they took effect. */
        uint8_t reply[8]={m->data[0],3,3,0,0x11,0x13,0,0};
        return mcc(d,MCC_RPN,false,reply,8,OUT_OTHER);
    }
    if(m->type==MCC_RLS && m->command){
        if(m->size!=2 || !m->data || !d->dlci || m->data[0]!=(uint8_t)((d->dlci<<2)|3u))return DBT_INVALID;
        dbt_result_t r=mcc(d,MCC_RLS,false,m->data,2,OUT_OTHER);if(r==DBT_OK)d->line_status=(uint8_t)(m->data[1]&15u);return r;
    }
    if((m->type==MCC_FCON || m->type==MCC_FCOFF) && m->command && !d->credit_mode && m->size==0){
        dbt_result_t r=mcc(d,m->type,false,0,0,OUT_OTHER);if(r==DBT_OK)d->aggregate_flow_off=m->type==MCC_FCOFF;return r;
    }
    if(m->type==MCC_TEST && m->command)return mcc(d,m->type,false,m->data,(uint8_t)m->size,OUT_OTHER);
    if(m->command){uint8_t unknown=(uint8_t)((m->type<<2)|3u);return mcc(d,MCC_NSC,false,&unknown,1,OUT_OTHER);}
    return DBT_UNSUPPORTED;
}
dbt_result_t dbt_rfcomm_dlc_receive(dbt_rfcomm_dlc_t *d,const uint8_t *bytes,uint16_t size){
    if(!d || d->in_callback || !bytes)return DBT_INVALID;
    dbt_rfcomm_frame_t f;dbt_result_t r=dbt_rfcomm_decode(bytes,size,d->credit_mode,&f);if(r!=DBT_OK)return r;
    bool is_command=f.type==DBT_RFCOMM_SABM || f.type==DBT_RFCOMM_DISC || f.type==DBT_RFCOMM_UIH;
    if(f.cr!=dbt_rfcomm_cr(!d->mux_initiator,is_command))return DBT_INVALID;
    if(!f.dlci && f.type==DBT_RFCOMM_UIH){dbt_rfcomm_mcc_t m;r=dbt_rfcomm_mcc_decode(f.data,f.size,&m);return r==DBT_OK?command(d,&m):r;}
    if(f.dlci!=d->dlci || !d->dlci)return DBT_INVALID;
    if(f.type==DBT_RFCOMM_SABM && d->state==DBT_RFCOMM_DLC_WAIT_SABM){
        r=frame(d,d->dlci,DBT_RFCOMM_UA,false,0,0,false,0,OUT_UA);if(r==DBT_OK)d->state=DBT_RFCOMM_DLC_UA_QUEUED;return r;
    }
    if(f.type==DBT_RFCOMM_UA && d->state==DBT_RFCOMM_DLC_WAIT_UA){d->state=DBT_RFCOMM_DLC_MSC;return DBT_OK;}
    if(f.type==DBT_RFCOMM_DISC && (d->state==DBT_RFCOMM_DLC_OPEN || d->state==DBT_RFCOMM_DLC_MSC)){
        r=frame(d,d->dlci,DBT_RFCOMM_UA,false,0,0,false,0,OUT_DISC_REPLY);if(r==DBT_OK)d->state=DBT_RFCOMM_DLC_CLOSE_REPLY;return r;
    }
    if((f.type==DBT_RFCOMM_UA || f.type==DBT_RFCOMM_DM) && d->state==DBT_RFCOMM_DLC_WAIT_DISC){
        d->state=DBT_RFCOMM_DLC_IDLE;signal_event(d,DBT_RFCOMM_DLC_EVENT_CLOSED);reset_activation(d);return DBT_OK;
    }
    if(f.type==DBT_RFCOMM_DM){fail(d);return DBT_OK;}
    if(f.type==DBT_RFCOMM_UIH && d->state==DBT_RFCOMM_DLC_OPEN){
        if((uint32_t)f.size+(f.has_credits?1u:0u)>d->max_frame)return DBT_INVALID;
        if(d->credit_mode){r=dbt_rfcomm_credit_receive(&d->credits,f.size!=0,f.has_credits?f.credits:0);if(r!=DBT_OK)return r;}
        if(f.size && d->hooks.data){d->in_callback=true;d->hooks.data(d->hooks.context,d->dlci,f.data,f.size);d->in_callback=false;}return DBT_OK;
    }
    return DBT_INVALID;
}
dbt_result_t dbt_rfcomm_dlc_send(dbt_rfcomm_dlc_t *d,const uint8_t *data,uint16_t size){
    if(!d || d->in_callback || d->state!=DBT_RFCOMM_DLC_OPEN || !size || !data)return DBT_INVALID;
    if(size>d->max_frame)return DBT_NO_SPACE;
    if(d->tx_ready || d->remote_flow_off || d->aggregate_flow_off)return DBT_BUSY;
    if(d->credit_mode){dbt_result_t r=dbt_rfcomm_credit_prepare(&d->credits,true,0);if(r!=DBT_OK)return r;}
    dbt_result_t r=frame(d,d->dlci,DBT_RFCOMM_UIH,true,data,size,false,0,OUT_DATA);
    if(r!=DBT_OK && d->credit_mode)(void)dbt_rfcomm_credit_cancel(&d->credits);
    return r;
}
dbt_result_t dbt_rfcomm_dlc_grant(dbt_rfcomm_dlc_t *d,uint8_t n){
    if(!d || d->in_callback || d->state!=DBT_RFCOMM_DLC_OPEN || !d->credit_mode || !n)return DBT_INVALID;
    if(d->tx_ready)return DBT_BUSY;
    dbt_result_t r=dbt_rfcomm_credit_prepare(&d->credits,false,n);if(r!=DBT_OK)return r;
    r=frame(d,d->dlci,DBT_RFCOMM_UIH,true,0,0,true,n,OUT_GRANT);
    if(r!=DBT_OK)(void)dbt_rfcomm_credit_cancel(&d->credits);
    return r;
}
dbt_result_t dbt_rfcomm_dlc_close(dbt_rfcomm_dlc_t *d){
    if(!d || d->in_callback || d->state!=DBT_RFCOMM_DLC_OPEN)return DBT_INVALID;
    dbt_result_t r=frame(d,d->dlci,DBT_RFCOMM_DISC,true,0,0,false,0,OUT_DISC);
    if(r==DBT_OK)d->state=DBT_RFCOMM_DLC_DISC_QUEUED;
    return r;
}
dbt_result_t dbt_rfcomm_dlc_peek(dbt_rfcomm_dlc_t *d,const uint8_t **bytes,uint16_t *size){
    if(!d || !bytes || !size || d->in_callback)return DBT_INVALID;
    if(!d->tx_ready)return DBT_EMPTY;
    d->tx_pinned=true;*bytes=d->tx;*size=d->tx_size;return DBT_OK;
}
dbt_result_t dbt_rfcomm_dlc_commit(dbt_rfcomm_dlc_t *d,uint32_t now){
    if(!d || d->in_callback || !d->tx_ready || !d->tx_pinned)return DBT_INVALID;
    uint8_t effect=d->tx_effect;d->tx_ready=d->tx_pinned=false;d->tx_effect=0;
    if(effect==OUT_PN){d->state=DBT_RFCOMM_DLC_WAIT_PN;d->deadline=now+d->t2_ms;}
    else if(effect==OUT_PN_REPLY){d->state=DBT_RFCOMM_DLC_WAIT_SABM;d->deadline=now+d->t1_ms;}
    else if(effect==OUT_SABM){d->state=DBT_RFCOMM_DLC_WAIT_UA;d->deadline=now+d->t1_ms;}
    else if(effect==OUT_UA){d->state=DBT_RFCOMM_DLC_MSC;d->deadline=now+d->t2_ms;}
    else if(effect==OUT_MSC){d->msc_sent=true;d->deadline=now+d->t2_ms;}
    else if(effect==OUT_MSC_REPLY){d->msc_reply=true;maybe_open(d);}
    else if(effect==OUT_DATA || effect==OUT_GRANT){if(d->credit_mode)(void)dbt_rfcomm_credit_commit(&d->credits);}
    else if(effect==OUT_DISC){d->state=DBT_RFCOMM_DLC_WAIT_DISC;d->deadline=now+d->t1_ms;}
    else if(effect==OUT_DISC_REPLY){d->state=DBT_RFCOMM_DLC_IDLE;signal_event(d,DBT_RFCOMM_DLC_EVENT_CLOSED);reset_activation(d);}
    return DBT_OK;
}
dbt_result_t dbt_rfcomm_dlc_cancel(dbt_rfcomm_dlc_t *d){
    if(!d || d->in_callback || !d->tx_ready || !d->tx_pinned)return DBT_INVALID;
    if(d->credit_mode && (d->tx_effect==OUT_DATA || d->tx_effect==OUT_GRANT))(void)dbt_rfcomm_credit_cancel(&d->credits);
    d->tx_ready=d->tx_pinned=false;d->tx_effect=0;fail(d);return DBT_OK;
}
bool dbt_rfcomm_dlc_queued_data(const dbt_rfcomm_dlc_t *d){return d && d->tx_ready && d->tx_effect==OUT_DATA;}
dbt_result_t dbt_rfcomm_dlc_abort_queued_data(dbt_rfcomm_dlc_t *d){
    if(!d || d->in_callback || !dbt_rfcomm_dlc_queued_data(d))return DBT_INVALID;
    if(d->credit_mode)(void)dbt_rfcomm_credit_cancel(&d->credits);
    d->tx_ready=d->tx_pinned=false;d->tx_effect=0;return DBT_OK;
}
void dbt_rfcomm_dlc_poll(dbt_rfcomm_dlc_t *d,uint32_t now){
    if(!d || d->in_callback)return;
    if((d->state==DBT_RFCOMM_DLC_WAIT_PN || d->state==DBT_RFCOMM_DLC_WAIT_SABM || d->state==DBT_RFCOMM_DLC_WAIT_UA ||
       d->state==DBT_RFCOMM_DLC_WAIT_DISC || (d->state==DBT_RFCOMM_DLC_MSC && d->msc_sent)) && due(now,d->deadline)){fail(d);return;}
    if(d->state==DBT_RFCOMM_DLC_MSC && !d->msc_sent && !d->tx_ready)(void)send_msc(d);
}
dbt_result_t dbt_rfcomm_dlc_shutdown(dbt_rfcomm_dlc_t *d,bool link_lost){
    if(!d || d->in_callback)return DBT_INVALID;
    if(d->tx_pinned)return DBT_BUSY;
    bool active=d->state!=DBT_RFCOMM_DLC_IDLE && d->state!=DBT_RFCOMM_DLC_FAILED;
    d->tx_ready=false;d->tx_effect=0;
    if(active)signal_event(d,link_lost?DBT_RFCOMM_DLC_EVENT_FAILED:DBT_RFCOMM_DLC_EVENT_CLOSED);
    d->state=DBT_RFCOMM_DLC_IDLE;reset_activation(d);return DBT_OK;
}
