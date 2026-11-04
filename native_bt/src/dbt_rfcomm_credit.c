#include "dbt_rfcomm_credit.h"
#define PN_TYPE 0x20u
static bool dlci_valid(uint8_t dlci){return dlci>=2 && dlci<=61;}
dbt_result_t dbt_rfcomm_pn_decode(const dbt_rfcomm_mcc_t *m,dbt_rfcomm_pn_t *out){
    if(!m || !out || m->type!=PN_TYPE || m->size!=8 || !m->data)return DBT_INVALID;
    const uint8_t *p=m->data;unsigned size=(unsigned)p[4]|((unsigned)p[5]<<8);
    if((p[0]&0xc0u) || !dlci_valid(p[0]) || p[7]>7 || size<23 || size>32767)return DBT_INVALID;
    /* Only initial credit-flow PN is supported; legacy/no-credit and
     * post-open PN cannot be accepted as a negotiated credit session. */
    if((p[1]>>4)!=(m->command?15u:14u))return DBT_UNSUPPORTED;
    dbt_rfcomm_pn_t value={(uint16_t)size,p[0],(uint8_t)(p[2]&63u),p[7],m->command};*out=value;return DBT_OK;
}
dbt_result_t dbt_rfcomm_pn_encode(const dbt_rfcomm_pn_t *p,uint8_t out[8],dbt_rfcomm_mcc_t *m){
    if(!p || !out || !m || !dlci_valid(p->dlci) || p->priority>63 || p->credits>7 || p->max_frame<23 || p->max_frame>32767)return DBT_INVALID;
    out[0]=p->dlci;out[1]=p->command?0xf0:0xe0;out[2]=p->priority;out[3]=0;
    out[4]=(uint8_t)p->max_frame;out[5]=(uint8_t)(p->max_frame>>8);out[6]=0;out[7]=p->credits;
    *m=(dbt_rfcomm_mcc_t){out,8,PN_TYPE,p->command};return DBT_OK;
}
dbt_result_t dbt_rfcomm_credit_init(dbt_rfcomm_credit_t *c,uint8_t peer,uint8_t local){
    if(!c || peer>7 || local>7)return DBT_INVALID;
    *c=(dbt_rfcomm_credit_t){peer,local,0,false,false};return DBT_OK;
}
dbt_result_t dbt_rfcomm_credit_prepare(dbt_rfcomm_credit_t *c,bool payload,uint8_t grant){
    if(!c)return DBT_INVALID;
    if(c->pending || (payload && !c->tx_available))return DBT_BUSY;
    if((uint32_t)c->rx_outstanding+grant>65535u)return DBT_NO_SPACE;
    c->pending=true;c->payload=payload;c->grant=grant;return DBT_OK;
}
dbt_result_t dbt_rfcomm_credit_commit(dbt_rfcomm_credit_t *c){
    if(!c || !c->pending)return DBT_INVALID;
    if(c->payload)--c->tx_available;
    c->rx_outstanding=(uint16_t)(c->rx_outstanding+c->grant);
    c->pending=false;c->payload=false;c->grant=0;return DBT_OK;
}
dbt_result_t dbt_rfcomm_credit_cancel(dbt_rfcomm_credit_t *c){
    if(!c || !c->pending)return DBT_INVALID;
    c->pending=false;c->payload=false;c->grant=0;return DBT_OK;
}
dbt_result_t dbt_rfcomm_credit_receive(dbt_rfcomm_credit_t *c,bool payload,uint8_t grant){
    if(!c || (payload && !c->rx_outstanding))return DBT_INVALID;
    if((uint32_t)c->tx_available+grant>65535u)return DBT_NO_SPACE;
    if(payload)--c->rx_outstanding;
    c->tx_available=(uint16_t)(c->tx_available+grant);return DBT_OK;
}
