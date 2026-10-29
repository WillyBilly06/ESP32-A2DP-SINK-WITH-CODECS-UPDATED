#include "dbt_ble_control.h"
#include <string.h>
static const int8_t presets[12][3]={{0,0,0},{6,2,0},{0,2,6},{-2,4,2},{4,0,4},{2,3,4},
                                     {3,0,2},{0,2,3},{5,2,4},{6,0,2},{2,3,3},{4,2,4}};
static void error(dbt_ble_control_result_t *r,uint8_t cmd,uint8_t code) {
    r->data[0]=0x11; r->data[1]=cmd; r->data[2]=code; r->size=3;
}
static void ack(dbt_ble_control_result_t *r,uint8_t cmd) { r->data[0]=0x10;r->data[1]=cmd;r->size=2; }
dbt_result_t dbt_ble_control_status(const dbt_ble_control_state_t *s,uint8_t *out,uint8_t *size) {
    if(!s || !out || !size || !s->valid || s->name_size>32 || s->firmware_size>32) return DBT_INVALID;
    unsigned n=0; out[n++]=0xf0;
    for(unsigned i=0;i<3;i++)out[n++]=(uint8_t)s->eq[i];
    out[n++]=s->control; memcpy(out+n,s->led,10);n+=10;out[n++]=s->sound;
    out[n++]=s->name_size;memcpy(out+n,s->name,s->name_size);n+=s->name_size;
    out[n++]=s->firmware_size;memcpy(out+n,s->firmware,s->firmware_size);n+=s->firmware_size;
    *size=(uint8_t)n;return DBT_OK;
}
dbt_result_t dbt_ble_control_execute(dbt_ble_control_state_t *s,const dbt_ble_control_hooks_t *h,
                                     const dbt_ble_control_peer_t *peer,const uint8_t *data,uint16_t n,dbt_ble_control_result_t *r) {
    if(!s || !h || !peer || !peer->generation || !data || !n || n>DBT_BLE_CONTROL_MAX_COMMAND || !r) return DBT_INVALID;
    memset(r,0,sizeof(*r)); uint8_t cmd=data[0];const uint8_t *p=data+1;unsigned len=n-1;
    if(cmd==0xff) { r->data[0]=0xff;r->size=1;return DBT_OK; }
    if(cmd==0xf0) {
        if(dbt_ble_control_status(s,r->data,&r->size)!=DBT_OK) error(r,cmd,3);
        return DBT_OK;
    }
    if(cmd==1 || cmd==2) {
        int8_t values[3];
        if((cmd==1 && len<3) || (cmd==2 && (!len || p[0]>=12))) { error(r,cmd,2);return DBT_OK; }
        if(cmd==1) for(unsigned i=0;i<3;i++) values[i]=(int8_t)(p[i]<128 ? (int)p[i] : (int)p[i]-256);
        else memcpy(values,presets[p[0]],3);
        if(!s->valid || !h->commit_eq || !h->publish_eq) { error(r,cmd,3);return DBT_OK; }
        if(!h->commit_eq(h->context,values[0],values[1],values[2])) { error(r,cmd,3);return DBT_OK; }
        r->durable_eq=true;memcpy(s->eq,values,3);
        if(!h->publish_eq(h->context,values[0],values[1],values[2])) { error(r,cmd,3);return DBT_OK; }
        ack(r,cmd);return DBT_OK;
    }
    if(cmd==3) {
        if(!len) { error(r,cmd,2);return DBT_OK; }
        uint8_t value=p[0]&7u;
        if(!s->valid || !h->commit_control || !h->publish_control ||
           !h->commit_control(h->context,value)) { error(r,cmd,3);return DBT_OK; }
        r->durable_control=true;s->control=value;
        if(!h->publish_control(h->context,value)) { error(r,cmd,3);return DBT_OK; }
        ack(r,cmd);return DBT_OK;
    }
    unsigned minimum;
    switch(cmd) {
      case 4:minimum=1;break; case 5:minimum=10;break;
      case 6:case 7:case 0x10:case 0x11:minimum=1;break;
      case 0x12:minimum=4;break;case 0x13:minimum=2;break;case 0x14:minimum=0;break;
      case 0x20:minimum=4;break;case 0x21:minimum=2;break;case 0x22:case 0x23:minimum=0;break;
      default:error(r,cmd,1);return DBT_OK;
    }
    if(len<minimum || (cmd==4 && len>32)) { error(r,cmd,2);return DBT_OK; }
    if(!s->valid || !h->command) { error(r,cmd,3);return DBT_OK; }
    uint8_t size=DBT_BLE_CONTROL_MAX_STATUS;
    if(!h->command(h->context,peer,cmd,p,(uint16_t)len,r->data,&size) || !size || size>DBT_BLE_CONTROL_MAX_STATUS) {
        error(r,cmd,3);return DBT_OK;
    }
    r->size=size;
    if(r->data[0]==0x10 && size==2 && r->data[1]==cmd) {
        if(cmd==4) {s->name_size=(uint8_t)len;memcpy(s->name,p,len);}
        else if(cmd==5) memcpy(s->led,p,10);
        else if(cmd==6) s->led[0]=p[0];
        else if(cmd==7) s->led[1]=p[0];
    }
    if(r->data[0]==0x06 && size==2) s->sound=r->data[1];
    return DBT_OK;
}
