#include "dbt_avdtp_sink.h"

static bool one(uint8_t value) { return value && !(value&(value-1u)); }
uint8_t dbt_a2dp_sbc_validate(const dbt_avdtp_endpoint_t *endpoint,const dbt_avdtp_cap_t *codec) {
    if(!endpoint || !codec || !codec->data || codec->category!=7 || codec->size!=6)return 0x18;
    dbt_avdtp_caps_t local;dbt_avdtp_cap_error_t error;
    if(dbt_avdtp_caps_validate(endpoint->capabilities,endpoint->size,DBT_AVDTP_CAPABILITIES,&local,&error)!=DBT_OK ||
       local.codec.size!=6 || !local.codec.data || local.codec.data[1]!=0)return 0xc2;
    const uint8_t *p=codec->data,*cap=local.codec.data;
    if(p[0]>>4 || p[1]!=0)return 0xc2;
    if(!one(p[2]&0xf0u))return 0xc3;
    if(!(p[2]&cap[2]&0xf0u))return 0xc4;
    if(!one(p[2]&0x0fu))return 0xc5;
    if(!(p[2]&cap[2]&0x0fu))return 0xc6;
    if(!one(p[3]&0xf0u))return 0xdd;
    if(!(p[3]&cap[3]&0xf0u))return 0xe3;
    if(!one(p[3]&0x0cu))return 0xc7;
    if(!(p[3]&cap[3]&0x0cu))return 0xc8;
    if(!one(p[3]&3u))return 0xc9;
    if(!(p[3]&cap[3]&3u))return 0xca;
    if(p[4]<2 || p[4]>250)return 0xcb;
    if(p[5]<2 || p[5]>250 || p[5]<p[4])return 0xcd;
    if(p[4]<cap[4] || p[4]>cap[5])return 0xcc;
    if(p[5]>cap[5])return 0xce;
    return 0;
}
