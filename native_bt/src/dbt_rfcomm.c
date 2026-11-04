#include "dbt_rfcomm.h"

/* Reflected x^8+x^2+x+1, initialized to all ones and complemented. */
uint8_t dbt_rfcomm_fcs(const uint8_t *data,uint8_t size){
    uint8_t remainder=0xff;
    for(unsigned i=0;i<size;++i){
        remainder^=data[i];
        for(unsigned bit=0;bit<8;++bit)remainder=(uint8_t)((remainder>>1)^((remainder&1u)?0xe0u:0u));
    }
    return (uint8_t)~remainder;
}
bool dbt_rfcomm_cr(bool initiator,bool command){return initiator==command;}
static bool control_valid(dbt_rfcomm_type_t t){
    return t==DBT_RFCOMM_SABM || t==DBT_RFCOMM_UA || t==DBT_RFCOMM_DM || t==DBT_RFCOMM_DISC || t==DBT_RFCOMM_UIH;
}
static bool frame_valid(const dbt_rfcomm_frame_t *f,bool flow){
    if(!f || f->dlci>63 || f->size>32767 || (f->size && !f->data) || !control_valid(f->type))return false;
    if(f->type!=DBT_RFCOMM_UIH && (f->size || f->has_credits))return false;
    if((f->type==DBT_RFCOMM_SABM || f->type==DBT_RFCOMM_UA || f->type==DBT_RFCOMM_DISC) && !f->pf)return false;
    if(f->type==DBT_RFCOMM_UIH && f->has_credits != (flow && f->dlci && f->pf))return false;
    if(f->type==DBT_RFCOMM_UIH && !f->dlci && f->pf)return false;
    return true;
}
dbt_result_t dbt_rfcomm_encode(const dbt_rfcomm_frame_t *f,bool flow,uint8_t *out,uint16_t cap,uint16_t *written){
    if(!out || !written || !frame_valid(f,flow))return DBT_INVALID;
    uint16_t header=f->size<128?3:4;
    uint32_t total=(uint32_t)header+f->size+1u+(f->has_credits?1u:0u);
    if(total>cap)return DBT_NO_SPACE;
    out[0]=(uint8_t)((f->dlci<<2)|(f->cr?2u:0u)|1u);
    out[1]=(uint8_t)(f->type|(f->pf?0x10u:0u));
    out[2]=(uint8_t)(((f->size&127u)<<1)|(header==3));
    if(header==4)out[3]=(uint8_t)(f->size>>7);
    unsigned at=header;if(f->has_credits)out[at++]=f->credits;
    for(unsigned i=0;i<f->size;++i)out[at++]=f->data[i];
    out[at++]=dbt_rfcomm_fcs(out,(uint8_t)(f->type==DBT_RFCOMM_UIH?2:header));
    *written=(uint16_t)at;return DBT_OK;
}
dbt_result_t dbt_rfcomm_decode(const uint8_t *wire,uint16_t size,bool flow,dbt_rfcomm_frame_t *out){
    if(!wire || !out || size<4 || !(wire[0]&1u))return DBT_INVALID;
    dbt_rfcomm_frame_t f={0};f.dlci=wire[0]>>2;f.cr=(wire[0]&2u)!=0;f.pf=(wire[1]&0x10u)!=0;
    f.type=(dbt_rfcomm_type_t)(wire[1]&0xefu);
    if(!control_valid(f.type))return DBT_UNSUPPORTED;
    unsigned header=(wire[2]&1u)?3u:4u;if(size<header+1u)return DBT_INVALID;
    f.size=(uint16_t)(wire[2]>>1);if(header==4)f.size|=(uint16_t)wire[3]<<7;
    f.has_credits=flow && f.dlci && f.type==DBT_RFCOMM_UIH && f.pf;
    unsigned at=header;if(f.has_credits){if(at>=size-1u)return DBT_INVALID;f.credits=wire[at++];}
    if((uint32_t)at+f.size+1u!=size)return DBT_INVALID;
    f.data=f.size?wire+at:0;
    if(!frame_valid(&f,flow) || wire[size-1u]!=dbt_rfcomm_fcs(wire,(uint8_t)(f.type==DBT_RFCOMM_UIH?2:header)))return DBT_INVALID;
    *out=f;return DBT_OK;
}
dbt_result_t dbt_rfcomm_mcc_decode(const uint8_t *data,uint16_t size,dbt_rfcomm_mcc_t *out){
    if(!data || !out || size<2 || !(data[0]&1u))return DBT_INVALID;
    if(!(data[1]&1u))return DBT_UNSUPPORTED;
    uint16_t n=data[1]>>1;if((uint32_t)n+2u!=size)return DBT_INVALID;
    dbt_rfcomm_mcc_t m={n?data+2:0,n,(uint8_t)(data[0]>>2),(data[0]&2u)!=0};*out=m;return DBT_OK;
}
dbt_result_t dbt_rfcomm_mcc_encode(const dbt_rfcomm_mcc_t *m,uint8_t *out,uint16_t cap,uint16_t *written){
    if(!m || !out || !written || m->type>63 || m->size>127 || (m->size && !m->data))return DBT_INVALID;
    if(cap<(uint32_t)m->size+2u)return DBT_NO_SPACE;
    out[0]=(uint8_t)((m->type<<2)|(m->command?2u:0u)|1u);out[1]=(uint8_t)((m->size<<1)|1u);
    for(unsigned i=0;i<m->size;++i)out[i+2]=m->data[i];
    *written=(uint16_t)(m->size+2u);return DBT_OK;
}
