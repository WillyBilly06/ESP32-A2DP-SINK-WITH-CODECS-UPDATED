#include "dbt_ertm.h"
/* Generated mathematically from x^16+x^15+x^2+1, not a reference code table. */
static const uint16_t crc_nibbles[16]={0x0000u,0xcc01u,0xd801u,0x1400u,0xf001u,0x3c00u,0x2800u,0xe401u,0xa001u,0x6c00u,0x7800u,0xb401u,0x5000u,0x9c01u,0x8801u,0x4400u};
static void zero(void *p,unsigned n){uint8_t *v=p;for(unsigned i=0;i<n;++i)v[i]=0;}
static void copy(void *a,const void *b,unsigned n){uint8_t *x=a;const uint8_t *y=b;for(unsigned i=0;i<n;++i)x[i]=y[i];}
static uint16_t read16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static void write16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
uint16_t dbt_ertm_crc(uint16_t seed,const uint8_t *bytes,uint16_t size){
    for(unsigned i=0;i<size;++i){seed^=bytes[i];seed=(uint16_t)((seed>>4)^crc_nibbles[seed&15u]);seed=(uint16_t)((seed>>4)^crc_nibbles[seed&15u]);}
    return seed;
}
static uint16_t payload_crc(uint16_t cid,const uint8_t *bytes,uint16_t size){
    uint8_t header[4];write16(header,(uint16_t)(size+2u));write16(header+2,cid);
    return dbt_ertm_crc(dbt_ertm_crc(0,header,4),bytes,size);
}
dbt_result_t dbt_ertm_frame_encode(uint16_t cid,bool fcs,const dbt_ertm_frame_t *f,uint8_t *output,uint16_t capacity,uint16_t *written){
    if(!f || !output || !written || cid<0x40 || f->req_seq>63 || f->tx_seq>63 || f->sar>3 || f->supervisor>3 ||
       (f->size && !f->data) || (f->supervisory && (f->size || (f->poll && f->final))) || (!f->supervisory && f->poll))return DBT_INVALID;
    uint32_t total=2u+(f->supervisory?0u:f->size+(f->sar==DBT_ERTM_START?2u:0u))+(fcs?2u:0u);
    if(total>UINT16_MAX || total>capacity)return DBT_NO_SPACE;
    uint16_t control=(uint16_t)(((uint16_t)f->req_seq<<8)|(f->final?0x80u:0u));
    if(f->supervisory)control|=(uint16_t)(1u|((unsigned)f->supervisor<<2)|(f->poll?0x10u:0u));
    else control|=(uint16_t)(((unsigned)f->tx_seq<<1)|((unsigned)f->sar<<14));
    write16(output,control);unsigned at=2;
    if(!f->supervisory){if(f->sar==DBT_ERTM_START){write16(output+at,f->sdu_length);at+=2;}if(f->size){copy(output+at,f->data,f->size);at+=f->size;}}
    if(fcs){write16(output+at,payload_crc(cid,output,(uint16_t)at));at+=2;}
    *written=(uint16_t)at;return DBT_OK;
}
dbt_ertm_wire_result_t dbt_ertm_frame_decode(uint16_t cid,bool fcs,const uint8_t *bytes,uint16_t size,dbt_ertm_frame_t *out){
    if(cid<0x40 || !bytes || !out || size<(fcs?4u:2u))return DBT_ERTM_WIRE_INVALID;
    uint16_t end=(uint16_t)(size-(fcs?2u:0u));
    if(fcs && payload_crc(cid,bytes,end)!=read16(bytes+end))return DBT_ERTM_WIRE_BAD_FCS;
    uint16_t c=read16(bytes);dbt_ertm_frame_t f;zero(&f,sizeof(f));
    f.supervisory=(c&1u)!=0;f.req_seq=(uint8_t)((c>>8)&63u);f.final=(c&0x80u)!=0;
    if(f.supervisory){
        f.supervisor=(uint8_t)((c>>2)&3u);f.poll=(c&0x10u)!=0;
        if(end!=2 || (f.poll && f.final))return DBT_ERTM_WIRE_INVALID;
    }else{
        f.tx_seq=(uint8_t)((c>>1)&63u);f.sar=(uint8_t)(c>>14);unsigned at=2;
        if(f.sar==DBT_ERTM_START){if(end<4)return DBT_ERTM_WIRE_INVALID;f.sdu_length=read16(bytes+at);at+=2;}
        f.data=bytes+at;f.size=(uint16_t)(end-at);
    }
    copy(out,&f,sizeof(f));return DBT_ERTM_WIRE_OK;
}
