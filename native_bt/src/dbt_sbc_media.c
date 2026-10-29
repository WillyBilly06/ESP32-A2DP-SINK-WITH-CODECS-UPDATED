#include "dbt_sbc_media.h"

static bool one(uint8_t v){return v && !(v&(v-1u));}
static unsigned index(uint8_t v,unsigned high){unsigned n=0;while(v!=(1u<<high)){v>>=1;++n;}return n;}
dbt_result_t dbt_sbc_configure(const uint8_t *p,uint16_t size,dbt_sbc_config_t *config){
    if(!p || !config || size!=6 || p[0]>>4 || p[1] || !one(p[2]&0xf0u) || !one(p[2]&15u) ||
       !one(p[3]&0xf0u) || !one(p[3]&12u) || !one(p[3]&3u) || p[4]<2 || p[5]>250 || p[5]<p[4])return DBT_INVALID;
    unsigned freq=index(p[2]&0xf0u,4),mode=index(p[2]&15u,0),blocks=index(p[3]&0xf0u,4);
    /* CIE bit order is reversed relative to the frame's ascending indexes. */
    freq=3u-freq;mode=3u-mode;blocks=3u-blocks;
    static const uint32_t rates[]={16000,32000,44100,48000};
    unsigned bands=(p[3]&4u)?8u:4u;
    dbt_sbc_config_t value={rates[freq],(uint16_t)((blocks+1u)*4u*bands),
        (uint8_t)((freq<<6)|(blocks<<4)|(mode<<2)|((p[3]&2u)?2u:0u)|(bands==8u)),
        (uint8_t)(mode?2:1),p[4],p[5]};
    *config=value;return DBT_OK;
}
static dbt_result_t dimensions(const dbt_sbc_config_t *c,const uint8_t *p,uint16_t size,uint16_t *bytes){
    if(!c || !p || size<4 || p[0]!=0x9c || p[1]!=c->format || p[2]<c->minimum_bitpool || p[2]>c->maximum_bitpool)return DBT_INVALID;
    unsigned mode=(p[1]>>2)&3u,bands=(p[1]&1u)?8u:4u,blocks=4u*(1u+((p[1]>>4)&3u)),channels=mode?2u:1u;
    static const uint32_t rates[]={16000,32000,44100,48000};
    if(c->rate!=rates[p[1]>>6] || c->samples!=blocks*bands || c->channels!=channels ||
       c->minimum_bitpool<2 || c->maximum_bitpool>250 || c->minimum_bitpool>c->maximum_bitpool)return DBT_INVALID;
    if(p[2]>(mode<2?16u:32u)*bands)return DBT_INVALID;
    unsigned bits=blocks*p[2]*(mode<2?channels:1u)+(mode==3?bands:0u);
    unsigned length=4u+bands*channels/2u+(bits+7u)/8u;
    if(length>DBT_SBC_MAX_FRAME_BYTES || length*8u*c->rate>(channels==1?320000u:512000u)*blocks*bands)return DBT_INVALID;
    *bytes=(uint16_t)length;return DBT_OK;
}
static uint8_t crc_bit(uint8_t crc,unsigned bit){
    unsigned feedback=((crc>>7)^bit)&1u;crc=(uint8_t)(crc<<1);return (uint8_t)(crc^(feedback?0x1du:0));
}
dbt_result_t dbt_sbc_frame_bytes(const dbt_sbc_config_t *config,uint8_t bitpool,uint16_t *size){
    if(!config || !size)return DBT_INVALID;
    uint8_t header[4]={0x9c,config->format,bitpool,0};return dimensions(config,header,4,size);
}
dbt_result_t dbt_sbc_frame_decode(const dbt_sbc_config_t *c,const uint8_t *p,uint16_t size,dbt_sbc_frame_t *frame){
    if(!frame)return DBT_INVALID;
    uint16_t bytes=0;if(dimensions(c,p,size,&bytes)!=DBT_OK || bytes>size)return DBT_INVALID;
    unsigned mode=(p[1]>>2)&3u,bands=(p[1]&1u)?8u:4u,channels=mode?2u:1u;
    uint8_t crc=15;
    for(unsigned i=0;i<16;++i)crc=crc_bit(crc,(p[1+i/8]>>(7u-i%8))&1u);
    unsigned bits=4u*bands*channels+(mode==3?bands:0u);
    for(unsigned i=0;i<bits;++i)crc=crc_bit(crc,(p[4+i/8]>>(7u-i%8))&1u);
    if(crc!=p[3])return DBT_INVALID;
    *frame=(dbt_sbc_frame_t){p,bytes,c->samples,(uint8_t)channels,p[2]};return DBT_OK;
}
void dbt_sbc_media_reset(dbt_sbc_media_t *s){
    if(!s || !s->initialized)return;
    if(s->fragment)s->hooks.release(s->hooks.context,s->fragment);
    s->fragment=0;s->size=s->capacity=s->remaining=0;
}
dbt_result_t dbt_sbc_media_init(dbt_sbc_media_t *s,dbt_sbc_media_hooks_t hooks,dbt_sbc_config_t config,uint32_t fragment_ms){
    if(!s || !hooks.allocate || !hooks.release || !hooks.frame || !fragment_ms || fragment_ms>INT32_MAX ||
       !config.rate || !config.samples || config.channels<1 || config.channels>2)return DBT_INVALID;
    uint8_t *p=(uint8_t *)s;for(unsigned i=0;i<sizeof(*s);++i)p[i]=0;
    s->hooks=hooks;s->config=config;s->fragment_ms=fragment_ms;s->initialized=true;return DBT_OK;
}
void dbt_sbc_media_expire(dbt_sbc_media_t *s,uint32_t now){
    if(s && s->fragment && (int32_t)(now-s->deadline)>=0){++s->rejected;dbt_sbc_media_reset(s);}
}
static dbt_result_t bad(dbt_sbc_media_t *s,dbt_result_t result){
    ++s->rejected;dbt_sbc_media_reset(s);return result;
}
dbt_result_t dbt_sbc_media_receive(dbt_sbc_media_t *s,const dbt_avdtp_media_t *m,uint32_t now){
    if(!s || !s->initialized)return DBT_INVALID;
    if(!m || !m->payload || m->size<2 || m->marker || m->payload_type<96)return bad(s,DBT_INVALID);
    dbt_sbc_media_expire(s,now);++s->packets;
    uint8_t header=m->payload[0],count=header&15u;bool fragmented=(header&128u)!=0,start=(header&64u)!=0,last=(header&32u)!=0;
    const uint8_t *p=m->payload+1;uint16_t size=(uint16_t)(m->size-1);
    if(!fragmented){
        if(s->fragment){++s->gaps;dbt_sbc_media_reset(s);}
        if(start || last || !count)return bad(s,DBT_INVALID);
        uint16_t cursor=0;dbt_sbc_frame_t frame;
        for(unsigned i=0;i<count;++i){
            if(dbt_sbc_frame_decode(&s->config,p+cursor,(uint16_t)(size-cursor),&frame)!=DBT_OK)return bad(s,DBT_INVALID);
            cursor=(uint16_t)(cursor+frame.size);
        }
        if(cursor!=size)return bad(s,DBT_INVALID);
        cursor=0;
        for(unsigned i=0;i<count;++i){
            (void)dbt_sbc_frame_decode(&s->config,p+cursor,(uint16_t)(size-cursor),&frame);
            dbt_result_t result=s->hooks.frame(s->hooks.context,&frame,m->timestamp+i*s->config.samples);
            if(result!=DBT_OK)return bad(s,result);
            ++s->frames;cursor=(uint16_t)(cursor+frame.size);
        }
        return DBT_OK;
    }
    if(!count || (start && last) || (last && count!=1) || (!last && count==1))return bad(s,DBT_INVALID);
    if(start){
        if(s->fragment){++s->gaps;dbt_sbc_media_reset(s);}
        uint16_t capacity=0;
        if(dimensions(&s->config,p,size,&capacity)!=DBT_OK || size>=capacity || (uint32_t)(count-1u)*size+1u>capacity)return bad(s,DBT_INVALID);
        s->fragment=s->hooks.allocate(s->hooks.context,capacity);
        if(!s->fragment)return bad(s,DBT_NO_SPACE);
        s->capacity=capacity;s->size=0;s->chunk_size=size;s->remaining=count;
        s->timestamp=m->timestamp;s->ssrc=m->ssrc;s->deadline=now+s->fragment_ms;s->next_sequence=m->sequence;
    }
    if(!s->fragment || count!=s->remaining || m->sequence!=s->next_sequence || m->timestamp!=s->timestamp || m->ssrc!=s->ssrc ||
       (!last && size!=s->chunk_size) || (last && size>s->chunk_size) || size>s->capacity-s->size){
        ++s->gaps;return bad(s,DBT_INVALID);
    }
    for(unsigned i=0;i<size;++i)s->fragment[s->size+i]=p[i];
    s->size=(uint16_t)(s->size+size);--s->remaining;++s->next_sequence;
    if(last){
        dbt_sbc_frame_t frame;
        if(s->size!=s->capacity || dbt_sbc_frame_decode(&s->config,s->fragment,s->size,&frame)!=DBT_OK)return bad(s,DBT_INVALID);
        dbt_result_t result=s->hooks.frame(s->hooks.context,&frame,s->timestamp);
        if(result==DBT_OK)++s->frames;else ++s->rejected;
        dbt_sbc_media_reset(s);return result;
    }
    return DBT_OK;
}
