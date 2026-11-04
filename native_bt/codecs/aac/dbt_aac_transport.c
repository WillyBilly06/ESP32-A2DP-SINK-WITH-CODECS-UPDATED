#include "dbt_aac.h"

static bool one(unsigned v){return v && !(v&(v-1u));}
dbt_result_t dbt_aac_configure(const uint8_t *p,uint16_t size,dbt_codec_config_t *config){
    if(!p || !config || size!=8 || p[0] || p[1]!=2)return DBT_UNSUPPORTED;
    if(!one(p[2]) || !(p[2]&0xc0u) || (p[4]&3u))return DBT_INVALID;
    unsigned frequency=((unsigned)p[3]<<4)|(p[4]>>4),channels=p[4]&12u;
    if(!one(frequency) || !one(channels))return DBT_INVALID;
    if(frequency!=0x10u && frequency!=8u)return DBT_UNSUPPORTED;
    dbt_codec_config_t next;
    uint8_t *bytes=(uint8_t *)&next;for(unsigned i=0;i<sizeof(next);++i)bytes[i]=0;
    next.rate=frequency==8u?48000:44100;next.samples=1024;next.sample_bytes=2;
    next.channels=channels==8u?1:2;next.cie_size=8;
    for(unsigned i=0;i<8;++i)next.cie[i]=p[i];
    *config=next;return DBT_OK;
}
uint8_t dbt_a2dp_aac_validate(const dbt_avdtp_endpoint_t *endpoint,const dbt_avdtp_cap_t *codec){
    if(!endpoint || !codec || codec->category!=7 || !codec->data || codec->size!=8)return 0x18;
    dbt_codec_config_t checked;
    if(dbt_aac_configure(codec->data,codec->size,&checked)!=DBT_OK)return 0x29;
    dbt_avdtp_caps_t local;dbt_avdtp_cap_error_t error;
    if(dbt_avdtp_caps_validate(endpoint->capabilities,endpoint->size,DBT_AVDTP_CAPABILITIES,&local,&error)!=DBT_OK || local.codec.size!=8)return 0x29;
    const uint8_t *p=codec->data,*cap=local.codec.data;
    if(cap[0] || cap[1]!=2 || !(p[2]&cap[2]) || (p[3]&cap[3])!=p[3] ||
       (p[4]&cap[4]&0xfcu)!=(p[4]&0xfcu) || ((p[5]&0x80u) && !(cap[5]&0x80u)))return 0x29;
    uint32_t bitrate=((uint32_t)(p[5]&127u)<<16)|((uint32_t)p[6]<<8)|p[7];
    uint32_t limit=((uint32_t)(cap[5]&127u)<<16)|((uint32_t)cap[6]<<8)|cap[7];
    return limit && bitrate>limit?0x29:0;
}
typedef struct {const uint8_t *data;uint32_t bits,pos;bool bad;} bits_t;
static uint32_t get(bits_t *b,unsigned n){
    if(n>32 || n>b->bits-b->pos){b->bad=true;return 0;}
    uint32_t value=0;
    for(unsigned i=0;i<n;++i){value=(value<<1)|((b->data[b->pos>>3]>>(7u-(b->pos&7u)))&1u);++b->pos;}
    return value;
}
dbt_result_t dbt_aac_latm_parse(dbt_aac_mux_t *mux,const dbt_codec_config_t *config,
    const uint8_t *data,uint16_t size,dbt_aac_unit_t *units,uint8_t capacity,uint8_t *count){
    if(!mux || !config || (config->channels!=1 && config->channels!=2) ||
       (config->rate!=44100 && config->rate!=48000) || !data || !size || size>DBT_AAC_MAX_MUX_BYTES || !units || !capacity || !count)return DBT_INVALID;
    bits_t b={data,(uint32_t)size*8u,0,false};dbt_aac_mux_t next=*mux;
    uint32_t same=get(&b,1);
    if(!same){
        if(get(&b,1)!=0 || get(&b,1)!=1)return b.bad?DBT_INVALID:DBT_UNSUPPORTED;
        unsigned frames=get(&b,6)+1u,programs=get(&b,4),layers=get(&b,3);
        if(b.bad)return DBT_INVALID;
        if(programs || layers)return DBT_UNSUPPORTED;
        unsigned object=get(&b,5),index=get(&b,4);
        uint32_t rate=0;
        if(index==15)rate=get(&b,24);
        else if(index==3)rate=48000;
        else if(index==4)rate=44100;
        unsigned channels=get(&b,4),flags=get(&b,3);
        if(b.bad)return DBT_INVALID;
        if(object!=2 || rate!=config->rate || channels!=config->channels || flags)return DBT_UNSUPPORTED;
        if(get(&b,3)!=0)return b.bad?DBT_INVALID:DBT_UNSUPPORTED;
        (void)get(&b,8);unsigned other=get(&b,1),crc=get(&b,1);
        if(b.bad)return DBT_INVALID;
        if(other || crc)return DBT_UNSUPPORTED;
        next.configured=true;next.subframes=(uint8_t)frames;
    }else if(!next.configured)return DBT_INVALID;
    if(!next.subframes || next.subframes>DBT_AAC_MAX_SUBFRAMES)return DBT_INVALID;
    if(next.subframes>capacity)return DBT_NO_SPACE;
    dbt_aac_unit_t found[DBT_AAC_MAX_SUBFRAMES];
    for(unsigned i=0;i<next.subframes;++i){
        uint32_t length=0,part;
        do{part=get(&b,8);length+=part;if(b.bad || length>768u*config->channels)return DBT_INVALID;}while(part==255);
        if(!length || length*8u>b.bits-b.pos)return DBT_INVALID;
        found[i].bit_offset=b.pos;found[i].size=(uint16_t)length;b.pos+=length*8u;
    }
    if(b.bits-b.pos>7u || get(&b,(unsigned)(b.bits-b.pos)) || b.bad)return DBT_INVALID;
    for(unsigned i=0;i<next.subframes;++i)units[i]=found[i];
    *count=next.subframes;*mux=next;return DBT_OK;
}
dbt_result_t dbt_aac_latm_copy(const uint8_t *data,uint16_t size,dbt_aac_unit_t unit,uint8_t *output,uint16_t capacity){
    if(!data || !output || !unit.size || unit.size>capacity || unit.bit_offset>(uint32_t)size*8u ||
       (uint32_t)unit.size*8u>(uint32_t)size*8u-unit.bit_offset)return DBT_INVALID;
    unsigned shift=unit.bit_offset&7u,offset=unit.bit_offset>>3;
    for(unsigned i=0;i<unit.size;++i){
        unsigned value=(unsigned)data[offset+i]<<8;
        if(shift)value|=data[offset+i+1u];
        output[i]=(uint8_t)(value>>(8u-shift));
    }
    return DBT_OK;
}
