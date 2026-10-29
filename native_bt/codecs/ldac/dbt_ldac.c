#include "dbt_ldac.h"

static bool one(unsigned v){return v && !(v&(v-1u));}
dbt_result_t dbt_ldac_configure(const uint8_t *p,uint16_t size,dbt_codec_config_t *out){
    if(!p || !out || size!=10 || p[0] || p[1]!=0xff || p[2]!=0x2d || p[3]!=1 || p[4] || p[5] || p[6]!=0xaa || p[7])return DBT_UNSUPPORTED;
    if(!one(p[8]) || !one(p[9]) || (p[9]&~7u))return DBT_INVALID;
    /* The existing SDK decoder rejects sample-rate IDs 4 and 5. */
    if(!(p[8]&0x3cu))return DBT_UNSUPPORTED;
    dbt_codec_config_t c={0};
    c.rate=p[8]==0x20?44100:p[8]==0x10?48000:p[8]==8?88200:96000;
    c.channels=p[9]==4?1:2;c.samples=c.rate<88200?128:256;c.sample_bytes=4;c.cie_size=10;
    for(unsigned i=0;i<10;++i)c.cie[i]=p[i];
    *out=c;return DBT_OK;
}
uint8_t dbt_a2dp_ldac_validate(const dbt_avdtp_endpoint_t *endpoint,const dbt_avdtp_cap_t *codec){
    if(!endpoint || !codec || codec->category!=7 || !codec->data)return 0x18;
    dbt_codec_config_t c;dbt_avdtp_caps_t local;dbt_avdtp_cap_error_t error;
    if(dbt_ldac_configure(codec->data,codec->size,&c)!=DBT_OK ||
       dbt_avdtp_caps_validate(endpoint->capabilities,endpoint->size,DBT_AVDTP_CAPABILITIES,&local,&error)!=DBT_OK || local.codec.size!=10)return 0x29;
    for(unsigned i=0;i<8;++i)if(codec->data[i]!=local.codec.data[i])return 0x29;
    return (codec->data[8]&local.codec.data[8]) && (codec->data[9]&local.codec.data[9])?0:0x29;
}
dbt_result_t dbt_ldac_packet_parse(const dbt_codec_config_t *c,const uint8_t *p,uint16_t size,
    dbt_ldac_frame_t *frames,uint8_t capacity,uint8_t *count){
    if(!c || !p || !size || !frames || !capacity || !count)return DBT_INVALID;
    dbt_codec_config_t checked;
    if(dbt_ldac_configure(c->cie,c->cie_size,&checked)!=DBT_OK || checked.rate!=c->rate ||
       checked.channels!=c->channels || checked.samples!=c->samples || checked.sample_bytes!=c->sample_bytes)return DBT_INVALID;
    unsigned header=p[0],n=header&15u;
    if((header&0xf0u) || !n)return DBT_UNSUPPORTED;
    if(n>capacity)return DBT_NO_SPACE;
    unsigned expected_rate=c->rate==44100?0:c->rate==48000?1:c->rate==88200?2:3;
    unsigned expected_mode=c->cie[9]==4?0:c->cie[9]==2?1:2;
    unsigned offset=1;dbt_ldac_frame_t found[DBT_LDAC_MAX_FRAMES];
    for(unsigned i=0;i<n;++i){
        if(size-offset<3 || p[offset]!=0xaa)return DBT_INVALID;
        unsigned rate=p[offset+1]>>5,mode=(p[offset+1]>>3)&3u;
        unsigned body=((((unsigned)p[offset+1]&7u)<<6)|(p[offset+2]>>2))+1u;
        unsigned minimum=mode==0?11u:22u,maximum=mode==0?256u:512u;
        if(rate!=expected_rate || mode!=expected_mode || body<minimum || body>maximum || body+3u>size-offset)return DBT_INVALID;
        found[i]=(dbt_ldac_frame_t){(uint16_t)offset,(uint16_t)(body+3u)};offset+=body+3u;
    }
    if(offset!=size)return DBT_INVALID;
    for(unsigned i=0;i<n;++i)frames[i]=found[i];
    *count=(uint8_t)n;return DBT_OK;
}
