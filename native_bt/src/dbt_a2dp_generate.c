#include "dbt_a2dp_generate.h"
#include "dbt_sbc_media.h"

static uint16_t le16(const uint8_t *p){return (uint16_t)((unsigned)p[0]|((unsigned)p[1]<<8));}
static uint32_t le32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void put16(uint8_t *p,uint16_t n){p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);}
static void put32(uint8_t *p,uint32_t n){for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(n>>(8u*i));}
static unsigned minimum(unsigned a,unsigned b){return a<b?a:b;}
static unsigned maximum(unsigned a,unsigned b){return a>b?a:b;}
static unsigned nonzero_limit(unsigned a,unsigned b){return a && b?minimum(a,b):a?a:b;}
static bool overlap(const void *a,unsigned an,const void *b,unsigned bn){uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;return an && bn && (x<=y?y-x<an:x-y<bn);}
static bool alias(uint8_t *data,unsigned cap,dbt_a2dp_generate_result_t *r,const void *p,unsigned n){return overlap(data,cap,p,n)||overlap(r,sizeof(*r),p,n);}
static dbt_result_t done(dbt_a2dp_generate_result_t *out,dbt_a2dp_generate_result_t *r,dbt_a2dp_generate_outcome_t why,dbt_result_t rc){r->outcome=why;*out=*r;return rc;}
static unsigned first(unsigned mask,const uint8_t *order,unsigned n){for(unsigned i=0;i<n;++i)if(mask&order[i])return order[i];return 0;}
static unsigned rate_bit(uint32_t rate,const uint32_t *rates,const uint8_t *bits,unsigned n){for(unsigned i=0;i<n;++i)if(rate==rates[i])return bits[i];return 0;}
static unsigned family(const dbt_avdtp_cap_t *c){
    if(c->size<2 || c->data[0]>>4)return 0;
    const uint8_t *p=c->data;if(p[1]==0)return DBT_A2DP_FAMILY_SBC;if(p[1]==2)return DBT_A2DP_FAMILY_AAC;
    if(p[1]!=0xff || c->size<8)return 0;
    uint32_t v=le32(p+2);unsigned id=le16(p+6);
    if(v==0x4f && id==1)return DBT_A2DP_FAMILY_APTX;
    if(v==0xd7 && id==0x24)return DBT_A2DP_FAMILY_APTX_HD;
    if(v==0x0a && id==2)return DBT_A2DP_FAMILY_APTX_LL;
    if(v==0x12d && id==0xaa)return DBT_A2DP_FAMILY_LDAC;
    if(v==0xe0 && id==1)return DBT_A2DP_FAMILY_ANDROID_OPUS;
    if(v==0x5f1 && id==0x1005)return DBT_A2DP_FAMILY_OPUS05;
    if(v==0x53a && id==0x4c35)return DBT_A2DP_FAMILY_LHDCV5;
    return 0;
}
static unsigned duration(uint32_t us,unsigned mask,bool android){
    static const uint32_t times[]={20000,10000,5000,2500,40000};
    static const uint8_t bits[]={8,4,2,1,16};
    for(unsigned i=0;i<5;++i){unsigned bit=android?(times[i]==20000?16:times[i]==10000?8:0):bits[i];
        if(bit && (!us || us==times[i]) && (mask&bit))return bit;}
    return 0;
}
static bool sbc(const uint8_t *a,const uint8_t *b,const dbt_a2dp_generate_request_t *r,uint8_t *p){
    static const uint32_t rates[]={16000,32000,44100,48000};static const uint8_t bits[]={128,64,32,16};
    static const uint8_t modes[]={1,2,4},blocks[]={16,32,64,128},bands[]={4,8},alloc[]={1,2};
    unsigned freq=rate_bit(r->rate,rates,bits,4),mode=r->codec.sbc.mode;
    unsigned common=a[2]&b[2];if(!freq || !(freq&common))return false;
    if(!mode)mode=r->channels==1?8:first(common,modes,3);
    if((r->channels==1 && mode!=8) || (r->channels==2 && mode!=1 && mode!=2 && mode!=4) || !(mode&common))return false;
    unsigned block=r->codec.sbc.blocks,band=r->codec.sbc.subbands,allocation=r->codec.sbc.allocation;
    common=a[3]&b[3];unsigned bm=block==4?128:block==8?64:block==12?32:block==16?16:0;
    if(block && !bm)return false;
    if(!block)bm=first(common,blocks,4);
    unsigned sm=band==4?8:band==8?4:0;if(band && !sm)return false;if(!band)sm=first(common,bands,2);
    if(!allocation)allocation=first(common,alloc,2);
    if(!bm || !(bm&common) || !sm || !(sm&common) || (allocation!=1 && allocation!=2) || !(allocation&common))return false;
    unsigned lo=maximum(a[4],b[4]),hi=minimum(a[5],b[5]);lo=maximum(lo,2);hi=minimum(hi,250);
    if(r->codec.sbc.minimum_bitpool){if(r->codec.sbc.minimum_bitpool<lo)return false;lo=r->codec.sbc.minimum_bitpool;}
    if(r->codec.sbc.maximum_bitpool){if(r->codec.sbc.maximum_bitpool>hi)return false;hi=r->codec.sbc.maximum_bitpool;}
    p[2]=(uint8_t)(freq|mode);p[3]=(uint8_t)(bm|sm|allocation);p[4]=(uint8_t)lo;p[5]=(uint8_t)hi;
    if(lo>hi || lo>250)return false;
    dbt_sbc_config_t config;
    if(dbt_sbc_configure(p,6,&config)!=DBT_OK)return false;
    uint16_t bytes;
    for(unsigned attempt=0;attempt<249;++attempt){
        if(dbt_sbc_frame_bytes(&config,(uint8_t)hi,&bytes)==DBT_OK){p[5]=(uint8_t)hi;return true;}
        if(r->codec.sbc.maximum_bitpool || hi<=lo)return false;
        --hi;
    }
    return false;
}
static bool build(const dbt_avdtp_cap_t *local,const dbt_avdtp_cap_t *peer,const dbt_a2dp_generate_request_t *r,uint8_t *p){
    const uint8_t *a=local->data,*b=peer->data;unsigned n=local->size;
    for(unsigned i=0;i<n;++i)p[i]=a[i];
    p[0]=0;
    switch(r->family){
    case DBT_A2DP_FAMILY_SBC:return sbc(a,b,r,p);
    case DBT_A2DP_FAMILY_AAC:{
        if(r->codec.aac.vbr<DBT_A2DP_AAC_VBR_AUTO || r->codec.aac.vbr>DBT_A2DP_AAC_VBR_REQUIRED)return false;
        unsigned freq=r->rate==44100?16:r->rate==48000?8:0;if(!freq)return false;
        unsigned object=r->codec.aac.object;if(!object)object=(a[2]&b[2]&0x40)?0x40:(a[2]&b[2]&0x80)?0x80:0;
        if((object!=0x40 && object!=0x80) || !(object&a[2]&b[2]))return false;
        bool vbr=(a[5]&b[5]&128)!=0;if(r->codec.aac.vbr==DBT_A2DP_AAC_CBR)vbr=false;
        if(r->codec.aac.vbr==DBT_A2DP_AAC_VBR_REQUIRED && !vbr)return false;
        unsigned x=((unsigned)(a[5]&127)<<16)|((unsigned)a[6]<<8)|a[7];
        unsigned y=((unsigned)(b[5]&127)<<16)|((unsigned)b[6]<<8)|b[7],limit=nonzero_limit(x,y),bitrate=r->codec.aac.maximum_bitrate_bps;
        if(bitrate>0x7fffff || (limit && bitrate>limit))return false;
        if(!bitrate)bitrate=limit;
        p[2]=(uint8_t)object;p[3]=(uint8_t)(freq>>4);p[4]=(uint8_t)((freq<<4)|(r->channels==1?8:4));
        p[5]=(uint8_t)((vbr?128:0)|(bitrate>>16));p[6]=(uint8_t)(bitrate>>8);p[7]=(uint8_t)bitrate;return true;}
    case DBT_A2DP_FAMILY_APTX:case DBT_A2DP_FAMILY_APTX_HD:case DBT_A2DP_FAMILY_APTX_LL:
        if(r->channels!=2 || (r->rate!=44100 && r->rate!=48000))return false;
        p[8]=(uint8_t)((r->rate==44100?32:16)|2);for(unsigned i=9;i<n;++i)p[i]=0;return true;
    case DBT_A2DP_FAMILY_LDAC:{
        static const uint32_t rates[]={44100,48000,88200,96000};static const uint8_t bits[]={32,16,8,4},modes[]={1,2};
        unsigned freq=rate_bit(r->rate,rates,bits,4),mode=r->codec.ldac.mode;
        if(!freq || !(freq&a[8]&b[8]))return false;
        if(!mode)mode=r->channels==1?4:first(a[9]&b[9],modes,2);
        if((r->channels==1 && mode!=4) || (r->channels==2 && mode!=1 && mode!=2) || !(mode&a[9]&b[9]))return false;
        p[8]=(uint8_t)freq;p[9]=(uint8_t)mode;return true;}
    case DBT_A2DP_FAMILY_ANDROID_OPUS:{
        unsigned frame=duration(r->codec.android_opus.duration_us,a[8]&b[8],true);
        if(r->rate!=48000 || !frame)return false;
        p[8]=(uint8_t)(128|frame|r->channels);return true;}
    case DBT_A2DP_FAMILY_OPUS05:{
        unsigned frame=duration(r->codec.opus05.duration_us,a[14]&b[14],false);uint32_t location=r->codec.opus05.location,mask=le32(a+10)&le32(b+10);
        if(r->rate!=48000 || !frame)return false;
        if(!location)location=r->channels==2?3:(mask&1)?1:(mask&2)?2:0;
        if((r->channels==2 && location!=3) || (r->channels==1 && location!=1 && location!=2) || (location&mask)!=location)return false;
        unsigned limit=nonzero_limit(le16(a+15),le16(b+15)),bitrate=r->codec.opus05.maximum_bitrate_field;
        if(limit && bitrate>limit)return false;
        if(!bitrate)bitrate=limit;
        p[8]=r->channels;p[9]=(uint8_t)(r->channels==2);put32(p+10,location);p[14]=(uint8_t)frame;put16(p+15,(uint16_t)bitrate);
        for(unsigned i=17;i<26;++i)p[i]=0;
        return true;}
    case DBT_A2DP_FAMILY_LHDCV5:{
        static const uint32_t rates[]={192000,96000,48000,44100};static const uint8_t bits[]={1,4,16,32};
        static const uint16_t mins[]={64,160,256,400},maxs[]={1000,400,500,900};
        unsigned freq=rate_bit(r->rate,rates,bits,4),depth=r->codec.lhdc.bits,common=a[9]&b[9]&7;
        if(r->channels!=2 || !freq || !(freq&a[8]&b[8]) || (r->codec.lhdc.features&~0x40u))return false;
        unsigned dm=depth==32?1:depth==24?2:depth==16?4:0;if(depth && !dm)return false;
        if(!depth)dm=(common&1)?1:(common&2)?2:(common&4)?4:0;
        if(!dm || !(dm&common))return false;
        unsigned lo=maximum(mins[(a[9]>>6)&3],mins[(b[9]>>6)&3]),hi=minimum(maxs[(a[9]>>4)&3],maxs[(b[9]>>4)&3]);
        if(r->codec.lhdc.minimum_kbps){if(r->codec.lhdc.minimum_kbps<lo)return false;lo=r->codec.lhdc.minimum_kbps;}
        if(r->codec.lhdc.maximum_kbps){if(r->codec.lhdc.maximum_kbps>hi)return false;hi=r->codec.lhdc.maximum_kbps;}
        if(lo>hi)return false;
        unsigned mi=4,ma=4;
        for(unsigned i=0;i<4;++i){if(mins[i]==lo)mi=i;if(maxs[i]==hi)ma=i;}if(mi==4 || ma==4)return false;
        p[8]=(uint8_t)freq;p[9]=(uint8_t)(dm|(mi<<6)|(ma<<4));p[10]=0x11;p[11]=r->codec.lhdc.features;p[12]=0;return true;}
    default:return false;
    }
}
static unsigned expected(unsigned family){switch(family){case DBT_A2DP_FAMILY_SBC:return 6;case DBT_A2DP_FAMILY_AAC:return 8;
    case DBT_A2DP_FAMILY_APTX:case DBT_A2DP_FAMILY_ANDROID_OPUS:return 9;case DBT_A2DP_FAMILY_APTX_LL:case DBT_A2DP_FAMILY_LDAC:return 10;
    case DBT_A2DP_FAMILY_APTX_HD:case DBT_A2DP_FAMILY_LHDCV5:return 13;case DBT_A2DP_FAMILY_OPUS05:return 26;default:return 0;}}

dbt_result_t dbt_a2dp_generate(const dbt_avdtp_endpoint_t *endpoints,uint8_t count,const dbt_avdtp_sep_t *peer,
    const uint8_t *capabilities,uint16_t size,const dbt_a2dp_generate_request_t *request,
    uint8_t *configuration,uint16_t capacity,dbt_a2dp_generate_result_t *out){
    if(!endpoints || !count || count>DBT_AVDTP_SINK_ENDPOINT_LIMIT || !peer || !request || !out ||
       !request->rate || request->channels<1 || request->channels>2 || request->family<DBT_A2DP_FAMILY_SBC || request->family>DBT_A2DP_FAMILY_LHDCV5 ||
       request->delay<DBT_A2DP_DELAY_DISABLED || request->delay>DBT_A2DP_DELAY_REQUIRED ||
       (!capabilities && size) || size>DBT_A2DP_SELECT_MAX_CAPABILITIES || (!configuration && capacity) ||
       overlap(configuration,capacity,out,sizeof(*out)) || alias(configuration,capacity,out,endpoints,(unsigned)count*sizeof(*endpoints)) ||
       alias(configuration,capacity,out,peer,sizeof(*peer)) || alias(configuration,capacity,out,request,sizeof(*request)) ||
       alias(configuration,capacity,out,capabilities,size))return DBT_INVALID;
    for(unsigned i=0;i<count;++i)if(alias(configuration,capacity,out,endpoints[i].capabilities,endpoints[i].size))return DBT_INVALID;
    dbt_a2dp_generate_result_t r={0};
    if(!peer->seid || peer->seid>62)return DBT_INVALID;
    if(peer->in_use || peer->sink || peer->media_type)return done(out,&r,DBT_A2DP_GENERATE_PEER_NOT_ELIGIBLE,peer->in_use?DBT_BUSY:DBT_UNSUPPORTED);
    dbt_avdtp_caps_t offered;
    if(dbt_avdtp_caps_validate(capabilities,size,DBT_AVDTP_CAPABILITIES,&offered,&r.capability_error)!=DBT_OK || (offered.present&0x82u)!=0x82u)
        return done(out,&r,DBT_A2DP_GENERATE_PEER_CAPS_INVALID,DBT_UNSUPPORTED);
    if(family(&offered.codec)!=(unsigned)request->family)return done(out,&r,DBT_A2DP_GENERATE_PEER_FAMILY_MISMATCH,DBT_UNSUPPORTED);
    unsigned bytes=expected((unsigned)request->family);
    if(offered.codec.size!=bytes)return done(out,&r,DBT_A2DP_GENERATE_LAYOUT_UNSUPPORTED,DBT_UNSUPPORTED);
    for(unsigned i=0;i<count;++i){dbt_avdtp_caps_t local;
        if(!endpoints[i].seid || endpoints[i].seid>62 || !endpoints[i].validate || !endpoints[i].capabilities || endpoints[i].size>DBT_A2DP_SELECT_MAX_CAPABILITIES ||
           dbt_avdtp_caps_validate(endpoints[i].capabilities,endpoints[i].size,DBT_AVDTP_CAPABILITIES,&local,&r.capability_error)!=DBT_OK ||
           (local.present&0x82u)!=0x82u || local.codec.data[0]>>4)return done(out,&r,DBT_A2DP_GENERATE_LOCAL_DESCRIPTOR_INVALID,DBT_INVALID);
        for(unsigned j=0;j<i;++j)if(endpoints[j].seid==endpoints[i].seid)return done(out,&r,DBT_A2DP_GENERATE_LOCAL_DESCRIPTOR_INVALID,DBT_INVALID);
    }
    bool saw=false,generated=false;
    for(unsigned i=0;i<count;++i){dbt_avdtp_caps_t local;dbt_avdtp_cap_error_t error;
        (void)dbt_avdtp_caps_validate(endpoints[i].capabilities,endpoints[i].size,DBT_AVDTP_CAPABILITIES,&local,&error);
        if(family(&local.codec)!=(unsigned)request->family)continue;
        saw=true;r.local_seid=endpoints[i].seid;
        if(local.codec.size!=bytes)return done(out,&r,DBT_A2DP_GENERATE_LOCAL_DESCRIPTOR_INVALID,DBT_INVALID);
        uint8_t cie[26];if(!build(&local.codec,&offered.codec,request,cie))continue;generated=true;r.codec_size=(uint8_t)bytes;
        dbt_a2dp_select_request_t exact={cie,(uint16_t)bytes,request->avdtp_version,request->delay};r.match_attempted=true;
        dbt_result_t rc=dbt_a2dp_select_exact(&endpoints[i],1,peer,capabilities,size,&exact,configuration,capacity,&r.selection);
        if(rc==DBT_OK)return done(out,&r,DBT_A2DP_GENERATE_SUCCESS,rc);
        if(rc==DBT_NO_SPACE)return done(out,&r,DBT_A2DP_GENERATE_NO_SPACE,rc);
        if(rc!=DBT_UNSUPPORTED)return done(out,&r,DBT_A2DP_GENERATE_VALIDATION_FAILED,rc);
    }
    return done(out,&r,!saw?DBT_A2DP_GENERATE_NO_LOCAL_FAMILY:generated?DBT_A2DP_GENERATE_VALIDATION_FAILED:DBT_A2DP_GENERATE_PARAMETERS_UNSUPPORTED,DBT_UNSUPPORTED);
}
