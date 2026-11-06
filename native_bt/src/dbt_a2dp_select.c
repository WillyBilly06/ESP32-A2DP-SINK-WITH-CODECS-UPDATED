#include "dbt_a2dp_select.h"

static bool overlap(const void *a,unsigned an,const void *b,unsigned bn){
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;return an && bn && (x<=y?y-x<an:x-y<bn);
}
static bool aliases(uint8_t *out,unsigned capacity,dbt_a2dp_select_result_t *result,const void *p,unsigned size){
    return overlap(out,capacity,p,size) || overlap(result,sizeof(*result),p,size);
}
static void copy(uint8_t *to,const uint8_t *from,unsigned size){for(unsigned i=0;i<size;++i)to[i]=from[i];}
static bool family(const dbt_avdtp_cap_t *a,const dbt_avdtp_cap_t *b){
    if(a->size<2 || b->size<2 || (a->data[0]>>4)!=(b->data[0]>>4) || a->data[1]!=b->data[1])return false;
    if(a->data[1]!=0xff)return true;
    if(a->size<8 || b->size<8)return false;
    for(unsigned i=2;i<8;++i)if(a->data[i]!=b->data[i])return false;
    return true;
}
static dbt_result_t finish(dbt_a2dp_select_result_t *out,const dbt_a2dp_select_result_t *r,
    dbt_a2dp_select_outcome_t outcome,dbt_result_t status){*out=*r;out->outcome=outcome;return status;}

dbt_result_t dbt_a2dp_select_exact(const dbt_avdtp_endpoint_t *endpoints,uint8_t count,
    const dbt_avdtp_sep_t *peer,const uint8_t *capabilities,uint16_t size,
    const dbt_a2dp_select_request_t *request,uint8_t *configuration,uint16_t capacity,
    dbt_a2dp_select_result_t *out){
    if(!endpoints || !count || count>DBT_AVDTP_SINK_ENDPOINT_LIMIT || !peer || !request || !out ||
       !request->codec || request->codec_size<2 || request->codec_size>DBT_A2DP_SELECT_MAX_CODEC ||
       (!capabilities && size) || size>DBT_A2DP_SELECT_MAX_CAPABILITIES || (!configuration && capacity) ||
       request->delay<DBT_A2DP_DELAY_DISABLED || request->delay>DBT_A2DP_DELAY_REQUIRED ||
       request->codec[0] || !peer->seid || peer->seid>62 ||
       overlap(configuration,capacity,out,sizeof(*out)) ||
       aliases(configuration,capacity,out,endpoints,(unsigned)count*sizeof(*endpoints)) ||
       aliases(configuration,capacity,out,peer,sizeof(*peer)) || aliases(configuration,capacity,out,request,sizeof(*request)) ||
       aliases(configuration,capacity,out,request->codec,request->codec_size) ||
       aliases(configuration,capacity,out,capabilities,size))return DBT_INVALID;
    for(unsigned i=0;i<count;++i)
        if(aliases(configuration,capacity,out,endpoints[i].capabilities,endpoints[i].size))return DBT_INVALID;
    dbt_a2dp_select_result_t r={0};r.remote_seid=peer->seid;r.peer_status=DBT_EMPTY;
    if(peer->in_use)return finish(out,&r,DBT_A2DP_SELECT_PEER_UNAVAILABLE,DBT_BUSY);
    if(peer->sink || peer->media_type)return finish(out,&r,DBT_A2DP_SELECT_NOT_AUDIO_SOURCE,DBT_UNSUPPORTED);
    dbt_avdtp_caps_t offered;
    r.peer_status=dbt_avdtp_caps_validate(capabilities,size,DBT_AVDTP_CAPABILITIES,&offered,&r.peer_error);
    if(r.peer_status!=DBT_OK)return finish(out,&r,DBT_A2DP_SELECT_PEER_CAPS_INVALID,DBT_INVALID);
    if((offered.present&0x82u)!=0x82u)return finish(out,&r,DBT_A2DP_SELECT_MISSING_SERVICE,DBT_UNSUPPORTED);
    if(offered.codec.data[0]>>4)return finish(out,&r,DBT_A2DP_SELECT_NOT_AUDIO_SOURCE,DBT_UNSUPPORTED);
    dbt_avdtp_cap_t selected={request->codec,DBT_AVDTP_MEDIA_CODEC,(uint8_t)request->codec_size};
    if(!family(&offered.codec,&selected))return finish(out,&r,DBT_A2DP_SELECT_PEER_INCOMPATIBLE,DBT_UNSUPPORTED);
    /* Validators compare a selected CIE to descriptor capabilities. Reuse that
     * exact contract for the Source offer, canonicalizing only media-type RFU.
     * Identity, ranges and codec-specific fields are not blindly ANDed. */
    uint8_t remote[4u+DBT_A2DP_SELECT_MAX_CODEC];
    remote[0]=1;remote[1]=0;remote[2]=7;remote[3]=offered.codec.size;
    copy(remote+4,offered.codec.data,offered.codec.size);remote[4]=0;
    bool saw_family=false,saw_local=false,saw_peer=false,delay=false;
    int chosen=-1;
    for(unsigned i=0;i<count;++i){
        const dbt_avdtp_endpoint_t *ep=&endpoints[i];dbt_avdtp_caps_t local;
        if(!ep->seid || ep->seid>62 || !ep->validate || !ep->capabilities || ep->size>DBT_A2DP_SELECT_MAX_CAPABILITIES)
            return finish(out,&r,DBT_A2DP_SELECT_LOCAL_DESCRIPTOR_INVALID,DBT_INVALID);
        for(unsigned j=0;j<i;++j)if(endpoints[j].seid==ep->seid)
            return finish(out,&r,DBT_A2DP_SELECT_LOCAL_DESCRIPTOR_INVALID,DBT_INVALID);
        if(dbt_avdtp_caps_validate(ep->capabilities,ep->size,DBT_AVDTP_CAPABILITIES,&local,&r.local_error)!=DBT_OK ||
           (local.present&0x82u)!=0x82u || local.codec.data[0]>>4)
            return finish(out,&r,DBT_A2DP_SELECT_LOCAL_DESCRIPTOR_INVALID,DBT_INVALID);
        if(!family(&local.codec,&selected))continue;
        saw_family=true;r.local_seid=ep->seid;
        uint8_t error=ep->validate(ep,&selected);
        if(error){r.local_codec_error=error;continue;}saw_local=true;
        dbt_avdtp_endpoint_t source={remote,(uint16_t)(4u+offered.codec.size),peer->seid,ep->validate,ep->raw_codeword_bytes,ep->raw_codeword_samples};
        error=ep->validate(&source,&selected);
        if(error){r.peer_codec_error=error;continue;}saw_peer=true;
        bool enabled=request->delay!=DBT_A2DP_DELAY_DISABLED && request->avdtp_version>=0x0103u &&
            (offered.present&(1u<<8)) && (local.present&(1u<<8));
        if(request->delay==DBT_A2DP_DELAY_REQUIRED && !enabled)continue;
        if(chosen<0){chosen=(int)i;delay=enabled;}
    }
    if(chosen<0)return finish(out,&r,!saw_family?DBT_A2DP_SELECT_NO_LOCAL_CODEC:
        !saw_local?DBT_A2DP_SELECT_LOCAL_INCOMPATIBLE:!saw_peer?DBT_A2DP_SELECT_PEER_INCOMPATIBLE:DBT_A2DP_SELECT_DELAY_UNSUPPORTED,DBT_UNSUPPORTED);
    r.local_seid=endpoints[chosen].seid;r.delay_enabled=delay;r.local_codec_error=r.peer_codec_error=0;
    r.required=(uint16_t)(4u+request->codec_size+(delay?2u:0u));
    if(capacity<r.required)return finish(out,&r,DBT_A2DP_SELECT_NO_SPACE,DBT_NO_SPACE);
    configuration[0]=1;configuration[1]=0;configuration[2]=7;configuration[3]=(uint8_t)request->codec_size;
    copy(configuration+4,request->codec,request->codec_size);
    if(delay){configuration[r.required-2u]=8;configuration[r.required-1u]=0;}
    r.configuration_size=r.required;
    return finish(out,&r,DBT_A2DP_SELECT_SUCCESS,DBT_OK);
}
