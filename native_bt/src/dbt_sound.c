#include "dbt_sound.h"
#include <string.h>
#define UPLOAD_TIMEOUT_MS 30000u
static uint16_t le16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static uint32_t le32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static bool same(dbt_sound_peer_t a,dbt_sound_peer_t b){return a.handle==b.handle && a.generation==b.generation;}
static uint32_t crc_add(uint32_t crc,const uint8_t *p,unsigned n){
    while(n--){crc^=*p++;for(unsigned j=0;j<8;j++)crc=(crc>>1)^(0xedb88320u & (0u-(crc&1u)));}return crc;
}
static bool read_part(dbt_sound_t *s,uint8_t type,bool temp,uint32_t at,uint8_t *p,uint16_t n){
    return temp?s->store.read_temp(s->store.context,at,p,n):s->store.read(s->store.context,type,at,p,n);
}
dbt_result_t dbt_sound_wav(dbt_sound_t *s,uint8_t type,bool temp,uint32_t size,dbt_sound_wav_t *out){
    if(!s || !out || type>3 || size<44)return DBT_INVALID;
    uint8_t b[16];if(!read_part(s,type,temp,0,b,12) || memcmp(b,"RIFF",4) || memcmp(b+8,"WAVE",4) || le32(b+4)!=size-8)return DBT_INVALID;
    dbt_sound_wav_t w={0};uint32_t pos=12;bool fmt=false,data=false;unsigned chunks=0;
    while(pos<=size-8){
        if(++chunks>256 || (temp && dbt_sound_poll(s,s->store.now_ms(s->store.context),0)))return DBT_INVALID;
        if(!read_part(s,type,temp,pos,b,8))return DBT_INVALID;
        uint32_t n=le32(b+4);pos+=8;if(n>size-pos)return DBT_INVALID;
        if(!memcmp(b,"fmt ",4)){
            if(fmt || n<16 || !read_part(s,type,temp,pos,b,16))return DBT_INVALID;
            if(le16(b)!=1)return DBT_UNSUPPORTED;
            w.channels=le16(b+2);w.rate=le32(b+4);uint32_t byte_rate=le32(b+8);
            w.align=le16(b+12);w.bits=le16(b+14);
            if((w.channels!=1 && w.channels!=2) || (w.bits!=8 && w.bits!=16 && w.bits!=24) ||
               w.rate<8000 || w.rate>192000 || w.align!=w.channels*(w.bits/8u) || byte_rate!=w.rate*w.align)return DBT_INVALID;
            fmt=true;
        } else if(!memcmp(b,"data",4)){
            if(!fmt || data || !n || n%w.align)return DBT_INVALID;
            w.data_offset=pos;w.data_bytes=n;w.frames=n/w.align;data=true;
        }
        if(n>size-pos || (n&1u && n==size-pos))return DBT_INVALID;
        pos+=n+(n&1u);
    }
    if(!fmt || !data || pos!=size)return DBT_INVALID;
    *out=w;return DBT_OK;
}
dbt_result_t dbt_sound_init(dbt_sound_t *s,dbt_sound_store_t store,bool muted){
    if(!s || !store.begin || !store.write || !store.seal || !store.read_temp || !store.commit || !store.abort ||
       !store.size || !store.read || !store.remove || !store.set_mute || !store.now_ms || !store.peer_valid)return DBT_INVALID;
    memset(s,0,sizeof(*s));s->store=store;s->muted=muted;
    atomic_init(&s->upload_phase,0);atomic_init(&s->owner_handle,0);atomic_init(&s->owner_generation,0);atomic_init(&s->identity_generation,0);
    return dbt_sound_refresh(s);
}
dbt_result_t dbt_sound_refresh(dbt_sound_t *s){
    if(!s)return DBT_INVALID;
    uint8_t exists=0;
    for(uint8_t i=0;i<4;i++){uint32_t size=0;dbt_result_t r=s->store.size(s->store.context,i,&size);
        if(r==DBT_OK)exists|=(uint8_t)(1u<<i);else if(r!=DBT_EMPTY)return r;}
    s->exists=exists;return DBT_OK;
}
uint8_t dbt_sound_status(const dbt_sound_t *s){return s?(uint8_t)(s->exists|(s->muted?0x80:0)):0;}
static void abort_upload(dbt_sound_t *s){if(s->active)s->store.abort(s->store.context);s->active=false;atomic_store_explicit(&s->upload_phase,0,memory_order_release);}
dbt_result_t dbt_sound_disconnect(dbt_sound_t *s,dbt_sound_peer_t peer){
    if(!s || !peer.generation)return DBT_INVALID;
    uint32_t phase=atomic_load_explicit(&s->upload_phase,memory_order_acquire);
    if(!phase || atomic_load_explicit(&s->owner_handle,memory_order_relaxed)!=peer.handle ||
       atomic_load_explicit(&s->identity_generation,memory_order_relaxed)!=peer.generation)return DBT_EMPTY;
    /* The runtime's link generation is globally unique and never recycled.
     * CAS the generation itself: a stale disconnect cannot cancel a newer
     * session between a separate owner check and a phase transition. */
    uint32_t expected=peer.generation;
    if(atomic_compare_exchange_strong_explicit(&s->owner_generation,&expected,0,memory_order_acq_rel,memory_order_acquire))return DBT_OK;
    return expected==0 && atomic_load_explicit(&s->identity_generation,memory_order_acquire)==peer.generation?DBT_BUSY:DBT_EMPTY;
}
bool dbt_sound_poll(dbt_sound_t *s,uint32_t now,dbt_sound_peer_t *owner){
    if(s && s->active && (atomic_load_explicit(&s->owner_generation,memory_order_acquire)!=s->peer.generation ||
       !s->store.peer_valid(s->store.context,s->peer) || (int32_t)(now-s->deadline)>=0)){
        if(owner)*owner=s->peer;
        abort_upload(s);return true;
    }
    return false;
}
static void response(dbt_sound_response_t *r,uint8_t a,uint8_t b,uint8_t c,uint8_t n){r->data[0]=a;r->data[1]=b;r->data[2]=c;r->size=n;}
static dbt_result_t failure(dbt_sound_t *s,dbt_sound_response_t *r,uint8_t code,bool cancel){if(cancel)abort_upload(s);response(r,0x33,code,0,2);return DBT_OK;}
dbt_result_t dbt_sound_command(dbt_sound_t *s,dbt_sound_peer_t peer,uint8_t cmd,const uint8_t *p,uint16_t n,uint32_t now,dbt_sound_response_t *r){
    if(!s || !r || !peer.generation || (!p && n) || !s->store.peer_valid(s->store.context,peer))return DBT_INVALID;
    r->size=0;dbt_sound_poll(s,now,0);
    if(s->active && !same(peer,s->peer))return failure(s,r,0x07,false);
    if(cmd==0x10){
        if(n<1){response(r,0x11,cmd,2,3);return DBT_OK;}
        bool muted=p[0]!=0;if(!s->store.set_mute(s->store.context,muted)){response(r,0x11,cmd,3,3);return DBT_OK;}
        s->muted=muted;response(r,0x10,cmd,0,2);return DBT_OK;
    }
    if(cmd==0x11){
        if(n<1 || p[0]>3){response(r,0x11,cmd,2,3);return DBT_OK;}
        if(s->active || s->playing || !s->store.remove(s->store.context,p[0]) || dbt_sound_refresh(s)!=DBT_OK){response(r,0x11,cmd,3,3);return DBT_OK;}
        response(r,0x10,cmd,0,2);return DBT_OK;
    }
    if(cmd==0x12){
        if(s->active || s->playing)return failure(s,r,0x07,false);
        if(n!=4 || p[0]>3)return failure(s,r,0x09,false);
        uint32_t size=(uint32_t)p[1]|((uint32_t)p[2]<<8)|((uint32_t)p[3]<<16);
        if(size<44)return failure(s,r,0x08,false);
        s->peer=peer;s->type=p[0];s->expected=size;s->written=0;s->sequence=0;s->crc=0xffffffffu;s->deadline=now+UPLOAD_TIMEOUT_MS;
        atomic_store_explicit(&s->owner_handle,peer.handle,memory_order_relaxed);
        atomic_store_explicit(&s->identity_generation,peer.generation,memory_order_relaxed);
        atomic_store_explicit(&s->owner_generation,peer.generation,memory_order_release);
        atomic_store_explicit(&s->upload_phase,1,memory_order_release);
        if(!s->store.begin(s->store.context,p[0],size)){atomic_store_explicit(&s->upload_phase,0,memory_order_release);return failure(s,r,0x06,false);}
        s->active=true;dbt_sound_poll(s,s->store.now_ms(s->store.context),0);
        if(!s->active)return failure(s,r,0x0b,false);
        response(r,0x31,0,0,1);return DBT_OK;
    }
    if(cmd==0x13){
        if(!s->active)return failure(s,r,0x05,false);
        if(n<2 || n>513 || p[0]!=s->sequence || (uint32_t)(n-1)>s->expected-s->written)return failure(s,r,0x03,true);
        if(!s->store.write(s->store.context,s->written,p+1,n-1))return failure(s,r,0x04,true);
        dbt_sound_poll(s,s->store.now_ms(s->store.context),0);if(!s->active)return failure(s,r,0x0b,false);
        s->crc=crc_add(s->crc,p+1,n-1);s->written+=n-1;s->sequence=(uint8_t)(s->sequence+1);s->deadline=now+UPLOAD_TIMEOUT_MS;
        response(r,0x31,0,0,1);return DBT_OK;
    }
    if(cmd==0x14){
        if(!s->active)return failure(s,r,0x05,false);
        if(n || s->written!=s->expected || !s->store.seal(s->store.context))return failure(s,r,0x05,true);
        uint32_t crc=0xffffffffu,at=0;uint8_t buf[256];
        while(at<s->expected){uint16_t count=(uint16_t)((s->expected-at)>sizeof(buf)?sizeof(buf):(s->expected-at));
            dbt_sound_poll(s,s->store.now_ms(s->store.context),0);if(!s->active)return failure(s,r,0x0b,false);
            if(!s->store.read_temp(s->store.context,at,buf,count))return failure(s,r,0x04,true);
            crc=crc_add(crc,buf,count);at+=count;}
        dbt_sound_wav_t wav;
        if(crc!=s->crc || dbt_sound_wav(s,s->type,true,s->expected,&wav)!=DBT_OK)return failure(s,r,0x08,true);
        dbt_sound_poll(s,s->store.now_ms(s->store.context),0);if(!s->active)return failure(s,r,0x0b,false);
        uint32_t expected=s->peer.generation;
        if(!atomic_compare_exchange_strong_explicit(&s->owner_generation,&expected,0,memory_order_acq_rel,memory_order_acquire))return failure(s,r,0x0b,true);
        atomic_store_explicit(&s->upload_phase,2,memory_order_release);
        if(!s->store.commit(s->store.context,s->type))return failure(s,r,0x04,true);
        s->active=false;atomic_store_explicit(&s->upload_phase,0,memory_order_release);
        if(dbt_sound_refresh(s)!=DBT_OK)return failure(s,r,0x04,false);
        response(r,0x32,0,0,1);return DBT_OK;
    }
    response(r,0x11,cmd,1,3);return DBT_OK;
}
