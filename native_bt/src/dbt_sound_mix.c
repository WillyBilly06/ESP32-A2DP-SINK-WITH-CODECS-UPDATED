#include "dbt_sound.h"
#include <limits.h>
#include <string.h>
#define RING 256u
#define CACHE 256u
#define UNITY 32767
#define DUCK 6554
#define OVERLAY 22938
struct dbt_sound_play {
    dbt_sound_t *sound;dbt_sound_output_t output;void *alloc_context;dbt_sound_free_fn release;
    dbt_sound_wav_t wav;uint8_t type;bool exclusive;
    uint32_t rate,produced,consumed,total,fade,cache_base,cache_count,stop_remaining;
    uint64_t position,step;
    int32_t duck,duck_step;
    _Atomic uint32_t write,read;
    _Atomic bool end,drained,stop_requested;bool stopping;
    uint8_t cache[CACHE*6];
    int32_t ring[RING*2];
};
static bool frame(dbt_sound_play_t *p,uint32_t index,int32_t *left,int32_t *right){
    if(index>=p->wav.frames)index=p->wav.frames-1;
    if(index<p->cache_base || index-p->cache_base>=p->cache_count){
        uint32_t remain=p->wav.frames-index,n=remain>CACHE?CACHE:remain;
        if(!p->sound->store.read(p->sound->store.context,p->type,p->wav.data_offset+index*p->wav.align,p->cache,(uint16_t)(n*p->wav.align)))return false;
        p->cache_base=index;p->cache_count=n;
    }
    const uint8_t *b=p->cache+(index-p->cache_base)*p->wav.align;int32_t values[2];
    for(unsigned ch=0;ch<p->wav.channels;ch++){
        if(p->wav.bits==8)values[ch]=((int32_t)b[ch]-128)*16777216;
        else {unsigned at=ch*(p->wav.bits/8u)+(p->wav.bits==24?1u:0u);
            uint16_t u=(uint16_t)(b[at]|((uint16_t)b[at+1]<<8));int32_t v=u<32768?(int32_t)u:(int32_t)u-65536;values[ch]=v*65536;}
    }
    *left=values[0];*right=values[p->wav.channels-1];return true;
}
static int32_t interpolate(int32_t a,int32_t b,uint32_t frac){return (int32_t)((int64_t)a+(((int64_t)b-a)*(frac>>16))/65536);}
dbt_result_t dbt_sound_play_open(dbt_sound_t *s,uint8_t type,uint32_t rate,bool exclusive,dbt_sound_output_t output,
                                 void *context,dbt_sound_alloc_fn alloc,dbt_sound_free_fn release,dbt_sound_play_t **out){
    if(!s || !out || !alloc || !release || !output.start || !output.stop || type>3 || rate<8000 || rate>192000)return DBT_INVALID;
    *out=0;if(s->muted || s->active || s->playing || !(s->exists&(1u<<type)))return DBT_BUSY;
    uint32_t size=0;dbt_sound_wav_t wav;
    if(s->store.size(s->store.context,type,&size)!=DBT_OK || dbt_sound_wav(s,type,false,size,&wav)!=DBT_OK)return DBT_INVALID;
    dbt_sound_play_t *p=alloc(context,sizeof(*p));if(!p)return DBT_NO_SPACE;
    memset(p,0,sizeof(*p));p->sound=s;p->output=output;p->alloc_context=context;p->release=release;p->wav=wav;p->type=type;p->rate=rate;
    /* Max-volume sound is always mixed as an overlay. */
    p->exclusive=exclusive && type!=3;p->step=((uint64_t)wav.rate<<32)/rate;
    uint64_t total=((uint64_t)wav.frames*rate+wav.rate-1)/wav.rate,limit=(uint64_t)rate*10u;
    p->total=(uint32_t)(total<limit?total:limit);p->fade=rate/100u;p->duck=UNITY;
    p->duck_step=(UNITY-DUCK+(int32_t)p->fade-1)/(int32_t)p->fade;
    atomic_init(&p->write,0);atomic_init(&p->read,0);atomic_init(&p->end,false);atomic_init(&p->drained,false);atomic_init(&p->stop_requested,false);
    if(!output.start(output.context,p->exclusive)){release(context,p);return DBT_BUSY;}
    s->playing=true;*out=p;return DBT_OK;
}
dbt_result_t dbt_sound_play_pump(dbt_sound_play_t *p,uint16_t max_frames){
    if(!p || !max_frames)return DBT_INVALID;
    if(atomic_load_explicit(&p->end,memory_order_acquire))return DBT_EMPTY;
    uint32_t written=atomic_load_explicit(&p->write,memory_order_relaxed);
    uint32_t read=atomic_load_explicit(&p->read,memory_order_acquire);
    if(written-read>RING)return DBT_INVALID;
    unsigned n=RING-(written-read);if(n>max_frames)n=max_frames;
    for(unsigned i=0;i<n && p->produced<p->total;i++){
        if(atomic_load_explicit(&p->stop_requested,memory_order_acquire))break;
        uint32_t idx=(uint32_t)(p->position>>32);int32_t a,b,c,d;
        if(!frame(p,idx,&a,&b) || !frame(p,idx+1,&c,&d)){
            atomic_store_explicit(&p->write,written,memory_order_release);
            atomic_store_explicit(&p->end,true,memory_order_release);return DBT_INVALID;
        }
        unsigned slot=(written%RING)*2;p->ring[slot]=interpolate(a,c,(uint32_t)p->position);p->ring[slot+1]=interpolate(b,d,(uint32_t)p->position);
        ++written;++p->produced;p->position+=p->step;
    }
    atomic_store_explicit(&p->write,written,memory_order_release);
    if(p->produced==p->total || atomic_load_explicit(&p->stop_requested,memory_order_acquire))atomic_store_explicit(&p->end,true,memory_order_release);
    return n?DBT_OK:DBT_BUSY;
}
bool dbt_sound_play_ready(const dbt_sound_play_t *p){
    if(!p)return false;
    bool ended=atomic_load_explicit(&p->end,memory_order_acquire);
    uint32_t written=atomic_load_explicit(&p->write,memory_order_acquire);
    uint32_t read=atomic_load_explicit(&p->read,memory_order_acquire);
    return written-read>=128u || (ended && written!=read);
}
uint16_t dbt_sound_play_mix_frames(const dbt_sound_play_t *p,uint16_t requested){
    if(!p || !requested)return 0;
    if(atomic_load_explicit(&p->end,memory_order_acquire) || atomic_load_explicit(&p->stop_requested,memory_order_acquire))return requested;
    uint32_t read=atomic_load_explicit(&p->read,memory_order_relaxed);
    uint32_t written=atomic_load_explicit(&p->write,memory_order_acquire),available=written-read;
    if(available>RING)return 0;
    return available<requested?(uint16_t)available:requested;
}
void dbt_sound_play_request_stop(dbt_sound_play_t *p){if(p)atomic_store_explicit(&p->stop_requested,true,memory_order_release);}
static int32_t clamp64(int64_t x){return x>INT32_MAX?INT32_MAX:x<INT32_MIN?INT32_MIN:(int32_t)x;}
void dbt_sound_mix(dbt_sound_play_t *p,int32_t *pcm,uint16_t frames){
    if(!p || !pcm)return;
    if(atomic_load_explicit(&p->stop_requested,memory_order_acquire) && !p->stopping){p->stopping=true;p->stop_remaining=p->fade;}
    uint32_t read=atomic_load_explicit(&p->read,memory_order_relaxed);
    bool ended=atomic_load_explicit(&p->end,memory_order_acquire);
    uint32_t written=atomic_load_explicit(&p->write,memory_order_acquire);
    for(unsigned i=0;i<frames;i++){
        bool have=read!=written;
        bool running=have || !ended;
        int32_t target=running?DUCK:UNITY;
        if(p->duck<target){p->duck+=p->duck_step;if(p->duck>target)p->duck=target;}
        else if(p->duck>target){p->duck-=p->duck_step;if(p->duck<target)p->duck=target;}
        int32_t fade=0;
        if(have){uint32_t remaining=p->total-1u-p->consumed,edge=p->consumed<remaining?p->consumed:remaining;
            fade=edge>=p->fade?UNITY:(int32_t)((uint64_t)edge*UNITY/p->fade);}
        if(p->stopping){int32_t stop_fade=(int32_t)((uint64_t)p->stop_remaining*UNITY/p->fade);if(fade>stop_fade)fade=stop_fade;}
        for(unsigned ch=0;ch<2;ch++){
            int64_t base=p->exclusive?0:((int64_t)pcm[i*2+ch]*p->duck)/32768;
            int64_t effect=have?((int64_t)p->ring[(read%RING)*2+ch]*fade)/32768:0;
            if(!p->exclusive)effect=effect*OVERLAY/32768;
            pcm[i*2+ch]=clamp64(base+effect);
        }
        if(have){++read;++p->consumed;}
        if(p->stopping && p->stop_remaining){--p->stop_remaining;}
        if(p->stopping && !p->stop_remaining){read=written;}
    }
    atomic_store_explicit(&p->read,read,memory_order_release);
    if(read==written && ended && (p->exclusive || p->duck==UNITY))
        atomic_store_explicit(&p->drained,true,memory_order_release);
}
bool dbt_sound_play_drained(const dbt_sound_play_t *p){
    return p && atomic_load_explicit(&p->drained,memory_order_acquire);
}
void dbt_sound_play_close(dbt_sound_play_t *p){
    if(!p)return; /* Caller has quiesced both producer and render. */
    p->output.stop(p->output.context);p->sound->playing=false;
    dbt_sound_free_fn release=p->release;void *context=p->alloc_context;memset(p,0,sizeof(*p));release(context,p);
}
dbt_result_t dbt_sound_play_queue(dbt_sound_t *s,uint8_t type,uint32_t rate,bool exclusive){
    if(!s || type>3 || rate<8000 || rate>192000)return DBT_INVALID;
    if(!s->playing || s->muted || !(s->exists&(1u<<type)))return DBT_BUSY;
    s->queued=true;s->queued_type=type;s->queued_rate=rate;s->queued_exclusive=exclusive && type!=3;return DBT_OK;
}
bool dbt_sound_play_take_queued(dbt_sound_t *s,uint8_t *type,uint32_t *rate,bool *exclusive){
    if(!s || !type || !rate || !exclusive || s->playing || !s->queued)return false;
    bool eligible=!s->muted && (s->exists&(1u<<s->queued_type));
    if(eligible){*type=s->queued_type;*rate=s->queued_rate;*exclusive=s->queued_exclusive;}
    s->queued=false;return eligible;
}
void dbt_sound_play_clear_queue(dbt_sound_t *s){if(s)s->queued=false;}
