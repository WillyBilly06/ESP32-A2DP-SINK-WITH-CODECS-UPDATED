#ifndef PROBE_PCM_LEVEL_H
#define PROBE_PCM_LEVEL_H
#include <stdint.h>
#include <stdbool.h>

/* Decoder-owner diagnostic only. Discard one second, then measure four seconds;
 * reads the signed top16 bits of each PCM container without changing audio.
 * Completed sums are published separately as lock-free32-bit words. */
typedef struct {
    uint32_t rate,channels,width,frames,remaining,skip,count,peak;
    uint64_t squares;
    bool complete;
} probe_pcm_level_t;
static inline void probe_pcm_level_start(probe_pcm_level_t *s,uint32_t rate,uint32_t channels,uint32_t width){
    *s=(probe_pcm_level_t){.rate=rate,.channels=channels,.width=width};
    if(rate && rate<=192000u && channels>=1u && channels<=2u && width>=2u && width<=4u){s->remaining=4u*rate;s->skip=rate;}
}
static inline bool probe_pcm_level_observe(probe_pcm_level_t *s,const void *pcm,uint32_t frames){
    if(!s->remaining || !pcm || !frames)return false;
    const uint8_t *p=pcm;uint32_t stride=s->width*s->channels;
    if(s->skip>=frames){s->skip-=frames;return false;}
    if(s->skip){p+=s->skip*stride;frames-=s->skip;s->skip=0;}
    if(frames>s->remaining)frames=s->remaining;
    for(uint32_t at=0;at<frames;++at){
        for(unsigned channel=0;channel<s->channels;++channel){
            const uint8_t *v=p+at*stride+channel*s->width+s->width-2u;
            int32_t sample=(int16_t)((uint32_t)v[0]|((uint32_t)v[1]<<8));
            uint32_t magnitude=sample<0?(uint32_t)-sample:(uint32_t)sample;
            if(magnitude>s->peak)s->peak=magnitude;
            s->squares+=(uint32_t)(sample*sample);++s->count;
        }
    }
    s->frames+=frames;s->remaining-=frames;
    if(s->remaining)return false;
    s->complete=true;return true;
}
#endif
