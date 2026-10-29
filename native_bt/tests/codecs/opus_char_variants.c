/* Decode the existing target vectors with one guarded fixed-point host build. */
#include "opus.h"
#include "dbt_opus_scratch.h"
#include "dbt_opus_guarded_api.h"
#include "probe_opus_vectors.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {OpusDecoder *decoder;const uint8_t *packet;int size;int16_t *pcm;int samples;} job_t;
static int work(void *p){job_t *j=p;return dbt_opus_arena_decode(j->decoder,j->packet+1,j->size-1,j->pcm,j->samples,0);}
static int reset(void *p){job_t *j=p;return opus_decoder_ctl(j->decoder,OPUS_RESET_STATE);}
int main(int argc,char **argv){
    int failed=0, dump=argc>1 && !strcmp(argv[1],"pcm"), strict=argc>1 && !strcmp(argv[1],"strict");
    for(unsigned k=0;k<sizeof(opus_vectors)/sizeof(opus_vectors[0]);++k){
        const opus_vector_t *v=&opus_vectors[k];int error=0,result=0;
        OpusDecoder *d=opus_decoder_create(48000,v->channels,&error);
        dbt_opus_scratch arena={0};
        if(!d || error || dbt_opus_scratch_init(&arena,dbt_opus_scratch_capacity(v->channels),malloc,free))return 2;
        int16_t pcm[3840];uint32_t hash=2166136261u;
        for(unsigned i=0;i<3;++i){
            job_t job={d,v->packet[i],v->bytes[i],pcm,v->samples};
            if(dbt_opus_scratch_run(&arena,work,&job,reset,&job,&result) || result!=v->samples)return 3;
            const uint8_t *bytes=(const uint8_t *)pcm;
            for(unsigned j=0;j<(unsigned)result*v->channels*2u;++j)hash=(hash^bytes[j])*16777619u;
            if(dump && k==2){
                for(unsigned start=0;start<(unsigned)result;start+=32){
                    fprintf(stderr,"%u %u ",i,start);
                    for(unsigned j=start;j<(unsigned)result && j<start+32;++j)fprintf(stderr,"%04x",(unsigned)(uint16_t)pcm[j]);
                    fputc('\n',stderr);
                }
            }
        }
        printf("%u %u %u\n",k,(unsigned)hash,(unsigned)v->hash);
        if(hash!=v->hash)failed=1;
        if(dbt_opus_scratch_destroy(&arena))return 4;
        opus_decoder_destroy(d);
    }
    return strict && failed ? 1 : 0;
}
