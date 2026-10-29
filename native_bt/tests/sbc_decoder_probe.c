#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "common/bt_target.h"
#include "oi_codec_sbc.h"
#include "dbt_sbc_decoder.h"
#include "dbt_sbc_worker.h"
#ifdef SBC_THREAD_TEST
#include <pthread.h>
#include <sched.h>
#include <time.h>
static bool stress_mode;
static _Atomic unsigned stress_frames;
static _Atomic bool stress_stop;
#endif

static dbt_sbc_decoder_t decoder;
static dbt_sbc_media_t stream;
static OI_CODEC_SBC_DECODER_CONTEXT reference;
static OI_UINT32 reference_data[CODEC_DATA_WORDS(2,SBC_CODEC_FAST_FILTER_BUFFERS)];
static size_t live,peak;
static int fail_next;
static int16_t *output;
static unsigned output_bytes;
static dbt_sbc_worker_t worker;
typedef union { struct {size_t size;} h; max_align_t align; } allocation_t;
static uint8_t *allocate(void *context,uint16_t size){
    (void)context;if(fail_next){fail_next=0;return 0;}
    allocation_t *a=malloc(sizeof(*a)+size);if(!a)return 0;
    a->h.size=size;live+=size;if(live>peak)peak=live;return (uint8_t *)(a+1);
}
static void release(void *context,uint8_t *data){
    (void)context;allocation_t *a=(allocation_t *)data-1;live-=a->h.size;free(a);
}
static dbt_result_t pcm(void *context,const int16_t *data,uint16_t samples,uint8_t channels,uint32_t rate,uint32_t timestamp){
    (void)context;(void)rate;(void)timestamp;unsigned bytes=samples*channels*2u;
#ifdef SBC_THREAD_TEST
    if(stress_mode){atomic_fetch_add(&stress_frames,1);return DBT_OK;}
#endif
    if(!output || output_bytes+bytes>8192)return DBT_NO_SPACE;
    memcpy((uint8_t *)output+output_bytes,data,bytes);output_bytes+=bytes;return DBT_OK;
}
void probe_close(void){dbt_sbc_media_reset(&stream);dbt_sbc_decoder_close(&decoder);}
int probe_open(const uint8_t *cie){
    dbt_sbc_config_t config;
    if(dbt_sbc_configure(cie,6,&config)!=DBT_OK)return 1;
    if(!decoder.hooks.allocate){
        dbt_sbc_decoder_hooks_t hooks={0,allocate,release,pcm};
        if(dbt_sbc_decoder_init(&decoder,hooks)!=DBT_OK)return 2;
    }
    dbt_sbc_media_reset(&stream);
    dbt_result_t result=dbt_sbc_decoder_open(&decoder,config);
    if(result!=DBT_OK)return (int)result;
    dbt_sbc_media_hooks_t hooks={&decoder,allocate,release,dbt_sbc_decoder_frame};
    return (int)dbt_sbc_media_init(&stream,hooks,config,100);
}
int probe_reference_open(unsigned channels){
    memset(reference_data,0,sizeof(reference_data));
    return (int)OI_CODEC_SBC_DecoderReset(&reference,reference_data,sizeof(reference_data),2,(OI_UINT8)channels,FALSE,FALSE);
}
int probe_reference(const uint8_t *frame,unsigned size,int16_t *pcm_data,unsigned *written){
    const OI_BYTE *cursor=frame;OI_UINT32 left=size,bytes=512;
    OI_STATUS status=OI_CODEC_SBC_DecodeFrame(&reference,&cursor,&left,pcm_data,&bytes);
    *written=bytes;return status==OI_OK && !left && cursor==frame+size?0:-1;
}
int probe_decode(const uint8_t *data,unsigned size,int16_t *pcm_data,unsigned *written){
    output=pcm_data;output_bytes=0;dbt_sbc_frame_t frame;
    dbt_result_t result=dbt_sbc_frame_decode(&decoder.config,data,(uint16_t)size,&frame);
    if(result==DBT_OK)result=dbt_sbc_decoder_frame(&decoder,&frame,0);
    *written=output_bytes;output=0;return (int)result;
}
int probe_packet(const uint8_t *data,unsigned size,unsigned sequence,unsigned timestamp,int16_t *pcm_data,unsigned *written){
    output=pcm_data;output_bytes=0;
    dbt_avdtp_media_t packet={data,timestamp,1,(uint16_t)size,(uint16_t)sequence,96,false};
    dbt_result_t result=dbt_sbc_media_receive(&stream,&packet,0);
    *written=output_bytes;output=0;return (int)result;
}
size_t probe_live(void){return live;}
size_t probe_peak(void){return peak;}
size_t probe_workspace(void){return decoder.allocated_bytes;}
void probe_peak_reset(void){peak=live;}
void probe_fail_next(void){fail_next=1;}
int probe_worker(const uint8_t *cie,const uint8_t *frame,unsigned size,int16_t *pcm_data,unsigned *written){
    if(live)return 20;
    dbt_sbc_worker_hooks_t hooks={0};hooks.decoder=(dbt_sbc_decoder_hooks_t){0,allocate,release,pcm};
    if(dbt_sbc_worker_init(&worker,hooks,672,2)!=DBT_OK || live)return 21;
    uint8_t configuration[10]={1,0,7,6};memcpy(configuration+4,cie,6);
    dbt_sink_job_t job={123,1,DBT_SINK_CONFIGURE,1,configuration,10};dbt_sbc_receipt_t receipt;
    if(dbt_sbc_worker_submit(&worker,&job)!=DBT_OK || live)return 22;
    (void)dbt_sbc_worker_step(&worker,0,2);
    if(dbt_sbc_worker_receipt(&worker,&receipt)!=DBT_OK || receipt.result!=DBT_SINK_WORK_READY || !live)return 23;
    job.operation=DBT_SINK_START;job.token=2;
    if(dbt_sbc_worker_submit(&worker,&job)!=DBT_OK)return 24;
    job.operation=DBT_SINK_SUSPEND;job.token=3;
    if(dbt_sbc_worker_submit(&worker,&job)!=DBT_OK)return 25;
    (void)dbt_sbc_worker_step(&worker,0,2);
    if(dbt_sbc_worker_receipt(&worker,&receipt)!=DBT_OK || dbt_sbc_worker_receipt(&worker,&receipt)!=DBT_OK)return 26;
    uint8_t packet[525];packet[0]=1;memcpy(packet+1,frame,size);
    dbt_avdtp_media_t media={packet,1000,1,(uint16_t)(size+1),1,96,false};
    if(dbt_sbc_worker_media(&worker,&media)!=DBT_BUSY)return 27;
    job.operation=DBT_SINK_START;job.token=4;
    if(dbt_sbc_worker_submit(&worker,&job)!=DBT_OK)return 28;
    (void)dbt_sbc_worker_step(&worker,0,2);
    if(dbt_sbc_worker_receipt(&worker,&receipt)!=DBT_OK || receipt.result!=DBT_SINK_WORK_READY)return 29;
    if(dbt_sbc_worker_media(&worker,&media)!=DBT_OK)return 30;
    ++media.sequence;if(dbt_sbc_worker_media(&worker,&media)!=DBT_OK)return 31;
    ++media.sequence;if(dbt_sbc_worker_media(&worker,&media)!=DBT_NO_SPACE)return 32;
    output=pcm_data;output_bytes=0;
    if(dbt_sbc_worker_step(&worker,0,2)!=2)return 33;
    *written=output_bytes;output=0;
    if(worker.decoder.frames!=2 || atomic_load(&worker.packets)!=2 || live!=atomic_load(&worker.resident_bytes))return 34;
    /* Incomplete fragment allocates only the current frame, then DROP must free
     * it along with the workspace and encoded queue before its receipt. */
    unsigned chunk=(size+2)/3;packet[0]=0xc3;memcpy(packet+1,frame,chunk);
    media.size=(uint16_t)(chunk+1);
    if(dbt_sbc_worker_media(&worker,&media)!=DBT_OK || dbt_sbc_worker_step(&worker,1,1)!=1)return 35;
    if(!worker.media.fragment || live!=atomic_load(&worker.resident_bytes))return 36;
    job.operation=DBT_SINK_DROP;job.token=5;
    if(dbt_sbc_worker_submit(&worker,&job)!=DBT_OK)return 37;
    if(dbt_sbc_worker_media(&worker,&media)!=DBT_BUSY)return 38;
    (void)dbt_sbc_worker_step(&worker,2,2);
    if(live || dbt_sbc_worker_receipt(&worker,&receipt)!=DBT_OK || receipt.token!=5 || receipt.result!=DBT_SINK_WORK_READY)return 39;
    return 0;
}
#ifdef SBC_THREAD_TEST
static double seconds(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec/1e9;}
static void *decoder_thread(void *unused){
    (void)unused;while(!atomic_load(&stress_stop)){(void)dbt_sbc_worker_step(&worker,1,2);sched_yield();}return 0;
}
static int job_receipt(dbt_sink_job_t *job,double deadline){
    dbt_result_t result;
    while((result=dbt_sbc_worker_submit(&worker,job))==DBT_BUSY){if(seconds()>deadline)return 1;sched_yield();}
    if(result!=DBT_OK)return 2;
    dbt_sbc_receipt_t receipt;
    while(dbt_sbc_worker_receipt(&worker,&receipt)==DBT_EMPTY){if(seconds()>deadline)return 3;sched_yield();}
    return receipt.token!=job->token || receipt.result!=DBT_SINK_WORK_READY?4:0;
}
static int worker_threads(const uint8_t *cie,const uint8_t *frame,unsigned size){
    dbt_sbc_worker_hooks_t hooks={0};hooks.decoder=(dbt_sbc_decoder_hooks_t){0,allocate,release,pcm};
    if(dbt_sbc_worker_init(&worker,hooks,672,4)!=DBT_OK)return 1;
    stress_mode=true;atomic_init(&stress_stop,false);atomic_init(&stress_frames,0);
    pthread_t thread;if(pthread_create(&thread,0,decoder_thread,0))return 2;
    uint8_t config[10]={1,0,7,6};memcpy(config+4,cie,6);double deadline=seconds()+15;
    dbt_sink_job_t job={123,1,DBT_SINK_CONFIGURE,1,config,10};int error=job_receipt(&job,deadline);
    job.token=2;job.operation=DBT_SINK_START;if(!error)error=job_receipt(&job,deadline);
    uint8_t payload[525];payload[0]=1;memcpy(payload+1,frame,size);
    for(unsigned n=0;n<500 && !error;++n){
        dbt_avdtp_media_t m={payload,n*128u,1,(uint16_t)(size+1),(uint16_t)(65500u+n),96,false};
        dbt_result_t status;
        while((status=dbt_sbc_worker_media(&worker,&m))==DBT_NO_SPACE){if(seconds()>deadline){error=5;break;}sched_yield();}
        if(!error && status!=DBT_OK)error=6;
    }
    while(!error && atomic_load(&worker.packets)!=500){if(seconds()>deadline){error=7;break;}sched_yield();}
    job.token=3;job.operation=DBT_SINK_DROP;if(!error)error=job_receipt(&job,deadline);
    atomic_store(&stress_stop,true);pthread_join(thread,0);stress_mode=false;
    if(!error && (live || atomic_load(&stress_frames)!=500))error=8;
    return error;
}
#endif

#ifdef SBC_SANITIZER_MAIN
int main(int argc,char **argv){
    if(argc!=2)return 1;
    FILE *file=fopen(argv[1],"rb");if(!file)return 2;
    uint8_t encoded[65536];size_t length=fread(encoded,1,sizeof(encoded),file);fclose(file);
    if(length<4)return 3;
    uint8_t f=encoded[1],pool=encoded[2];unsigned mode=(f>>2)&3u,channels=mode?2u:1u;
    uint8_t cie[]={0,0,(uint8_t)((0x80u>>(f>>6))|(8u>>mode)),
        (uint8_t)((0x80u>>((f>>4)&3u))|((f&1u)?4u:8u)|((f&2u)?2u:1u)),2,pool};
    if(probe_open(cie) || probe_reference_open(channels))return 4;
    unsigned offset=0;int16_t actual[4096],expected[4096];
    while(offset<length){
        dbt_sbc_frame_t frame;
        if(dbt_sbc_frame_decode(&decoder.config,encoded+offset,(uint16_t)(length-offset),&frame)!=DBT_OK)return 5;
        unsigned a=0,b=0;
        int candidate_status=probe_decode(frame.data,frame.size,actual,&a);
        int reference_status=probe_reference(frame.data,frame.size,expected,&b);
        if(candidate_status || reference_status || a!=b || memcmp(actual,expected,a)){
            unsigned difference=0;while(difference<a/2 && difference<b/2 && actual[difference]==expected[difference])++difference;
            fprintf(stderr,"SBC comparison offset=%u candidate=%d reference=%d bytes=%u/%u first_sample=%u values=%d/%d\n",
                    offset,candidate_status,reference_status,a,b,difference,actual[difference],expected[difference]);return 6;
        }
        offset+=frame.size;
    }
    probe_close();if(live)return 7;
    dbt_sbc_config_t config;dbt_sbc_frame_t first;
    if(dbt_sbc_configure(cie,6,&config)!=DBT_OK || dbt_sbc_frame_decode(&config,encoded,(uint16_t)length,&first)!=DBT_OK)return 8;
    unsigned produced=0;if(probe_worker(cie,first.data,first.size,actual,&produced) || !produced)return 9;
#ifdef SBC_THREAD_TEST
    int thread_result=worker_threads(cie,first.data,first.size);
    if(thread_result){fprintf(stderr,"SBC worker two-thread test failed %d\n",thread_result);return 10;}
#endif
    puts("SBC PCM reference comparison and allocation cleanup passed");return 0;
}
#endif
