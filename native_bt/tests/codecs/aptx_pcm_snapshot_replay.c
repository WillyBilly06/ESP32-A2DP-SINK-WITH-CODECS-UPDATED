#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freeaptx.h"
#include "dbt_pcm_snapshot.h"
extern size_t dbt_aptx_backend_size(void);
extern uint32_t dbt_aptx_backend_snapshot_schema(void);
static void read_blob(const char *directory,const char *name,void *data,size_t size){
    char p[1024];snprintf(p,sizeof(p),"%s/%s.bin",directory,name);FILE *f=fopen(p,"rb");
    if(!f || fread(data,1,size,f)!=size || fgetc(f)!=EOF){fprintf(stderr,"bad blob %s\n",name);exit(2);}fclose(f);
}
static int compare(const char *stage,const void *expected,const void *actual,size_t size){
    const uint8_t *a=expected,*b=actual;for(size_t i=0;i<size;++i)if(a[i]!=b[i]){
        printf("{\"stage\":\"%s\",\"match\":false,\"first_byte\":%zu,\"expected\":%u,\"actual\":%u}\n",stage,i,a[i],b[i]);return 1;}
    printf("{\"stage\":\"%s\",\"match\":true,\"bytes\":%zu}\n",stage,size);return 0;
}
int main(int argc,char **argv){
    const uint16_t endian=1;
    if(argc!=10 || *(const uint8_t *)&endian!=1)return 2;
    size_t context_size=strtoul(argv[2],0,10),input_size=strtoul(argv[5],0,10),expected_used=strtoul(argv[6],0,10),pcm_size=strtoul(argv[7],0,10);
    uint32_t context_schema=(uint32_t)strtoul(argv[3],0,10),dsp_schema=(uint32_t)strtoul(argv[4],0,10);
    if(sizeof(void*)!=4 || context_size!=dbt_aptx_backend_size() || context_schema!=dbt_aptx_backend_snapshot_schema() || dsp_schema!=dbt_pcm_snapshot_dsp_schema() || sizeof(fixed_dsp_t)!=strtoul(argv[9],0,10)){
        fprintf(stderr,"ABI mismatch: ctx %zu/%zu schema %u/%u DSP %u/%u size %zu\n",context_size,dbt_aptx_backend_size(),context_schema,dbt_aptx_backend_snapshot_schema(),dsp_schema,dbt_pcm_snapshot_dsp_schema(),sizeof(fixed_dsp_t));return 2;}
    if(!input_size || input_size>1800 || !pcm_size || pcm_size>1024 || pcm_size%8)return 2;
    void *context=malloc(context_size);uint8_t input[1800],actual[1024],expected[1024],handoff[1024];size_t written=0;
    read_blob(argv[1],"context",context,context_size);read_blob(argv[1],"encoded",input,input_size);
    size_t used=aptx_decode32(context,input,input_size,actual,sizeof(actual),&written);
    printf("{\"stage\":\"decode_lengths\",\"used\":%zu,\"expected_used\":%zu,\"written\":%zu,\"expected_written\":%zu}\n",used,expected_used,written,pcm_size);
    int failed=used!=expected_used || written!=pcm_size;
    read_blob(argv[1],"decoded",expected,pcm_size);failed|=compare("decoder_target_vs_desktop",expected,actual,pcm_size);
    read_blob(argv[1],"input_ring",handoff,pcm_size);failed|=compare("decoder_to_input_ring",expected,handoff,pcm_size);
    fixed_dsp_t dsp;int32_t dsp_pcm[256],dsp_expected[256],final[256];
    read_blob(argv[1],"dsp_state",&dsp,sizeof(dsp));read_blob(argv[1],"dsp_input",dsp_pcm,sizeof(dsp_pcm));read_blob(argv[1],"dsp_output",dsp_expected,sizeof(dsp_expected));
    if(!fixed_dsp_process_block(&dsp,dsp_pcm,128))return 2;
    failed|=compare("dsp_target_vs_desktop",dsp_expected,dsp_pcm,sizeof(dsp_pcm));
    read_blob(argv[1],"final_output",final,sizeof(final));
    int mix_difference=compare("dsp_to_post_mix",dsp_expected,final,sizeof(final));
    if(!strtoul(argv[8],0,10))failed|=mix_difference;
    free(context);return failed?1:0;
}
