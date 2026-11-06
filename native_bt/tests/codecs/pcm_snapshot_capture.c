#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "dbt_pcm_snapshot.h"
int main(int argc,char **argv){
    (void)argv;uint8_t context[4404]={0},encoded[1800]={0};int32_t pcm[256],copied[256];fixed_dsp_t dsp;
    for(unsigned i=0;i<256;++i)pcm[i]=(int32_t)(i*1024u);
    assert(!dbt_pcm_snapshot_wanted(true,0));dbt_pcm_snapshot_arm();
    assert(!dbt_pcm_snapshot_wanted(false,0));assert(!dbt_pcm_snapshot_wanted(true,UINT32_MAX-1000u));
    assert(!dbt_pcm_snapshot_wanted(true,1498));assert(dbt_pcm_snapshot_wanted(true,1499));
    assert(!dbt_pcm_snapshot_decode_begin(context,sizeof(context),123,encoded,1801,pcm,48000,1499));
    assert(dbt_pcm_snapshot_decode_begin(context,sizeof(context),123,encoded,sizeof(encoded),pcm,48000,1499));
    dbt_pcm_snapshot_decode_end(192,sizeof(pcm));memset(context,0xA5,sizeof(context));
    assert(!dbt_pcm_snapshot_wanted(true,4000));
    if(argc>1){dbt_pcm_snapshot_retire();dbt_pcm_snapshot_handoff(pcm,128,8,pcm,512,pcm+128,512);dbt_pcm_snapshot_log(true);return 0;}
    dbt_pcm_snapshot_handoff(encoded,128,8,pcm,512,pcm+128,512);
    memcpy(copied,pcm,sizeof(pcm));dbt_pcm_snapshot_handoff(pcm,128,8,copied,504,(uint8_t*)copied+504,520);
    assert(fixed_dsp_init(&dsp,48000,90,500,1024));
    assert(!dbt_pcm_snapshot_dsp_begin(&dsp,pcm,127,false));
    assert(dbt_pcm_snapshot_dsp_begin(&dsp,pcm,128,false));assert(fixed_dsp_process_block(&dsp,pcm,128));
    dbt_pcm_snapshot_dsp_end(pcm);dbt_pcm_snapshot_final(&dsp);
    assert(!dbt_pcm_snapshot_dsp_begin(&dsp,pcm,128,false));
    dbt_pcm_snapshot_log(false);
    for(unsigned i=0;i<110;++i)dbt_pcm_snapshot_log(true);
    return 0;
}
