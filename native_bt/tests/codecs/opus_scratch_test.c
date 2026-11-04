#include "dbt_opus_scratch.h"
#include "opus.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FORCE_MODE_REQUEST 11002
#define SILK_MODE 1000
#define HYBRID_MODE 1001
#define CELT_MODE 1002

typedef struct { OpusEncoder *enc; OpusDecoder *dec; int samples, channels, len, resets;
    unsigned char packet[4000]; opus_int16 pcm[1920*2]; } job;
static int encode(void *v) {
    job *j=v; return opus_encode(j->enc,j->pcm,j->samples,j->packet,sizeof(j->packet));
}
static int decode(void *v) {
    job *j=v; return opus_decode(j->dec,j->len ? j->packet : NULL,j->len,j->pcm,j->samples,0);
}
static int reset(void *v) {
    job *j=v; j->resets++; memset(j->pcm,0,sizeof(j->pcm));
    return opus_decoder_ctl(j->dec,OPUS_RESET_STATE);
}
static int noop_reset(void *v) { (void)v; return 0; }
static void *fail_malloc(size_t n) { (void)n; return NULL; }
static unsigned pcm_hash(const job *j, int samples) {
    unsigned h=2166136261u;
    for (int i=0;i<samples*j->channels;i++) { h^=(unsigned short)j->pcm[i]; h*=16777619u; }
    return h;
}
static void print_pcm(const job *j, int samples) {
    fputs("pcm:",stdout);
    for (int i=0;i<samples*j->channels;i++) printf("%04x",(unsigned short)j->pcm[i]);
    putchar('\n');
}
static int overflow(void *v) { (void)v; dbt_opus_scratch_push(SIZE_MAX,2,4); return 0; }
static int failed_reset(void *v) { (void)v; return -1; }
static void compare_reset_to_fresh(dbt_opus_scratch *a, job *j) {
    job fresh={0}; int error, actual, expected;
    fresh.samples=j->samples; fresh.channels=j->channels; fresh.len=j->len;
    memcpy(fresh.packet,j->packet,(size_t)j->len);
    fresh.dec=opus_decoder_create(48000,j->channels,&error);
    assert(fresh.dec && error==0);
    assert(dbt_opus_scratch_run(a,decode,j,reset,j,&actual)==0);
    assert(dbt_opus_scratch_run(a,decode,&fresh,reset,&fresh,&expected)==0);
    assert(actual==j->samples && actual==expected);
    assert(memcmp(j->pcm,fresh.pcm,(size_t)actual*j->channels*sizeof(j->pcm[0]))==0);
    opus_decoder_destroy(fresh.dec);
}
static int nested(void *v) {
    dbt_opus_scratch *s=v; size_t m=dbt_opus_scratch_mark();
    int result=99;
    assert(dbt_opus_scratch_push(32,1,16));
    assert(dbt_opus_scratch_run(s,nested,s,noop_reset,NULL,&result)==DBT_OPUS_SCRATCH_BUSY);
    dbt_opus_scratch_restore(m); assert(s->used==m); return 123;
}
int main(void) {
    dbt_opus_scratch a={0}, tiny={0}, limited={0}, failed={0}; int r, out;
    assert(dbt_opus_scratch_init(&failed,100,fail_malloc,free)==DBT_OPUS_SCRATCH_EXHAUSTED);
    assert(dbt_opus_scratch_init(&a,262144,malloc,free)==0);
    assert(dbt_opus_scratch_init(&a,1,malloc,free)==DBT_OPUS_SCRATCH_BAD_ARGUMENT);
    assert(dbt_opus_scratch_run(&a,nested,&a,noop_reset,NULL,&out)==0 && out==123);
    assert(dbt_opus_scratch_run(&a,overflow,NULL,failed_reset,NULL,&out)==DBT_OPUS_SCRATCH_RESET_FAILED);
    assert(dbt_opus_scratch_init(&tiny,32,malloc,free)==0);
    assert(dbt_opus_scratch_init(&limited,5000,malloc,free)==0);
    for (int ch=1;ch<=2;ch++) for(int samples=120;samples<=1920;samples*=2) {
        job j={0}; j.samples=samples; j.channels=ch;
        j.enc=opus_encoder_create(48000,ch,OPUS_APPLICATION_AUDIO,&r); assert(j.enc && r==0);
        j.dec=opus_decoder_create(48000,ch,&r); assert(j.dec && r==0);
        for (int i=0;i<samples*ch;i++) j.pcm[i]=(opus_int16)((i*97)%20000-10000);
        assert(opus_encoder_ctl(j.enc,FORCE_MODE_REQUEST,CELT_MODE)==0);
        assert(dbt_opus_scratch_run(&a,encode,&j,noop_reset,NULL,&out)==0 && out>2);
        j.len=out; a.peak=0;
        assert(dbt_opus_scratch_run(&a,decode,&j,reset,&j,&out)==0 && out==samples);
        printf("CELT ch=%d samples=%d hash=%08x\n",ch,samples,pcm_hash(&j,out));
        print_pcm(&j,out);
        j.len=0;
        assert(dbt_opus_scratch_run(&a,decode,&j,reset,&j,&out)==0 && out==samples);
        printf("PLC ch=%d samples=%d hash=%08x\n",ch,samples,pcm_hash(&j,out));
        print_pcm(&j,out);
        opus_decoder_destroy(j.dec); opus_encoder_destroy(j.enc);
    }
    for (int ch=1;ch<=2;ch++) for (int samples=480;samples<=1920;samples*=2) {
        job j={0}; j.samples=samples; j.channels=ch;
        j.enc=opus_encoder_create(48000,ch,OPUS_APPLICATION_VOIP,&r); assert(j.enc && !r);
        j.dec=opus_decoder_create(48000,ch,&r); assert(j.dec && !r);
        for (int mode=SILK_MODE;mode<=CELT_MODE;mode++) {
            assert(opus_encoder_ctl(j.enc,FORCE_MODE_REQUEST,mode)==0);
            assert(opus_encoder_ctl(j.enc,OPUS_SET_FORCE_CHANNELS(ch))==0);
            assert(opus_encoder_ctl(j.enc,OPUS_SET_BITRATE(mode==SILK_MODE?16000:48000))==0);
            assert(opus_encoder_ctl(j.enc,OPUS_SET_BANDWIDTH(mode==SILK_MODE?OPUS_BANDWIDTH_WIDEBAND:OPUS_BANDWIDTH_FULLBAND))==0);
            for (int n=0;n<5;n++) {
                for (int i=0;i<j.samples*ch;i++) j.pcm[i]=(opus_int16)(((i+n*11)*53)%24000-12000);
                assert(dbt_opus_scratch_run(&a,encode,&j,noop_reset,NULL,&out)==0 && out>2);
                j.len=out; a.peak=0;
                assert(dbt_opus_scratch_run(&a,decode,&j,reset,&j,&out)==0 && out==j.samples);
                if (n==4) {
                    unsigned toc=j.packet[0]>>3;
                    assert(mode==SILK_MODE ? toc<12 : mode==HYBRID_MODE ? toc>=12 && toc<16 : toc>=16);
                    assert(((j.packet[0] & 4)!=0)==(ch==2));
                }
                printf("mode=%d ch=%d toc=%02x hash=%08x\n",mode,ch,j.packet[0],pcm_hash(&j,out));
                print_pcm(&j,out);
            }
            j.len=0;
            assert(dbt_opus_scratch_run(&a,decode,&j,reset,&j,&out)==0 && out==j.samples);
            printf("modePLC=%d ch=%d hash=%08x\n",mode,ch,pcm_hash(&j,out));
            print_pcm(&j,out);
        }
        /* Force failure during a real decode; output and decoder are reset. */
        assert(opus_encoder_ctl(j.enc,FORCE_MODE_REQUEST,CELT_MODE)==0);
        assert(dbt_opus_scratch_run(&a,encode,&j,noop_reset,NULL,&out)==0 && out>2); j.len=out;
        memset(j.pcm,0x55,sizeof(j.pcm)); out=991;
        #ifndef OPUS_SCRATCH_REFERENCE
        assert(dbt_opus_scratch_run(&tiny,decode,&j,reset,&j,&out)==DBT_OPUS_SCRATCH_EXHAUSTED);
        assert(out==991 && j.pcm[0]==0 && tiny.used==0 && j.resets==1);
        compare_reset_to_fresh(&a,&j);
        if (samples>=960) {
            out=992;
            assert(dbt_opus_scratch_run(&limited,decode,&j,reset,&j,&out)==DBT_OPUS_SCRATCH_EXHAUSTED);
            assert(out==992 && j.pcm[0]==0 && limited.used==0 && j.resets==2);
            compare_reset_to_fresh(&a,&j);
        }
        #endif
        opus_decoder_destroy(j.dec); opus_encoder_destroy(j.enc);
    }
    assert(dbt_opus_scratch_destroy(&tiny)==0);
    assert(dbt_opus_scratch_destroy(&limited)==0);
    assert(dbt_opus_scratch_destroy(&a)==0);
    puts("opus scratch tests passed"); return 0;
}
