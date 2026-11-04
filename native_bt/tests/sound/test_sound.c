#include "dbt_sound.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define ASSERT(x) do {if(!(x)){fprintf(stderr,"assertion %d: %s\n",__LINE__,#x);return 1;}}while(0)
typedef struct {
    uint8_t files[4][8192],tmp[8192];uint32_t lengths[4],tmp_size,now;
    dbt_sound_t *sound;dbt_sound_peer_t peer;bool valid,started,muted,output_exclusive;
    int fail_write,corrupt,fail_seal,fail_commit,fail_mute,cancel_begin,cancel_write,cancel_commit,alloc_fail,output_fail,fail_read;
    dbt_result_t cancel_result;unsigned starts,stops;
    unsigned allocations;
} fixture_t;
static bool begin(void *v,uint8_t type,uint32_t n){fixture_t *f=v;(void)type;if(n>sizeof(f->tmp))return false;f->tmp_size=0;f->started=true;if(f->cancel_begin)f->cancel_result=dbt_sound_disconnect(f->sound,f->peer);return true;}
static bool write_file(void *v,uint32_t off,const uint8_t *p,uint16_t n){fixture_t *f=v;if(f->fail_write || !f->started || off!=f->tmp_size || off+n>sizeof(f->tmp))return false;memcpy(f->tmp+off,p,n);f->tmp_size+=n;if(f->cancel_write)f->cancel_result=dbt_sound_disconnect(f->sound,f->peer);return true;}
static bool seal(void *v){fixture_t *f=v;return f->started && !f->fail_seal;}
static bool read_temp(void *v,uint32_t off,uint8_t *p,uint16_t n){fixture_t *f=v;if(off+n>f->tmp_size)return false;memcpy(p,f->tmp+off,n);if(f->corrupt && n)p[0]^=1;return true;}
static bool commit(void *v,uint8_t type){fixture_t *f=v;if(f->cancel_commit)f->cancel_result=dbt_sound_disconnect(f->sound,f->peer);if(f->fail_commit)return false;memcpy(f->files[type],f->tmp,f->tmp_size);f->lengths[type]=f->tmp_size;f->started=false;return true;}
static void abort_file(void *v){fixture_t *f=v;f->started=false;f->tmp_size=0;}
static dbt_result_t size_of(void *v,uint8_t t,uint32_t *n){fixture_t *f=v;if(!f->lengths[t])return DBT_EMPTY;*n=f->lengths[t];return DBT_OK;}
static bool read_file(void *v,uint8_t t,uint32_t off,uint8_t *p,uint16_t n){fixture_t *f=v;if(f->fail_read || off+n>f->lengths[t])return false;memcpy(p,f->files[t]+off,n);return true;}
static bool remove_file(void *v,uint8_t t){fixture_t *f=v;if(!f->lengths[t])return false;f->lengths[t]=0;return true;}
static bool mute(void *v,bool b){fixture_t *f=v;if(f->fail_mute)return false;f->muted=b;return true;}
static uint32_t clock_ms(void *v){return ((fixture_t*)v)->now;}
static bool peer_valid(void *v,dbt_sound_peer_t p){fixture_t *f=v;return f->valid && p.handle==f->peer.handle && p.generation==f->peer.generation;}
static bool output_start(void *v,bool exclusive){fixture_t *f=v;if(f->output_fail)return false;f->starts++;f->output_exclusive=exclusive;return true;}
static void output_stop(void *v){((fixture_t*)v)->stops++;}
static void *allocate(void *v,size_t n){fixture_t *f=v;if(f->alloc_fail)return NULL;void *p=malloc(n);if(p)++f->allocations;return p;}
static void release(void *v,void *p){fixture_t *f=v;if(p)--f->allocations;free(p);}
static void put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void put32(uint8_t *p,uint32_t v){put16(p,(uint16_t)v);put16(p+2,(uint16_t)(v>>16));}
static uint32_t wav(uint8_t *p,uint16_t bits,uint16_t channels,uint32_t rate,uint32_t frames){
    uint16_t align=(uint16_t)(bits/8*channels);uint32_t bytes=frames*align,pos=0;
    memcpy(p,"RIFF",4);put32(p+4,0);memcpy(p+8,"WAVEfmt ",8);put32(p+16,16);put16(p+20,1);put16(p+22,channels);put32(p+24,rate);put32(p+28,rate*align);put16(p+32,align);put16(p+34,bits);
    memcpy(p+36,"JUNK",4);put32(p+40,3);p[44]=1;p[45]=2;p[46]=3;p[47]=0;
    memcpy(p+48,"data",4);put32(p+52,bytes);pos=56;
    for(uint32_t i=0;i<frames;i++)for(unsigned c=0;c<channels;c++){
        int16_t x=(int16_t)((i%64<32?1:-1)*(5000+(int)c*1000));uint16_t u=(uint16_t)x;
        if(bits==8)p[pos++]=(uint8_t)((x/256)+128);else if(bits==16){p[pos++]=(uint8_t)u;p[pos++]=(uint8_t)(u>>8);}
        else {p[pos++]=0;p[pos++]=(uint8_t)u;p[pos++]=(uint8_t)(u>>8);}
    }
    if(bytes&1)p[pos++]=0;
    put32(p+4,pos-8);return pos;
}
static int cmd(fixture_t *f,uint8_t command,const uint8_t *data,uint16_t n,dbt_sound_response_t *r){return dbt_sound_command(f->sound,f->peer,command,data,n,f->now,r);}
static int upload(fixture_t *f,uint8_t type,const uint8_t *p,uint32_t n,uint16_t chunk){
    dbt_sound_response_t r;uint8_t start[4]={type,(uint8_t)n,(uint8_t)(n>>8),(uint8_t)(n>>16)};
    if(cmd(f,0x12,start,4,&r)!=DBT_OK || r.data[0]!=0x31)return 0;
    uint32_t at=0;uint8_t seq=0,buf[513];
    while(at<n){uint16_t len=(uint16_t)((n-at)>chunk?chunk:n-at);buf[0]=seq++;memcpy(buf+1,p+at,len);
        if(cmd(f,0x13,buf,len+1,&r)!=DBT_OK || r.data[0]!=0x31)return 0;
        at+=len;}
    return cmd(f,0x14,0,0,&r)==DBT_OK && r.data[0]==0x32;
}
int main(void){
    fixture_t f={0};dbt_sound_t s;f.sound=&s;f.peer=(dbt_sound_peer_t){7,10};f.valid=true;
    dbt_sound_store_t store={&f,begin,write_file,seal,read_temp,commit,abort_file,size_of,read_file,remove_file,mute,clock_ms,peer_valid};
    ASSERT(dbt_sound_init(&s,store,false)==DBT_OK && dbt_sound_status(&s)==0);
    uint8_t input[8192];uint32_t n=wav(input,16,2,48000,512);ASSERT(upload(&f,0,input,n,7)); /* wraps seq8 */
    ASSERT(f.lengths[0]==n && !memcmp(f.files[0],input,n) && dbt_sound_status(&s)==1);
    dbt_sound_response_t r;uint8_t one=1;ASSERT(cmd(&f,0x10,&one,1,&r)==DBT_OK && r.data[0]==0x10 && dbt_sound_status(&s)==0x81);
    f.fail_mute=1;one=0;ASSERT(cmd(&f,0x10,&one,1,&r)==DBT_OK && r.data[0]==0x11 && s.muted);f.fail_mute=0;
    ASSERT(cmd(&f,0x10,&one,1,&r)==DBT_OK && !s.muted);
    uint8_t start[4]={1,(uint8_t)n,(uint8_t)(n>>8),(uint8_t)(n>>16)},part[3]={1,0,0};
    ASSERT(cmd(&f,0x12,start,4,&r)==DBT_OK && r.data[0]==0x31);
    ASSERT(cmd(&f,0x13,part,3,&r)==DBT_OK && r.data[0]==0x33 && !s.active && !f.started);
    ASSERT(cmd(&f,0x12,start,4,&r)==DBT_OK);ASSERT(dbt_sound_disconnect(&s,(dbt_sound_peer_t){7,9})==DBT_EMPTY);
    ASSERT(dbt_sound_disconnect(&s,f.peer)==DBT_OK);dbt_sound_peer_t expired={0};ASSERT(dbt_sound_poll(&s,f.now,&expired) && expired.generation==10);ASSERT(!s.active && !f.started);
    f.cancel_begin=1;ASSERT(cmd(&f,0x12,start,4,&r)==DBT_OK && r.data[0]==0x33 && f.cancel_result==DBT_OK && !s.active);f.cancel_begin=0;
    ASSERT(cmd(&f,0x12,start,4,&r)==DBT_OK);f.cancel_write=1;part[0]=0;ASSERT(cmd(&f,0x13,part,3,&r)==DBT_OK && r.data[0]==0x33 && !s.active);f.cancel_write=0;
    ASSERT(cmd(&f,0x12,start,4,&r)==DBT_OK);f.now+=30000;ASSERT(dbt_sound_poll(&s,f.now,&expired));ASSERT(!s.active);
    f.corrupt=1;ASSERT(!upload(&f,1,input,n,256) && !f.started && !f.lengths[1]);f.corrupt=0;
    f.fail_write=1;ASSERT(!upload(&f,1,input,n,256) && !f.started);f.fail_write=0;
    f.fail_seal=1;ASSERT(!upload(&f,1,input,n,256) && !f.started);f.fail_seal=0;
    f.fail_commit=1;ASSERT(!upload(&f,1,input,n,256) && !f.started);f.fail_commit=0;
    input[0]='X';ASSERT(!upload(&f,1,input,n,256) && !f.started && !f.lengths[1]);input[0]='R';
    f.cancel_commit=1;ASSERT(upload(&f,1,input,n,256) && f.cancel_result==DBT_BUSY);f.cancel_commit=0;
    ASSERT(upload(&f,2,input,wav(input,8,1,44100,513),128));
    ASSERT(upload(&f,3,input,wav(input,24,2,96000,512),255));
    dbt_sound_output_t output={&f,output_start,output_stop};dbt_sound_play_t *play=0;
    dbt_sound_output_t missing={0};ASSERT(dbt_sound_play_open(&s,0,48000,false,missing,&f,allocate,release,&play)==DBT_INVALID && !play);
    f.alloc_fail=1;ASSERT(dbt_sound_play_open(&s,0,48000,false,output,&f,allocate,release,&play)==DBT_NO_SPACE);f.alloc_fail=0;
    f.output_fail=1;ASSERT(dbt_sound_play_open(&s,0,48000,false,output,&f,allocate,release,&play)==DBT_BUSY && !play && !f.allocations);f.output_fail=0;
    ASSERT(dbt_sound_play_open(&s,3,48000,true,output,&f,allocate,release,&play)==DBT_OK && !f.output_exclusive);
    ASSERT(dbt_sound_play_queue(&s,0,48000,true)==DBT_OK && dbt_sound_play_queue(&s,2,96000,true)==DBT_OK);
    ASSERT(!dbt_sound_play_ready(play));ASSERT(dbt_sound_play_pump(play,64)==DBT_OK && !dbt_sound_play_ready(play));
    ASSERT(dbt_sound_play_pump(play,64)==DBT_OK && dbt_sound_play_ready(play));
    int32_t pcm[64*2];unsigned loops=0;bool heard=false;
    while(!dbt_sound_play_drained(play) && loops++<1000){
        dbt_result_t q=dbt_sound_play_pump(play,64);ASSERT(q==DBT_OK || q==DBT_BUSY || q==DBT_EMPTY);
        for(unsigned i=0;i<128;i++)pcm[i]=1000000000;
        dbt_sound_mix(play,pcm,64);for(unsigned i=0;i<128;i++)if(pcm[i]!=1000000000)heard=true;
    }
    ASSERT(dbt_sound_play_drained(play) && heard);dbt_sound_play_close(play);ASSERT(!s.playing && f.starts==f.stops);
    uint8_t next_type;uint32_t next_rate;bool next_exclusive;
    ASSERT(dbt_sound_play_take_queued(&s,&next_type,&next_rate,&next_exclusive) && next_type==2 && next_rate==96000 && next_exclusive);
    ASSERT(dbt_sound_play_open(&s,2,96000,true,output,&f,allocate,release,&play)==DBT_OK && f.output_exclusive);
    ASSERT(dbt_sound_play_pump(play,64)==DBT_OK);memset(pcm,0,sizeof(pcm));dbt_sound_mix(play,pcm,64);ASSERT(pcm[0]==0 && pcm[1]==0);dbt_sound_play_close(play);
    ASSERT(dbt_sound_play_open(&s,0,48000,true,output,&f,allocate,release,&play)==DBT_OK && f.output_exclusive);
    ASSERT(dbt_sound_play_pump(play,64)==DBT_OK);memset(pcm,0,sizeof(pcm));dbt_sound_mix(play,pcm,64);
    ASSERT(pcm[0]==0 && pcm[1]==0 && pcm[2]==680000 && pcm[3]==816000);
    dbt_sound_play_request_stop(play);ASSERT(dbt_sound_play_pump(play,64)==DBT_OK);
    loops=0;while(!dbt_sound_play_drained(play) && loops++<20){memset(pcm,0,sizeof(pcm));dbt_sound_mix(play,pcm,64);}
    ASSERT(dbt_sound_play_drained(play));dbt_sound_play_close(play);
    ASSERT(!f.allocations && f.starts==f.stops);
    ASSERT(dbt_sound_play_open(&s,0,48000,false,output,&f,allocate,release,&play)==DBT_OK);
    f.fail_read=1;ASSERT(dbt_sound_play_pump(play,64)==DBT_INVALID);f.fail_read=0;
    memset(pcm,0,sizeof(pcm));dbt_sound_mix(play,pcm,64);ASSERT(dbt_sound_play_drained(play));dbt_sound_play_close(play);
    one=3;ASSERT(cmd(&f,0x11,&one,1,&r)==DBT_OK && r.data[0]==0x10 && !(dbt_sound_status(&s)&8));
    puts("sound protocol, WAV, lifecycle and mixer tests passed");return 0;
}
