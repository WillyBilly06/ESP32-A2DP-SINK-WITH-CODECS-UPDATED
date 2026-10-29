#include "dbt_sbc_media.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
/* First frame of our FFmpeg-encoded 44.1 kHz stereo silence fixture, SBC 328k.
 * Fixed external encoder bytes exercise CRC independently of our parser. */
static const char silence[]=
"9cb9369700000000000000007776db6eeedb6ddddb6dbbbb6db7776db6eeedb6ddddb6dbbbb6db"
"7776db6eeedb6ddddb6dbbbb6db7776db6eeedb6ddddb6dbbbb6db7776db6eeedb6ddddb6dbbbb6db"
"7776db6eeedb6ddddb6dbbbb6db7776db6eeedb6ddddb6dbbbb6db7776db6eeedb6ddddb6dbbbb6db";
static dbt_sbc_media_t stream;
static dbt_sbc_config_t config;
static uint8_t frame[120],packet[260],arena[526];
static unsigned allocated,released,calls,errors;
static uint32_t timestamp;
static bool used,oom;
static uint16_t allocated_size;
static unsigned hex(char c){return (unsigned)(c<='9'?c-'0':c-'a'+10);}
static void copy(uint8_t *out,const uint8_t *in,unsigned n){for(unsigned i=0;i<n;++i)out[i]=in[i];}
static uint8_t *allocate(void *context,uint16_t size){
    (void)context;if(oom || used || size>524)return 0;
    arena[0]=0x5a;arena[size+1]=0xa5;allocated_size=size;used=true;++allocated;return arena+1;
}
static void release(void *context,uint8_t *p){
    (void)context;if(p!=arena+1 || !used || arena[0]!=0x5a || arena[allocated_size+1u]!=0xa5)++errors;
    used=false;++released;
}
static dbt_result_t receive_frame(void *context,const dbt_sbc_frame_t *f,uint32_t t){
    (void)context;++calls;timestamp=t;if(f->size!=120 || f->samples!=128 || f->channels!=2 || f->bitpool!=54)++errors;
    for(unsigned i=0;i<120;++i)if(f->data[i]!=frame[i]){++errors;break;}
    return DBT_OK;
}
static bool fixture(void){
    for(unsigned i=0;i<sizeof(frame);++i)frame[i]=(uint8_t)((hex(silence[2*i])<<4)|hex(silence[2*i+1]));
    allocated=released=calls=errors=0;used=oom=false;
    const uint8_t cie[]={0,0,0x22,0x15,2,54};
    dbt_sbc_media_hooks_t hooks={0,allocate,release,receive_frame};
    return dbt_sbc_configure(cie,sizeof(cie),&config)==DBT_OK &&
        dbt_sbc_media_init(&stream,hooks,config,100)==DBT_OK;
}
static dbt_avdtp_media_t media(uint16_t size,uint16_t sequence){
    dbt_avdtp_media_t result={packet,1000,0x12345678,size,sequence,96,false};return result;
}
static dbt_result_t fragment(unsigned part,uint16_t sequence,uint32_t now){
    packet[0]=(uint8_t)(part==0?0xc3:part==1?0x82:0xa1);copy(packet+1,frame+40*part,40);
    dbt_avdtp_media_t m=media(41,sequence);return dbt_sbc_media_receive(&stream,&m,now);
}
EXPORT int test_sbc_golden_crc_and_negotiated_format(void){
    CHECK(fixture());CHECK(config.rate==44100 && config.samples==128 && config.format==0xb9 && config.channels==2);
    dbt_sbc_frame_t view={0};CHECK(dbt_sbc_frame_decode(&config,frame,120,&view)==DBT_OK && view.size==120);
    frame[12]^=1;CHECK(dbt_sbc_frame_decode(&config,frame,120,&view)==DBT_OK); /* audio bits are outside SBC header CRC */
    frame[12]^=1;frame[4]^=1;CHECK(dbt_sbc_frame_decode(&config,frame,120,&view)==DBT_INVALID);frame[4]^=1;
    CHECK(dbt_sbc_frame_decode(&config,frame,119,&view)==DBT_INVALID);
    config.maximum_bitpool=53;CHECK(dbt_sbc_frame_decode(&config,frame,120,&view)==DBT_INVALID);
    config.maximum_bitpool=54;config.rate=UINT32_MAX;CHECK(dbt_sbc_frame_decode(&config,frame,120,&view)==DBT_INVALID);
    return 0;
}
EXPORT int test_sbc_packet_atomic_validation_and_timestamps(void){
    CHECK(fixture());packet[0]=2;copy(packet+1,frame,120);copy(packet+121,frame,120);
    dbt_avdtp_media_t m=media(241,10);
    CHECK(dbt_sbc_media_receive(&stream,&m,0)==DBT_OK && calls==2 && timestamp==1128 && !allocated && !errors);
    calls=0;packet[125]^=1;
    CHECK(dbt_sbc_media_receive(&stream,&m,1)==DBT_INVALID && calls==0);packet[125]^=1;
    packet[0]=0x12;CHECK(dbt_sbc_media_receive(&stream,&m,2)==DBT_OK && calls==2); /* ignore RFA */
    m.size=240;CHECK(dbt_sbc_media_receive(&stream,&m,3)==DBT_INVALID);
    return 0;
}
EXPORT int test_sbc_fragment_lifecycle_sequence_wrap_and_lazy_storage(void){
    CHECK(fixture());CHECK(fragment(0,65535,0)==DBT_OK && used && allocated==1 && !calls);
    CHECK(fragment(1,0,1)==DBT_OK && !calls);
    CHECK(fragment(2,1,2)==DBT_OK && calls==1 && !used && allocated==released && !errors && timestamp==1000);
    CHECK(fragment(0,10,3)==DBT_OK);
    packet[0]=1;copy(packet+1,frame,120);dbt_avdtp_media_t m=media(121,11);
    CHECK(dbt_sbc_media_receive(&stream,&m,4)==DBT_OK && !used && calls==2 && stream.gaps==1);
    CHECK(allocated==released && !errors);return 0;
}
EXPORT int test_sbc_fragment_errors_deadlines_and_oom(void){
    CHECK(fixture());oom=true;CHECK(fragment(0,1,0)==DBT_NO_SPACE && !used);oom=false;
    CHECK(fragment(0,1,1)==DBT_OK && fragment(1,3,2)==DBT_INVALID && !used && !calls);
    CHECK(fragment(0,1,3)==DBT_OK);packet[0]=0x82;copy(packet+1,frame+40,40);
    dbt_avdtp_media_t m=media(41,2);m.timestamp++;
    CHECK(dbt_sbc_media_receive(&stream,&m,4)==DBT_INVALID && !used);
    CHECK(fragment(0,1,UINT32_MAX-49u)==DBT_OK);
    dbt_sbc_media_expire(&stream,49);CHECK(used);dbt_sbc_media_expire(&stream,50);CHECK(!used);
    CHECK(fragment(0,1,51)==DBT_OK);dbt_sbc_media_reset(&stream);
    CHECK(allocated==released && !errors);return 0;
}
EXPORT int test_sbc_generated_inputs_keep_output_and_allocations_bounded(void){
    uint32_t random=0x323477u;CHECK(fixture());
    for(unsigned n=0;n<4096;++n){
        for(unsigned j=0;j<sizeof(packet);++j){random=random*1664525u+1013904223u;packet[j]=(uint8_t)(random>>24);}
        dbt_avdtp_media_t m=media((uint16_t)(n%sizeof(packet)),(uint16_t)n);
        (void)dbt_sbc_media_receive(&stream,&m,n);
        dbt_sbc_media_reset(&stream);CHECK(!used && allocated==released);
    }
    CHECK(!errors);return 0;
}
