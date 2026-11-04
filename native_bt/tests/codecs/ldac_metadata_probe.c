#include "dbt_ldac.h"
#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
EXPORT int test_ldac_metadata(void){
    const uint8_t ratebits[]={0x20,0x10,8,4},modes[]={4,2,1};
    const uint32_t rates[]={44100,48000,88200,96000};
    uint8_t cie[]={0,255,0x2d,1,0,0,0xaa,0,0,0};
    for(unsigned r=0;r<4;++r)for(unsigned m=0;m<3;++m){
        cie[8]=ratebits[r];cie[9]=modes[m];dbt_codec_config_t c;
        CHECK(dbt_ldac_configure(cie,10,&c)==DBT_OK);
        CHECK(c.rate==rates[r] && c.channels==(m==0?1:2) && c.samples==(r<2?128:256) && c.sample_bytes==4);
        uint8_t packet[51]={0};packet[0]=2;
        for(unsigned f=0;f<2;++f){unsigned off=1+f*25;packet[off]=0xaa;packet[off+1]=(uint8_t)((r<<5)|(m<<3));packet[off+2]=(uint8_t)(21<<2);}
        dbt_ldac_frame_t output[2]={{65535,65535},{65535,65535}};uint8_t count=77;
        CHECK(dbt_ldac_packet_parse(&c,packet,sizeof(packet),output,1,&count)==DBT_NO_SPACE && count==77 && output[0].offset==65535);
        CHECK(dbt_ldac_packet_parse(&c,packet,sizeof(packet),output,2,&count)==DBT_OK && count==2);
        CHECK(output[0].offset==1 && output[0].size==25 && output[1].offset==26 && output[1].size==25);
        count=77;output[0].offset=65535;
        CHECK(dbt_ldac_packet_parse(&c,packet,50,output,2,&count)==DBT_INVALID && count==77 && output[0].offset==65535);
        packet[27]^=0x20;CHECK(dbt_ldac_packet_parse(&c,packet,51,output,2,&count)==DBT_INVALID && count==77);
        packet[0]=0x81;CHECK(dbt_ldac_packet_parse(&c,packet,51,output,2,&count)==DBT_UNSUPPORTED);
    }
    cie[8]=2;cie[9]=1;dbt_codec_config_t sentinel={0};sentinel.rate=77;
    CHECK(dbt_ldac_configure(cie,10,&sentinel)==DBT_UNSUPPORTED && sentinel.rate==77);
    cie[8]=0x10;cie[9]=4;CHECK(dbt_ldac_configure(cie,10,&sentinel)==DBT_OK);
    uint8_t mono[15]={1,0xaa,0x20,40};dbt_ldac_frame_t frame;uint8_t count=0;
    CHECK(dbt_ldac_packet_parse(&sentinel,mono,sizeof(mono),&frame,1,&count)==DBT_OK && frame.size==14);
    cie[8]=0x10;uint8_t caps[]={1,0,7,10,0,255,0x2d,1,0,0,0xaa,0,0x30,3};
    cie[9]=1;
    dbt_avdtp_endpoint_t endpoint={caps,sizeof(caps),1,dbt_a2dp_ldac_validate};
    dbt_avdtp_cap_t cap={cie,7,10};CHECK(dbt_a2dp_ldac_validate(&endpoint,&cap)==0);
    caps[12]=0x20;CHECK(dbt_a2dp_ldac_validate(&endpoint,&cap)!=0);
    return 0;
}
