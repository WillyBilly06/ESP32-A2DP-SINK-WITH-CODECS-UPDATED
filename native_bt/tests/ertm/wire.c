#include "dbt_ertm.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
static void zero(void *p,unsigned n){uint8_t *v=p;for(unsigned i=0;i<n;++i)v[i]=0;}
static bool equal(const void *a,const void *b,unsigned n){const uint8_t *x=a,*y=b;for(unsigned i=0;i<n;++i)if(x[i]!=y[i])return false;return true;}
/* Independent bit-serial polynomial oracle, not the nibble implementation. */
static uint16_t crc(uint16_t v,const uint8_t *p,unsigned n){for(unsigned i=0;i<n;++i){v^=p[i];for(unsigned bit=0;bit<8;++bit)v=(uint16_t)((v>>1)^((v&1u)?0xa001u:0u));}return v;}
EXPORT int ertm_wire_golden(void){
    const uint8_t information[]={2,0,0,1,2,3,4,5,6,7,8,9,0x38,0x61};
    const uint8_t ready[]={1,1,0xd4,0x14};uint8_t output[32];uint16_t size=0;
    dbt_ertm_frame_t f;zero(&f,sizeof(f));f.data=information+2;f.size=10;f.tx_seq=1;
    CHECK(dbt_ertm_frame_encode(0x40,true,&f,output,sizeof(output),&size)==DBT_OK && size==14 && equal(information,output,14));
    dbt_ertm_frame_t parsed;zero(&parsed,sizeof(parsed));
    CHECK(dbt_ertm_frame_decode(0x40,true,information,sizeof(information),&parsed)==DBT_ERTM_WIRE_OK && parsed.tx_seq==1 && parsed.size==10 && !parsed.supervisory && equal(parsed.data,information+2,10));
    zero(&f,sizeof(f));f.supervisory=true;f.req_seq=1;
    CHECK(dbt_ertm_frame_encode(0x40,true,&f,output,sizeof(output),&size)==DBT_OK && size==4 && equal(ready,output,4));
    CHECK(dbt_ertm_frame_decode(0x40,true,ready,4,&parsed)==DBT_ERTM_WIRE_OK && parsed.supervisory && parsed.req_seq==1 && parsed.supervisor==DBT_ERTM_RR);
    CHECK(dbt_ertm_frame_decode(0x41,true,ready,4,&parsed)==DBT_ERTM_WIRE_BAD_FCS);
    for(unsigned at=0;at<sizeof(information);++at)for(unsigned bit=0;bit<8;++bit){
        for(unsigned i=0;i<sizeof(information);++i){output[i]=information[i];}
        output[at]^=(uint8_t)(1u<<bit);
        CHECK(dbt_ertm_frame_decode(0x40,true,output,sizeof(information),&parsed)==DBT_ERTM_WIRE_BAD_FCS);
    }
    return 0;
}
EXPORT int ertm_wire_all_enhanced_fields(void){
    uint8_t payload[97],out[110],header[4];for(unsigned i=0;i<97;++i)payload[i]=(uint8_t)(i*73u+19u);
    for(unsigned fields=0;fields<65536u;++fields){
        dbt_ertm_frame_t f,p;zero(&f,sizeof(f));f.tx_seq=(uint8_t)(fields&63u);f.req_seq=(uint8_t)((fields>>6)&63u);
        f.sar=(uint8_t)((fields>>12)&3u);f.final=(fields&0x4000u)!=0;bool fcs=(fields&0x8000u)!=0;
        f.data=payload;f.size=(uint16_t)(fields%98u);f.sdu_length=1000;uint16_t size=0;
        CHECK(dbt_ertm_frame_encode(0x97,fcs,&f,out,sizeof(out),&size)==DBT_OK);
        unsigned expected=((unsigned)f.sar<<14)|((unsigned)f.req_seq<<8)|(f.final?128u:0u)|((unsigned)f.tx_seq<<1);
        CHECK(out[0]==(uint8_t)expected && out[1]==(uint8_t)(expected>>8));
        if(fcs){header[0]=(uint8_t)size;header[1]=(uint8_t)(size>>8);header[2]=0x97;header[3]=0;
            uint16_t sum=crc(crc(0,header,4),out,size-2u);CHECK(out[size-2u]==(uint8_t)sum && out[size-1u]==(uint8_t)(sum>>8));}
        CHECK(dbt_ertm_frame_decode(0x97,fcs,out,size,&p)==DBT_ERTM_WIRE_OK);
        CHECK(!p.supervisory && p.tx_seq==f.tx_seq && p.req_seq==f.req_seq && p.sar==f.sar && p.final==f.final && p.size==f.size && equal(p.data,payload,p.size));
        if(f.sar==DBT_ERTM_START)CHECK(p.sdu_length==1000);
    }
    for(unsigned super=0;super<4;++super)for(unsigned req=0;req<64;++req)for(unsigned flags=0;flags<3;++flags)for(unsigned fcs=0;fcs<2;++fcs){
        dbt_ertm_frame_t f,p;zero(&f,sizeof(f));f.supervisory=true;f.supervisor=(uint8_t)super;f.req_seq=(uint8_t)req;f.poll=flags==1;f.final=flags==2;uint16_t size=0;
        CHECK(dbt_ertm_frame_encode(0x80,fcs!=0,&f,out,sizeof(out),&size)==DBT_OK && size==(fcs?4:2));
        CHECK(out[0]==(uint8_t)(1u|(super<<2)|(f.poll?16u:0u)|(f.final?128u:0u)) && out[1]==req);
        CHECK(dbt_ertm_frame_decode(0x80,fcs!=0,out,size,&p)==DBT_ERTM_WIRE_OK && p.supervisory && p.supervisor==super && p.req_seq==req && p.poll==f.poll && p.final==f.final);
    }
    return 0;
}
EXPORT int ertm_wire_transactional_bounds(void){
    dbt_ertm_frame_t f,old;zero(&f,sizeof(f));uint8_t output[8];for(unsigned i=0;i<8;++i)output[i]=0xa5;uint16_t size=999;
    f.tx_seq=64;CHECK(dbt_ertm_frame_encode(0x40,true,&f,output,8,&size)==DBT_INVALID && size==999 && output[0]==0xa5);
    f.tx_seq=0;f.size=1;CHECK(dbt_ertm_frame_encode(0x40,true,&f,output,8,&size)==DBT_INVALID);
    f.data=output;f.size=65535;f.sar=DBT_ERTM_START;CHECK(dbt_ertm_frame_encode(0x40,true,&f,output,65535,&size)==DBT_NO_SPACE && size==999 && output[0]==0xa5);
    f.size=0;CHECK(dbt_ertm_frame_encode(0x40,true,&f,output,5,&size)==DBT_NO_SPACE && output[0]==0xa5);
    zero(&f,sizeof(f));f.supervisory=f.poll=f.final=true;CHECK(dbt_ertm_frame_encode(0x40,false,&f,output,8,&size)==DBT_INVALID);
    f.poll=false;CHECK(dbt_ertm_frame_encode(0x3f,false,&f,output,8,&size)==DBT_INVALID);
    zero(&old,sizeof(old));old.tx_seq=33;f=old;
    uint8_t short_start[]={0,0x40,0};CHECK(dbt_ertm_frame_decode(0x40,false,short_start,3,&f)==DBT_ERTM_WIRE_INVALID && equal(&f,&old,sizeof(f)));
    uint8_t extra_s[]={1,0,9};CHECK(dbt_ertm_frame_decode(0x40,false,extra_s,3,&f)==DBT_ERTM_WIRE_INVALID && equal(&f,&old,sizeof(f)));
    uint8_t poll_final[]={0x91,0};CHECK(dbt_ertm_frame_decode(0x40,false,poll_final,2,&f)==DBT_ERTM_WIRE_INVALID && equal(&f,&old,sizeof(f)));
    CHECK(dbt_ertm_frame_decode(0x40,true,extra_s,3,&f)==DBT_ERTM_WIRE_INVALID && equal(&f,&old,sizeof(f)));
    return 0;
}
