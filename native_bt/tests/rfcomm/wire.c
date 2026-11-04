#include "dbt_rfcomm.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
static void zero(void *p,unsigned n){uint8_t *b=p;for(unsigned i=0;i<n;++i)b[i]=0;}
static bool equal(const void *a,const void *b,unsigned n){const uint8_t *x=a,*y=b;for(unsigned i=0;i<n;++i)if(x[i]!=y[i])return false;return true;}
/* GSM 07.10 fixture also appears in Zephyr's public CRC tests; RFCOMM omits flags. */
EXPORT int rfcomm_wire_vectors(void){
    const uint8_t sabm[]={3,0x3f,1,0x1c},ua[]={3,0x73,1,0xd7};
    const uint8_t mcc[]={3,0xef,21,0x83,17,2,0xf0,7,0,122,0,0,7,0x70};
    uint8_t out[64];uint16_t n=0;dbt_rfcomm_frame_t f;
    CHECK(dbt_rfcomm_decode(sabm,sizeof(sabm),false,&f)==DBT_OK && f.dlci==0 && f.type==DBT_RFCOMM_SABM && f.pf && f.cr);
    CHECK(dbt_rfcomm_encode(&f,false,out,sizeof(out),&n)==DBT_OK && n==sizeof(sabm) && equal(out,sabm,n));
    CHECK(dbt_rfcomm_decode(ua,sizeof(ua),false,&f)==DBT_OK && f.type==DBT_RFCOMM_UA && f.pf);
    CHECK(dbt_rfcomm_encode(&f,false,out,sizeof(out),&n)==DBT_OK && n==sizeof(ua) && equal(out,ua,n));
    CHECK(dbt_rfcomm_decode(mcc,sizeof(mcc),false,&f)==DBT_OK && f.type==DBT_RFCOMM_UIH && f.size==10);
    dbt_rfcomm_mcc_t m;CHECK(dbt_rfcomm_mcc_decode(f.data,f.size,&m)==DBT_OK && m.command && m.type==0x20 && m.size==8 && m.data[1]==0xf0);
    CHECK(dbt_rfcomm_encode(&f,false,out,sizeof(out),&n)==DBT_OK && n==sizeof(mcc) && equal(out,mcc,n));
    CHECK(dbt_rfcomm_cr(true,true) && !dbt_rfcomm_cr(true,false) && !dbt_rfcomm_cr(false,true) && dbt_rfcomm_cr(false,false));
    return 0;
}
EXPORT int rfcomm_wire_lengths_and_credits(void){
    static uint8_t input[32767],out[32780];
    for(unsigned i=0;i<sizeof(input);++i)input[i]=(uint8_t)(i*23u);
    unsigned lengths[]={0,1,126,127,128,129,254,255,256,1024,16384,32767};
    for(unsigned k=0;k<sizeof(lengths)/sizeof(lengths[0]);++k)for(unsigned credit=0;credit<2;++credit){
        dbt_rfcomm_frame_t f={input,(uint16_t)lengths[k],35,149,DBT_RFCOMM_UIH,true,credit!=0,credit!=0},got;
        uint16_t n=0;CHECK(dbt_rfcomm_encode(&f,true,out,sizeof(out),&n)==DBT_OK);
        CHECK(dbt_rfcomm_decode(out,n,true,&got)==DBT_OK && got.dlci==35 && got.size==f.size && got.has_credits==f.has_credits && (!credit || got.credits==149));
        CHECK(!got.size || equal(got.data,input,got.size));
        CHECK(dbt_rfcomm_decode(out,n-1,true,&got)==DBT_INVALID);
        CHECK(dbt_rfcomm_encode(&f,true,out,(uint16_t)(n-1u),&n)==DBT_NO_SPACE);
    }
    return 0;
}
EXPORT int rfcomm_wire_transaction_and_corruption(void){
    uint8_t out[40];for(unsigned i=0;i<sizeof(out);++i)out[i]=0xa5;uint16_t n=999;
    dbt_rfcomm_frame_t f={0};f.type=DBT_RFCOMM_SABM;f.dlci=0;f.pf=true;f.cr=true;
    CHECK(dbt_rfcomm_encode(&f,false,out,3,&n)==DBT_NO_SPACE && n==999 && out[0]==0xa5);
    f.pf=false;CHECK(dbt_rfcomm_encode(&f,false,out,sizeof(out),&n)==DBT_INVALID && n==999 && out[0]==0xa5);
    f.pf=true;CHECK(dbt_rfcomm_encode(&f,false,out,sizeof(out),&n)==DBT_OK);
    dbt_rfcomm_frame_t x,old;zero(&x,sizeof(x));x.dlci=77;old=x;
    for(unsigned pos=0;pos<4;++pos)for(unsigned bit=0;bit<8;++bit){
        uint8_t altered[4];for(unsigned i=0;i<4;++i)altered[i]=out[i];altered[pos]^=(uint8_t)(1u<<bit);
        dbt_result_t result=dbt_rfcomm_decode(altered,4,false,&x);
        CHECK(result!=DBT_OK && equal(&x,&old,sizeof(x)));
    }
    uint8_t missing[]={3,0xff,1,0};CHECK(dbt_rfcomm_decode(missing,sizeof(missing),true,&x)==DBT_INVALID);
    return 0;
}
EXPORT int rfcomm_mcc_bounded_and_fuzz(void){
    uint8_t bytes[130],out[130];uint32_t seed=0x12345678;uint16_t n=0;
    for(unsigned len=0;len<=127;++len){for(unsigned i=0;i<len;++i)bytes[i]=(uint8_t)i;
        dbt_rfcomm_mcc_t m={bytes,(uint16_t)len,0x20,true},d;
        CHECK(dbt_rfcomm_mcc_encode(&m,out,sizeof(out),&n)==DBT_OK && n==len+2);
        CHECK(dbt_rfcomm_mcc_decode(out,n,&d)==DBT_OK && d.type==m.type && d.command && d.size==len && (!len || equal(d.data,bytes,len)));
    }
    for(unsigned i=0;i<100000;++i){seed=seed*1664525u+1013904223u;unsigned len=seed%64u;
        for(unsigned j=0;j<len;++j){seed=seed*1664525u+1013904223u;bytes[j]=(uint8_t)(seed>>24);}
        dbt_rfcomm_frame_t frame;dbt_rfcomm_mcc_t item;
        (void)dbt_rfcomm_decode(bytes,(uint16_t)len,(seed&1u)!=0,&frame);
        (void)dbt_rfcomm_mcc_decode(bytes,(uint16_t)len,&item);
    }
    return 0;
}
