#include "dbt_queue.h"
#include <stdio.h>
#include <string.h>

void reference_queue_init(dbt_queue_t *);
dbt_result_t reference_queue_push(dbt_queue_t *,const uint8_t *,uint16_t,bool);
dbt_result_t reference_queue_push_sensitive(dbt_queue_t *,const uint8_t *,uint16_t,bool);
dbt_result_t reference_queue_pop(dbt_queue_t *,uint8_t *,uint16_t,uint16_t *);
uint32_t reference_queue_used(const dbt_queue_t *);

static dbt_queue_t actual,reference;
static uint8_t input[DBT_MAX_H4_BYTES],aout[DBT_MAX_H4_BYTES+32],rout[DBT_MAX_H4_BYTES+32];
static unsigned long long cases;
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL line%d case%llu\n",__LINE__,cases);return 1;}}while(0)
static int equal(void){
    return atomic_load(&actual.read)==atomic_load(&reference.read) &&
        atomic_load(&actual.written)==atomic_load(&reference.written) &&
        atomic_load(&actual.rejected)==atomic_load(&reference.rejected) &&
        atomic_load(&actual.high_water)==atomic_load(&reference.high_water) &&
        !memcmp(actual.bytes,reference.bytes,DBT_QUEUE_BYTES);
}
static void reset(uint32_t cursor){
    dbt_queue_init(&actual);reference_queue_init(&reference);
    atomic_store(&actual.read,cursor);atomic_store(&actual.written,cursor);
    atomic_store(&reference.read,cursor);atomic_store(&reference.written,cursor);
    for(unsigned i=0;i<DBT_QUEUE_BYTES;++i)actual.bytes[i]=reference.bytes[i]=(uint8_t)(i*79u+cursor+31u);
    for(unsigned i=0;i<DBT_MAX_H4_BYTES;++i)input[i]=(uint8_t)(i*137u+cursor+17u);
}
static int pop(uint16_t capacity){
    uint16_t an=0,rn=0;
    memset(aout,0x5c,sizeof(aout));memset(rout,0x5c,sizeof(rout));
    dbt_result_t ar=dbt_queue_pop(&actual,aout+16,capacity,&an);
    dbt_result_t rr=reference_queue_pop(&reference,rout+16,capacity,&rn);
    CHECK(ar==rr && an==rn && equal() && !memcmp(aout,rout,sizeof(aout)));
    for(unsigned i=0;i<16;++i)CHECK(aout[i]==0x5c);
    unsigned extent=ar==DBT_OK?an:0;
    for(unsigned i=16u+extent;i<sizeof(aout);++i)CHECK(aout[i]==0x5c);
    ++cases;return 0;
}
static int single(uint32_t cursor,uint16_t length,bool sensitive){
    reset(cursor);
    dbt_result_t ar=sensitive?dbt_queue_push_sensitive(&actual,input,length,false):dbt_queue_push(&actual,input,length,false);
    dbt_result_t rr=sensitive?reference_queue_push_sensitive(&reference,input,length,false):reference_queue_push(&reference,input,length,false);
    CHECK(ar==DBT_OK && rr==DBT_OK && equal());
    CHECK(!pop((uint16_t)(length-1u)));
    CHECK(atomic_load(&actual.read)==cursor);
    CHECK(!pop(DBT_MAX_H4_BYTES));
    CHECK(!memcmp(aout+16,input,length));
    CHECK(dbt_queue_used(&actual)==0);
    for(unsigned i=0;i<length;++i){
        unsigned index=(cursor+2u+i)&(DBT_QUEUE_BYTES-1u);
        CHECK(actual.bytes[index]==(sensitive?0:input[i]));
    }
    CHECK(!pop(DBT_MAX_H4_BYTES));return 0;
}
static uint32_t state=0x39a4c6e1;
static uint32_t random_word(void){state^=state<<13;state^=state>>17;state^=state<<5;return state;}
int main(void){
    const uint16_t lengths[]={1,2,3,4,5,6,7,8,9,15,16,17,127,128,129,565,566,567,585,586,587,588,589,1028,1029};
    for(unsigned offset=0;offset<DBT_QUEUE_BYTES;++offset)
        for(unsigned i=0;i<sizeof(lengths)/sizeof(lengths[0]);++i)
            for(unsigned sensitive=0;sensitive<2;++sensitive)
                if(single(0xfffff000u+offset,lengths[i],sensitive!=0))return 1;
    reset(UINT32_MAX-71u);
    for(unsigned i=0;i<60000;++i){
        uint32_t word=random_word();
        if((word&7u)<5u){
            uint16_t length=(uint16_t)(1u+(word>>8)%DBT_MAX_H4_BYTES);
            bool sensitive=(word&8u)!=0,reserve=(word&16u)!=0;
            dbt_result_t ar=sensitive?dbt_queue_push_sensitive(&actual,input,length,reserve):dbt_queue_push(&actual,input,length,reserve);
            dbt_result_t rr=sensitive?reference_queue_push_sensitive(&reference,input,length,reserve):reference_queue_push(&reference,input,length,reserve);
            CHECK(ar==rr && equal());++cases;
        }else if(pop((uint16_t)((word>>8)%(DBT_MAX_H4_BYTES+1u))))return 1;
    }
    while(dbt_queue_used(&actual))if(pop(DBT_MAX_H4_BYTES))return 1;
    CHECK(!pop(DBT_MAX_H4_BYTES));
    printf("PASS %llu exact queue operations; all4096 ring positions, 25 lengths, normal/sensitive, short output, counter wrap, randomized saturation and guard bytes\n",cases);
    return 0;
}
