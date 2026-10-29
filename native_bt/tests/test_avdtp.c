#include "dbt_avdtp.h"
#include "dbt_avdtp_caps.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

typedef struct {
    dbt_avdtp_t signal;
    uint8_t storage[4][2050], saved[2048];
    bool used[4], fail, echo;
    unsigned allocated, released, errors, commands, responses, responded;
    uint16_t sizes[4], saved_size;
    uint8_t last_label, last_signal, last_type;
    dbt_avdtp_outcome_t outcome;
} test_peer_t;
static test_peer_t a, b;
static void zero(void *data, unsigned size) { uint8_t *p=data; for(unsigned i=0;i<size;++i)p[i]=0; }
static bool equal(const uint8_t *x,const uint8_t *y,unsigned n) {
    for(unsigned i=0;i<n;++i) if(x[i]!=y[i])return false;
    return true;
}
static uint8_t *allocate(void *context,uint16_t size) {
    test_peer_t *p=context;
    if(p->fail || size>2048 || !size)return 0;
    for(unsigned i=0;i<4;++i) if(!p->used[i]) {
        p->used[i]=true;p->sizes[i]=size;p->storage[i][0]=0x5a;p->storage[i][size+1u]=0xa5;
        ++p->allocated;return p->storage[i]+1;
    }
    return 0;
}
static void release(void *context,uint8_t *data) {
    test_peer_t *p=context;
    for(unsigned i=0;i<4;++i) if(data==p->storage[i]+1) {
        if(!p->used[i] || p->storage[i][0]!=0x5a || p->storage[i][p->sizes[i]+1u]!=0xa5)++p->errors;
        p->used[i]=false;++p->released;
        for(unsigned j=0;j<2048;++j)p->storage[i][j+1]=0xdd;
        return;
    }
    ++p->errors;
}
static void save(test_peer_t *p,const dbt_avdtp_message_t *m) {
    if(m->size>sizeof(p->saved)){++p->errors;return;}
    p->saved_size=m->size;p->last_signal=m->signal;p->last_label=m->label;p->last_type=(uint8_t)m->type;
    for(unsigned i=0;i<m->size;++i)p->saved[i]=m->data[i];
}
static void command(void *context,const dbt_avdtp_message_t *m) {
    test_peer_t *p=context;++p->commands;save(p,m);
    if(p->echo && dbt_avdtp_respond(&p->signal,m->label,m->signal,DBT_AVDTP_ACCEPT,
                                   m->data,m->size,100)!=DBT_OK)++p->errors;
}
static void complete(void *context,dbt_avdtp_outcome_t outcome,const dbt_avdtp_message_t *m) {
    test_peer_t *p=context;++p->responses;p->outcome=outcome;if(m)save(p,m);
}
static void responded(void *context,uint8_t label,uint8_t signal,dbt_avdtp_type_t type) {
    test_peer_t *p=context;(void)label;(void)signal;(void)type;++p->responded;
}
static dbt_avdtp_hooks_t hooks(test_peer_t *p) {
    dbt_avdtp_hooks_t result={p,allocate,release,command,complete,responded,0};return result;
}
static bool fixture(test_peer_t *p,uint16_t mtu) {
    zero(p,sizeof(*p));return dbt_avdtp_init(&p->signal,hooks(p),123,mtu,mtu,2048,1000,2000)==DBT_OK;
}
static dbt_result_t transfer(test_peer_t *from,test_peer_t *to,uint32_t now) {
    const uint8_t *bytes=0;uint16_t size=0;
    dbt_result_t result=dbt_avdtp_tx_peek(&from->signal,&bytes,&size);
    if(result!=DBT_OK)return result;
    /* Commit is transport admission, preceding the simulated peer receive. */
    uint8_t wire[2048];for(unsigned i=0;i<size;++i)wire[i]=bytes[i];
    result=dbt_avdtp_tx_commit(&from->signal,now);
    return result==DBT_OK?dbt_avdtp_receive(&to->signal,123,wire,size,now):result;
}

EXPORT int test_avdtp_control_correlation_and_lazy_storage(void) {
    CHECK(fixture(&a,48));uint8_t label=99,body=4;
    CHECK(dbt_avdtp_command(&a.signal,DBT_AVDTP_OPEN,&body,1,10,&label)==DBT_OK && label==0);
    body=8;const uint8_t *wire=0;uint16_t size=0;
    CHECK(dbt_avdtp_tx_peek(&a.signal,&wire,&size)==DBT_OK && size==3);
    CHECK(wire[0]==0 && wire[1]==6 && wire[2]==4 && a.allocated==0);
    uint8_t reply[]={2,6};
    CHECK(dbt_avdtp_receive(&a.signal,123,reply,2,11)==DBT_EMPTY && !a.responses);
    CHECK(dbt_avdtp_tx_commit(&a.signal,12)==DBT_OK);
    CHECK(dbt_avdtp_wait_ms(&a.signal,12)==2000 && a.signal.tx[0].data==0);
    reply[0]=0x12;CHECK(dbt_avdtp_receive(&a.signal,123,reply,2,13)==DBT_EMPTY);
    reply[0]=2;reply[1]=7;CHECK(dbt_avdtp_receive(&a.signal,123,reply,2,14)==DBT_EMPTY);
    reply[1]=6;CHECK(dbt_avdtp_receive(&a.signal,124,reply,2,15)==DBT_EMPTY);
    CHECK(dbt_avdtp_receive(&a.signal,123,reply,2,16)==DBT_OK);
    CHECK(a.responses==1 && a.outcome==DBT_AVDTP_RESPONSE && a.last_signal==6 && !a.signal.tx[0].used);
    for(unsigned i=1;i<=17;++i) {
        CHECK(dbt_avdtp_command(&a.signal,DBT_AVDTP_DISCOVER,0,0,20,&label)==DBT_OK && label==(i&15));
        CHECK(dbt_avdtp_tx_peek(&a.signal,&wire,&size)==DBT_OK);
        CHECK(dbt_avdtp_tx_commit(&a.signal,20)==DBT_OK);
        reply[0]=(uint8_t)((label<<4)|2);reply[1]=1;
        CHECK(dbt_avdtp_receive(&a.signal,123,reply,2,21)==DBT_OK);
    }
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK && a.allocated==0 && !a.errors);
    return 0;
}

EXPORT int test_avdtp_fragmented_duplex_and_transport_pins(void) {
    CHECK(fixture(&a,48) && fixture(&b,48));a.echo=b.echo=true;
    uint8_t body[193],label=0;for(unsigned i=0;i<sizeof(body);++i)body[i]=(uint8_t)(i*17u+3u);
    CHECK(dbt_avdtp_command(&a.signal,DBT_AVDTP_SECURITY_CONTROL,body,sizeof(body),10,&label)==DBT_OK);
    for(unsigned i=0;i<sizeof(body);++i)body[i]=0;
    const uint8_t *packet=0;uint16_t size=0;
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK && size==48 && packet[1]==5 && packet[3]==3);
    const uint8_t *saved=packet;
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_BUSY);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_BUSY);
    CHECK(dbt_avdtp_tx_cancel(&a.signal)==DBT_OK);
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK && packet==saved && packet[3]==3);
    CHECK(dbt_avdtp_tx_cancel(&a.signal)==DBT_OK);
    for(unsigned i=0;i<5;++i)CHECK(transfer(&a,&b,100+i)==DBT_OK);
    CHECK(b.commands==1 && b.saved_size==193 && b.saved[192]==(uint8_t)(192u*17u+3u));
    CHECK(a.signal.tx[0].waiting && !a.signal.tx[0].data);
    for(unsigned i=0;i<5;++i)CHECK(transfer(&b,&a,110+i)==DBT_OK);
    CHECK(a.responses==1 && a.saved_size==193 && a.saved[192]==b.saved[192] && b.responded==1);
    for(unsigned i=0;i<sizeof(body);++i)body[i]=(uint8_t)(i*17u+3u);
    CHECK(equal(a.saved,body,sizeof(body)));
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK && dbt_avdtp_clear(&b.signal)==DBT_OK);
    CHECK(a.allocated==a.released && b.allocated==b.released && !a.errors && !b.errors);
    return 0;
}

EXPORT int test_avdtp_receive_sequence_limits_and_abort(void) {
    CHECK(fixture(&a,48));uint8_t first[]={4,3,3,4,8,1}; /* start, 3 packets, SetConfiguration */
    CHECK(dbt_avdtp_receive(&a.signal,123,first,sizeof(first),0)==DBT_OK);
    CHECK(a.commands==0 && a.signal.rx_capacity==13);
    uint8_t middle[]={8,2,3,4,5,6},last[]={12,7,8};
    CHECK(dbt_avdtp_receive(&a.signal,123,middle,sizeof(middle),1)==DBT_OK);
    CHECK(dbt_avdtp_receive(&a.signal,123,last,sizeof(last),2)==DBT_OK);
    CHECK(a.commands==1 && a.saved_size==10 && a.saved[0]==4 && a.saved[9]==8);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK && a.allocated==a.released);
    for(unsigned bad=0;bad<5;++bad) {
        CHECK(fixture(&a,48));CHECK(dbt_avdtp_receive(&a.signal,123,first,sizeof(first),0)==DBT_OK);
        uint8_t broken[]={8,2,3,4,5,6};uint16_t length=sizeof(broken);
        if(bad==0)broken[0]=12; /* end too early */
        if(bad==1)broken[0]=0x18; /* different label */
        if(bad==2)broken[0]=10; /* response type changed */
        if(bad==3)--length; /* short continue */
        if(bad==4){broken[0]=0;broken[1]=1;length=2;} /* interleaved single */
        CHECK(dbt_avdtp_receive(&a.signal,123,broken,length,1)==DBT_INVALID);
        CHECK(!a.signal.rx && !a.commands && a.allocated==a.released);
        CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
    }
    CHECK(fixture(&a,48));CHECK(dbt_avdtp_receive(&a.signal,123,first,sizeof(first),0)==DBT_OK);
    uint8_t abort[]={0x10,10,4};
    CHECK(dbt_avdtp_receive(&a.signal,123,abort,sizeof(abort),1)==DBT_OK);
    CHECK(a.commands==1 && a.last_signal==10 && !a.signal.rx);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK && a.allocated==a.released);
    CHECK(fixture(&a,48));a.signal.maximum_message=5;
    CHECK(dbt_avdtp_receive(&a.signal,123,first,sizeof(first),0)==DBT_INVALID && !a.allocated);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
    return 0;
}

EXPORT int test_avdtp_deadlines_wrap_and_pinned_timeout(void) {
    CHECK(fixture(&a,48));uint8_t label=0;const uint8_t *packet=0;uint16_t size=0;
    uint32_t start=UINT32_MAX-499u;
    CHECK(dbt_avdtp_command(&a.signal,1,0,0,start,&label)==DBT_OK);
    CHECK(dbt_avdtp_poll(&a.signal,start+999u)==DBT_OK);
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK);
    CHECK(dbt_avdtp_tx_commit(&a.signal,start+999u)==DBT_OK);
    CHECK(dbt_avdtp_poll(&a.signal,start+2998u)==DBT_OK);
    CHECK(dbt_avdtp_poll(&a.signal,start+2999u)==DBT_INVALID);
    CHECK(a.responses==1 && a.outcome==DBT_AVDTP_TIMEOUT);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK && a.responses==1);
    CHECK(fixture(&a,48));uint8_t body[100]={0x81};
    CHECK(dbt_avdtp_command(&a.signal,11,body,sizeof(body),0,&label)==DBT_OK);
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK && packet[3]==0x81);
    CHECK(dbt_avdtp_poll(&a.signal,1000)==DBT_INVALID && packet[3]==0x81);
    CHECK(a.responses==1 && a.released==0 && dbt_avdtp_clear(&a.signal)==DBT_BUSY);
    CHECK(dbt_avdtp_tx_commit(&a.signal,1001)==DBT_INVALID);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK && a.allocated==a.released && a.responses==1);
    CHECK(fixture(&a,48));uint8_t command_packet[]={0,1};
    CHECK(dbt_avdtp_receive(&a.signal,123,command_packet,2,5)==DBT_OK);
    CHECK(dbt_avdtp_poll(&a.signal,2004)==DBT_OK && dbt_avdtp_poll(&a.signal,2005)==DBT_INVALID);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
    CHECK(fixture(&a,48));uint8_t first[]={4,2,3,4};
    CHECK(dbt_avdtp_receive(&a.signal,123,first,sizeof(first),5)==DBT_OK);
    CHECK(dbt_avdtp_poll(&a.signal,2005)==DBT_INVALID && a.allocated==a.released);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
    CHECK(fixture(&a,48));
    CHECK(dbt_avdtp_command(&a.signal,1,0,0,0,&label)==DBT_OK);
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK);
    CHECK(dbt_avdtp_tx_commit(&a.signal,1000)==DBT_INVALID && a.outcome==DBT_AVDTP_TIMEOUT);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK && a.responses==1);
    CHECK(fixture(&a,48));
    CHECK(dbt_avdtp_receive(&a.signal,123,command_packet,2,0)==DBT_OK);
    CHECK(dbt_avdtp_respond(&a.signal,0,1,DBT_AVDTP_ACCEPT,0,0,2000)==DBT_INVALID && a.signal.failed);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
    return 0;
}

EXPORT int test_avdtp_oom_retries_and_response_priority(void) {
    CHECK(fixture(&a,48));uint8_t body[80]={4},label=0;
    a.fail=true;
    CHECK(dbt_avdtp_command(&a.signal,11,body,sizeof(body),0,&label)==DBT_NO_SPACE);
    CHECK(!a.signal.tx[0].used && a.signal.next_label==0);
    a.fail=false;CHECK(dbt_avdtp_command(&a.signal,11,body,sizeof(body),0,&label)==DBT_OK);
    a.fail=true;const uint8_t *packet=0;uint16_t size=0;
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_NO_SPACE && !a.signal.pinned);
    a.fail=false;CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK);
    CHECK(dbt_avdtp_tx_commit(&a.signal,1)==DBT_OK);
    uint8_t incoming[]={0x10,1};
    CHECK(dbt_avdtp_receive(&a.signal,123,incoming,2,2)==DBT_OK);
    a.fail=true;CHECK(dbt_avdtp_respond(&a.signal,1,1,DBT_AVDTP_ACCEPT,body,sizeof(body),2)==DBT_NO_SPACE);
    CHECK(a.signal.remote_pending && !a.signal.tx[1].used);
    a.fail=false;CHECK(dbt_avdtp_respond(&a.signal,1,1,DBT_AVDTP_ACCEPT,0,0,2)==DBT_OK);
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK && ((packet[0]>>2)&3)==3);
    CHECK(dbt_avdtp_tx_commit(&a.signal,3)==DBT_OK); /* finish first fragmented message */
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK && packet[0]==0x12 && packet[1]==1);
    CHECK(dbt_avdtp_tx_commit(&a.signal,4)==DBT_OK && !a.signal.remote_pending);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK && a.responses==1 && a.outcome==DBT_AVDTP_CANCELLED);
    CHECK(a.allocated==a.released && !a.errors);
    CHECK(fixture(&a,48));a.fail=true;uint8_t first[]={4,2,3,4};
    CHECK(dbt_avdtp_receive(&a.signal,123,first,sizeof(first),0)==DBT_NO_SPACE && !a.signal.rx);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
    return 0;
}

EXPORT int test_avdtp_unknown_signals_and_command_backpressure(void) {
    CHECK(fixture(&a,48));uint8_t unknown[]={0x50,0xfe};
    CHECK(dbt_avdtp_receive(&a.signal,123,unknown,2,0)==DBT_OK && !a.commands);
    const uint8_t *packet=0;uint16_t size=0;
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK && packet[0]==0x51 && packet[1]==62 && size==2);
    CHECK(dbt_avdtp_tx_commit(&a.signal,1)==DBT_OK && a.responded==1);
    uint8_t incoming[]={0,0xc1}; /* known signal with RFA set */
    CHECK(dbt_avdtp_receive(&a.signal,123,incoming,2,2)==DBT_OK && a.commands==1 && a.last_signal==1);
    CHECK(dbt_avdtp_receive(&a.signal,123,incoming,2,3)==DBT_BUSY && a.commands==1);
    CHECK(dbt_avdtp_respond(&a.signal,1,1,DBT_AVDTP_ACCEPT,0,0,3)==DBT_INVALID);
    CHECK(dbt_avdtp_respond(&a.signal,0,1,DBT_AVDTP_GENERAL_REJECT,incoming,1,3)==DBT_INVALID);
    CHECK(dbt_avdtp_respond(&a.signal,0,1,DBT_AVDTP_ACCEPT,0,0,3)==DBT_OK);
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK);
    CHECK(dbt_avdtp_tx_commit(&a.signal,4)==DBT_OK);
    incoming[1]=0;CHECK(dbt_avdtp_receive(&a.signal,123,incoming,2,5)==DBT_INVALID && a.commands==1);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
    return 0;
}

EXPORT int test_avdtp_rtp_media_headers_and_bounds(void) {
    uint8_t packet[100]={0x80,0xe0,0x12,0x34,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef,0x9c};
    dbt_avdtp_media_t media={0};
    CHECK(dbt_avdtp_media_decode(packet,13,&media)==DBT_OK);
    CHECK(media.sequence==0x1234 && media.timestamp==0x01234567 && media.ssrc==0x89abcdef &&
          media.marker && media.payload_type==96 && media.size==1 && media.payload[0]==0x9c);
    packet[0]=0xb2; /* two CSRCs, extension, padding */
    packet[22]=0;packet[23]=2;packet[32]=0x81;packet[33]=0x82;packet[34]=0;packet[35]=2;
    CHECK(dbt_avdtp_media_decode(packet,36,&media)==DBT_OK && media.size==2 && media.payload==packet+32);
    dbt_avdtp_media_t before=media;
    packet[35]=0;CHECK(dbt_avdtp_media_decode(packet,36,&media)==DBT_INVALID && media.payload==before.payload);
    packet[35]=5;CHECK(dbt_avdtp_media_decode(packet,36,&media)==DBT_INVALID);
    packet[35]=2;packet[22]=0xff;packet[23]=0xff;
    CHECK(dbt_avdtp_media_decode(packet,36,&media)==DBT_INVALID);
    packet[0]=0x8f;CHECK(dbt_avdtp_media_decode(packet,36,&media)==DBT_INVALID);
    packet[0]=0x40;CHECK(dbt_avdtp_media_decode(packet,36,&media)==DBT_INVALID);
    CHECK(dbt_avdtp_media_decode(packet,11,&media)==DBT_INVALID);
    return 0;
}

EXPORT int test_avdtp_generated_packets_reclaim_storage(void) {
    uint32_t random=0x9128731u;uint8_t bytes[96];
    for(unsigned n=0;n<4096;++n) {
        CHECK(fixture(&a,48));
        for(unsigned j=0;j<sizeof(bytes);++j){random=random*1664525u+1013904223u;bytes[j]=(uint8_t)(random>>24);}
        unsigned length=n%sizeof(bytes);
        (void)dbt_avdtp_receive(&a.signal,123,bytes,(uint16_t)length,0);
        /* Also exercise a second, independently malformed continuation. */
        bytes[0]=(uint8_t)((bytes[0]&0xf3u)|8u);
        (void)dbt_avdtp_receive(&a.signal,123,bytes,(uint16_t)length,1);
        dbt_avdtp_media_t media;
        (void)dbt_avdtp_media_decode(bytes,(uint16_t)length,&media);
        CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
        CHECK(a.allocated==a.released && !a.errors);
    }
    return 0;
}

EXPORT int test_avdtp_reply_lengths_and_fragment_capacity(void) {
    CHECK(fixture(&a,48));uint8_t label=0;const uint8_t *packet=0;uint16_t size=0;
    CHECK(dbt_avdtp_command(&a.signal,DBT_AVDTP_SET_CONFIGURATION,0,0,0,&label)==DBT_OK);
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK);
    CHECK(dbt_avdtp_tx_commit(&a.signal,1)==DBT_OK);
    uint8_t bad[]={2,3,0x81};
    CHECK(dbt_avdtp_receive(&a.signal,123,bad,sizeof(bad),2)==DBT_INVALID && !a.responses);
    bad[0]=3;CHECK(dbt_avdtp_receive(&a.signal,123,bad,sizeof(bad),3)==DBT_INVALID && !a.responses);
    uint8_t reject[]={3,3,7,0x29};
    CHECK(dbt_avdtp_receive(&a.signal,123,reject,sizeof(reject),4)==DBT_OK && a.responses==1);
    CHECK(a.last_type==DBT_AVDTP_REJECT && a.saved[0]==7 && a.saved[1]==0x29);
    uint8_t incoming[]={0x10,3,4,8};
    CHECK(dbt_avdtp_receive(&a.signal,123,incoming,sizeof(incoming),5)==DBT_OK);
    CHECK(dbt_avdtp_respond(&a.signal,1,3,DBT_AVDTP_ACCEPT,incoming,1,5)==DBT_INVALID);
    CHECK(dbt_avdtp_respond(&a.signal,1,3,DBT_AVDTP_REJECT,reject+2,2,5)==DBT_OK);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
    CHECK(fixture(&a,48));a.signal.maximum_message=4;
    /* Worst-case final length exceeds the bound, but the actual short end fits. */
    uint8_t start[]={4,2,11,0x81,0x82,0x83},end[]={12,0x84};
    CHECK(dbt_avdtp_receive(&a.signal,123,start,sizeof(start),0)==DBT_OK && a.signal.rx_capacity==4);
    CHECK(dbt_avdtp_receive(&a.signal,123,end,sizeof(end),1)==DBT_OK && a.saved_size==4);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK && a.allocated==a.released && !a.errors);
    CHECK(fixture(&a,48));a.signal.maximum_message=4;
    CHECK(dbt_avdtp_receive(&a.signal,123,start,sizeof(start),0)==DBT_OK);
    uint8_t too_long[]={12,1,2};
    CHECK(dbt_avdtp_receive(&a.signal,123,too_long,sizeof(too_long),1)==DBT_INVALID);
    CHECK(!a.commands && a.allocated==a.released && !a.errors);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
    CHECK(fixture(&a,48));a.signal.maximum_message=UINT16_MAX;
    uint8_t one=1;
    CHECK(dbt_avdtp_command(&a.signal,11,&one,12000,0,&label)==DBT_NO_SPACE && !a.allocated);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK);
    return 0;
}

EXPORT int test_avdtp_simultaneous_local_and_remote_transactions(void) {
    CHECK(fixture(&a,48));uint8_t label=0;const uint8_t *packet=0;uint16_t size=0;
    CHECK(dbt_avdtp_command(&a.signal,DBT_AVDTP_OPEN,0,0,0,&label)==DBT_OK);
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK);
    CHECK(dbt_avdtp_tx_commit(&a.signal,1)==DBT_OK);
    uint8_t incoming[]={0x70,DBT_AVDTP_DISCOVER};
    CHECK(dbt_avdtp_receive(&a.signal,123,incoming,sizeof(incoming),2)==DBT_OK);
    CHECK(a.commands==1 && a.signal.tx[0].waiting);
    CHECK(dbt_avdtp_respond(&a.signal,7,1,DBT_AVDTP_ACCEPT,0,0,2)==DBT_OK);
    CHECK(dbt_avdtp_tx_peek(&a.signal,&packet,&size)==DBT_OK && packet[0]==0x72);
    uint8_t response[]={2,6};
    CHECK(dbt_avdtp_receive(&a.signal,123,response,sizeof(response),3)==DBT_OK && a.responses==1);
    CHECK(packet[0]==0x72 && a.signal.remote_pending && a.signal.pinned);
    CHECK(dbt_avdtp_tx_commit(&a.signal,4)==DBT_OK && a.responded==1);
    CHECK(dbt_avdtp_clear(&a.signal)==DBT_OK && a.allocated==a.released);
    return 0;
}

EXPORT int test_avdtp_capabilities_configuration_and_alternatives(void) {
    const uint8_t capabilities[]={7,6,0,0,0x3f,0xff,2,53,1,0,4,2,2,0,4,2,1,0,8,0};
    dbt_avdtp_caps_t view={0};dbt_avdtp_cap_error_t error={0};
    CHECK(dbt_avdtp_caps_validate(capabilities,sizeof(capabilities),DBT_AVDTP_CAPABILITIES,&view,&error)==DBT_OK);
    CHECK(view.present==0x192 && view.codec.size==6 && view.codec.data==capabilities+2);
    uint16_t cursor=0;dbt_avdtp_cap_t cap;unsigned protection=0;
    while(dbt_avdtp_cap_next(capabilities,sizeof(capabilities),&cursor,&cap)==DBT_OK)
        if(cap.category==4)++protection;
    CHECK(protection==2 && cursor==sizeof(capabilities));
    CHECK(dbt_avdtp_caps_validate(capabilities,sizeof(capabilities),DBT_AVDTP_CONFIGURATION,&view,&error)==DBT_INVALID);
    CHECK(error.category==4 && error.code==0x18);
    const uint8_t configuration[]={1,0,7,6,0,0,0x21,0x15,2,53,8,0};
    CHECK(dbt_avdtp_caps_validate(configuration,sizeof(configuration),DBT_AVDTP_CONFIGURATION,&view,&error)==DBT_OK);
    CHECK(dbt_avdtp_caps_validate(configuration,sizeof(configuration),DBT_AVDTP_RECONFIGURATION,&view,&error)==DBT_INVALID);
    CHECK(error.category==1 && error.code==0x1a);
    CHECK(dbt_avdtp_caps_validate(configuration+2,8,DBT_AVDTP_RECONFIGURATION,&view,&error)==DBT_OK);
    CHECK(dbt_avdtp_caps_validate(0,0,DBT_AVDTP_CONFIGURATION,&view,&error)==DBT_INVALID && error.category==0);
    const uint8_t future[]={0xfe,1,0xa5,1,0,7,2,0,0xff};
    CHECK(dbt_avdtp_caps_validate(future,sizeof(future),DBT_AVDTP_CAPABILITIES,&view,&error)==DBT_OK);
    CHECK(dbt_avdtp_caps_validate(future,sizeof(future),DBT_AVDTP_CONFIGURATION,&view,&error)==DBT_INVALID && error.code==0x17);
    return 0;
}

EXPORT int test_avdtp_capability_format_errors_and_generated_tlvs(void) {
    dbt_avdtp_caps_t view={0};dbt_avdtp_cap_error_t error={0};
    uint8_t bytes[]={1,1,0};
    CHECK(dbt_avdtp_caps_validate(bytes,sizeof(bytes),DBT_AVDTP_CAPABILITIES,&view,&error)==DBT_INVALID && error.code==0x23);
    bytes[0]=4;
    CHECK(dbt_avdtp_caps_validate(bytes,sizeof(bytes),DBT_AVDTP_CAPABILITIES,&view,&error)==DBT_INVALID && error.code==0x27);
    bytes[0]=3;
    CHECK(dbt_avdtp_caps_validate(bytes,sizeof(bytes),DBT_AVDTP_CAPABILITIES,&view,&error)==DBT_INVALID && error.code==0x25);
    uint8_t recovery[]={3,3,0,1,1};
    CHECK(dbt_avdtp_caps_validate(recovery,sizeof(recovery),DBT_AVDTP_CAPABILITIES,&view,&error)==DBT_INVALID && error.code==0x22);
    recovery[2]=1;recovery[3]=25;
    CHECK(dbt_avdtp_caps_validate(recovery,sizeof(recovery),DBT_AVDTP_CAPABILITIES,&view,&error)==DBT_INVALID && error.code==0x25);
    recovery[3]=24;
    CHECK(dbt_avdtp_caps_validate(recovery,sizeof(recovery),DBT_AVDTP_CAPABILITIES,&view,&error)==DBT_OK);
    uint8_t mux[]={2,0,6,3,0,8,8};
    CHECK(dbt_avdtp_caps_validate(mux,sizeof(mux),DBT_AVDTP_CAPABILITIES,&view,&error)==DBT_INVALID && error.code==0x28);
    uint8_t good_mux[]={6,5,0,8,8,16,16,2,0};
    CHECK(dbt_avdtp_caps_validate(good_mux,sizeof(good_mux),DBT_AVDTP_CAPABILITIES,&view,&error)==DBT_OK);
    uint8_t truncated[]={7,255,0};uint16_t cursor=0;dbt_avdtp_cap_t cap={0};
    CHECK(dbt_avdtp_cap_next(truncated,sizeof(truncated),&cursor,&cap)==DBT_INVALID && !cursor && !cap.data);
    uint32_t random=12561;uint8_t generated[160];
    for(unsigned n=0;n<4096;++n){
        for(unsigned j=0;j<sizeof(generated);++j){random=random*1664525u+1013904223u;generated[j]=(uint8_t)(random>>24);}
        for(unsigned mode=0;mode<3;++mode){
            (void)dbt_avdtp_caps_validate(generated,(uint16_t)(n%sizeof(generated)),(dbt_avdtp_cap_mode_t)mode,&view,&error);
        }
    }
    return 0;
}
