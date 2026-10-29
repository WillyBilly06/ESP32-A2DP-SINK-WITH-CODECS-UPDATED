#include "dbt_channels.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do { if(!(x))return __LINE__; } while(0)
static dbt_acl_t acl;
static dbt_channels_t channels;
static void fixture(void) {
    dbt_controller_info_t info={0};info.classic_acl_bytes=27;info.classic_acl_packets=2;
    (void)dbt_acl_init(&acl,&info);
    acl.links[0]=(dbt_link_t){.used=true,.handle=1,.generation=1,.kind=DBT_LINK_CLASSIC};
    acl.links[1]=(dbt_link_t){.used=true,.handle=2,.generation=2,.kind=DBT_LINK_CLASSIC};
    (void)dbt_channels_init(&channels,&acl,(dbt_channel_hooks_t){0},512,1000,60000);
}
static int send_response(uint16_t handle,uint8_t id,const uint8_t *body,unsigned n,uint32_t now) {
    uint8_t packet[16]={0x0b,id,0,0};packet[2]=(uint8_t)n;
    for(unsigned i=0;i<n;++i)packet[4+i]=body[i];
    const dbt_link_t *link=dbt_acl_find(&acl,handle);
    if(dbt_channels_receive(&channels,handle,link->generation,1,packet,(uint16_t)(n+4))!=DBT_OK)return __LINE__;
    dbt_channels_poll(&channels,now,8);return 0;
}
EXPORT int feature_discovery_success_and_generation(void) {
    fixture();uint32_t mask=0xaabbccdd;dbt_signal_view_t view;
    CHECK(dbt_channels_discover_features(&channels,1,10)==DBT_OK);
    CHECK(dbt_channels_peer_features(&channels,1,&mask)==DBT_BUSY && mask==0xaabbccdd);
    CHECK(dbt_channels_tx_peek(&channels,&view)==DBT_OK && view.size==6 && view.data[0]==0x0a && view.data[4]==2);
    uint8_t id=view.data[1];uint8_t body[]={2,0,0,0,0x88,0,0x12,0x34};
    CHECK(send_response(1,id,body,8,11)==0); /* ignored before HCI admission */
    CHECK(dbt_channels_peer_features(&channels,1,&mask)==DBT_BUSY);
    CHECK(dbt_channels_tx_commit(&channels,20)==DBT_OK);
    CHECK(send_response(1,(uint8_t)(id+1),body,8,21)==0);
    CHECK(dbt_channels_peer_features(&channels,1,&mask)==DBT_BUSY);
    CHECK(send_response(1,id,body,8,22)==0);
    CHECK(dbt_channels_peer_features(&channels,1,&mask)==DBT_OK && mask==0x34120088u);
    acl.links[0].generation=3;
    mask=7;CHECK(dbt_channels_peer_features(&channels,1,&mask)==DBT_BUSY && mask==7);
    CHECK(dbt_channels_discover_features(&channels,1,30)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&channels,&view)==DBT_OK && view.generation==3);
    return 0;
}
EXPORT int feature_discovery_timeout_and_reject(void) {
    fixture();uint32_t mask=9;dbt_signal_view_t view;
    CHECK(dbt_channels_discover_features(&channels,1,0xfffffff0u)==DBT_OK);
    CHECK(dbt_channels_wait_ms(&channels,0xfffffff1u)==999);
    CHECK(dbt_channels_tx_peek(&channels,&view)==DBT_OK);
    CHECK(dbt_channels_tx_commit(&channels,0xfffffff1u)==DBT_OK);
    dbt_channels_poll(&channels,0x000003d9u,4);
    CHECK(dbt_channels_peer_features(&channels,1,&mask)==DBT_UNSUPPORTED && mask==9);
    CHECK(dbt_channels_discover_features(&channels,2,50)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&channels,&view)==DBT_OK);uint8_t id=view.data[1];
    CHECK(dbt_channels_tx_commit(&channels,51)==DBT_OK);
    uint8_t reject[]={1,id,2,0,0,0};
    CHECK(dbt_channels_receive(&channels,2,2,1,reject,sizeof(reject))==DBT_OK);
    dbt_channels_poll(&channels,52,4);
    CHECK(dbt_channels_peer_features(&channels,2,&mask)==DBT_UNSUPPORTED);
    return 0;
}
EXPORT int feature_discovery_short_and_cancel(void) {
    fixture();uint32_t mask=11;dbt_signal_view_t view;
    CHECK(dbt_channels_discover_features(&channels,1,0)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&channels,&view)==DBT_OK);uint8_t id=view.data[1];
    CHECK(dbt_channels_tx_commit(&channels,1)==DBT_OK);
    uint8_t short_success[]={2,0,0,0};
    CHECK(send_response(1,id,short_success,4,2)==0);
    CHECK(dbt_channels_peer_features(&channels,1,&mask)==DBT_UNSUPPORTED && mask==11);
    CHECK(dbt_channels_discover_features(&channels,2,3)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&channels,&view)==DBT_OK);
    CHECK(dbt_channels_tx_cancel(&channels)==DBT_OK);
    CHECK(dbt_channels_peer_features(&channels,2,&mask)==DBT_UNSUPPORTED);
    return 0;
}
EXPORT int feature_discovery_ids_and_clear(void) {
    fixture();dbt_signal_view_t view;uint16_t cid=0;uint32_t mask=0;
    CHECK(dbt_channels_open(&channels,1,0x19,256,48,DBT_SECURITY_NONE,1,&cid)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&channels,&view)==DBT_OK && view.data[0]==2);
    uint8_t connection_id=view.data[1];CHECK(dbt_channels_tx_commit(&channels,2)==DBT_OK);
    CHECK(dbt_channels_discover_features(&channels,1,3)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&channels,&view)==DBT_OK && view.data[0]==0x0a && view.data[1]!=connection_id);
    dbt_channels_clear(&channels);
    CHECK(dbt_channels_tx_peek(&channels,&view)==DBT_EMPTY);
    CHECK(dbt_channels_peer_features(&channels,1,&mask)==DBT_BUSY);
    CHECK(dbt_channels_discover_features(&channels,1,4)==DBT_OK);
    return 0;
}
