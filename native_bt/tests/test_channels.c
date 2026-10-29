#include "dbt_channels.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(expression) do { if (!(expression)) return __LINE__; } while (0)
static dbt_acl_t acl;
static dbt_channels_t manager;
static uint32_t now;
static unsigned opened, closed, failed, reconfigured, data_calls, fixed_calls, parameter_calls;
static dbt_channel_reason_t last_reason;
static uint16_t last_result, data_size, data_cid;
static bool permit, idle, parameters;
static dbt_result_t security_progress;
static unsigned security_starts;
static dbt_result_t security_request(void *context,const dbt_link_t *link,uint16_t psm,dbt_security_t level,bool start){
    (void)context;(void)link;(void)psm;(void)level;if(start)++security_starts;return security_progress;
}
static uint8_t wire[48], command[256], received_data[16];
static uint16_t wire_size;
static uint8_t wire_id;
static void w16(uint8_t *data, uint16_t value) { data[0]=(uint8_t)value; data[1]=(uint8_t)(value>>8); }
static uint16_t r16(const uint8_t *data) { return (uint16_t)(data[0]|((uint16_t)data[1]<<8)); }
static bool authorize(void *context,const dbt_link_t *link,uint16_t psm,dbt_security_t security) {
    (void)context;(void)link;(void)psm;(void)security;return permit;
}
static bool tx_idle(void *context,uint16_t handle,uint16_t cid) {(void)context;(void)handle;(void)cid;return idle;}
static bool le_parameters(void *context,const dbt_link_t *link,uint16_t minimum,uint16_t maximum,uint16_t latency,uint16_t timeout) {
    (void)context;(void)link;(void)minimum;(void)maximum;(void)latency;(void)timeout;++parameter_calls;return parameters;
}
static void event(void *context,dbt_channel_event_t kind,const dbt_channel_t *channel,dbt_channel_reason_t reason,uint16_t result) {
    (void)context;(void)channel;last_reason=reason;last_result=result;
    if(kind==DBT_CH_EVENT_OPEN)++opened;
    else if(kind==DBT_CH_EVENT_CLOSED)++closed;
    else if(kind==DBT_CH_EVENT_FAILED)++failed;
    else ++reconfigured;
}
static void data(void *context,const dbt_channel_t *channel,const uint8_t *bytes,uint16_t size) {
    (void)context;++data_calls;data_size=size;data_cid=channel->local_cid;
    for(unsigned i=0;i<size && i<sizeof(received_data);++i)received_data[i]=bytes[i];
}
static void fixed(void *context,const dbt_link_t *link,uint16_t cid,const uint8_t *bytes,uint16_t size) {
    (void)context;(void)link;(void)cid;(void)bytes;(void)size;++fixed_calls;
}
static dbt_result_t link_event(uint8_t code,const uint8_t *bytes,uint16_t size,dbt_link_change_t *change) {
    dbt_hci_packet_t packet={0};packet.type=4;packet.event=code;packet.payload=bytes;packet.size=size;
    return dbt_acl_event(&acl,&packet,change);
}
static void connect_link(uint16_t handle,bool le) {
    dbt_link_change_t change;
    uint8_t classic[11]={0};classic[1]=(uint8_t)handle;classic[9]=1;
    uint8_t low_energy[19]={1,0,0,0,0,0};low_energy[2]=(uint8_t)handle;
    (void)link_event(le?0x3e:3,le?low_energy:classic,le?19:11,&change);
}
static void fixture(void) {
    dbt_controller_info_t info={0};info.classic_acl_bytes=27;info.classic_acl_packets=3;
    info.le_acl_bytes=27;info.le_acl_packets=2;
    (void)dbt_acl_init(&acl,&info);connect_link(1,false);connect_link(2,false);connect_link(3,true);
    dbt_channel_hooks_t hooks={0};hooks.authorize=authorize;hooks.tx_idle=tx_idle;hooks.event=event;
    hooks.data=data;hooks.fixed_data=fixed;hooks.le_parameters=le_parameters;
    (void)dbt_channels_init(&manager,&acl,hooks,4096,1000,60000);
    permit=idle=parameters=true;opened=closed=failed=reconfigured=data_calls=fixed_calls=parameter_calls=0;
    now=0;wire_size=0;last_reason=DBT_CH_REASON_NORMAL;last_result=0;
}
static dbt_result_t rx(uint16_t handle,uint16_t cid,uint8_t code,uint8_t id,const uint8_t *body,uint16_t size) {
    command[0]=code;command[1]=id;w16(command+2,size);
    for(unsigned i=0;i<size;++i)command[4+i]=body[i];
    const dbt_link_t *link=dbt_acl_find(&acl,handle);
    return dbt_channels_receive(&manager,handle,link->generation,cid,command,(uint16_t)(4u+size));
}
static void poll(void) {dbt_channels_poll(&manager,++now,32);}
static int take(uint8_t code) {
    dbt_signal_view_t view;
    CHECK(dbt_channels_tx_peek(&manager,&view)==DBT_OK);
    CHECK(view.data[0]==code && view.size<=sizeof(wire));
    wire_size=view.size;wire_id=view.data[1];
    for(unsigned i=0;i<view.size;++i)wire[i]=view.data[i];
    return 0;
}
static int commit(void) {CHECK(dbt_channels_tx_commit(&manager,++now)==DBT_OK);return 0;}
static int connected_outgoing(uint16_t handle,uint16_t mtu,uint16_t peer,uint16_t *local,uint8_t *configuration_id) {
    CHECK(dbt_channels_open(&manager,handle,0x19,mtu,48,DBT_SECURITY_NONE,++now,local)==DBT_OK);
    int result=take(2);if(result)return result;
    CHECK(wire_size==8 && r16(wire+4)==0x19 && r16(wire+6)==*local);
    uint8_t id=wire_id;
    result=commit();if(result)return result;
    uint8_t response[8]={0};w16(response,peer);w16(response+2,*local);
    CHECK(rx(handle,1,3,id,response,8)==DBT_OK);poll();
    result=take(4);if(result)return result;
    CHECK(wire_size==12 && r16(wire+4)==peer && r16(wire+6)==0 &&
          wire[8]==1 && wire[9]==2 && r16(wire+10)==mtu);
    *configuration_id=wire_id;
    return commit();
}
static int open_channel(uint16_t handle,uint16_t mtu,uint16_t peer,uint16_t peer_mtu,uint16_t *local) {
    uint8_t config_id=0;int result=connected_outgoing(handle,mtu,peer,local,&config_id);if(result)return result;
    uint8_t peer_config[8]={0};w16(peer_config,*local);peer_config[4]=1;peer_config[5]=2;w16(peer_config+6,peer_mtu);
    CHECK(rx(handle,1,4,99,peer_config,8)==DBT_OK);poll();
    uint8_t own_response[6]={0};w16(own_response,*local);
    CHECK(rx(handle,1,5,config_id,own_response,6)==DBT_OK);poll();
    CHECK(dbt_channels_find(&manager,handle,*local)->state==DBT_CH_CONFIGURING);
    result=take(5);if(result)return result;
    CHECK(r16(wire+4)==peer && r16(wire+8)==0 && r16(wire+12)==peer_mtu);
    result=commit();if(result)return result;
    CHECK(dbt_channels_find(&manager,handle,*local)->state==DBT_CH_OPEN);
    return 0;
}

EXPORT int test_channels_outgoing_and_asymmetric_mtu(void) {
    fixture();uint16_t local=0;
    int result=open_channel(1,256,0x80,1000,&local);if(result)return result;
    const dbt_channel_t *channel=dbt_channels_find(&manager,1,local);
    CHECK(opened==1 && local==0x40 && channel->rx_mtu==256 && channel->tx_mtu==1000);
    dbt_l2cap_tx_t tx;uint8_t body[3]={1,2,3};
    CHECK(dbt_channels_prepare_data(&manager,1,local,body,3,&tx)==DBT_OK && tx.cid==0x80);
    CHECK(dbt_channels_prepare_data(&manager,1,local,body,1001,&tx)==DBT_NO_SPACE);
    CHECK(dbt_channels_receive(&manager,1,channel->link_generation,local,body,3)==DBT_OK);
    CHECK(data_calls==1 && data_cid==local && data_size==3 && received_data[2]==3);
    CHECK(dbt_channels_receive(&manager,1,channel->link_generation,local,body,257)==DBT_OK);
    CHECK(data_calls==1 && manager.oversized_dynamic_pdus==1);
    return 0;
}

EXPORT int test_channels_incoming_and_duplicate_requests(void) {
    fixture();dbt_service_t service={0x19,1024,48,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&manager,service)==DBT_OK);
    uint8_t request[4]={0x19,0,0xa0,0};
    CHECK(rx(1,1,2,7,request,4)==DBT_OK);request[0]=0;poll();
    int result=take(3);if(result)return result;
    CHECK(r16(wire+4)==0x40 && r16(wire+6)==0xa0 && r16(wire+8)==0);
    CHECK(!dbt_channels_find(&manager,1,0x40)->connection_ready);
    result=commit();if(result)return result;poll();
    result=take(4);if(result)return result;uint8_t config_id=wire_id;
    result=commit();if(result)return result;
    uint8_t peer_config[4]={0x40,0,0,0};
    CHECK(rx(1,1,4,8,peer_config,4)==DBT_OK);poll();
    result=take(5);if(result)return result;
    CHECK(r16(wire+4)==0xa0 && r16(wire+12)==672);
    result=commit();if(result)return result;
    uint8_t response[10]={0x40,0,0,0,0,0,1,2,0,2};
    CHECK(rx(1,1,5,config_id,response,10)==DBT_OK);poll();
    CHECK(opened==1 && dbt_channels_find(&manager,1,0x40)->rx_mtu==512);
    request[0]=0x19;
    CHECK(rx(1,1,2,7,request,4)==DBT_OK);poll();result=take(3);if(result)return result;
    CHECK(r16(wire+4)==0x40 && r16(wire+8)==0);result=commit();if(result)return result;
    CHECK(rx(1,1,4,8,peer_config,4)==DBT_OK);poll();result=take(5);if(result)return result;
    result=commit();if(result)return result;
    CHECK(opened==1 && reconfigured==0);
    unsigned count=0;for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)count+=manager.channels[i].state!=DBT_CH_FREE;
    CHECK(count==1);
    return 0;
}

EXPORT int test_channels_option_errors_hints_and_continuation(void) {
    fixture();uint16_t local=0;uint8_t config_id=0;
    int result=connected_outgoing(1,256,0x80,&local,&config_id);if(result)return result;
    uint8_t options[8]={0x40,0,0,0,0x20,2,0x78,0x56};
    CHECK(rx(1,1,4,10,options,8)==DBT_OK);poll();result=take(5);if(result)return result;
    CHECK(r16(wire+8)==3 && wire[10]==0x20 && wire[11]==2 && wire[13]==0x56);
    result=commit();if(result)return result;
    options[4]=0xa0;
    CHECK(rx(1,1,4,11,options,8)==DBT_OK);poll();result=take(5);if(result)return result;
    CHECK(r16(wire+8)==0 && wire_size==14);result=commit();if(result)return result;
    uint8_t own_response[6]={0x40,0,0,0,0,0};
    CHECK(rx(1,1,5,config_id,own_response,6)==DBT_OK);poll();CHECK(opened==1);

    fixture();result=connected_outgoing(1,256,0x80,&local,&config_id);if(result)return result;
    uint8_t part1[8]={0x40,0,1,0,1,2,0x84,3}; /* peer RX=900, more options follow */
    CHECK(rx(1,1,4,20,part1,8)==DBT_OK);poll();result=take(5);if(result)return result;
    CHECK(r16(wire+6)==1 && r16(wire+8)==0);result=commit();if(result)return result;
    CHECK(!dbt_channels_find(&manager,1,local)->peer_configured);
    uint8_t part2[8]={0x40,0,0,0,2,2,0xff,0xff};
    CHECK(rx(1,1,4,21,part2,8)==DBT_OK);poll();result=take(5);if(result)return result;
    CHECK(r16(wire+6)==0 && r16(wire+12)==900);result=commit();if(result)return result;
    CHECK(rx(1,1,5,config_id,own_response,6)==DBT_OK);poll();
    CHECK(opened==1 && dbt_channels_find(&manager,1,local)->tx_mtu==900);
    return 0;
}

EXPORT int test_channels_response_continuation_and_mtu_retry(void) {
    fixture();uint16_t local=0;uint8_t config_id=0;
    int result=connected_outgoing(1,256,0x80,&local,&config_id);if(result)return result;
    uint8_t response[10]={0x40,0,1,0,0,0,1,2,128,0};
    CHECK(rx(1,1,5,config_id,response,10)==DBT_OK);poll();
    result=take(4);if(result)return result;CHECK(wire_size==8 && r16(wire+4)==0x80);
    config_id=wire_id;result=commit();if(result)return result;
    uint8_t final[6]={0x40,0,0,0,0,0};
    CHECK(rx(1,1,5,config_id,final,6)==DBT_OK);poll();
    CHECK(dbt_channels_find(&manager,1,local)->local_configured && dbt_channels_find(&manager,1,local)->rx_mtu==128);
    CHECK(opened==0);
    fixture();result=connected_outgoing(1,256,0x80,&local,&config_id);if(result)return result;
    response[2]=0;response[4]=1;
    CHECK(rx(1,1,5,config_id,response,10)==DBT_OK);poll();
    result=take(4);if(result)return result;CHECK(wire_size==12 && r16(wire+10)==128 && wire_id!=config_id);
    result=commit();if(result)return result;
    return 0;
}

EXPORT int test_channels_timers_pending_and_unsent_responses(void) {
    fixture();uint16_t local=0;dbt_signal_view_t view;
    CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,0,&local)==DBT_OK);
    CHECK(dbt_channels_wait_ms(&manager,0)==1000);
    uint8_t response[8]={0x80,0,0x40,0,0,0,0,0};
    CHECK(rx(1,1,3,1,response,8)==DBT_OK);poll();
    CHECK(dbt_channels_find(&manager,1,local)->state==DBT_CH_CONNECTING && manager.unknown_responses==1);
    CHECK(dbt_channels_tx_peek(&manager,&view)==DBT_OK);
    CHECK(dbt_channels_tx_commit(&manager,500)==DBT_OK);
    dbt_channels_poll(&manager,1499,32);CHECK(failed==0);
    dbt_channels_poll(&manager,1500,32);CHECK(failed==1 && last_reason==DBT_CH_REASON_RESPONSE_TIMEOUT);

    fixture();CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,0,&local)==DBT_OK);
    dbt_channels_poll(&manager,1000,32);CHECK(failed==1 && last_reason==DBT_CH_REASON_LOCAL_TIMEOUT);
    CHECK(dbt_channels_tx_peek(&manager,&view)==DBT_EMPTY);

    fixture();CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,0,&local)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&manager,&view)==DBT_OK);uint8_t id=view.data[1];
    CHECK(dbt_channels_tx_commit(&manager,0)==DBT_OK);
    uint8_t waiting[8]={0,0,0,0,1,0,1,0};w16(waiting+2,local);
    CHECK(rx(1,1,3,id,waiting,8)==DBT_OK);dbt_channels_poll(&manager,1,32);
    for(uint32_t time=59000;time<=295000;time+=59000) {
        CHECK(rx(1,1,3,id,waiting,8)==DBT_OK);dbt_channels_poll(&manager,time,32);CHECK(failed==0);
    }
    dbt_channels_poll(&manager,300000,32);CHECK(failed==0);
    dbt_channels_poll(&manager,300001,32);CHECK(failed==1);
    return 0;
}

EXPORT int test_channels_multi_command_backpressure_and_malformed(void) {
    fixture();uint8_t cframe[48];
    for(unsigned i=0;i<12;++i) {cframe[4*i]=8;cframe[4*i+1]=(uint8_t)(i+1);cframe[4*i+2]=cframe[4*i+3]=0;}
    const dbt_link_t *link=dbt_acl_find(&acl,1);
    CHECK(dbt_channels_receive(&manager,1,link->generation,1,cframe,48)==DBT_OK);
    dbt_channels_poll(&manager,1,32);CHECK(manager.tx_count==8 && manager.rx_count==1);
    for(unsigned i=0;i<8;++i) {int result=take(9);if(result)return result;CHECK(wire_id==i+1);result=commit();if(result)return result;}
    poll();CHECK(manager.rx_count==0 && manager.tx_count==4);
    for(unsigned i=8;i<12;++i) {int result=take(9);if(result)return result;CHECK(wire_id==i+1);result=commit();if(result)return result;}
    cframe[4]=8;cframe[5]=30;cframe[6]=9;cframe[7]=0;
    CHECK(dbt_channels_receive(&manager,1,link->generation,1,cframe,8)==DBT_INVALID);
    CHECK(manager.rx_count==0 && manager.tx_count==0);
    CHECK(dbt_channels_receive(&manager,3,dbt_acl_find(&acl,3)->generation,5,cframe,8)==DBT_INVALID);
    return 0;
}

EXPORT int test_channels_generation_and_pinned_bytes(void) {
    fixture();uint16_t local=0;dbt_signal_view_t original,again;
    CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,0,&local)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&manager,&original)==DBT_OK);
    uint8_t saved[8];for(unsigned i=0;i<8;++i)saved[i]=original.data[i];
    uint8_t disconnected[4]={0,1,0,0x13};dbt_link_change_t change;
    CHECK(link_event(5,disconnected,4,&change)==DBT_OK);dbt_channels_link_change(&manager,&change);
    connect_link(1,false);
    CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,1,&local)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&manager,&again)==DBT_OK && again.generation==original.generation);
    for(unsigned i=0;i<8;++i)CHECK(saved[i]==again.data[i]);
    CHECK(dbt_channels_tx_cancel(&manager)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&manager,&again)==DBT_OK && again.generation!=original.generation);
    CHECK(dbt_channels_receive(&manager,1,original.generation,1,saved,8)==DBT_INVALID);
    return 0;
}

EXPORT int test_channels_information_security_fixed_and_le(void) {
    fixture();uint8_t info[2]={2,0};
    CHECK(rx(1,1,0x0a,1,info,2)==DBT_OK);poll();int result=take(0x0b);if(result)return result;
    CHECK(r16(wire+6)==0 && wire[8]==0x80 && wire[9]==0);result=commit();if(result)return result;
    info[0]=3;CHECK(rx(1,1,0x0a,2,info,2)==DBT_OK);poll();result=take(0x0b);if(result)return result;
    CHECK(wire_size==16 && wire[8]==2);result=commit();if(result)return result;
    uint8_t bytes[2]={1,2};uint32_t generation=dbt_acl_find(&acl,3)->generation;
    CHECK(dbt_channels_receive(&manager,3,generation,4,bytes,2)==DBT_UNSUPPORTED);
    CHECK(dbt_channels_register_fixed(&manager,DBT_LINK_LE,4)==DBT_OK);
    CHECK(dbt_channels_receive(&manager,3,generation,4,bytes,2)==DBT_OK && fixed_calls==1);
    uint8_t update[8]={8,0,16,0,0,0,100,0};
    CHECK(rx(3,5,0x12,3,update,8)==DBT_OK);poll();result=take(0x13);if(result)return result;
    CHECK(r16(wire+4)==0 && parameter_calls==1);result=commit();if(result)return result;
    update[2]=0x80;update[3]=0x0c;update[6]=10;
    CHECK(rx(3,5,0x12,4,update,8)==DBT_OK);poll();result=take(0x13);if(result)return result;
    CHECK(r16(wire+4)==1 && parameter_calls==1);result=commit();if(result)return result;
    uint8_t connect[4]={0x19,0,0x80,0};
    CHECK(rx(3,5,2,5,connect,4)==DBT_OK);poll();result=take(1);if(result)return result;
    CHECK(r16(wire+4)==0);result=commit();if(result)return result;
    dbt_service_t service={0x19,256,48,DBT_SECURITY_AUTHENTICATED};
    CHECK(dbt_channels_register(&manager,service)==DBT_OK);manager.hooks.authorize=0;
    CHECK(rx(1,1,2,6,connect,4)==DBT_OK);poll();result=take(3);if(result)return result;
    CHECK(r16(wire+8)==3);result=commit();if(result)return result;
    return 0;
}

EXPORT int test_channels_reconfiguration_and_disconnect(void) {
    fixture();uint16_t local=0;int result=open_channel(1,256,0x80,1000,&local);if(result)return result;
    idle=false;uint8_t config[8]={0x40,0,0,0,1,2,0,2};
    CHECK(rx(1,1,4,44,config,8)==DBT_OK);poll();
    CHECK(manager.rx_deferred && manager.rx_count==1 && dbt_channels_wait_ms(&manager,now)==UINT32_MAX);
    uint8_t information[2]={2,0};CHECK(rx(2,1,0x0a,45,information,2)==DBT_OK);poll();
    result=take(0x0b);if(result)return result;result=commit();if(result)return result;
    CHECK(manager.rx_count==1 && manager.rx_deferred);
    idle=true;poll();result=take(5);if(result)return result;result=commit();if(result)return result;
    result=take(4);if(result)return result;uint8_t id=wire_id;result=commit();if(result)return result;
    uint8_t response[6]={0x40,0,0,0,0,0};
    CHECK(rx(1,1,5,id,response,6)==DBT_OK);poll();CHECK(reconfigured==1 && opened==1);
    CHECK(dbt_channels_find(&manager,1,local)->tx_mtu==512);
    CHECK(dbt_channels_close(&manager,1,local,++now)==DBT_OK);
    result=take(6);if(result)return result;id=wire_id;
    CHECK(r16(wire+4)==0x80 && r16(wire+6)==local);result=commit();if(result)return result;
    uint8_t wrong[4]={0x81,0,0x40,0};CHECK(rx(1,1,7,id,wrong,4)==DBT_OK);poll();CHECK(closed==0);
    wrong[0]=0x80;CHECK(rx(1,1,7,id,wrong,4)==DBT_OK);poll();
    CHECK(closed==1 && dbt_channels_find(&manager,1,local)==0);
    return 0;
}

EXPORT int test_channels_late_data_and_reconfiguration(void){
    fixture();uint16_t local=0;int result=open_channel(1,672,0x80,1000,&local);if(result)return result;
    uint32_t generation=dbt_acl_find(&acl,1)->generation;
    uint8_t media[608];for(unsigned i=0;i<sizeof(media);++i)media[i]=(uint8_t)i;
    media[0]=0x80;media[1]=96;
    CHECK(dbt_channels_receive(&manager,1,generation,local,media,sizeof(media))==DBT_OK && data_calls==1);
    CHECK(dbt_channels_close(&manager,1,local,++now)==DBT_OK);
    CHECK(dbt_channels_receive(&manager,1,generation,local,media,sizeof(media))==DBT_OK);
    CHECK(data_calls==1 && manager.ignored_dynamic_pdus==1);
    result=take(6);if(result)return result;uint8_t id=wire_id;result=commit();if(result)return result;
    uint8_t response[4]={0x80,0,0,0};w16(response+2,local);
    CHECK(rx(1,1,7,id,response,sizeof(response))==DBT_OK);poll();
    CHECK(closed==1 && !dbt_channels_find(&manager,1,local));
    CHECK(dbt_channels_receive(&manager,1,generation,local,media,sizeof(media))==DBT_OK);
    CHECK(data_calls==1 && manager.ignored_dynamic_pdus==2);
    CHECK(dbt_channels_receive(&manager,1,generation+1,local,media,sizeof(media))==DBT_INVALID);
    fixture();result=open_channel(1,672,0x80,1000,&local);if(result)return result;
    generation=dbt_acl_find(&acl,1)->generation;
    uint8_t config[8]={0,0,0,0,1,2,0,2};w16(config,local);
    CHECK(rx(1,1,4,44,config,sizeof(config))==DBT_OK);poll();
    CHECK(dbt_channels_find(&manager,1,local)->state==DBT_CH_CONFIGURING);
    CHECK(dbt_channels_receive(&manager,1,generation,local,media,sizeof(media))==DBT_OK && data_calls==1);
    CHECK(dbt_channels_receive(&manager,1,generation,local,media,673)==DBT_OK);
    CHECK(data_calls==1 && manager.ignored_dynamic_pdus==0 && manager.oversized_dynamic_pdus==1);
    return 0;
}
EXPORT int test_channels_response_cids_and_closed_disconnect(void){
    fixture();uint16_t local=0;
    CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,++now,&local)==DBT_OK);
    int result=take(2);if(result)return result;uint8_t id=wire_id;result=commit();if(result)return result;
    uint8_t response[8]={0x80,0,0,0,1,0,0,0};w16(response+2,local+1);
    CHECK(rx(1,1,3,id,response,sizeof(response))==DBT_OK);poll();
    const dbt_channel_t *channel=dbt_channels_find(&manager,1,local);
    CHECK(channel && channel->pending_extended && manager.unknown_responses==0);
    response[4]=2;CHECK(rx(1,1,3,id,response,sizeof(response))==DBT_OK);poll();
    CHECK(!dbt_channels_find(&manager,1,local) && failed==1);
    CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,++now,&local)==DBT_OK);
    result=take(2);if(result)return result;id=wire_id;result=commit();if(result)return result;
    response[4]=0;w16(response+2,local+1);CHECK(rx(1,1,3,id,response,sizeof(response))==DBT_OK);poll();
    CHECK(!dbt_channels_find(&manager,1,local) && failed==2 && last_reason==DBT_CH_REASON_PROTOCOL);
    uint8_t closed_pair[4];w16(closed_pair,local);w16(closed_pair+2,0x80);
    CHECK(rx(1,1,6,99,closed_pair,sizeof(closed_pair))==DBT_OK);poll();
    result=take(7);if(result)return result;
    CHECK(wire_id==99 && r16(wire+4)==local && r16(wire+6)==0x80);
    result=commit();if(result)return result;
    CHECK(!dbt_channels_find(&manager,1,local) && opened==0 && closed==0 && failed==2);
    return 0;
}
EXPORT int test_channels_oversized_signaling_request_identity(void){
    fixture();uint8_t body[48];for(unsigned i=0;i<sizeof(body);++i)body[i]=(uint8_t)i;
    CHECK(rx(1,1,9,11,body,sizeof(body))==DBT_OK);poll();
    dbt_signal_view_t view;CHECK(dbt_channels_tx_peek(&manager,&view)==DBT_EMPTY && !manager.rx_count && manager.oversized_signaling_pdus==1);
    /* A recognized response precedes the first request in this oversized
     * multi-command C-frame. Reject must echo the request's ID, not 11. */
    command[0]=9;command[1]=11;w16(command+2,40);
    for(unsigned i=0;i<40;++i)command[4+i]=0;
    command[44]=8;command[45]=22;w16(command+46,4);for(unsigned i=48;i<52;++i)command[i]=0;
    const dbt_link_t *link=dbt_acl_find(&acl,1);
    CHECK(dbt_channels_receive(&manager,1,link->generation,1,command,52)==DBT_OK);poll();
    int result=take(1);if(result)return result;
    CHECK(wire_id==22 && r16(wire+4)==1 && r16(wire+6)==48 && manager.oversized_signaling_pdus==2);
    result=commit();if(result)return result;CHECK(dbt_channels_tx_peek(&manager,&view)==DBT_EMPTY);return 0;
}
EXPORT int test_channels_hinted_options_and_reconfig_tx(void){
    fixture();uint16_t local;int result=open_channel(1,256,0x80,1000,&local);if(result)return result;
    uint8_t config[15]={0,0,0,0,0x84,9,3,1,1,0,0,0,0,0,0};w16(config,local);
    CHECK(rx(1,1,4,55,config,sizeof(config))==DBT_OK);poll();result=take(5);if(result)return result;
    CHECK(r16(wire+8)==0 && wire_size==14);
    const dbt_channel_t *channel=dbt_channels_find(&manager,1,local);
    CHECK(channel && channel->reconfiguring && channel->state==DBT_CH_CONFIGURING);
    uint8_t bytes[2]={1,2};dbt_l2cap_tx_t tx;
    CHECK(dbt_channels_prepare_data(&manager,1,local,bytes,2,&tx)==DBT_OK && tx.cid==0x80);
    result=commit();if(result)return result;
    /* Drain the locally generated configuration request before the next peer
     * continuation; Basic data remains permitted in this state. */
    result=take(4);if(result)return result;uint8_t id=wire_id;result=commit();if(result)return result;
    uint8_t response[6]={0};w16(response,local);CHECK(rx(1,1,5,id,response,sizeof(response))==DBT_OK);poll();
    CHECK(dbt_channels_find(&manager,1,local)->state==DBT_CH_OPEN);
    uint8_t mtu_hint[8]={0,0,0,0,0x81,2,1,0};w16(mtu_hint,local);
    CHECK(rx(1,1,4,56,mtu_hint,sizeof(mtu_hint))==DBT_OK);poll();result=take(5);if(result)return result;
    CHECK(r16(wire+8)==0 && r16(wire+12)==1000);
    /* Ignoring a hint does not waive the profile's independent minimum MTU.
     * Its implied default 672 still needs adjustment for this service. */
    fixture();dbt_service_t service={0x19,1000,1000,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&manager,service)==DBT_OK);
    uint8_t incoming[4]={0x19,0,0x80,0};CHECK(rx(1,1,2,60,incoming,sizeof(incoming))==DBT_OK);poll();
    result=take(3);if(result)return result;local=r16(wire+4);result=commit();if(result)return result;
    result=take(4);if(result)return result;result=commit();if(result)return result;
    w16(mtu_hint,local);CHECK(rx(1,1,4,61,mtu_hint,sizeof(mtu_hint))==DBT_OK);poll();result=take(5);if(result)return result;
    CHECK(r16(wire+8)==1 && r16(wire+12)==1000);return 0;
}
static _Alignas(8) uint8_t le_storage[4][1032];
static bool le_used[4],le_oom;
static uint16_t le_sizes[4],le_policy;
static unsigned le_allocated,le_released,le_errors,le_security_calls;
static uint8_t *le_alloc(void *context,uint16_t size){
    (void)context;if(le_oom || !size || size>1024)return 0;
    for(unsigned i=0;i<4;++i)if(!le_used[i]){le_used[i]=true;le_sizes[i]=size;le_storage[i][size]=0xa5;++le_allocated;return le_storage[i];}
    return 0;
}
static void le_free(void *context,uint8_t *bytes){
    (void)context;for(unsigned i=0;i<4;++i){if(bytes==le_storage[i]){
        if(!le_used[i] || le_storage[i][le_sizes[i]]!=0xa5)++le_errors;
        le_used[i]=false;++le_released;return;
    }}
    ++le_errors;
}
static uint16_t qualify(void *context,const dbt_link_t *link,uint16_t spsm,dbt_security_t security,uint8_t minimum){
    (void)context;(void)link;(void)spsm;(void)security;(void)minimum;++le_security_calls;return le_policy;
}
static void le_fixture(void){
    fixture();for(unsigned i=0;i<4;++i)le_used[i]=false;
    le_allocated=le_released=le_errors=le_security_calls=0;le_oom=false;le_policy=0;
    manager.hooks.allocate=le_alloc;manager.hooks.release=le_free;
}
static dbt_result_t le_request(uint8_t id,uint16_t spsm,uint16_t cid,uint16_t mtu,uint16_t mps,uint16_t credits){
    uint8_t body[10];w16(body,spsm);w16(body+2,cid);w16(body+4,mtu);w16(body+6,mps);w16(body+8,credits);
    return rx(3,5,0x14,id,body,sizeof(body));
}
EXPORT int test_le_coc_signaling_validation_and_oom(void){
    const dbt_le_service_t s={128,{1000,27,1},DBT_SECURITY_NONE,0};
    le_fixture();CHECK(dbt_channels_register_le(&manager,&s,1)==DBT_OK && !le_allocated);
    const uint16_t cases[][5]={{129,0x60,100,23,2},{128,0x80,100,23,9},{128,0x60,22,23,11},{128,0x60,100,22,11},{128,0x60,100,65534,11}};
    for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);++i){
        CHECK(le_request((uint8_t)(20u+i),cases[i][0],cases[i][1],cases[i][2],cases[i][3],1)==DBT_OK);poll();
        int result=take(0x15);if(result)return result;CHECK(wire_size==14 && r16(wire+12)==cases[i][4] && r16(wire+4)==0);
        result=commit();if(result)return result;CHECK(!le_allocated && !opened);
    }
    le_oom=true;CHECK(le_request(30,128,0x60,100,23,1)==DBT_OK);poll();int result=take(0x15);if(result)return result;
    CHECK(r16(wire+12)==4 && !dbt_channels_find(&manager,3,0x40));result=commit();if(result)return result;
    le_oom=false;CHECK(le_request(31,128,0x60,100,23,1)==DBT_OK);poll();result=take(0x15);if(result)return result;
    CHECK(r16(wire+12)==0 && !opened && le_allocated==1);uint16_t cid=r16(wire+4);
    uint8_t sdu[]={1,0,42};const dbt_link_t *link=dbt_acl_find(&acl,3);
    CHECK(dbt_channels_receive(&manager,3,link->generation,cid,sdu,sizeof(sdu))==DBT_OK && !data_calls);
    result=commit();if(result)return result;CHECK(opened==1);
    CHECK(le_request(32,128,0x60,100,23,1)==DBT_OK);poll();result=take(0x15);if(result)return result;CHECK(r16(wire+12)==10);result=commit();if(result)return result;
    dbt_channels_clear(&manager);CHECK(le_allocated==le_released && !le_errors);return 0;
}
EXPORT int test_le_coc_security_and_key_size_qualification(void){
    const dbt_le_service_t s={128,{1000,27,1},DBT_SECURITY_AUTHENTICATED,16};
    le_fixture();CHECK(dbt_channels_register_le(&manager,&s,1)==DBT_OK);
    CHECK(le_request(40,128,0x60,100,23,1)==DBT_OK);poll();int result=take(0x15);if(result)return result;
    CHECK(r16(wire+12)==8 && !le_allocated);result=commit();if(result)return result;
    dbt_link_change_t change;uint8_t cipher[]={0,3,0,1};CHECK(link_event(8,cipher,sizeof(cipher),&change)==DBT_OK);
    CHECK(le_request(41,128,0x60,100,23,1)==DBT_OK);poll();result=take(0x15);if(result)return result;CHECK(r16(wire+12)==5);result=commit();if(result)return result;
    manager.hooks.le_security=qualify;
    for(unsigned code=5;code<=8;++code){le_policy=(uint16_t)code;CHECK(le_request((uint8_t)(42u+code),128,0x60,100,23,1)==DBT_OK);poll();
        result=take(0x15);if(result)return result;CHECK(r16(wire+12)==code && !le_allocated);result=commit();if(result)return result;}
    le_policy=0;permit=false;CHECK(le_request(51,128,0x60,100,23,1)==DBT_OK);poll();result=take(0x15);if(result)return result;
    CHECK(r16(wire+12)==6 && !le_allocated);result=commit();if(result)return result;
    permit=true;CHECK(le_request(52,128,0x60,100,23,1)==DBT_OK);poll();result=take(0x15);if(result)return result;
    CHECK(r16(wire+12)==0 && !opened);result=commit();if(result)return result;CHECK(opened==1 && le_security_calls>=6);
    dbt_channels_clear(&manager);CHECK(le_allocated==le_released && !le_errors);return 0;
}
EXPORT int test_le_coc_duplicate_grant_admission_and_deadline(void){
    const dbt_le_service_t s={128,{1000,27,1},DBT_SECURITY_NONE,0};
    le_fixture();CHECK(dbt_channels_register_le(&manager,&s,1)==DBT_OK);
    CHECK(le_request(60,128,0x60,100,23,1)==DBT_OK);poll();
    CHECK(le_request(60,128,0x60,100,23,1)==DBT_OK);poll();
    int result=take(0x15);if(result)return result;uint16_t cid=r16(wire+4);result=commit();if(result)return result;
    const dbt_link_t *link=dbt_acl_find(&acl,3);uint8_t data_packet[]={1,0,77};
    CHECK(dbt_channels_receive(&manager,3,link->generation,cid,data_packet,sizeof(data_packet))==DBT_OK && data_calls==1);
    result=take(0x15);if(result)return result;result=commit();if(result)return result;CHECK(opened==1 && le_allocated==1);
    /* Replayed success must not grant the initial receive credit again. */
    CHECK(dbt_channels_receive(&manager,3,link->generation,cid,data_packet,sizeof(data_packet))==DBT_OK && data_calls==1);
    CHECK(dbt_channels_find(&manager,3,cid)->state==DBT_CH_DISCONNECTING);dbt_channels_clear(&manager);
    CHECK(le_allocated==le_released && !le_errors);
    le_fixture();CHECK(dbt_channels_register_le(&manager,&s,1)==DBT_OK);
    CHECK(le_request(61,128,0x60,100,23,1)==DBT_OK);poll();result=take(0x15);if(result)return result;cid=r16(wire+4);result=commit();if(result)return result;
    link=dbt_acl_find(&acl,3);CHECK(dbt_channels_receive(&manager,3,link->generation,cid,data_packet,sizeof(data_packet))==DBT_OK);
    CHECK(dbt_channels_grant_le(&manager,3,cid,1)==DBT_OK && dbt_channels_grant_le(&manager,3,cid,1)==DBT_BUSY);
    /* The staged grant is not usable until its whole indication is admitted. */
    CHECK(dbt_channels_receive(&manager,3,link->generation,cid,data_packet,sizeof(data_packet))==DBT_OK && data_calls==1);
    dbt_channels_clear(&manager);CHECK(le_allocated==le_released);
    le_fixture();CHECK(dbt_channels_open_le(&manager,3,&s,now,&cid)==DBT_OK);result=take(0x14);if(result)return result;
    CHECK(wire_size==14 && r16(wire+4)==128 && r16(wire+6)==cid);result=commit();if(result)return result;
    now+=1001;dbt_channels_poll(&manager,now,4);
    CHECK(!dbt_channels_find(&manager,3,cid) && failed==1 && le_allocated==le_released && !le_errors);return 0;
}
EXPORT int test_le_coc_security_revoke_during_setup_and_open(void){
    const dbt_le_service_t service={128,{1000,27,1},DBT_SECURITY_AUTHENTICATED,16};
    le_fixture();manager.hooks.le_security=qualify;
    dbt_link_change_t change;uint8_t cipher[]={0,3,0,1};CHECK(link_event(8,cipher,sizeof(cipher),&change)==DBT_OK);
    CHECK(dbt_channels_register_le(&manager,&service,1)==DBT_OK);
    CHECK(le_request(80,128,0x60,100,23,1)==DBT_OK);poll();int result=take(0x15);if(result)return result;
    uint16_t cid=r16(wire+4);CHECK(!r16(wire+12) && !opened);
    le_policy=7;result=commit();if(result)return result;
    CHECK(!opened && dbt_channels_find(&manager,3,cid)->state==DBT_CH_DISCONNECTING);
    result=take(6);if(result)return result;uint8_t id=wire_id;result=commit();if(result)return result;
    uint8_t response[4];w16(response,0x60);w16(response+2,cid);CHECK(rx(3,5,7,id,response,4)==DBT_OK);poll();
    CHECK(failed==1 && !opened && le_allocated==le_released);
    le_fixture();manager.hooks.le_security=qualify;CHECK(link_event(8,cipher,sizeof(cipher),&change)==DBT_OK);
    CHECK(dbt_channels_register_le(&manager,&service,1)==DBT_OK);
    CHECK(le_request(81,128,0x60,100,23,1)==DBT_OK);poll();result=take(0x15);if(result)return result;cid=r16(wire+4);
    result=commit();if(result)return result;CHECK(opened==1);
    cipher[3]=0;CHECK(link_event(8,cipher,sizeof(cipher),&change)==DBT_OK);poll();
    CHECK(dbt_channels_find(&manager,3,cid)->state==DBT_CH_DISCONNECTING && !closed);
    result=take(6);if(result)return result;id=wire_id;result=commit();if(result)return result;
    w16(response+2,cid);CHECK(rx(3,5,7,id,response,4)==DBT_OK);poll();
    CHECK(closed==1 && le_allocated==le_released && !le_errors);
    le_fixture();manager.hooks.le_security=qualify;cipher[3]=1;CHECK(link_event(8,cipher,sizeof(cipher),&change)==DBT_OK);
    CHECK(dbt_channels_open_le(&manager,3,&service,now,&cid)==DBT_OK);result=take(0x14);if(result)return result;
    id=wire_id;result=commit();if(result)return result;le_policy=7;
    uint8_t success[10]={0x60,0,100,0,23,0,1,0,0,0};CHECK(rx(3,5,0x15,id,success,10)==DBT_OK);poll();
    CHECK(!opened && dbt_channels_find(&manager,3,cid)->state==DBT_CH_DISCONNECTING);
    dbt_channels_clear(&manager);CHECK(le_allocated==le_released && !le_errors);return 0;
}
EXPORT int test_le_coc_grant_queue_backpressure(void){
    const dbt_le_service_t service={128,{1000,27,1},DBT_SECURITY_NONE,0};
    le_fixture();CHECK(dbt_channels_register_le(&manager,&service,1)==DBT_OK);
    CHECK(le_request(90,128,0x60,100,23,1)==DBT_OK);poll();int result=take(0x15);if(result)return result;
    uint16_t cid=r16(wire+4);result=commit();if(result)return result;
    const dbt_link_t *link=dbt_acl_find(&acl,3);uint8_t whole[]={1,0,55};
    CHECK(dbt_channels_receive(&manager,3,link->generation,cid,whole,sizeof(whole))==DBT_OK && data_calls==1);
    for(unsigned i=0;i<DBT_SIGNAL_TX_SLOTS;++i){CHECK(rx(1,1,8,(uint8_t)(100u+i),0,0)==DBT_OK);poll();}
    CHECK(dbt_channels_grant_le(&manager,3,cid,1)==DBT_BUSY);
    for(unsigned i=0;i<DBT_SIGNAL_TX_SLOTS;++i){result=take(9);if(result)return result;result=commit();if(result)return result;}
    CHECK(dbt_channels_grant_le(&manager,3,cid,1)==DBT_OK);result=take(0x16);if(result)return result;
    CHECK(r16(wire+4)==cid && r16(wire+6)==1);result=commit();if(result)return result;
    CHECK(dbt_channels_receive(&manager,3,link->generation,cid,whole,sizeof(whole))==DBT_OK && data_calls==2);
    CHECK(dbt_channels_find(&manager,3,cid)->state==DBT_CH_OPEN);
    dbt_channels_clear(&manager);CHECK(le_allocated==le_released && !le_errors);return 0;
}
EXPORT uint32_t test_channels_bytes(void) {return sizeof(manager);}

EXPORT int test_channels_limits_ids_and_protocol_rejections(void) {
    fixture();uint16_t local=0;uint8_t previous=0;
    for(unsigned i=0;i<270;++i) {
        CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,++now,&local)==DBT_OK);
        int result=take(2);if(result)return result;CHECK(wire_id!=0 && wire_id!=previous);previous=wire_id;
        result=commit();if(result)return result;
        uint8_t refused[8]={0,0,0,0,2,0,0,0};w16(refused+2,local);
        CHECK(rx(1,1,3,previous,refused,8)==DBT_OK);poll();CHECK(dbt_channels_find(&manager,1,local)==0);
    }
    CHECK(failed==270);
    fixture();
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i) {
        CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,++now,&local)==DBT_OK);
    }
    CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,++now,&local)!=DBT_OK);
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i) {int result=take(2);if(result)return result;result=commit();if(result)return result;}
    dbt_service_t service={0x19,256,48,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&manager,service)==DBT_OK);
    uint8_t request[4]={0x19,0,0x90,0};CHECK(rx(1,1,2,31,request,4)==DBT_OK);poll();
    int result=take(3);if(result)return result;CHECK(r16(wire+8)==4);result=commit();if(result)return result;
    fixture();uint8_t config_id=0;
    result=connected_outgoing(1,256,0x80,&local,&config_id);if(result)return result;
    uint8_t unsupported[15]={0x40,0,0,0,4,9,3,1,1,0,0,0,0,0,0};
    CHECK(rx(1,1,4,32,unsupported,15)==DBT_OK);poll();result=take(5);if(result)return result;
    CHECK(r16(wire+8)==1 && wire[10]==4 && wire[11]==9 && wire[12]==0);
    result=commit();if(result)return result;
    uint8_t bad_success[17]={0x40,0,0,0,0,0,4,9,3,0,0,0,0,0,0,0,0};
    CHECK(rx(1,1,5,config_id,bad_success,17)==DBT_OK);poll();
    CHECK(failed==1 && opened==0 && last_reason==DBT_CH_REASON_PROTOCOL);
    return 0;
}

EXPORT int test_channels_timer_wrap_and_late_pending_bound(void) {
    fixture();uint16_t local=0;dbt_signal_view_t view;
    CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,UINT32_MAX-499u,&local)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&manager,&view)==DBT_OK);
    uint8_t id=view.data[1];CHECK(dbt_channels_tx_commit(&manager,UINT32_MAX-499u)==DBT_OK);
    dbt_channels_poll(&manager,499,32);CHECK(failed==0);
    dbt_channels_poll(&manager,500,32);CHECK(failed==1);
    fixture();CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_NONE,0,&local)==DBT_OK);
    CHECK(dbt_channels_tx_peek(&manager,&view)==DBT_OK);id=view.data[1];
    CHECK(dbt_channels_tx_commit(&manager,0)==DBT_OK);
    uint8_t waiting[8]={0,0,0,0,1,0,0,0};w16(waiting+2,local);
    CHECK(rx(1,1,3,id,waiting,8)==DBT_OK);dbt_channels_poll(&manager,1,32);
    CHECK(rx(1,1,3,id,waiting,8)==DBT_OK);dbt_channels_poll(&manager,300001,32);
    CHECK(failed==1 && dbt_channels_find(&manager,1,local)==0);
    return 0;
}

EXPORT int test_channels_incoming_waits_for_verified_security(void){
    fixture();manager.hooks.security_request=security_request;security_progress=DBT_BUSY;security_starts=0;
    dbt_service_t service={0x19,256,48,DBT_SECURITY_ENCRYPTED};CHECK(dbt_channels_register(&manager,service)==DBT_OK);
    uint8_t request[]={0x19,0,0x80,0};CHECK(rx(1,1,2,22,request,4)==DBT_OK);poll();
    const dbt_channel_t *channel=dbt_channels_find(&manager,1,0x40);CHECK(channel && channel->state==DBT_CH_SECURING && security_starts==1);
    uint32_t deadline=channel->deadline;
    int result=take(3);if(result)return result;CHECK(r16(wire+8)==1 && r16(wire+10)==1);
    acl.links[0].encrypted=true;security_progress=DBT_OK;poll();
    CHECK(channel->state==DBT_CH_SECURING && manager.tx_count==1 && !channel->connection_ready);
    result=commit();if(result)return result;poll();result=take(3);if(result)return result;
    CHECK(r16(wire+8)==0 && channel->state==DBT_CH_CONFIGURING && !channel->connection_ready);
    result=commit();if(result)return result;CHECK(channel->connection_ready);
    result=take(4);if(result)return result;uint8_t config_id=wire_id;result=commit();if(result)return result;
    uint8_t own_response[6]={0x40,0,0,0,0,0};CHECK(rx(1,1,5,config_id,own_response,6)==DBT_OK);
    uint8_t peer_request[8]={0x40,0,0,0,1,2,0,1};CHECK(rx(1,1,4,23,peer_request,8)==DBT_OK);poll();
    result=take(5);if(result)return result;result=commit();if(result)return result;
    CHECK(opened==1 && channel->state==DBT_CH_OPEN && deadline>1000);
    permit=false;uint8_t payload[]={1,2};dbt_l2cap_tx_t tx;
    CHECK(dbt_channels_receive(&manager,1,channel->link_generation,0x40,payload,2)==DBT_UNSUPPORTED);
    CHECK(dbt_channels_prepare_data(&manager,1,0x40,payload,2,&tx)==DBT_BUSY && data_calls==0);return 0;
}
EXPORT int test_channels_security_duplicates_failure_and_timeout(void){
    fixture();manager.hooks.security_request=security_request;security_progress=DBT_BUSY;security_starts=0;
    dbt_service_t service={0x19,256,48,DBT_SECURITY_ENCRYPTED};CHECK(dbt_channels_register(&manager,service)==DBT_OK);
    uint8_t request[]={0x19,0,0x80,0};CHECK(rx(1,1,2,22,request,4)==DBT_OK);poll();
    uint32_t deadline=dbt_channels_find(&manager,1,0x40)->deadline;
    int result=take(3);if(result)return result;result=commit();if(result)return result;
    CHECK(rx(1,1,2,22,request,4)==DBT_OK);poll();result=take(3);if(result)return result;
    CHECK(r16(wire+8)==1 && security_starts==1 && dbt_channels_find(&manager,1,0x40)->deadline==deadline);
    result=commit();if(result)return result;security_progress=DBT_INVALID;poll();
    result=take(3);if(result)return result;CHECK(r16(wire+8)==3 && failed==1 && last_reason==DBT_CH_REASON_SECURITY);
    result=commit();if(result)return result;
    fixture();manager.hooks.security_request=security_request;security_progress=DBT_BUSY;
    CHECK(dbt_channels_register(&manager,service)==DBT_OK);CHECK(rx(1,1,2,22,request,4)==DBT_OK);poll();
    deadline=dbt_channels_find(&manager,1,0x40)->deadline;result=take(3);if(result)return result;result=commit();if(result)return result;
    dbt_channels_poll(&manager,deadline,32);result=take(3);if(result)return result;
    CHECK(r16(wire+8)==3 && failed==1 && last_reason==DBT_CH_REASON_LOCAL_TIMEOUT);return 0;
}
EXPORT int test_channels_outgoing_security_wait_and_cipher_gate(void){
    fixture();manager.hooks.security_request=security_request;security_progress=DBT_BUSY;security_starts=0;uint16_t cid=0;
    CHECK(dbt_channels_open(&manager,1,0x19,256,48,DBT_SECURITY_ENCRYPTED,++now,&cid)==DBT_OK);
    dbt_signal_view_t view;CHECK(dbt_channels_tx_peek(&manager,&view)==DBT_EMPTY);
    CHECK(dbt_channels_find(&manager,1,cid)->state==DBT_CH_SECURING && security_starts==1);
    security_progress=DBT_OK;poll();CHECK(dbt_channels_tx_peek(&manager,&view)==DBT_EMPTY);
    acl.links[0].encrypted=true;poll();int result=take(2);if(result)return result;
    CHECK(r16(wire+4)==0x19 && r16(wire+6)==cid && dbt_channels_find(&manager,1,cid)->state==DBT_CH_CONNECTING);return 0;
}
