#include "probe_ble_parameters.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"failed line %u\n",__LINE__);return 1;}}while(0)
static dbt_runtime_t runtime;
static struct {bool pending,submitted;uint16_t opcode;uint8_t data[14],size;uint32_t token;void *context;dbt_command_done_t done;} command;
static probe_ble_parameters_record_t records[128];
static unsigned count,submissions;
static uint32_t next_token;
static dbt_result_t le_result=DBT_EMPTY;
static uint16_t le_wire;
static bool le_started;
dbt_result_t dbt_channels_request_le_parameters(dbt_channels_t *m,uint16_t handle,uint16_t min,uint16_t max,uint16_t latency,uint16_t timeout,uint32_t now){
    (void)m;(void)now;if(handle||min!=18||max!=24||latency||timeout!=400)return DBT_INVALID;
    if(le_result==DBT_BUSY)return DBT_BUSY;le_started=true;le_result=DBT_BUSY;return DBT_OK;
}
dbt_result_t dbt_channels_le_parameters_result(const dbt_channels_t *m,uint16_t handle,uint16_t *result){
    (void)m;(void)handle;if(le_result==DBT_OK)*result=le_wire;return le_result;
}
dbt_result_t dbt_channels_cancel_le_parameters(dbt_channels_t *m,uint16_t handle){(void)m;(void)handle;le_result=DBT_CANCELLED;return DBT_OK;}
uint16_t dbt_read_le16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
const dbt_link_t *dbt_acl_find(const dbt_acl_t *a,uint16_t handle){
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)if(a->links[i].used&&a->links[i].handle==handle)return &a->links[i];return 0;
}
dbt_result_t dbt_host_command_tracked(dbt_host_t *host,uint16_t opcode,const uint8_t *data,uint8_t size,
    dbt_command_response_t response,dbt_command_done_t done,void *context,uint32_t now,uint32_t *token){
    (void)now;if(host->command_count==DBT_COMMAND_SLOTS)return DBT_BUSY;
    if(command.pending||size>14||response!=DBT_COMMAND_STATUS)return DBT_INVALID;
    command.pending=true;command.submitted=false;command.opcode=opcode;command.size=size;memcpy(command.data,data,size);
    command.token=++next_token;command.context=context;command.done=done;*token=command.token;++submissions;++host->command_count;return DBT_OK;
}
static void status(uint8_t value){
    dbt_command_done_t callback=command.done;void *context=command.context;uint16_t opcode=command.opcode;
    command.pending=false;runtime.host.command_count=0;callback(context,opcode,value?DBT_COMMAND_REJECTED:DBT_COMMAND_SUCCESS,value,0,0);
}
dbt_command_cancel_result_t dbt_host_cancel_command(dbt_host_t *host,uint32_t token){
    if(!command.pending||command.token!=token)return DBT_COMMAND_NOT_FOUND;
    if(command.submitted)return DBT_COMMAND_ALREADY_SUBMITTED;
    dbt_command_done_t callback=command.done;void *context=command.context;uint16_t opcode=command.opcode;
    command.pending=false;host->command_count=0;callback(context,opcode,DBT_COMMAND_ABORTED,0xff,0,0);return DBT_COMMAND_CANCELLED;
}
void probe_ble_parameters_log(const probe_ble_parameters_record_t *r){if(count<128)records[count++]=*r;}
static void add(uint32_t gen,bool central){
    le_result=DBT_EMPTY;le_started=false;
    runtime.acl.links[0]=(dbt_link_t){.handle=0,.generation=gen,.used=true,.kind=DBT_LINK_LE,.role=central?0:1};
    dbt_link_change_t c={DBT_LINK_ADDED,0,gen};probe_ble_parameters_link(&runtime,&c);
}
static void remove_link(uint32_t gen){runtime.acl.links[0].used=false;dbt_link_change_t c={DBT_LINK_REMOVED,0,gen};probe_ble_parameters_link(&runtime,&c);}
static void event(const uint8_t *data,uint16_t size){dbt_hci_packet_t p={.type=DBT_H4_EVENT,.event=0x3e,.payload=data,.size=size};probe_ble_parameters_event(&runtime,&p);}
static void poll(uint32_t now){runtime.now_ms=now;probe_ble_parameters_poll(&runtime,now,false);}
int main(void){
    runtime.ready=true;runtime.host.state=DBT_HOST_IDENTIFIED;runtime.host.controller.le_features[0]=2;
    uint8_t initial[19]={1,0,0,0,1,0,0,0,0,0,0,0,6,0,0,0,0x90,1,0};
    uint8_t features[12]={4,0,0,0,2},updated[10]={3,0,0,0,24,0,0,0,0x90,1};
    add(1,false);event(initial,sizeof(initial));CHECK(records[count-1].interval==6);
    CHECK(probe_ble_parameters_wait_ms(0)==0);poll(0);
    CHECK(command.pending&&command.opcode==0x2016&&command.size==2&&dbt_read_le16(command.data)==0);
    status(0);CHECK(probe_ble_parameters_wait_ms(0)>0);event(features,sizeof(features));poll(1);
    CHECK(command.opcode==0x2013&&command.size==14&&dbt_read_le16(command.data+2)==18&&dbt_read_le16(command.data+4)==24);
    CHECK(!dbt_read_le16(command.data+6)&&dbt_read_le16(command.data+8)==400&&!dbt_read_le16(command.data+10)&&!dbt_read_le16(command.data+12));
    status(0);event(updated,sizeof(updated));CHECK(records[count-1].stage==4&&records[count-1].interval==24&&records[count-1].status==0);
    CHECK(probe_ble_parameters_wait_ms(1)==UINT32_MAX);remove_link(1);
    /* No retry spin while the tracked command queue is full. */
    add(2,false);runtime.host.command_count=DBT_COMMAND_SLOTS;unsigned before=submissions;poll(2);
    CHECK(submissions==before&&probe_ble_parameters_wait_ms(2)>0);
    runtime.host.command_count=0;CHECK(probe_ble_parameters_wait_ms(2)==0);poll(2);remove_link(2);
    CHECK(!command.pending&&probe_ble_parameters_wait_ms(2)==UINT32_MAX);
    /* Disconnect cannot recycle a callback context still owned by HCI. */
    add(3,false);poll(3);command.submitted=true;remove_link(3);add(4,false);before=submissions;poll(4);CHECK(submissions==before);
    status(0);CHECK(probe_ble_parameters_wait_ms(4)==0);poll(4);
    event(features,sizeof(features));CHECK(command.pending&&command.opcode==0x2016); /* late old event before new admission */
    status(0);event(features,sizeof(features));poll(5);remove_link(4);CHECK(!command.pending);
    /* Admission success is not an applied update; timeout leaves link intact. */
    add(5,false);poll(10);status(0);poll(50000);CHECK(runtime.acl.links[0].used&&probe_ble_parameters_wait_ms(50000)==UINT32_MAX);remove_link(5);
    /* The original LE signaling path supports a controller/peer without LL CPR. */
    runtime.host.controller.le_features[0]=0;add(6,false);before=submissions;poll(50001);CHECK(submissions==before&&le_started);
    event(updated,sizeof(updated));le_result=DBT_OK;le_wire=0;poll(50001);CHECK(probe_ble_parameters_wait_ms(50001)==UINT32_MAX&&records[count-1].l2cap_result==0);remove_link(6);
    runtime.host.controller.le_features[0]=2;add(7,false);poll(50002);status(0);features[4]=0;event(features,sizeof(features));before=submissions;poll(50003);CHECK(submissions==before&&le_started);
    le_result=DBT_OK;le_wire=1;poll(50003);CHECK(records[count-1].stage==5&&records[count-1].l2cap_result==1);remove_link(7);features[4]=2;
    /* One asynchronous collision retry, no repeated controller submissions. */
    add(8,true);poll(50004);status(0x0c);CHECK(probe_ble_parameters_wait_ms(50004)==500);
    before=submissions;poll(50503);CHECK(submissions==before);poll(50504);CHECK(submissions==before+1);status(0x0c);CHECK(probe_ble_parameters_wait_ms(50504)==UINT32_MAX);remove_link(8);
    /* Restart hold cancels queued preference work without another command. */
    add(9,false);poll(50505);probe_ble_parameters_poll(&runtime,50505,true);CHECK(!command.pending&&probe_ble_parameters_wait_ms(50505)==UINT32_MAX);remove_link(9);
    runtime.host.controller.le_features[0]=0;add(10,false);poll(50506);CHECK(le_started);
    probe_ble_parameters_poll(&runtime,50506,true);CHECK(le_result==DBT_CANCELLED&&probe_ble_parameters_wait_ms(50506)==UINT32_MAX);remove_link(10);
    add(11,false);le_result=DBT_BUSY;poll(50507);CHECK(!le_started&&probe_ble_parameters_wait_ms(50507)>0);
    le_result=DBT_EMPTY;CHECK(probe_ble_parameters_wait_ms(50507)==0);poll(50507);CHECK(le_started);
    le_result=DBT_OK;le_wire=0;poll(50508);remove_link(11); /* old controller event is still pending */
    add(12,false);poll(50509);CHECK(!le_started&&probe_ble_parameters_wait_ms(50509)>0);
    event(updated,sizeof(updated));CHECK(probe_ble_parameters_wait_ms(50509)==0);poll(50509);CHECK(le_started);
    poll(100510);CHECK(le_result==DBT_CANCELLED&&runtime.acl.links[0].used&&probe_ble_parameters_wait_ms(100510)==UINT32_MAX);remove_link(12);
    puts("PASS BLE policy bytes, HCI/LE admission versus actual update, queue backpressure, callback lifetime, reuse, timeout, bounded retry and hold; fake transports only");return 0;
}
