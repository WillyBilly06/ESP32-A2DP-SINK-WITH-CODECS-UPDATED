#include "dbt_host.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do { if(!(x))return __LINE__; } while(0)
typedef struct {
    dbt_host_t host;
    bool ready,accept;
    unsigned sends,callbacks;
    uint16_t last_opcode;
    dbt_command_outcome_t last_outcome;
    uint8_t sent[259];uint16_t sent_size;
    uint32_t cancel_on_event;
    dbt_command_cancel_result_t cancel_result;
    bool enqueue_in_callback;
} fixture_t;
static fixture_t f,g;
static void zero(void *p,unsigned n){uint8_t *x=p;for(unsigned i=0;i<n;++i)x[i]=0;}
static bool can_send(void *p){return ((fixture_t *)p)->ready;}
static bool send(void *p,uint8_t *data,uint16_t n){
    fixture_t *s=p;if(!s->ready || !s->accept)return false;
    for(unsigned i=0;i<n;++i)s->sent[i]=data[i];s->sent_size=n;s->ready=false;++s->sends;return true;
}
static void done(void *p,uint16_t opcode,dbt_command_outcome_t outcome,uint8_t status,const uint8_t *data,uint8_t size){
    fixture_t *s=p;(void)status;(void)data;(void)size;++s->callbacks;s->last_opcode=opcode;s->last_outcome=outcome;
    if(s->enqueue_in_callback){s->enqueue_in_callback=false;if(dbt_host_command(&s->host,0xfc55,0,0,DBT_COMMAND_COMPLETE,0,0,20)!=DBT_OK)++s->callbacks;}
}
static void observer(void *p,const dbt_hci_packet_t *packet){
    fixture_t *s=p;if(packet->type==DBT_H4_EVENT && packet->event==5 && s->cancel_on_event){
        uint32_t token=s->cancel_on_event;s->cancel_on_event=0;
        s->cancel_result=dbt_host_cancel_command(&s->host,token);
    }
}
static int init(fixture_t *s){
    zero(s,sizeof(*s));s->accept=true;
    dbt_transport_t transport={s,can_send,send};
    CHECK(dbt_host_init(&s->host,transport,observer,s,100)==DBT_OK);
    s->host.state=DBT_HOST_IDENTIFIED;s->host.command_credits=1;return 0;
}
static dbt_result_t tracked(fixture_t *s,uint16_t opcode,const uint8_t *p,uint8_t n,uint32_t *token){
    return dbt_host_command_tracked(&s->host,opcode,p,n,DBT_COMMAND_COMPLETE,done,s,10,token);
}
static bool no_pattern(const dbt_host_t *host,uint8_t value){
    for(unsigned s=0;s<DBT_COMMAND_SLOTS;++s)
        for(unsigned i=0;i<sizeof(host->commands[s].parameters);++i)
            if(host->commands[s].parameters[i]==value)return false;
    return true;
}
EXPORT int cancel_queued_order_and_wipe(void){
    CHECK(init(&f)==0);uint8_t a[]={0x11},secret[16],c[]={0x33},d[]={0x44};
    for(unsigned i=0;i<sizeof(secret);++i)secret[i]=0xa7;
    uint32_t ta=0,tb=0,tc=0,td=0;
    CHECK(tracked(&f,0xfc01,a,sizeof(a),&ta)==DBT_OK);
    CHECK(tracked(&f,0x201a,secret,sizeof(secret),&tb)==DBT_OK);
    CHECK(tracked(&f,0xfc03,c,sizeof(c),&tc)==DBT_OK);
    CHECK(tracked(&f,0xfc04,d,sizeof(d),&td)==DBT_OK);
    uint32_t unchanged=99;CHECK(tracked(&f,0xfc05,0,0,&unchanged)==DBT_NO_SPACE && unchanged==99);
    CHECK(dbt_host_cancel_command(&f.host,tb)==DBT_COMMAND_CANCELLED);
    CHECK(f.callbacks==1 && f.last_opcode==0x201a && f.last_outcome==DBT_COMMAND_ABORTED);
    CHECK(f.host.command_count==3 && f.host.command_credits==1 && !f.host.pending_opcode && no_pattern(&f.host,0xa7));
    CHECK(f.host.commands[0].token==ta && f.host.commands[1].token==tc && f.host.commands[2].token==td);
    CHECK(f.host.commands[0].deadline==110 && f.host.commands[1].deadline==110 && f.host.commands[2].deadline==110);
    CHECK(dbt_host_cancel_command(&f.host,tb)==DBT_COMMAND_NOT_FOUND);
    CHECK(dbt_host_cancel_command(&f.host,0)==DBT_COMMAND_NOT_FOUND);
    f.enqueue_in_callback=true;CHECK(dbt_host_cancel_command(&f.host,tc)==DBT_COMMAND_CANCELLED);
    CHECK(f.callbacks==2 && f.host.command_count==3 && f.host.commands[0].token==ta && f.host.commands[1].token==td &&
          f.host.commands[2].opcode==0xfc55 && f.host.commands[2].token==0);
    return 0;
}
EXPORT int cancel_observer_prevents_submit(void){
    CHECK(init(&f)==0);uint8_t secret[16];for(unsigned i=0;i<16;++i)secret[i]=0xa5;
    uint32_t token=0;CHECK(tracked(&f,0x201a,secret,16,&token)==DBT_OK);
    uint8_t disconnection[]={4,5,4,0,1,0,0x13};
    CHECK(dbt_host_receive(&f.host,disconnection,sizeof(disconnection))==DBT_OK);
    f.cancel_on_event=token;f.ready=true;dbt_host_poll(&f.host,20,4);
    CHECK(f.cancel_result==DBT_COMMAND_CANCELLED && f.callbacks==1 && f.sends==0 && f.host.command_credits==1 &&
          !f.host.command_count && no_pattern(&f.host,0xa5));
    return 0;
}
EXPORT int cancel_admitted_and_following(void){
    CHECK(init(&f)==0);uint8_t key[16];for(unsigned i=0;i<16;++i)key[i]=0xb6;
    uint32_t first=0,second=0;CHECK(tracked(&f,0x201a,key,16,&first)==DBT_OK);
    f.ready=true;dbt_host_poll(&f.host,11,0);
    CHECK(f.sends==1 && f.host.pending_opcode==0x201a && f.host.command_credits==0 && f.host.tx_retained);
    CHECK(dbt_host_cancel_command(&f.host,first)==DBT_COMMAND_ALREADY_SUBMITTED && f.callbacks==0);
    CHECK(tracked(&f,0xfc01,key,16,&second)==DBT_OK);
    CHECK(dbt_host_cancel_command(&f.host,second)==DBT_COMMAND_CANCELLED && f.callbacks==1 && f.last_opcode==0xfc01);
    CHECK(f.host.pending_opcode==0x201a && f.host.command_count==1 && f.host.command_credits==0 && no_pattern(&f.host,0xb6));
    uint8_t complete[]={4,0x0e,4,1,0x1a,0x20,0};
    CHECK(dbt_host_receive(&f.host,complete,sizeof(complete))==DBT_OK);
    dbt_host_poll(&f.host,12,4);
    CHECK(f.callbacks==2 && f.last_opcode==0x201a && f.last_outcome==DBT_COMMAND_SUCCESS && !f.host.command_count && f.host.command_credits==1);
    CHECK(dbt_host_cancel_command(&f.host,first)==DBT_COMMAND_NOT_FOUND);
    f.ready=true;dbt_host_poll(&f.host,13,0);
    CHECK(!f.host.tx_retained);
    for(unsigned i=0;i<sizeof(f.host.tx_command);++i)CHECK(f.host.tx_command[i]==0);
    return 0;
}
EXPORT int cancel_reinit_and_host_identity(void){
    CHECK(init(&f)==0);CHECK(init(&g)==0);uint32_t old=0,next=0,other=0;
    CHECK(tracked(&f,0xfc01,0,0,&old)==DBT_OK);
    dbt_host_abort(&f.host,DBT_FAULT_BAD_PACKET);
    CHECK(dbt_host_cancel_command(&f.host,old)==DBT_COMMAND_NOT_FOUND);
    CHECK(init(&f)==0);CHECK(tracked(&f,0xfc02,0,0,&next)==DBT_OK);
    CHECK(tracked(&g,0xfc03,0,0,&other)==DBT_OK);
    CHECK(old && next && other && old!=next && old!=other && next!=other);
    CHECK(dbt_host_cancel_command(&f.host,old)==DBT_COMMAND_NOT_FOUND);
    CHECK(dbt_host_cancel_command(&f.host,other)==DBT_COMMAND_NOT_FOUND);
    CHECK(dbt_host_cancel_command(&g.host,other)==DBT_COMMAND_CANCELLED);
    CHECK(dbt_host_cancel_command(&f.host,next)==DBT_COMMAND_CANCELLED);
    return 0;
}
EXPORT int cancel_wrap_and_rejected_transport(void){
    CHECK(init(&f)==0);uint8_t complete[]={4,0x0e,4,1,1,0xfc,0};uint32_t temp=0;
    for(unsigned i=0;i<2;++i){
        CHECK(tracked(&f,0xfc01,0,0,&temp)==DBT_OK);f.ready=true;dbt_host_poll(&f.host,11+i*2,0);
        CHECK(dbt_host_receive(&f.host,complete,sizeof(complete))==DBT_OK);dbt_host_poll(&f.host,12+i*2,4);
    }
    CHECK(f.host.command_head==2 && f.host.command_count==0);
    uint8_t secret[16];for(unsigned i=0;i<16;++i)secret[i]=0xc9;
    uint32_t a=0,b=0,c=0,d=0;
    CHECK(tracked(&f,0xfc10,0,0,&a)==DBT_OK && tracked(&f,0x201a,secret,16,&b)==DBT_OK &&
          tracked(&f,0xfc12,0,0,&c)==DBT_OK && tracked(&f,0xfc13,0,0,&d)==DBT_OK);
    CHECK(dbt_host_cancel_command(&f.host,b)==DBT_COMMAND_CANCELLED && no_pattern(&f.host,0xc9));
    CHECK(f.host.command_head==2 && f.host.command_count==3 && f.host.commands[2].token==a &&
          f.host.commands[3].token==c && f.host.commands[0].token==d && f.host.commands[1].token==0);
    CHECK(dbt_host_cancel_command(&f.host,a)==DBT_COMMAND_CANCELLED);
    CHECK(dbt_host_cancel_command(&f.host,c)==DBT_COMMAND_CANCELLED);
    CHECK(dbt_host_cancel_command(&f.host,d)==DBT_COMMAND_CANCELLED && !f.host.command_count);
    uint32_t rejected=0;CHECK(tracked(&f,0x201a,secret,16,&rejected)==DBT_OK);
    f.ready=true;f.accept=false;unsigned previous=f.sends;dbt_host_poll(&f.host,20,0);
    CHECK(f.sends==previous && !f.host.pending_opcode && f.host.command_credits==1 && f.host.command_count==1);
    for(unsigned i=0;i<sizeof(f.host.tx_command);++i)CHECK(f.host.tx_command[i]==0);
    CHECK(dbt_host_cancel_command(&f.host,rejected)==DBT_COMMAND_CANCELLED && no_pattern(&f.host,0xc9));
    return 0;
}
EXPORT int cancel_abort_preserves_transport_borrow(void){
    CHECK(init(&f)==0);uint8_t secret[16];for(unsigned i=0;i<16;++i)secret[i]=0xd7;
    uint32_t token;CHECK(tracked(&f,0x201a,secret,16,&token)==DBT_OK);
    f.ready=true;dbt_host_poll(&f.host,11,0);CHECK(f.host.tx_retained && f.sends==1);
    dbt_host_abort(&f.host,DBT_FAULT_BAD_PACKET);
    CHECK(f.callbacks==1 && f.last_outcome==DBT_COMMAND_ABORTED && f.host.tx_retained);
    CHECK(dbt_host_cancel_command(&f.host,token)==DBT_COMMAND_NOT_FOUND);
    for(unsigned i=0;i<16;++i)CHECK(f.host.tx_command[i+4]==0xd7);
    f.ready=true;dbt_host_poll(&f.host,12,0);CHECK(!f.host.tx_retained);
    for(unsigned i=0;i<sizeof(f.host.tx_command);++i)CHECK(f.host.tx_command[i]==0);
    return 0;
}
