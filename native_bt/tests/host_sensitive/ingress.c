#include "dbt_host.h"
#include <string.h>
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
typedef struct {
    dbt_host_t host;
    bool ready,abort_in_callback,alive_after_abort;
    unsigned observed;
    uint8_t last[64]; uint16_t last_size;
} fixture_t;
static fixture_t f;
static bool ready(void *v) { return ((fixture_t *)v)->ready; }
static bool send(void *v,uint8_t *p,uint16_t n) { (void)p; (void)n; ((fixture_t *)v)->ready=false; return true; }
static void observe(void *v,const dbt_hci_packet_t *p) {
    fixture_t *x=v; ++x->observed; x->last_size=p->size;
    if(p->size<=sizeof(x->last)) memcpy(x->last,p->payload,p->size);
    if(x->abort_in_callback) {
        uint8_t before=p->size?p->payload[0]:0;
        dbt_host_abort(&x->host,DBT_FAULT_CONTROLLER);
        x->alive_after_abort=!p->size || p->payload[0]==before;
    }
}
static void init(void) {
    memset(&f,0,sizeof(f)); f.ready=true; dbt_transport_t t={&f,ready,send};
    (void)dbt_host_init(&f.host,t,observe,&f,2000); f.host.state=DBT_HOST_IDENTIFIED;
}
static bool zero(const uint8_t *p,unsigned n) { for(unsigned i=0;i<n;i++) if(p[i]) return false; return true; }
static void acl(uint8_t *frame,uint16_t payload,uint8_t pb,uint8_t fill) {
    frame[0]=2; frame[1]=1; frame[2]=(uint8_t)(pb<<4); frame[3]=(uint8_t)payload; frame[4]=(uint8_t)(payload>>8);
    for(unsigned i=0;i<payload;i++) frame[5+i]=(uint8_t)(fill+i);
}
static bool ring_zero(uint32_t start,unsigned n) {
    for(unsigned i=0;i<n;i++) if(f.host.ingress.bytes[(start+2u+i)&(DBT_QUEUE_BYTES-1u)]) return false;
    return true;
}
EXPORT int host_sensitive_acl_and_events(void) {
    init(); uint8_t a[53],b[12],event[26]; acl(a,48,2,0x71); acl(b,7,1,0x41);
    uint32_t sa=atomic_load(&f.host.ingress.written);
    CHECK(dbt_host_receive(&f.host,a,sizeof(a))==DBT_OK);
    uint32_t sb=atomic_load(&f.host.ingress.written);
    CHECK(dbt_host_receive(&f.host,b,sizeof(b))==DBT_OK);
    CHECK(dbt_host_poll(&f.host,1,2)==2 && f.observed==2 && f.last_size==7 && !memcmp(f.last,b+5,7));
    CHECK(ring_zero(sa,sizeof(a)) && ring_zero(sb,sizeof(b)) && zero(f.host.rx_packet,sizeof(a)));
    event[0]=4; event[1]=0x18; event[2]=23; memset(event+3,0x53,23);
    uint32_t se=atomic_load(&f.host.ingress.written);
    CHECK(dbt_host_receive(&f.host,event,sizeof(event))==DBT_OK && dbt_host_poll(&f.host,2,1)==1);
    CHECK(ring_zero(se,sizeof(event)) && zero(f.host.rx_packet,sizeof(event)));
    uint8_t dh[]={4,0x3e,1,9}; se=atomic_load(&f.host.ingress.written);
    CHECK(dbt_host_receive(&f.host,dh,sizeof(dh))==DBT_OK && dbt_host_poll(&f.host,3,1)==1);
    CHECK(ring_zero(se,sizeof(dh)) && zero(f.host.rx_packet,sizeof(a)));
    return 0;
}
EXPORT int host_sensitive_abort_and_borrow(void) {
    init(); uint8_t a[25],b[27]; acl(a,20,2,0x71); acl(b,22,1,0x91);
    uint32_t sa=atomic_load(&f.host.ingress.written); CHECK(dbt_host_receive(&f.host,a,sizeof(a))==DBT_OK);
    uint32_t sb=atomic_load(&f.host.ingress.written); CHECK(dbt_host_receive(&f.host,b,sizeof(b))==DBT_OK);
    f.abort_in_callback=true;
    CHECK(dbt_host_poll(&f.host,1,8)==1 && f.alive_after_abort && f.observed==1);
    CHECK(f.host.state==DBT_HOST_FAILED && !dbt_queue_used(&f.host.ingress));
    CHECK(ring_zero(sa,sizeof(a)) && ring_zero(sb,sizeof(b)) && zero(f.host.rx_packet,sizeof(f.host.rx_packet)));
    CHECK(dbt_host_receive(&f.host,a,sizeof(a))==DBT_OK && !dbt_queue_used(&f.host.ingress));
    return 0;
}
EXPORT int host_sensitive_abort_with_queued_and_late(void) {
    init(); uint8_t a[25],b[17]; acl(a,20,2,0x51); acl(b,12,1,0x61);
    uint32_t sa=atomic_load(&f.host.ingress.written); CHECK(dbt_host_receive(&f.host,a,sizeof(a))==DBT_OK);
    dbt_host_abort(&f.host,DBT_FAULT_CONTROLLER);
    CHECK(ring_zero(sa,sizeof(a)) && !dbt_queue_used(&f.host.ingress));
    /* Model a producer that passed the stop gate before abort and published
     * after the first drain. The next owner poll must discard that record. */
    uint32_t sb=atomic_load(&f.host.ingress.written);
    CHECK(dbt_queue_push_sensitive(&f.host.ingress,b,sizeof(b),true)==DBT_OK);
    CHECK(dbt_host_wait_ms(&f.host,2)==0 && dbt_host_poll(&f.host,2,8)==0);
    CHECK(ring_zero(sb,sizeof(b)) && !dbt_queue_used(&f.host.ingress));
    CHECK(zero(f.host.rx_packet,sizeof(f.host.rx_packet)));
    return 0;
}
EXPORT int host_sensitive_error_paths(void) {
    init(); uint8_t a[23]; acl(a,18,2,0x81);
    uint32_t start=atomic_load(&f.host.ingress.written); CHECK(dbt_host_receive(&f.host,a,sizeof(a))==DBT_OK);
    uint8_t malformed[]={4,0xff,1}; CHECK(dbt_host_receive(&f.host,malformed,sizeof(malformed))==DBT_INVALID);
    CHECK(dbt_host_poll(&f.host,1,8)==0 && f.host.fault==DBT_FAULT_BAD_PACKET);
    CHECK(ring_zero(start,sizeof(a)) && zero(f.host.rx_packet,sizeof(f.host.rx_packet)));
    init(); acl(a,18,2,0x81); start=atomic_load(&f.host.ingress.written);
    CHECK(dbt_host_receive(&f.host,a,sizeof(a))==DBT_OK);
    f.host.ingress.bytes[(start+2u)&(DBT_QUEUE_BYTES-1u)]=0xff; /* forced corruption after publish */
    CHECK(dbt_host_poll(&f.host,1,8)==0 && f.host.fault_stage==3);
    CHECK(ring_zero(start,sizeof(a)) && zero(f.host.rx_packet,sizeof(f.host.rx_packet)));
    init(); acl(a,18,2,0x81); start=atomic_load(&f.host.ingress.written);
    CHECK(dbt_host_receive(&f.host,a,sizeof(a))==DBT_OK);
    f.host.ingress.bytes[start&(DBT_QUEUE_BYTES-1u)]=0;
    f.host.ingress.bytes[(start+1u)&(DBT_QUEUE_BYTES-1u)]=0x80; /* zero-length corrupt record */
    CHECK(dbt_host_poll(&f.host,1,8)==0 && f.host.fault_stage==2);
    CHECK(ring_zero(start,sizeof(a)) && !dbt_queue_used(&f.host.ingress));
    return 0;
}
EXPORT int host_sensitive_control_reserve(void) {
    init(); uint8_t a[1026]; acl(a,1021,2,0x31);
    for(unsigned i=0;i<3;i++) CHECK(dbt_host_receive(&f.host,a,sizeof(a))==DBT_OK);
    CHECK(dbt_host_receive(&f.host,a,sizeof(a))==DBT_NO_SPACE);
    CHECK(dbt_host_poll(&f.host,1,8)==0 && f.host.fault==DBT_FAULT_RX_OVERRUN);
    CHECK(!dbt_queue_used(&f.host.ingress) && zero(f.host.rx_packet,sizeof(f.host.rx_packet)));
    return 0;
}
