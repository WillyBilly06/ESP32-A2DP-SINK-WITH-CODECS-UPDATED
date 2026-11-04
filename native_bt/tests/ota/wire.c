#include "dbt_ota.h"
#include <assert.h>
#include <string.h>

typedef struct {
    uint8_t slot[512];uint32_t length,now;unsigned writes,aborts,finishes,activations;
    bool grant,allocated,verified,boot_selected,fail_begin,fail_write,fail_finish,fail_abort;
    dbt_ble_control_peer_t peer;
    dbt_ota_manifest_t manifest;
} fake_t;
static bool grant(void *v,const dbt_ble_control_peer_t *p,uint32_t n,dbt_ota_manifest_t *out) {
    fake_t *f=v;if(!f->grant || p->handle!=f->peer.handle || p->generation!=f->peer.generation || n!=f->manifest.length)return false;
    *out=f->manifest;return true;
}
static uint32_t now(void *v){return ((fake_t *)v)->now;}
static bool begin(void *v,const dbt_ota_manifest_t *m) {
    fake_t *f=v;if(f->fail_begin || f->allocated || m->length>sizeof(f->slot))return false;
    memset(f->slot,0xff,sizeof(f->slot));f->length=0;f->allocated=true;f->verified=false;return true;
}
static bool write_flash(void *v,uint32_t off,const uint8_t *p,uint16_t n) {
    fake_t *f=v;if(f->fail_write || !f->allocated || off!=f->length || off+n>sizeof(f->slot))return false;
    memcpy(f->slot+off,p,n);f->length+=n;f->writes++;return true;
}
static bool finish(void *v,const dbt_ota_manifest_t *m) {
    fake_t *f=v;f->finishes++;f->allocated=false;
    /* Fake backend models checksum/project/chip/SHA rejection by matching a
     * known complete image. It is not evidence of ESP image validation. */
    f->verified=!f->fail_finish && m->length==9 && !memcmp(f->slot,"123456789",9);
    return f->verified;
}
static bool abort_job(void *v) {fake_t *f=v;f->aborts++;if(f->fail_abort)return false;f->allocated=f->verified=false;return true;}
static bool activate(void *v,const dbt_ota_manifest_t *m){fake_t *f=v;(void)m;if(!f->verified)return false;f->activations++;f->boot_selected=true;return true;}
static void setup(dbt_ota_t *s,fake_t *f) {
    memset(f,0,sizeof(*f));f->peer=(dbt_ble_control_peer_t){7,1};f->grant=true;
    f->manifest.length=9;f->manifest.crc32=0xcbf43926;f->manifest.trusted_digest=true;memcpy(f->manifest.project,"owned",6);
    const uint8_t digest[]={0x15,0xe2,0xb0,0xd3,0xc3,0x38,0x91,0xeb,0xb0,0xf1,0xef,0x60,0x9e,0xc4,0x19,0x42,
                            0x0c,0x20,0xe3,0x20,0xce,0x94,0xc6,0x5f,0xbc,0x8c,0x33,0x12,0x44,0x8e,0xb2,0x25};
    memcpy(f->manifest.sha256,digest,32);
    dbt_ota_hooks_t h={f,grant,now,begin,write_flash,finish,abort_job,activate};assert(dbt_ota_init(s,&h,100));
}
static void run(dbt_ota_t *s,const dbt_ble_control_peer_t *p,uint8_t command,const uint8_t *data,uint16_t n,uint8_t code,uint8_t param) {
    uint8_t out[82],size=sizeof(out);assert(dbt_ota_command(s,p,command,data,n,out,&size));
    assert(size && out[0]==code);if(size==2)assert(out[1]==param);
}
int main(void) {
    dbt_ota_t s;fake_t f;setup(&s,&f);uint8_t start[]={9,0,0,0},a[]={0,'1','2','3','4'},b[]={1,'5','6','7','8','9'};
    run(&s,&f.peer,0x20,start,4,0x21,0);run(&s,&f.peer,0x21,a,sizeof(a),0x20,44);
    dbt_ble_control_peer_t stale={7,2};run(&s,&stale,0x21,b,sizeof(b),0x23,0x12);assert(f.writes==1);
    run(&s,&f.peer,0x21,b,sizeof(b),0x20,100);run(&s,&f.peer,0x22,0,0,0x22,0);
    assert(s.state==DBT_OTA_VERIFIED && f.finishes==1 && !f.boot_selected);
    assert(!dbt_ota_activate(&s,&stale) && !f.boot_selected);
    assert(dbt_ota_activate(&s,&f.peer) && f.boot_selected && f.activations==1);
    setup(&s,&f);run(&s,&f.peer,0x20,start,4,0x21,0);run(&s,&f.peer,0x21,a,sizeof(a),0x20,44);
    run(&s,&f.peer,0x21,a,sizeof(a),0x23,0x11);assert(!f.allocated && s.state==DBT_OTA_IDLE && !f.boot_selected);
    setup(&s,&f);run(&s,&f.peer,0x20,start,4,0x21,0);run(&s,&f.peer,0x21,a,sizeof(a),0x20,44);
    b[0]=2;run(&s,&f.peer,0x21,b,sizeof(b),0x23,0x11);b[0]=1;
    assert(!f.allocated && !f.boot_selected);
    setup(&s,&f);f.manifest.length=257;uint8_t long_start[]={1,1,0,0};run(&s,&f.peer,0x20,long_start,4,0x21,0);
    for(unsigned i=0;i<257;i++) {uint8_t chunk[]={(uint8_t)i,'1'};
        run(&s,&f.peer,0x21,chunk,2,0x20,(uint8_t)(((i+1u)*100u)/257u));}
    run(&s,&f.peer,0x22,0,0,0x23,0x12);assert(!f.finishes && !f.boot_selected);
    setup(&s,&f);run(&s,&f.peer,0x20,start,4,0x21,0);run(&s,&f.peer,0x22,0,0,0x23,0x12);assert(!f.allocated);
    setup(&s,&f);run(&s,&f.peer,0x20,start,4,0x21,0);uint8_t over[]={0,1,2,3,4,5,6,7,8,9,10};
    run(&s,&f.peer,0x21,over,sizeof(over),0x23,0x11);assert(!f.writes && !f.allocated);
    setup(&s,&f);f.manifest.crc32=0;run(&s,&f.peer,0x20,start,4,0x21,0);run(&s,&f.peer,0x21,a,sizeof(a),0x20,44);
    run(&s,&f.peer,0x21,b,sizeof(b),0x20,100);run(&s,&f.peer,0x22,0,0,0x23,0x12);assert(!f.finishes);
    setup(&s,&f);f.fail_write=true;run(&s,&f.peer,0x20,start,4,0x21,0);run(&s,&f.peer,0x21,a,sizeof(a),0x23,0x11);assert(!f.allocated);
    setup(&s,&f);f.fail_begin=true;run(&s,&f.peer,0x20,start,4,0x23,0x10);assert(!f.allocated);
    setup(&s,&f);f.fail_abort=true;run(&s,&f.peer,0x20,start,4,0x21,0);dbt_ota_request_cancel(&s);dbt_ota_tick(&s);assert(s.state==DBT_OTA_FAULT);
    setup(&s,&f);run(&s,&f.peer,0x20,start,4,0x21,0);f.now=100;dbt_ota_tick(&s);assert(s.state==DBT_OTA_IDLE && !f.allocated);
    setup(&s,&f);run(&s,&f.peer,0x20,start,4,0x21,0);f.grant=false;dbt_ota_tick(&s);assert(s.state==DBT_OTA_IDLE && !f.allocated);
    setup(&s,&f);run(&s,&f.peer,0x20,start,4,0x21,0);run(&s,&f.peer,0x21,a,sizeof(a),0x20,44);
    /* Power loss: a fresh state has no activation authority and never selects
     * the partially written fake slot, which remains solely on the inactive side. */
    dbt_ota_t rebooted;dbt_ota_hooks_t hooks=s.hooks;assert(dbt_ota_init(&rebooted,&hooks,100));
    assert(!dbt_ota_activate(&rebooted,&f.peer) && !f.boot_selected);
    /* Finished but not activated also remains unselected across reboot. */
    setup(&s,&f);run(&s,&f.peer,0x20,start,4,0x21,0);run(&s,&f.peer,0x21,a,sizeof(a),0x20,44);
    run(&s,&f.peer,0x21,b,sizeof(b),0x20,100);run(&s,&f.peer,0x22,0,0,0x22,0);
    hooks=s.hooks;assert(dbt_ota_init(&rebooted,&hooks,100));assert(!dbt_ota_activate(&rebooted,&f.peer) && !f.boot_selected);
    setup(&s,&f);f.manifest.trusted_digest=false;
    run(&s,&f.peer,0x20,start,4,0x21,0);a[0]=1;b[0]=2;
    run(&s,&f.peer,0x21,a,sizeof(a),0x20,44);run(&s,&f.peer,0x21,b,sizeof(b),0x20,100);
    run(&s,&f.peer,0x22,0,0,0x22,0);assert(!f.boot_selected);a[0]=0;b[0]=1;
    setup(&s,&f);dbt_ble_control_state_t state={0};state.valid=true;
    dbt_ble_control_hooks_t control={0};control.context=&s;control.command=dbt_ota_control_command;
    dbt_ble_control_result_t result;uint8_t wire_start[]={0x20,9,0,0,0};
    assert(dbt_ble_control_execute(&state,&control,&f.peer,wire_start,sizeof(wire_start),&result)==DBT_OK &&
           result.size==1 && result.data[0]==0x21);
    dbt_ota_request_cancel(&s);dbt_ota_tick(&s);assert(s.state==DBT_OTA_IDLE && !f.allocated);
    return 0;
}
