#include "dbt_ota_esp.h"
#include "esp_ota_ops.h"
#include "esp_image_format.h"
#include "mbedtls/sha256.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
static const esp_partition_t running={0,0x10,0x10000,1024,"ota_0"};
static const esp_partition_t update={0,0x11,0x20000,1024,"ota_1"};
static const esp_partition_t recovery={0,0x12,0x30000,1024,"recovery"};
static const esp_partition_t *boot=&running;
static uint8_t flash[1024];static size_t written;static bool image_ok=true,hash_appended=true,read_ok=true,valid=true,slot_recorded,oom;static unsigned boot_writes,select_failure;
void *ota_test_calloc(size_t count,size_t size){if(oom)return 0;void *p=malloc(count*size);if(p)memset(p,0,count*size);return p;}
static const uint8_t known_sha[32]={0x15,0xe2,0xb0,0xd3,0xc3,0x38,0x91,0xeb,0xb0,0xf1,0xef,0x60,0x9e,0xc4,0x19,0x42,
    0x0c,0x20,0xe3,0x20,0xce,0x94,0xc6,0x5f,0xbc,0x8c,0x33,0x12,0x44,0x8e,0xb2,0x25};
const esp_partition_t *esp_partition_find_first(int type,int sub,const char *label) {
    (void)type;(void)sub;if(!strcmp(label,"ota_0"))return &running;if(!strcmp(label,"ota_1"))return &update;
    if(!strcmp(label,"recovery"))return &recovery;return 0;
}
const esp_partition_t *esp_ota_get_running_partition(void){return &running;}
const esp_partition_t *esp_ota_get_boot_partition(void){return boot;}
int esp_ota_resume(const esp_partition_t *p,size_t n,size_t off,esp_ota_handle_t *h){assert(p==&update && n==OTA_WITH_SEQUENTIAL_WRITES && !off);memset(flash,0xff,sizeof(flash));written=0;*h=1;return 0;}
int esp_ota_write(esp_ota_handle_t h,const void *p,size_t n){assert(h==1 && written+n<=sizeof(flash));memcpy(flash+written,p,n);written+=n;return 0;}
int esp_ota_end(esp_ota_handle_t h){assert(h==1);return image_ok?0:1;}
int esp_ota_abort(esp_ota_handle_t h){assert(h==1);return 0;}
int esp_ota_get_partition_description(const esp_partition_t *p,esp_app_desc_t *d){assert(p);memset(d,0,sizeof(*d));memcpy(d->project_name,"native_bt_probe",16);return 0;}
int esp_ota_get_state_partition(const esp_partition_t *p,esp_ota_img_states_t *s){if(p==&update)return slot_recorded?0:ESP_ERR_NOT_FOUND;assert(p==&running);*s=ESP_OTA_IMG_VALID;return valid?0:1;}
int esp_ota_set_boot_partition(const esp_partition_t *p){assert(p==&update);if(select_failure==1)return 1;boot=p;boot_writes++;return select_failure?1:0;}
int esp_image_verify(int mode,const esp_partition_pos_t *p,esp_image_metadata_t *m){assert(mode==0 && p->offset==update.address);m->image_len=9;m->image.hash_appended=hash_appended;return image_ok?0:1;}
int esp_partition_read(const esp_partition_t *p,size_t off,void *out,size_t n){assert(p==&update && off+n<=sizeof(flash));if(!read_ok)return 1;memcpy(out,flash+off,n);return 0;}
void mbedtls_sha256_init(mbedtls_sha256_context *s){memset(s,0,sizeof(*s));}
int mbedtls_sha256_starts(mbedtls_sha256_context *s,int mode){(void)s;assert(mode==0);return 0;}
int mbedtls_sha256_update(mbedtls_sha256_context *s,const unsigned char *p,size_t n){assert(s->size+n<=sizeof(s->bytes));memcpy(s->bytes+s->size,p,n);s->size+=n;return 0;}
int mbedtls_sha256_finish(mbedtls_sha256_context *s,unsigned char *out){memset(out,0,32);if(s->size==9 && !memcmp(s->bytes,"123456789",9))memcpy(out,known_sha,32);return 0;}
void mbedtls_sha256_free(mbedtls_sha256_context *s){memset(s,0,sizeof(*s));}
#ifndef DBT_OTA_SHIM_LIB
static void staged(dbt_ota_esp_t *s,dbt_ota_manifest_t *m) {
    assert(dbt_ota_esp_init(s,"ota_1") && dbt_ota_esp_begin(s,m));
    assert(!dbt_ota_esp_write(s,1,(const uint8_t *)"123",3));
    assert(dbt_ota_esp_write(s,0,(const uint8_t *)"123456789",9));
}
typedef struct {dbt_ota_esp_t esp;} control_context_t;
static bool owner_grant(void *v,const dbt_ble_control_peer_t *p,uint32_t n,dbt_ota_manifest_t *m) {
    (void)v;if(p->handle!=7 || p->generation!=42 || n!=9)return false;
    memset(m,0,sizeof(*m));m->length=n;memcpy(m->project,"native_bt_probe",16);return true;
}
static uint32_t clock_ms(void *v){(void)v;return 1;}
static bool control_begin(void *v,const dbt_ota_manifest_t *m){return dbt_ota_esp_begin(&((control_context_t *)v)->esp,m);}
static bool control_write(void *v,uint32_t off,const uint8_t *p,uint16_t n){return dbt_ota_esp_write(&((control_context_t *)v)->esp,off,p,n);}
static bool control_finish(void *v,const dbt_ota_manifest_t *m){return dbt_ota_esp_finish(&((control_context_t *)v)->esp,m);}
static bool control_abort(void *v){return dbt_ota_esp_abort(&((control_context_t *)v)->esp);}
static bool control_activate(void *v,const dbt_ota_manifest_t *m){return dbt_ota_esp_activate(&((control_context_t *)v)->esp,m);}
int main(void) {
    dbt_ota_manifest_t m={0};m.length=9;memcpy(m.project,"native_bt_probe",16);memcpy(m.sha256,known_sha,32);
    dbt_ota_esp_t s;assert(dbt_ota_esp_init(&s,"ota_0") && !dbt_ota_esp_begin(&s,&m));
    assert(!dbt_ota_esp_init(&s,"recovery"));
    assert(dbt_ota_esp_init(&s,"missing") && !dbt_ota_esp_begin(&s,&m));
    assert(dbt_ota_esp_init(&s,"ota_1"));slot_recorded=true;assert(dbt_ota_esp_begin(&s,&m));assert(dbt_ota_esp_abort(&s));slot_recorded=false;
    oom=true;assert(!dbt_ota_esp_begin(&s,&m) && !s.session);oom=false;
    m.length=1025;assert(!dbt_ota_esp_begin(&s,&m));m.length=9;
    boot=&recovery;assert(!dbt_ota_esp_begin(&s,&m));boot=&running;
    staged(&s,&m);assert(dbt_ota_esp_finish(&s,&m) && boot==&running && !boot_writes);
    valid=false;assert(!dbt_ota_esp_activate(&s,&m) && !boot_writes);valid=true;
    flash[0]='X';assert(!dbt_ota_esp_activate(&s,&m) && !boot_writes);flash[0]='1';
    assert(dbt_ota_esp_activate(&s,&m) && boot==&update && boot_writes==1);assert(dbt_ota_esp_abort(&s));
    boot=&running;staged(&s,&m);image_ok=false;assert(!dbt_ota_esp_finish(&s,&m) && !s.session);image_ok=true;
    staged(&s,&m);read_ok=false;assert(!dbt_ota_esp_finish(&s,&m) && !s.session);read_ok=true;
    staged(&s,&m);m.trusted_digest=true;m.sha256[0]^=1;assert(!dbt_ota_esp_finish(&s,&m) && !s.session);m.sha256[0]^=1;m.trusted_digest=false;
    staged(&s,&m);hash_appended=false;assert(!dbt_ota_esp_finish(&s,&m) && !s.session);hash_appended=true;
    staged(&s,&m);m.project[0]='x';assert(!dbt_ota_esp_finish(&s,&m) && !s.session);m.project[0]='n';
    staged(&s,&m);assert(dbt_ota_esp_abort(&s) && !s.session && boot==&running);
    /* Full baseline control-parser -> OTA state -> SDK shim, reusing an
     * historically recorded inactive slot under an explicit peer grant. */
    control_context_t app;assert(dbt_ota_esp_init(&app.esp,"ota_1"));slot_recorded=true;
    dbt_ota_hooks_t hooks={&app,owner_grant,clock_ms,control_begin,control_write,control_finish,control_abort,control_activate};
    dbt_ota_t ota;assert(dbt_ota_init(&ota,&hooks,100));
    dbt_ble_control_hooks_t ble={0};ble.context=&ota;ble.command=dbt_ota_control_command;
    dbt_ble_control_state_t display={0};display.valid=true;dbt_ble_control_peer_t peer={7,42};dbt_ble_control_result_t reply;
    uint8_t begin_wire[]={0x20,9,0,0,0},data_wire[]={0x21,1,'1','2','3','4','5','6','7','8','9'},end_wire[]={0x22};
    assert(dbt_ble_control_execute(&display,&ble,&peer,begin_wire,sizeof(begin_wire),&reply)==DBT_OK && reply.data[0]==0x21);
    assert(dbt_ble_control_execute(&display,&ble,&peer,data_wire,sizeof(data_wire),&reply)==DBT_OK && reply.data[0]==0x20 && reply.data[1]==100);
    assert(dbt_ble_control_execute(&display,&ble,&peer,end_wire,sizeof(end_wire),&reply)==DBT_OK && reply.data[0]==0x22 && boot==&running);
    dbt_ota_request_cancel(&ota);dbt_ota_tick(&ota);assert(ota.state==DBT_OTA_IDLE && !app.esp.session && boot==&running);
    return 0;
}
#endif
