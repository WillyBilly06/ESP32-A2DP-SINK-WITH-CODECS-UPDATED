#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
/* This fixture exercises the actual store/cache with opaque synthetic tags.
 * Cryptographic bond-tag generation is covered by the real-PSA reconnect
 * fixture; it must never be invoked by this cache test. */
#define DBT_SHA256_H
static unsigned unexpected_hash_calls;
static int dbt_sha256(const uint8_t *p,size_t n,uint8_t out[32],int is224){
    (void)p;(void)n;(void)out;(void)is224;++unexpected_hash_calls;return -1;
}
#include "../../esp32_probe/main/probe_cccd_store.c"
#define CHECK(x) do {if(!(x))return __LINE__;}while(0)
typedef struct {char key[14];bool present;size_t length;uint8_t value[80];} slot_t;
static slot_t disk[2],staged;
static unsigned opens,gets,sets,commits,closes;
static bool fail_open,fail_get,fail_set,fail_commit,persist_on_error;
static int index_for(const char *key,bool create){
    for(unsigned i=0;i<2;++i)if(disk[i].present&&!strcmp(disk[i].key,key))return (int)i;
    if(create)for(unsigned i=0;i<2;++i)if(!disk[i].present)return (int)i;
    return -1;
}
esp_err_t nvs_open(const char *ns,int mode,nvs_handle_t *out){
    ++opens;if(strcmp(ns,"native_cccd")||mode!=NVS_READWRITE||fail_open)return 1;*out=1;return ESP_OK;
}
void nvs_close(nvs_handle_t h){(void)h;++closes;}
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *out,size_t *n){
    (void)h;++gets;if(fail_get)return 1;int i=index_for(key,false);if(i<0)return ESP_ERR_NVS_NOT_FOUND;
    if(*n<disk[i].length){*n=disk[i].length;return 1;}
    *n=disk[i].length;memcpy(out,disk[i].value,*n);return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *data,size_t n){
    (void)h;++sets;if(fail_set||n>sizeof(staged.value)||strlen(key)!=13)return 1;
    staged=(slot_t){.present=true,.length=n};memcpy(staged.key,key,14);memcpy(staged.value,data,n);return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h){
    (void)h;++commits;if(!fail_commit||persist_on_error){int i=index_for(staged.key,true);if(i<0)return 1;disk[i]=staged;}
    return fail_commit?1:ESP_OK;
}
static const uint8_t a[6]={1,2,3,4,5,6},b[6]={1,2,3,4,5,7};
static const uint8_t tag1[32]={1},tag2[32]={[0]=1,[31]=2};
static void reset(void){memset(disk,0,sizeof disk);memset(&staged,0,sizeof staged);memset(&cccd_cache,0,sizeof cccd_cache);
    poisoned=fail_open=fail_get=fail_set=fail_commit=persist_on_error=false;opens=gets=sets=commits=closes=0;}
static int reuse_and_durability(void){
    reset();uint16_t v[4]={2,1,0,1},loaded[4];
    CHECK(probe_cccd_load(0,a,tag1,loaded)==DBT_EMPTY&&!cccd_cache.valid);
    CHECK(probe_cccd_save(0,a,tag1,v)==DBT_OK&&cccd_cache.valid);
    unsigned before_get=gets,before_set=sets,before_commit=commits;
    for(unsigned i=0;i<64;++i){v[3]^=1;CHECK(probe_cccd_save(0,a,tag1,v)==DBT_OK);}
    CHECK(gets==before_get&&sets==before_set+64&&commits==before_commit+64);
    unsigned before_open=opens;before_get=gets;before_set=sets;before_commit=commits;
    CHECK(probe_cccd_save(0,a,tag1,v)==DBT_OK);
    CHECK(opens==before_open&&gets==before_get&&sets==before_set&&commits==before_commit);
    /* Simulated reboot drops only volatile state, then verifies actual backing. */
    memset(&cccd_cache,0,sizeof cccd_cache);poisoned=false;
    CHECK(probe_cccd_load(0,a,tag1,loaded)==DBT_OK&&!memcmp(v,loaded,sizeof v)&&gets==before_get+1);
    CHECK(opens==closes);return 0;
}
static int identity_and_incarnation(void){
    reset();uint16_t va[4]={2,1,1,0},vb[4]={0,0,0,1},zero[4]={0},loaded[4];
    CHECK(probe_cccd_save(0,a,tag1,va)==DBT_OK);unsigned n=gets;
    CHECK(probe_cccd_save(0,b,tag1,vb)==DBT_OK&&gets==n+1);
    CHECK(probe_cccd_load(0,a,tag1,loaded)==DBT_OK&&!memcmp(loaded,va,sizeof va));
    n=gets;CHECK(probe_cccd_save(0,a,tag2,zero)==DBT_OK&&gets==n+1);
    CHECK(probe_cccd_load(0,a,tag1,loaded)==DBT_EMPTY);
    CHECK(probe_cccd_load(0,a,tag2,loaded)==DBT_OK&&!memcmp(loaded,zero,sizeof zero));
    CHECK(probe_cccd_load(0,b,tag1,loaded)==DBT_OK&&!memcmp(loaded,vb,sizeof vb));return 0;
}
static int invalidation_and_validation(void){
    reset();uint16_t v[4]={2,1,0,0},loaded[4];CHECK(probe_cccd_save(0,a,tag1,v)==DBT_OK);
    unsigned n=sets;uint16_t bad[4]={1,1,0,0};
    CHECK(probe_cccd_save(0,a,tag1,bad)==DBT_INVALID&&sets==n&&cccd_cache.valid);
    fail_get=true;CHECK(probe_cccd_load(0,a,tag1,loaded)==DBT_INVALID&&!cccd_cache.valid);
    CHECK(probe_cccd_save(0,a,tag1,v)==DBT_INVALID&&sets==n&&!poisoned);fail_get=false;
    CHECK(probe_cccd_load(0,a,tag1,loaded)==DBT_OK);
    disk[0].length=50;CHECK(probe_cccd_load(0,a,tag1,loaded)==DBT_INVALID&&!cccd_cache.valid);
    CHECK(probe_cccd_save(0,a,tag1,v)==DBT_INVALID&&sets==n);disk[0].length=51;
    CHECK(probe_cccd_load(0,a,tag1,loaded)==DBT_OK);
    fail_open=true;v[3]=1;CHECK(probe_cccd_save(0,a,tag1,v)==DBT_INVALID&&!cccd_cache.valid&&!poisoned);
    fail_open=false;CHECK(probe_cccd_save(0,a,tag1,v)==DBT_OK);return 0;
}
static int uncertain_writes(void){
    for(unsigned persisted=0;persisted<2;++persisted){
        reset();uint16_t old[4]={2,1,0,0},next[4]={2,1,0,1},loaded[4];
        CHECK(probe_cccd_save(0,a,tag1,old)==DBT_OK);fail_commit=true;persist_on_error=persisted!=0;
        CHECK(probe_cccd_save(0,a,tag1,next)==DBT_INVALID&&poisoned&&!cccd_cache.valid);
        CHECK(probe_cccd_save(0,a,tag1,old)==DBT_INVALID&&probe_cccd_load(0,a,tag1,loaded)==DBT_INVALID);
        fail_commit=false;poisoned=false;memset(&cccd_cache,0,sizeof cccd_cache);
        CHECK(probe_cccd_load(0,a,tag1,loaded)==DBT_OK);
        CHECK(!memcmp(loaded,persisted?next:old,sizeof old));
    }
    reset();uint16_t v[4]={2,1,0,0};CHECK(probe_cccd_save(0,a,tag1,v)==DBT_OK);
    fail_set=true;v[3]=1;CHECK(probe_cccd_save(0,a,tag1,v)==DBT_INVALID&&poisoned&&!cccd_cache.valid);
    return 0;
}
int main(void){
    int (*cases[])(void)={reuse_and_durability,identity_and_incarnation,invalidation_and_validation,uncertain_writes};
    for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);++i){int line=cases[i]();if(line){fprintf(stderr,"FAIL CCCD cache case%u line%d\n",i,line);return 1;}}
    CHECK(!unexpected_hash_calls);
    puts("PASS actual CCCD store:64 changed saves without duplicate reads, verified no-op, re-read on load, identity/incarnation isolation, malformed/read/open errors and both uncertain-commit outcomes; mocked NVS, no crypto or physical power-loss proof");return 0;
}
