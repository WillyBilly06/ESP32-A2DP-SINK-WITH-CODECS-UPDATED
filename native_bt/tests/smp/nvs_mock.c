#include "nvs.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* Bounded multi-key NVS model. A power cycle discards staged entries while
 * preserving committed entries. No claim about physical flash semantics. */
#define MAX_KEYS 64
#define MAX_BYTES 256
typedef struct {
    char key[16]; unsigned char durable[MAX_BYTES],pending[MAX_BYTES];
    size_t durable_size,pending_size;
    bool used,have_durable,have_pending,pending_delete;
} entry_t;
static entry_t entries[MAX_KEYS];
static bool fail_commit,off,commit_after_failure;
static unsigned commit_calls,fail_commit_call,cut_commit_call,fail_read_call,read_calls,fail_erase_call,erase_calls;
static char read_key[16],erase_key[16];
struct nvs_opaque_iterator_t { unsigned index; };
static entry_t *find(const char *key,bool create) {
    entry_t *empty=NULL;
    for (unsigned i=0;i<MAX_KEYS;i++) {
        if (entries[i].used && !strcmp(entries[i].key,key)) return &entries[i];
        if (!entries[i].used && !empty) empty=&entries[i];
    }
    if (!create || !empty || strlen(key)>=sizeof(empty->key)) return NULL;
    empty->used=true; memcpy(empty->key,key,strlen(key)+1); return empty;
}
void fake_nvs_reset(void) {
    memset(entries,0,sizeof(entries)); fail_commit=false; off=false; commit_after_failure=false;
    commit_calls=fail_commit_call=cut_commit_call=fail_read_call=read_calls=fail_erase_call=erase_calls=0;
    memset(read_key,0,sizeof(read_key)); memset(erase_key,0,sizeof(erase_key));
}
void fake_nvs_fail_commit(bool fail) { fail_commit=fail; }
void fake_nvs_fail_commit_at(unsigned call,bool applied) { commit_calls=0; fail_commit_call=call; commit_after_failure=applied; }
void fake_nvs_cut_after_commit(unsigned call) { commit_calls=0; cut_commit_call=call; }
void fake_nvs_fail_read(const char *key,unsigned call) {
    memset(read_key,0,sizeof(read_key)); if(key && strlen(key)<sizeof(read_key)) memcpy(read_key,key,strlen(key)+1);
    read_calls=0; fail_read_call=call;
}
void fake_nvs_fail_erase(const char *key,unsigned call) {
    memset(erase_key,0,sizeof(erase_key)); if(key && strlen(key)<sizeof(erase_key)) memcpy(erase_key,key,strlen(key)+1);
    erase_calls=0; fail_erase_call=call;
}
void fake_nvs_power_cycle(void) {
    for (unsigned i=0;i<MAX_KEYS;i++) {
        memset(entries[i].pending,0,sizeof(entries[i].pending)); entries[i].have_pending=false; entries[i].pending_delete=false;
        if (entries[i].used && !entries[i].have_durable) memset(&entries[i],0,sizeof(entries[i]));
    }
    off=false; fail_commit=false; fail_commit_call=cut_commit_call=fail_read_call=fail_erase_call=0;
    commit_calls=read_calls=erase_calls=0;
}
void fake_nvs_corrupt(void) {
    for (unsigned i=0;i<MAX_KEYS;i++) if(entries[i].have_durable && strcmp(entries[i].key,"smp_txn_v1")) {
        entries[i].durable[0]^=1; return;
    }
}
void fake_nvs_corrupt_key(const char *key,unsigned offset) {
    entry_t *e=find(key,false); if(e && e->have_durable && offset<e->durable_size) e->durable[offset]^=1;
}
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *out,size_t *n) {
    (void)h; if(off) return 1;
    if (fail_read_call && !strcmp(key,read_key) && ++read_calls==fail_read_call) return 1;
    entry_t *e=find(key,false);
    if (!e || (e->have_pending ? e->pending_delete : !e->have_durable)) return ESP_ERR_NVS_NOT_FOUND;
    const unsigned char *data=e->have_pending?e->pending:e->durable;
    size_t size=e->have_pending?e->pending_size:e->durable_size;
    if(!out) { *n=size; return ESP_OK; }
    if(*n<size) { *n=size; return 1; }
    memcpy(out,data,size); *n=size; return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *data,size_t n) {
    (void)h; if(off || n>MAX_BYTES) return 1;
    entry_t *e=find(key,true); if(!e) return 1;
    memcpy(e->pending,data,n); e->pending_size=n; e->have_pending=true; e->pending_delete=false; return ESP_OK;
}
esp_err_t nvs_erase_key(nvs_handle_t h,const char *key) {
    (void)h; if(off) return 1;
    if (fail_erase_call && !strcmp(key,erase_key) && ++erase_calls==fail_erase_call) return 1;
    entry_t *e=find(key,false);
    if (!e || (e->have_pending ? e->pending_delete : !e->have_durable)) return ESP_ERR_NVS_NOT_FOUND;
    e->have_pending=true; e->pending_delete=true; return ESP_OK;
}
static void apply(void) {
    for (unsigned i=0;i<MAX_KEYS;i++) if(entries[i].have_pending) {
        entry_t *e=&entries[i];
        if(e->pending_delete) { e->have_durable=false; memset(e->durable,0,sizeof(e->durable)); }
        else { memcpy(e->durable,e->pending,e->pending_size); e->durable_size=e->pending_size; e->have_durable=true; }
        memset(e->pending,0,sizeof(e->pending)); e->have_pending=false; e->pending_delete=false;
    }
}
esp_err_t nvs_commit(nvs_handle_t h) {
    (void)h; if(off) return 1;
    ++commit_calls;
    if(fail_commit || (fail_commit_call && commit_calls==fail_commit_call)) {
        if(commit_after_failure) apply();
        return 1;
    }
    apply();
    if(cut_commit_call && commit_calls==cut_commit_call) off=true;
    return ESP_OK;
}
esp_err_t nvs_entry_find_in_handle(nvs_handle_t h,nvs_type_t type,nvs_iterator_t *out) {
    (void)h; (void)type; if(off || !out) return 1;
    unsigned i=0; while(i<MAX_KEYS && !entries[i].have_durable) ++i;
    if(i==MAX_KEYS) { *out=NULL; return ESP_ERR_NVS_NOT_FOUND; }
    *out=malloc(sizeof(**out)); if(!*out) return 1; (*out)->index=i; return ESP_OK;
}
esp_err_t nvs_entry_next(nvs_iterator_t *it) {
    if(off || !it || !*it) return 1;
    unsigned i=(*it)->index+1; while(i<MAX_KEYS && !entries[i].have_durable) ++i;
    if(i==MAX_KEYS) { free(*it); *it=NULL; return ESP_ERR_NVS_NOT_FOUND; }
    (*it)->index=i; return ESP_OK;
}
esp_err_t nvs_entry_info(nvs_iterator_t it,nvs_entry_info_t *out) {
    if(off || !it || !out) return 1;
    memset(out,0,sizeof(*out)); memcpy(out->key,entries[it->index].key,16); out->type=NVS_TYPE_BLOB; return ESP_OK;
}
void nvs_release_iterator(nvs_iterator_t it) { free(it); }
