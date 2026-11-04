#include "dbt_ota_esp.h"
#include "esp_ota_ops.h"
#include "esp_image_format.h"
#include "esp_partition.h"
#include "dbt_sha256.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    const esp_partition_t *partition;esp_ota_handle_t handle;
    dbt_sha256_context stream;uint8_t stream_digest[32];
    uint32_t offset;bool active,verified,hash_live;
} session_t;
bool dbt_ota_esp_init(dbt_ota_esp_t *s,const char *label) {
    if(!s || !label || !label[0] || strlen(label)>16 || !strcmp(label,"recovery"))return false;
    s->label=label;s->session=0;return true;
}
static bool same_partition(const esp_partition_t *a,const esp_partition_t *b) {
    return a && b && a->address==b->address && a->size==b->size && a->type==b->type && a->subtype==b->subtype;
}
static bool safe_slot(const esp_partition_t *p,uint32_t size) {
    const esp_partition_t *run=esp_ota_get_running_partition(),*boot=esp_ota_get_boot_partition();
    esp_ota_img_states_t running_state;
    return p && p->type==ESP_PARTITION_TYPE_APP && p->subtype>=ESP_PARTITION_SUBTYPE_APP_OTA_MIN &&
           p->subtype<ESP_PARTITION_SUBTYPE_APP_OTA_MAX && size && size<=p->size &&
           run && boot && same_partition(run,boot) && !same_partition(p,run) && !same_partition(p,boot) &&
           run->subtype>=ESP_PARTITION_SUBTYPE_APP_OTA_MIN && run->subtype<ESP_PARTITION_SUBTYPE_APP_OTA_MAX &&
           esp_ota_get_state_partition(run,&running_state)==ESP_OK &&
           (running_state==ESP_OTA_IMG_VALID || running_state==ESP_OTA_IMG_UNDEFINED);
}
bool dbt_ota_esp_begin(void *context,const dbt_ota_manifest_t *m) {
    dbt_ota_esp_t *s=context;
    if(!s || !m || !s->label || s->session)return false;
    const esp_partition_t *p=esp_partition_find_first(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_ANY,s->label);
    if(!safe_slot(p,m->length))return false;
    session_t *job=calloc(1,sizeof(*job));if(!job)return false;
    job->partition=p;
    dbt_sha256_init(&job->stream);job->hash_live=true;
    if(dbt_sha256_starts(&job->stream,0)!=0) {dbt_sha256_free(&job->stream);free(job);return false;}
    /* begin() invalidates the inactive otadata record with rollback enabled.
     * Resume at offset zero with lazy sequential erase avoids that mutation. */
    if(esp_ota_resume(p,OTA_WITH_SEQUENTIAL_WRITES,0,&job->handle)!=ESP_OK) {dbt_sha256_free(&job->stream);free(job);return false;}
    job->active=true;s->session=job;return true;
}
bool dbt_ota_esp_write(void *context,uint32_t offset,const uint8_t *data,uint16_t size) {
    dbt_ota_esp_t *s=context;session_t *job=s?s->session:0;
    if(!job || !job->active || !data || !size || offset!=job->offset || size>job->partition->size-offset)return false;
    if(esp_ota_write(job->handle,data,size)!=ESP_OK || dbt_sha256_update(&job->stream,data,size)!=0)return false;
    job->offset+=size;return true;
}
static bool verify(const session_t *job,const dbt_ota_manifest_t *m) {
    if(!job || !m || job->offset!=m->length || !safe_slot(job->partition,m->length))return false;
    esp_partition_pos_t pos={.offset=job->partition->address,.size=job->partition->size};
    esp_image_metadata_t image;
    if(esp_image_verify(ESP_IMAGE_VERIFY,&pos,&image)!=ESP_OK || !image.image.hash_appended ||
       !image.image_len || image.image_len>m->length)return false;
    esp_app_desc_t candidate,running;
    const esp_partition_t *run=esp_ota_get_running_partition();
    if(esp_ota_get_partition_description(job->partition,&candidate)!=ESP_OK ||
       esp_ota_get_partition_description(run,&running)!=ESP_OK ||
       memcmp(candidate.project_name,m->project,sizeof(m->project)) ||
       memcmp(running.project_name,m->project,sizeof(m->project)))return false;
    dbt_sha256_context hash;dbt_sha256_init(&hash);
    bool ok=dbt_sha256_starts(&hash,0)==0;uint8_t bytes[512],digest[32];
    for(uint32_t off=0;ok && off<m->length;) {
        uint32_t count=m->length-off;if(count>sizeof(bytes))count=sizeof(bytes);
        if(esp_partition_read(job->partition,off,bytes,count)!=ESP_OK || dbt_sha256_update(&hash,bytes,count)!=0)ok=false;
        off+=count;
    }
    if(ok)ok=dbt_sha256_finish(&hash,digest)==0 && !memcmp(digest,job->stream_digest,sizeof(digest)) &&
             (!m->trusted_digest || !memcmp(digest,m->sha256,sizeof(digest)));
    dbt_sha256_free(&hash);memset(bytes,0,sizeof(bytes));memset(digest,0,sizeof(digest));return ok;
}
bool dbt_ota_esp_finish(void *context,const dbt_ota_manifest_t *m) {
    dbt_ota_esp_t *s=context;session_t *job=s?s->session:0;
    if(!job || !job->active)return false;
    if(!job->hash_live || dbt_sha256_finish(&job->stream,job->stream_digest)!=0) {
        (void)dbt_ota_esp_abort(context);return false;
    }
    dbt_sha256_free(&job->stream);job->hash_live=false;
    esp_err_t result=esp_ota_end(job->handle);job->active=false;
    if(result!=ESP_OK || !verify(job,m)) {free(job);s->session=0;return false;}
    job->verified=true;return true;
}
bool dbt_ota_esp_abort(void *context) {
    dbt_ota_esp_t *s=context;session_t *job=s?s->session:0;
    if(!job)return true;
    bool ok=!job->active || esp_ota_abort(job->handle)==ESP_OK;
    if(ok) {if(job->hash_live)dbt_sha256_free(&job->stream);memset(job,0,sizeof(*job));free(job);s->session=0;}
    return ok;
}
bool dbt_ota_esp_activate(void *context,const dbt_ota_manifest_t *m) {
    dbt_ota_esp_t *s=context;session_t *job=s?s->session:0;
    const esp_partition_t *run=esp_ota_get_running_partition();esp_ota_img_states_t state;
    if(!job || !job->verified || !run || run->subtype<ESP_PARTITION_SUBTYPE_APP_OTA_MIN ||
       run->subtype>=ESP_PARTITION_SUBTYPE_APP_OTA_MAX || esp_ota_get_state_partition(run,&state)!=ESP_OK ||
       (state!=ESP_OTA_IMG_VALID && state!=ESP_OTA_IMG_UNDEFINED) || !verify(job,m))return false;
    return esp_ota_set_boot_partition(job->partition)==ESP_OK;
}
