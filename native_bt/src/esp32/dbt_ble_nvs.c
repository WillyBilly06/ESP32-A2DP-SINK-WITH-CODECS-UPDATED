#include "dbt_ble_nvs.h"
#include "nvs.h"
#include <string.h>
static bool missing_or_ok(esp_err_t e){return e==ESP_OK || e==ESP_ERR_NVS_NOT_FOUND;}
static int8_t signed_byte(uint8_t v){return (int8_t)(v<128?(int)v:(int)v-256);}
dbt_result_t dbt_ble_nvs_load_state(dbt_ble_control_state_t *state) {
    if(!state || state->name_size>32 || state->firmware_size>32)return DBT_INVALID;
    dbt_ble_control_state_t next=*state; nvs_handle_t h;
    esp_err_t e=nvs_open("audio",NVS_READONLY,&h);
    if(e!=ESP_OK && e!=ESP_ERR_NVS_NOT_FOUND)return DBT_INVALID;
    if(e==ESP_OK) {
        uint8_t v;uint32_t packed;char name[33];size_t length=sizeof(name);
        e=nvs_get_str(h,"devname",name,&length);
        if(!missing_or_ok(e) || (e==ESP_OK && (!length || length>sizeof(name) || name[length-1]!=0))) goto bad_audio;
        if(e==ESP_OK) {next.name_size=(uint8_t)(length-1);memcpy(next.name,name,next.name_size);}
        e=nvs_get_u8(h,"ctrl",&v);if(!missing_or_ok(e))goto bad_audio;if(e==ESP_OK)next.control=v&7u;
        e=nvs_get_u32(h,"eq_v1",&packed);if(!missing_or_ok(e))goto bad_audio;
        if(e==ESP_OK && (packed>>24)==1u) {
            next.eq[0]=signed_byte((uint8_t)packed);next.eq[1]=signed_byte((uint8_t)(packed>>8));
            next.eq[2]=signed_byte((uint8_t)(packed>>16));
        } else {
            static const char *keys[]={"eq_bass","eq_mid","eq_treb"};
            for(unsigned i=0;i<3;i++) {int8_t gain; e=nvs_get_i8(h,keys[i],&gain);
                if(!missing_or_ok(e))goto bad_audio;
                if(e==ESP_OK)next.eq[i]=gain;}
        }
        e=nvs_get_u8(h,"led_effect",&v);if(!missing_or_ok(e))goto bad_audio;if(e==ESP_OK)next.led[0]=v;
        e=nvs_get_u8(h,"sound_muted",&v);if(!missing_or_ok(e))goto bad_audio;
        if(e==ESP_OK)next.sound=(uint8_t)((next.sound&0x7fu)|(v?0x80u:0u));
        nvs_close(h);
    }
    e=nvs_open("led",NVS_READONLY,&h);
    if(e!=ESP_OK && e!=ESP_ERR_NVS_NOT_FOUND)return DBT_INVALID;
    if(e==ESP_OK) {
        uint8_t blob[10],brightness;size_t len=sizeof(blob);
        e=nvs_get_blob(h,"settings",blob,&len);
        if(!missing_or_ok(e) || (e==ESP_OK && len!=sizeof(blob))) {nvs_close(h);return DBT_INVALID;}
        if(e==ESP_OK) {
            next.led[1]=(uint8_t)(((unsigned)blob[0]*25u+32u)>>6);
            next.led[2]=blob[8];memcpy(next.led+3,blob+1,6);next.led[9]=blob[7];
        }
        else {
            e=nvs_get_u8(h,"bright",&brightness);if(!missing_or_ok(e)){nvs_close(h);return DBT_INVALID;}
            if(e==ESP_OK)next.led[1]=(uint8_t)(((unsigned)brightness*25u+32u)>>6);
        }
        nvs_close(h);
    }
    next.valid=true;*state=next;return DBT_OK;
bad_audio:
    nvs_close(h);return DBT_INVALID;
}
bool dbt_ble_nvs_commit_eq(void *context,int8_t bass,int8_t mid,int8_t treble) {
    (void)context; nvs_handle_t handle;
    if(nvs_open("audio",NVS_READWRITE,&handle)!=ESP_OK) return false;
    uint32_t packed=0x01000000u|(uint8_t)bass|((uint32_t)(uint8_t)mid<<8)|((uint32_t)(uint8_t)treble<<16);
    esp_err_t result=nvs_set_u32(handle,"eq_v1",packed);
    if(result==ESP_OK) result=nvs_commit(handle);
    nvs_close(handle); return result==ESP_OK;
}
bool dbt_ble_nvs_commit_control(void *context,uint8_t value) {
    (void)context; nvs_handle_t handle;
    if(nvs_open("audio",NVS_READWRITE,&handle)!=ESP_OK) return false;
    esp_err_t result=nvs_set_u8(handle,"ctrl",value);
    if(result==ESP_OK) result=nvs_commit(handle);
    nvs_close(handle); return result==ESP_OK;
}
