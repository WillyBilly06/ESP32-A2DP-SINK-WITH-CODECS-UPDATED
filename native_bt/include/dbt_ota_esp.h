#ifndef DBT_OTA_ESP_H
#define DBT_OTA_ESP_H
#include "dbt_ota.h"
/* Platform object and immutable label must outlive the worker. A caller must
 * explicitly select an owned OTA slot; no automatic next-slot selection. */
typedef struct { const char *label;void *session; } dbt_ota_esp_t;
bool dbt_ota_esp_init(dbt_ota_esp_t *,const char *approved_slot_label);
bool dbt_ota_esp_begin(void *,const dbt_ota_manifest_t *);
bool dbt_ota_esp_write(void *,uint32_t,const uint8_t *,uint16_t);
bool dbt_ota_esp_finish(void *,const dbt_ota_manifest_t *);
bool dbt_ota_esp_abort(void *);
bool dbt_ota_esp_activate(void *,const dbt_ota_manifest_t *);
#endif
