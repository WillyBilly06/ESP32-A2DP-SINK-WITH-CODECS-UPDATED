#ifndef ESP_OTA_OPS_H
#define ESP_OTA_OPS_H
#include "esp_partition.h"
typedef int esp_err_t;
typedef unsigned esp_ota_handle_t;
#define OTA_WITH_SEQUENTIAL_WRITES 0xfffffffeu
typedef enum {ESP_OTA_IMG_VALID=2,ESP_OTA_IMG_UNDEFINED=0xffffffffu} esp_ota_img_states_t;
typedef struct {char project_name[32];} esp_app_desc_t;
const esp_partition_t *esp_ota_get_running_partition(void);
const esp_partition_t *esp_ota_get_boot_partition(void);
int esp_ota_resume(const esp_partition_t *,size_t,size_t,esp_ota_handle_t *);
int esp_ota_write(esp_ota_handle_t,const void *,size_t);
int esp_ota_end(esp_ota_handle_t);
int esp_ota_abort(esp_ota_handle_t);
int esp_ota_get_partition_description(const esp_partition_t *,esp_app_desc_t *);
int esp_ota_get_state_partition(const esp_partition_t *,esp_ota_img_states_t *);
int esp_ota_set_boot_partition(const esp_partition_t *);
#endif
