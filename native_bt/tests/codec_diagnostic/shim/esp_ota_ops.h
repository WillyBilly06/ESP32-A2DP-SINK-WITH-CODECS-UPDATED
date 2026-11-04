#pragma once
#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 259
#define ESP_PARTITION_TYPE_APP 0
#define ESP_PARTITION_SUBTYPE_APP_OTA_0 16
#define ESP_PARTITION_SUBTYPE_APP_OTA_1 17
typedef struct {uint8_t subtype;uint32_t address,size;} esp_partition_t;
typedef struct {char project_name[32],version[32];uint8_t app_elf_sha256[32];} esp_app_desc_t;
typedef enum {ESP_OTA_IMG_NEW,ESP_OTA_IMG_PENDING_VERIFY,ESP_OTA_IMG_VALID} esp_ota_img_states_t;
const esp_partition_t *esp_ota_get_running_partition(void);
const esp_partition_t *esp_partition_find_first(int,int,const char *);
esp_err_t esp_ota_get_state_partition(const esp_partition_t *,esp_ota_img_states_t *);
esp_err_t esp_ota_get_partition_description(const esp_partition_t *,esp_app_desc_t *);
esp_err_t esp_ota_set_boot_partition(const esp_partition_t *);
int xPortGetCoreID(void);
