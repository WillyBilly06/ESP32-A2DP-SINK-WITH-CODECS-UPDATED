#ifndef ESP_PARTITION_H
#define ESP_PARTITION_H
#include <stdint.h>
#include <stddef.h>
#define ESP_OK 0
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_PARTITION_TYPE_APP 0
#define ESP_PARTITION_SUBTYPE_APP_OTA_MIN 0x10
#define ESP_PARTITION_SUBTYPE_APP_OTA_MAX 0x20
#define ESP_PARTITION_SUBTYPE_ANY 0xff
typedef struct {int type,subtype;uint32_t address,size;const char *label;} esp_partition_t;
typedef struct {uint32_t offset,size;} esp_partition_pos_t;
const esp_partition_t *esp_partition_find_first(int,int,const char *);
int esp_partition_read(const esp_partition_t *,size_t,void *,size_t);
#endif
