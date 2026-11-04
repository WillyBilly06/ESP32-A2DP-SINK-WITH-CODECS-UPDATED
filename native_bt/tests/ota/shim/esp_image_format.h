#ifndef ESP_IMAGE_FORMAT_H
#define ESP_IMAGE_FORMAT_H
#include "esp_partition.h"
#define ESP_IMAGE_VERIFY 0
typedef struct {struct {uint8_t hash_appended;} image;uint32_t image_len;} esp_image_metadata_t;
int esp_image_verify(int,const esp_partition_pos_t *,esp_image_metadata_t *);
#endif
