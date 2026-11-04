#ifndef TEST_ESP_SPIFFS_H
#define TEST_ESP_SPIFFS_H
#include <stddef.h>
typedef int esp_err_t;
#define ESP_OK 0
esp_err_t esp_spiffs_info(const char *,size_t *,size_t *);
#endif
