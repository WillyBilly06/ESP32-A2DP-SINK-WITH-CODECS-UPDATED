#ifndef TEST_ESP_COEXIST_H
#define TEST_ESP_COEXIST_H
#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 0x103
typedef enum {ESP_COEX_ST_TYPE_WIFI,ESP_COEX_ST_TYPE_BLE,ESP_COEX_ST_TYPE_BT} esp_coex_status_type_t;
#define ESP_COEX_BT_ST_A2DP_STREAMING 0x10
#define ESP_COEX_BT_ST_A2DP_PAUSED 0x20
esp_err_t esp_coex_status_bit_set(esp_coex_status_type_t,uint32_t);
esp_err_t esp_coex_status_bit_clear(esp_coex_status_type_t,uint32_t);
#endif
