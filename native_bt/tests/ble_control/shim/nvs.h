#include <stdint.h>
#include <stddef.h>
typedef uint32_t nvs_handle_t;
typedef int esp_err_t;
#define ESP_OK 0
#define NVS_READWRITE 1
#define NVS_READONLY 0
#define ESP_ERR_NVS_NOT_FOUND 0x1102
esp_err_t nvs_open(const char *,int,nvs_handle_t *);
esp_err_t nvs_set_u32(nvs_handle_t,const char *,uint32_t);
esp_err_t nvs_set_u8(nvs_handle_t,const char *,uint8_t);
esp_err_t nvs_commit(nvs_handle_t);
void nvs_close(nvs_handle_t);
esp_err_t nvs_get_u8(nvs_handle_t,const char *,uint8_t *);
esp_err_t nvs_get_u32(nvs_handle_t,const char *,uint32_t *);
esp_err_t nvs_get_i8(nvs_handle_t,const char *,int8_t *);
esp_err_t nvs_get_str(nvs_handle_t,const char *,char *,size_t *);
esp_err_t nvs_get_blob(nvs_handle_t,const char *,void *,size_t *);
