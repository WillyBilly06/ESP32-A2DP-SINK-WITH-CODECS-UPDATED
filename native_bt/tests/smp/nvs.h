#ifndef TEST_NVS_H
#define TEST_NVS_H
#include <stddef.h>
#include <stdint.h>
typedef uint32_t nvs_handle_t;
typedef int32_t esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NVS_NOT_FOUND 0x1102
typedef enum { NVS_TYPE_BLOB=0x42,NVS_TYPE_ANY=0xff } nvs_type_t;
typedef struct { char namespace_name[16],key[16]; nvs_type_t type; } nvs_entry_info_t;
typedef struct nvs_opaque_iterator_t *nvs_iterator_t;
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *out,size_t *n);
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *data,size_t n);
esp_err_t nvs_erase_key(nvs_handle_t h,const char *key);
esp_err_t nvs_commit(nvs_handle_t h);
esp_err_t nvs_entry_find_in_handle(nvs_handle_t,nvs_type_t,nvs_iterator_t *);
esp_err_t nvs_entry_next(nvs_iterator_t *);
esp_err_t nvs_entry_info(nvs_iterator_t,nvs_entry_info_t *);
void nvs_release_iterator(nvs_iterator_t);
#endif
