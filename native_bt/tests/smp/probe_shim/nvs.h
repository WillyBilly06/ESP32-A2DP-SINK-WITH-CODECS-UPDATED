#ifndef PROBE_TEST_NVS_H
#define PROBE_TEST_NVS_H
#include <stdint.h>
#include <stddef.h>
typedef uint32_t nvs_handle_t;
typedef int esp_err_t;
#define ESP_OK 0
#define NVS_READWRITE 1
#define ESP_ERR_NVS_NOT_FOUND 0x1102
typedef enum { NVS_TYPE_BLOB=0x42,NVS_TYPE_ANY=0xff } nvs_type_t;
typedef struct { char namespace_name[16],key[16]; nvs_type_t type; } nvs_entry_info_t;
typedef struct nvs_opaque_iterator_t *nvs_iterator_t;
esp_err_t nvs_open(const char *,int,nvs_handle_t *);
void nvs_close(nvs_handle_t);
esp_err_t nvs_get_blob(nvs_handle_t,const char *,void *,size_t *);
esp_err_t nvs_set_blob(nvs_handle_t,const char *,const void *,size_t);
esp_err_t nvs_erase_key(nvs_handle_t,const char *);
esp_err_t nvs_commit(nvs_handle_t);
esp_err_t nvs_entry_find_in_handle(nvs_handle_t,nvs_type_t,nvs_iterator_t *);
esp_err_t nvs_entry_next(nvs_iterator_t *);
esp_err_t nvs_entry_info(nvs_iterator_t,nvs_entry_info_t *);
void nvs_release_iterator(nvs_iterator_t);
#endif
