#include <stddef.h>
#include <stdint.h>
#define MALLOC_CAP_INTERNAL 1u
#define MALLOC_CAP_8BIT 2u
size_t heap_caps_get_free_size(uint32_t);
size_t heap_caps_get_minimum_free_size(uint32_t);
void esp_heap_trace_alloc_hook(void *,size_t,uint32_t);
void esp_heap_trace_free_hook(void *);
