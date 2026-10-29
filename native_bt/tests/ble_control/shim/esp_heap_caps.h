#include <stddef.h>
#include <stdint.h>
#define MALLOC_CAP_INTERNAL 1u
#define MALLOC_CAP_8BIT 2u
void *heap_caps_malloc(size_t,uint32_t);
void heap_caps_free(void *);
