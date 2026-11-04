#include <stddef.h>
#define MALLOC_CAP_8BIT 1
#define MALLOC_CAP_DMA 2
void *heap_caps_malloc(size_t,unsigned);void heap_caps_free(void*);
