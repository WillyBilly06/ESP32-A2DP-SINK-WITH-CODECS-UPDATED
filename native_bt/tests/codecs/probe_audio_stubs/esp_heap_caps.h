#include <stddef.h>
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
void *heap_caps_malloc(size_t,unsigned);void *heap_caps_aligned_alloc(size_t,size_t,unsigned);void heap_caps_free(void *);size_t heap_caps_get_free_size(unsigned);size_t heap_caps_get_minimum_free_size(unsigned);size_t heap_caps_get_largest_free_block(unsigned);
