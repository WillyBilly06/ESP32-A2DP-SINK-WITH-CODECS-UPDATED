#include "opus_defines.h"
#include "osi/allocator.h"

static OPUS_INLINE void *opus_alloc (size_t size)
{
   return osi_malloc(size);
}

static OPUS_INLINE void opus_free (void *ptr)
{
   osi_free(ptr);
}
