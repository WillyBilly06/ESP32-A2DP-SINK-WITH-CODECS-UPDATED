#include "ldac.h"
#include <malloc.h>
/***************************************************************************************************
    Align Memory
***************************************************************************************************/
#define LDAC_ALLOC_LINE 8

DECLFUNC size_t align_ldac(size_t size) {
  if (LDAC_ALLOC_LINE != 0) {
    size = (((size - 1) / LDAC_ALLOC_LINE) + 1) * LDAC_ALLOC_LINE;
  }

  return size;
}

/***************************************************************************************************
    Clear Allocate Memory
***************************************************************************************************/
DECLFUNC void* calloc_ldac(SFINFO* p_sfinfo, size_t nmemb, size_t size) {
  char* p_mempos;

  p_mempos = p_sfinfo->p_mempos;
  if (p_mempos) {
    p_sfinfo->p_mempos = &p_mempos[nmemb * align_ldac(size)];
    return p_mempos;
  } else {
    return calloc(nmemb, size);
  }
}
