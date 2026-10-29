#ifndef DBT_LDAC_PORT_H
#define DBT_LDAC_PORT_H
#include <stddef.h>
#include <stdint.h>
#include "ldacBT.h"
void *dbt_ldac_mem_alloc(size_t size);
void *dbt_ldac_mem_calloc(size_t count,size_t size);
void dbt_ldac_mem_free(void *ptr);
extern uint32_t dbt_ldac_port_bit_limit;
extern int dbt_ldac_port_overread;
int dbt_ldac_bind_pcm(HANDLE_LDAC_BT handle,void *pcm,unsigned samples,unsigned channels);
#endif
