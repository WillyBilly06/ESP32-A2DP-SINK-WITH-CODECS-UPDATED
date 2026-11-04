#ifndef DBT_BOOT_POLICY_H
#define DBT_BOOT_POLICY_H
#include <stdbool.h>
#include <stdint.h>
/* Slot/project identity only; not authenticity or qualification. */
bool dbt_boot_owned_slot(uint8_t,uint8_t,const char *,const char *);
#endif
