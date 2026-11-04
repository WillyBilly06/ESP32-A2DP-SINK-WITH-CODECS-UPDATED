#ifndef PROBE_CCCD_STORE_H
#define PROBE_CCCD_STORE_H
#include "dbt_smp_bond.h"

#define PROBE_CCCD_COUNT 4u
#define PROBE_CCCD_TAG_SIZE 32u
/* Worker only. The digest is opaque and binds the stable identity and the
 * committed bond's key material, excluding mutable identity-promotion flags. */
bool probe_cccd_bond_tag(const dbt_smp_bond_record_t *,uint8_t out[PROBE_CCCD_TAG_SIZE]);
/* Worker only, serialized by one worker. A failure may mean an ambiguous NVS
 * mutation; the provider then stays poisoned for the lifetime of the boot. */
dbt_result_t probe_cccd_load(uint8_t type,const uint8_t identity[6],const uint8_t tag[32],uint16_t values[PROBE_CCCD_COUNT]);
dbt_result_t probe_cccd_save(uint8_t type,const uint8_t identity[6],const uint8_t tag[32],const uint16_t values[PROBE_CCCD_COUNT]);
#endif
