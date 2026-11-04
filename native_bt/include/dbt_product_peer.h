#ifndef DBT_PRODUCT_PEER_H
#define DBT_PRODUCT_PEER_H
#include "dbt_ota.h"
#include <stdatomic.h>
/* One owner publishes a complete snapshot; workers may take bounded reads.
 * Grants are explicit, connection-generation scoped and never inferred from
 * handle reuse or from controller command admission. */
typedef struct {
    dbt_ble_control_peer_t peer;
    bool encrypted,bonded,allow_name,allow_ota;
    dbt_ota_manifest_t ota;
} dbt_product_peer_value_t;
typedef struct {
    _Atomic uint32_t sequence,handle,generation,flags,length,crc,sha[8],project[8];
} dbt_product_peer_t;
void dbt_product_peer_init(dbt_product_peer_t *);
bool dbt_product_peer_publish(dbt_product_peer_t *,const dbt_product_peer_value_t *);
bool dbt_product_peer_read(const dbt_product_peer_t *,dbt_product_peer_value_t *);
#endif
