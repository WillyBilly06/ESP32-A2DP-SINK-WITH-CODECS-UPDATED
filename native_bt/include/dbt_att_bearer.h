#ifndef DBT_ATT_BEARER_H
#define DBT_ATT_BEARER_H

#include "dbt_att.h"
#include "dbt_runtime.h"

struct dbt_att_bearer;
typedef struct {
    struct dbt_att_bearer *owner;
    uint32_t generation;
    uint16_t handle;
    bool active;
} dbt_att_binding_t;
typedef struct dbt_att_bearer {
    dbt_runtime_t *runtime;
    dbt_att_server_t *server;
    dbt_att_binding_t bindings[DBT_MAX_LINKS];
} dbt_att_bearer_t;

/* Zero-initialize before first use. Call after runtime.ready. Does not replace the runtime's hooks:
 * the owner dispatches link/fixed-data events here alongside SMP/other profile owners. */
dbt_result_t dbt_att_bearer_init(dbt_att_bearer_t *bearer, dbt_runtime_t *runtime,
                                 dbt_att_server_t *server);
dbt_result_t dbt_att_bearer_link(dbt_att_bearer_t *bearer, const dbt_link_change_t *change);
dbt_result_t dbt_att_bearer_receive(dbt_att_bearer_t *bearer, const dbt_link_t *link, uint16_t cid,
                                    const uint8_t *data, uint16_t size);
void dbt_att_bearer_poll(dbt_att_bearer_t *bearer, uint32_t now_ms);
uint32_t dbt_att_bearer_wait_ms(const dbt_att_bearer_t *bearer, uint32_t now_ms);
/* Abort/drain runtime first, so it has returned every borrowed ATT payload. */
dbt_result_t dbt_att_bearer_clear(dbt_att_bearer_t *bearer);

#endif
