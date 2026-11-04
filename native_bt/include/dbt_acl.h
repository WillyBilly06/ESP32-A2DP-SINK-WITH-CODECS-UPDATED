#ifndef DBT_ACL_H
#define DBT_ACL_H

#include "dbt_host.h"

#define DBT_MAX_LINKS 4u
typedef enum { DBT_LINK_CLASSIC, DBT_LINK_LE } dbt_link_kind_t;
typedef struct {
    uint32_t generation;
    uint16_t handle;
    uint16_t outstanding;
    dbt_link_kind_t kind;
    uint8_t peer[6];
    uint8_t peer_address_type;
    uint8_t role;
    bool used;
    bool encrypted; /* Does not imply authenticated pairing or adequate key size. */
} dbt_link_t;
typedef struct { uint16_t capacity, available; } dbt_acl_pool_t;
typedef struct {
    dbt_link_t links[DBT_MAX_LINKS];
    dbt_acl_pool_t pools[2];
    uint16_t mtu[2];
    uint32_t generation;
    uint32_t unknown_completions;
    bool shared_pool;
} dbt_acl_t;
typedef enum { DBT_LINK_UNCHANGED, DBT_LINK_ADDED, DBT_LINK_REMOVED } dbt_link_change_kind_t;
typedef struct {
    dbt_link_change_kind_t kind;
    uint16_t handle;
    uint32_t generation;
} dbt_link_change_t;

/* All calls run on the same host owner. No locks or allocations are needed. */
dbt_result_t dbt_acl_init(dbt_acl_t *acl, const dbt_controller_info_t *controller);
const dbt_link_t *dbt_acl_find(const dbt_acl_t *acl, uint16_t handle);
dbt_result_t dbt_acl_event(dbt_acl_t *acl, const dbt_hci_packet_t *packet,
                          dbt_link_change_t *change);
/* Complete H4 ACL fragment. Caller retains storage until transport can_send again.
 * Returns BUSY without consuming credits if either capacity gate is closed.
 * Success proves transport submission only, not RF delivery or peer consumption. */
dbt_result_t dbt_acl_submit(dbt_acl_t *acl, dbt_transport_t transport,
                           uint8_t *packet, uint16_t size);

#endif
