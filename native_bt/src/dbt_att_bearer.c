#include "dbt_att_bearer.h"

static const dbt_att_connection_t *connection(const dbt_att_bearer_t *bearer,
                                              const dbt_link_t *link) {
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_att_connection_t *c = &bearer->server->connections[i];
        if (c->used && c->handle == link->handle && c->generation == link->generation)
            return c;
    }
    return 0;
}
static void admitted(void *context, dbt_tx_outcome_t outcome) {
    dbt_att_binding_t *binding = context;
    dbt_att_bearer_t *bearer = binding->owner;
    binding->active = false;
    if (outcome == DBT_TX_ADMITTED)
        (void)dbt_att_tx_admitted(bearer->server, binding->handle, binding->generation,
                                  bearer->runtime->now_ms);
    else
        dbt_att_close(bearer->server, binding->handle, binding->generation);
}
dbt_result_t dbt_att_bearer_init(dbt_att_bearer_t *bearer, dbt_runtime_t *runtime,
                                 dbt_att_server_t *server) {
    if (!bearer || !runtime || !server || !server->attributes)
        return DBT_INVALID;
    if (!runtime->ready)
        return DBT_BUSY;
    if (server->mtu > runtime->maximum_mtu)
        return DBT_NO_SPACE;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        if (bearer->bindings[i].active)
            return DBT_BUSY;
    }
    dbt_result_t result = dbt_channels_register_fixed(&runtime->channels, DBT_LINK_LE, 4);
    if (result != DBT_OK)
        return result;
    bearer->runtime = runtime;
    bearer->server = server;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        bearer->bindings[i].owner = bearer;
        bearer->bindings[i].active = false;
        const dbt_link_t *link = &runtime->acl.links[i];
        if (link->used && link->kind == DBT_LINK_LE && !connection(bearer, link)) {
            result = dbt_att_open(server, link);
            if (result != DBT_OK)
                return result;
        }
    }
    return DBT_OK;
}
dbt_result_t dbt_att_bearer_link(dbt_att_bearer_t *bearer, const dbt_link_change_t *change) {
    if (!bearer || !bearer->runtime || !change)
        return DBT_INVALID;
    if (change->kind == DBT_LINK_REMOVED) {
        for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
            if (bearer->bindings[i].active && bearer->bindings[i].handle == change->handle &&
                bearer->bindings[i].generation == change->generation)
                return DBT_BUSY;
        dbt_att_close(bearer->server, change->handle, change->generation);
    } else if (change->kind == DBT_LINK_ADDED) {
        const dbt_link_t *link = dbt_acl_find(&bearer->runtime->acl, change->handle);
        if (!link || link->generation != change->generation)
            return DBT_INVALID;
        if (link->kind == DBT_LINK_LE && !connection(bearer, link))
            return dbt_att_open(bearer->server, link);
    }
    return DBT_OK;
}
dbt_result_t dbt_att_bearer_receive(dbt_att_bearer_t *bearer, const dbt_link_t *link, uint16_t cid,
                                    const uint8_t *data, uint16_t size) {
    if (!bearer || !bearer->runtime || !link)
        return DBT_INVALID;
    if (cid != 4 || link->kind != DBT_LINK_LE)
        return DBT_UNSUPPORTED;
    return dbt_att_receive(bearer->server, link, data, size, bearer->runtime->now_ms);
}
void dbt_att_bearer_poll(dbt_att_bearer_t *bearer, uint32_t now_ms) {
    if (!bearer || !bearer->runtime)
        return;
    dbt_att_poll(bearer->server, now_ms);
    if (!bearer->runtime->ready)
        return;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_link_t *link = &bearer->runtime->acl.links[i];
        dbt_att_binding_t *binding = &bearer->bindings[i];
        if (!link->used || link->kind != DBT_LINK_LE || binding->active ||
            bearer->runtime->tx[i].active)
            continue;
        dbt_att_view_t view;
        if (dbt_att_tx_peek(bearer->server, link->handle, link->generation, &view) != DBT_OK)
            continue;
        binding->handle = link->handle;
        binding->generation = link->generation;
        dbt_result_t result = dbt_runtime_send_fixed(bearer->runtime, link->handle, 4, view.data,
                                                     view.size, admitted, binding);
        if (result == DBT_OK)
            binding->active = true;
    }
}
uint32_t dbt_att_bearer_wait_ms(const dbt_att_bearer_t *bearer, uint32_t now_ms) {
    if (!bearer || !bearer->runtime)
        return UINT32_MAX;
    uint32_t wait = dbt_att_wait_ms(bearer->server, now_ms);
    if (!bearer->runtime->ready)
        return wait;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_link_t *link = &bearer->runtime->acl.links[i];
        if (!link->used || link->kind != DBT_LINK_LE || bearer->bindings[i].active ||
            bearer->runtime->tx[i].active)
            continue;
        const dbt_att_connection_t *c = connection(bearer, link);
        if (c && !c->failed && (c->response_ready || c->notice_ready))
            return 0;
    }
    return wait;
}
dbt_result_t dbt_att_bearer_clear(dbt_att_bearer_t *bearer) {
    if (!bearer || !bearer->runtime)
        return DBT_INVALID;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (bearer->bindings[i].active)
            return DBT_BUSY;
    dbt_att_clear(bearer->server);
    bearer->runtime = 0;
    bearer->server = 0;
    return DBT_OK;
}
