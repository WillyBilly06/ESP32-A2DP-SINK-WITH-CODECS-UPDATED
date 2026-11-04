#include "dbt_avctp_bearer.h"

static bool matches(const dbt_avctp_bearer_t *b, const dbt_channel_t *channel) {
    return channel && channel->handle == b->handle && channel->local_cid == b->cid &&
        channel->serial == b->serial && channel->link_generation == b->generation;
}
static const dbt_channel_t *current(const dbt_avctp_bearer_t *b) {
    const dbt_channel_t *channel = dbt_channels_find(&b->runtime->channels, b->handle, b->cid);
    return matches(b, channel) && channel->state == DBT_CH_OPEN &&
        channel->rx_mtu == b->signal->rx_mtu && channel->tx_mtu == b->signal->tx_mtu ? channel : 0;
}
static void admitted(void *context, dbt_tx_outcome_t outcome) {
    dbt_avctp_bearer_t *b = context;
    b->active = false;
    if (outcome == DBT_TX_ADMITTED) {
        (void)dbt_avctp_tx_commit(b->signal, b->runtime->now_ms);
    } else {
        (void)dbt_avctp_tx_cancel(b->signal);
        (void)dbt_avctp_bearer_clear(b);
    }
}
dbt_result_t dbt_avctp_bearer_init(dbt_avctp_bearer_t *b, dbt_runtime_t *runtime,
    dbt_avctp_t *signal, const dbt_channel_t *channel) {
    if (!b || !runtime || !runtime->ready || !signal || signal->failed || signal->browsing || !channel ||
        (channel->psm != (signal->browsing ? 0x1b : 0x17)) || channel->state != DBT_CH_OPEN || !channel->serial ||
        signal->binding != channel->serial || signal->rx_mtu != channel->rx_mtu ||
        signal->tx_mtu != channel->tx_mtu || signal->pinned) return DBT_INVALID;
    const dbt_channel_t *live = dbt_channels_find(&runtime->channels, channel->handle,
                                                channel->local_cid);
    const dbt_link_t *link = dbt_acl_find(&runtime->acl, channel->handle);
    if (!live || live->serial != channel->serial || live->link_generation != channel->link_generation ||
        live->state != DBT_CH_OPEN || live->psm != (signal->browsing ? 0x1b : 0x17) || live->rx_mtu != signal->rx_mtu ||
        live->tx_mtu != signal->tx_mtu || !link || link->kind != DBT_LINK_CLASSIC ||
        link->generation != channel->link_generation) return DBT_INVALID;
    *b = (dbt_avctp_bearer_t){runtime, signal, channel->serial, channel->link_generation,
                             channel->handle, channel->local_cid, false, true};
    return DBT_OK;
}
dbt_result_t dbt_avctp_bearer_receive(dbt_avctp_bearer_t *b, const dbt_channel_t *channel,
    const uint8_t *data, uint16_t size) {
    if (!b || !b->attached || !matches(b, channel) || !current(b)) return DBT_INVALID;
    return dbt_avctp_receive(b->signal, b->serial, data, size, b->runtime->now_ms);
}
void dbt_avctp_bearer_event(dbt_avctp_bearer_t *b, dbt_channel_event_t event,
    const dbt_channel_t *channel) {
    if (b && b->attached && matches(b, channel) &&
        (event == DBT_CH_EVENT_CLOSED || event == DBT_CH_EVENT_FAILED))
        (void)dbt_avctp_bearer_clear(b);
}
dbt_result_t dbt_avctp_bearer_poll(dbt_avctp_bearer_t *b, uint32_t now) {
    if (!b || !b->attached) return DBT_INVALID;
    if (!current(b) || b->runtime->host.state == DBT_HOST_FAILED) {
        (void)dbt_avctp_bearer_clear(b);
        return DBT_INVALID;
    }
    if (dbt_avctp_poll(b->signal, now) != DBT_OK) return DBT_INVALID;
    if (b->active) return DBT_OK;
    const uint8_t *bytes = 0;
    uint16_t size = 0;
    dbt_result_t result = dbt_avctp_tx_peek(b->signal, &bytes, &size);
    if (result == DBT_EMPTY) return DBT_OK;
    if (result != DBT_OK) return result;
    result = dbt_runtime_send(b->runtime, b->handle, b->cid, bytes, size, admitted, b);
    if (result == DBT_OK) b->active = true;
    else (void)dbt_avctp_tx_cancel(b->signal);
    return result == DBT_BUSY ? DBT_OK : result;
}
uint32_t dbt_avctp_bearer_wait_ms(const dbt_avctp_bearer_t *b, uint32_t now) {
    if (!b || !b->attached) return UINT32_MAX;
    if (!current(b) || b->runtime->host.state == DBT_HOST_FAILED) return 0;
    uint32_t wait = dbt_avctp_wait_ms(b->signal, now);
    if (!b->active && !b->signal->failed &&
        b->signal->tx_used) {
        for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
            if (b->runtime->acl.links[i].used && b->runtime->acl.links[i].handle == b->handle &&
                !b->runtime->tx[i].active) return 0;
    }
    return wait;
}
dbt_result_t dbt_avctp_bearer_clear(dbt_avctp_bearer_t *b) {
    if (!b || !b->attached) return DBT_INVALID;
    if (b->active) return DBT_BUSY;
    dbt_result_t result = dbt_avctp_clear(b->signal);
    if (result == DBT_OK) b->attached = false;
    return result;
}
