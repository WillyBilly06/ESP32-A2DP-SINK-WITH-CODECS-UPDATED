#include "dbt_sdp_bearer.h"

static void release_reply(dbt_sdp_bearer_t *bearer, dbt_sdp_reply_t *reply) {
    if (reply->bytes && reply->bytes != reply->error)
        bearer->runtime->hooks.release(bearer->runtime->hooks.context, reply->bytes);
    reply->bytes = 0;
    reply->used = reply->active = false;
}

static void sent(void *context, dbt_tx_outcome_t outcome) {
    /* Runtime no longer borrows the response, including after channel abort.
     * Release is done by poll/event with access to the owning allocator. */
    (void)outcome;
    dbt_sdp_reply_t *reply = context;
    reply->active = false;
    reply->used = false;
}

dbt_result_t dbt_sdp_bearer_init(dbt_sdp_bearer_t *bearer, dbt_runtime_t *runtime,
                                 const dbt_sdp_t *server, uint16_t maximum_pdu) {
    if (!bearer || !runtime || !runtime->ready || !server || !server->database_state ||
        maximum_pdu < 48 || maximum_pdu > runtime->maximum_mtu)
        return DBT_INVALID;
    dbt_service_t service = {1, maximum_pdu, 48, DBT_SECURITY_NONE};
    dbt_result_t result = dbt_channels_register(&runtime->channels, service);
    if (result != DBT_OK)
        return result;
    uint8_t *bytes = (uint8_t *)bearer;
    for (unsigned i = 0; i < sizeof(*bearer); ++i)
        bytes[i] = 0;
    bearer->runtime = runtime;
    bearer->server = server;
    bearer->maximum_pdu = maximum_pdu;
    return DBT_OK;
}

dbt_result_t dbt_sdp_bearer_receive(dbt_sdp_bearer_t *bearer, const dbt_channel_t *channel,
                                    const uint8_t *data, uint16_t size) {
    if (!bearer || !bearer->runtime || !channel || (!data && size) || !bearer->runtime->ready ||
        bearer->runtime->host.state != DBT_HOST_IDENTIFIED)
        return DBT_INVALID;
    if (channel->psm != 1)
        return DBT_UNSUPPORTED;
    const dbt_channel_t *current =
        dbt_channels_find(&bearer->runtime->channels, channel->handle, channel->local_cid);
    if (!current || current->serial != channel->serial || current->state != DBT_CH_OPEN ||
        current->link_generation != channel->link_generation || !channel->serial ||
        size > channel->rx_mtu || size > bearer->maximum_pdu)
        return DBT_INVALID;
    dbt_sdp_reply_t *slot = 0;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        dbt_sdp_reply_t *reply = &bearer->replies[i];
        if (reply->used && reply->handle == channel->handle &&
            reply->generation == channel->link_generation && reply->serial == channel->serial)
            return DBT_BUSY;
        if (!reply->used && !reply->active && !slot)
            slot = reply;
    }
    if (!slot)
        return DBT_NO_SPACE;
    release_reply(bearer, slot);
    uint16_t capacity =
        channel->tx_mtu < bearer->maximum_pdu ? channel->tx_mtu : bearer->maximum_pdu;
    uint16_t planned = 0;
    dbt_result_t result =
        dbt_sdp_respond(bearer->server, channel->serial, data, size, 0, capacity, &planned);
    if (result != DBT_OK)
        return result;
    slot->bytes = planned <= sizeof(slot->error)
                      ? slot->error
                      : bearer->runtime->hooks.allocate(bearer->runtime->hooks.context, planned);
    if (!slot->bytes) {
        slot->bytes = slot->error;
        planned = 7;
        slot->error[0] = 1;
        slot->error[1] = data[1];
        slot->error[2] = data[2];
        slot->error[3] = 0;
        slot->error[4] = 2;
        slot->error[5] = 0;
        slot->error[6] = 6;
    } else {
        uint16_t actual = 0;
        result = dbt_sdp_respond(bearer->server, channel->serial, data, size, slot->bytes, planned,
                                 &actual);
        if (result != DBT_OK || actual != planned) {
            release_reply(bearer, slot);
            return result == DBT_OK ? DBT_INVALID : result;
        }
    }
    slot->size = planned;
    slot->handle = channel->handle;
    slot->cid = channel->local_cid;
    slot->serial = channel->serial;
    slot->generation = channel->link_generation;
    slot->used = true;
    slot->active = false;
    return DBT_OK;
}

void dbt_sdp_bearer_event(dbt_sdp_bearer_t *bearer, dbt_channel_event_t event,
                          const dbt_channel_t *channel) {
    if (!bearer || !bearer->runtime || !channel ||
        (event != DBT_CH_EVENT_CLOSED && event != DBT_CH_EVENT_FAILED))
        return;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        dbt_sdp_reply_t *reply = &bearer->replies[i];
        if (reply->bytes && !reply->active && reply->serial == channel->serial &&
            reply->generation == channel->link_generation && reply->handle == channel->handle)
            release_reply(bearer, reply);
    }
}

dbt_result_t dbt_sdp_bearer_poll(dbt_sdp_bearer_t *bearer) {
    if (!bearer || !bearer->runtime)
        return DBT_INVALID;
    if (bearer->runtime->host.state == DBT_HOST_FAILED) {
        (void)dbt_sdp_bearer_clear(bearer);
        return DBT_INVALID;
    }
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        dbt_sdp_reply_t *reply = &bearer->replies[i];
        if (reply->active)
            continue;
        if (!reply->used) {
            release_reply(bearer, reply);
            continue;
        }
        const dbt_channel_t *channel =
            dbt_channels_find(&bearer->runtime->channels, reply->handle, reply->cid);
        if (!channel || channel->serial != reply->serial ||
            channel->link_generation != reply->generation) {
            release_reply(bearer, reply);
            continue;
        }
        dbt_result_t result = dbt_runtime_send(bearer->runtime, reply->handle, reply->cid,
                                               reply->bytes, reply->size, sent, reply);
        if (result == DBT_OK)
            reply->active = true;
        else if (result != DBT_BUSY)
            return result;
    }
    return DBT_OK;
}

uint32_t dbt_sdp_bearer_wait_ms(const dbt_sdp_bearer_t *bearer) {
    if (!bearer || !bearer->runtime || !bearer->runtime->ready)
        return UINT32_MAX;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        const dbt_sdp_reply_t *reply = &bearer->replies[i];
        if (!reply->active && reply->bytes && !reply->used)
            return 0;
        if (!reply->used || reply->active)
            continue;
        const dbt_link_t *link = dbt_acl_find(&bearer->runtime->acl, reply->handle);
        const dbt_channel_t *channel =
            dbt_channels_find(&bearer->runtime->channels, reply->handle, reply->cid);
        if (!link || link->generation != reply->generation || !channel ||
            channel->serial != reply->serial)
            return 0;
        for (unsigned j = 0; j < DBT_MAX_LINKS; ++j)
            if (bearer->runtime->acl.links[j].used &&
                bearer->runtime->acl.links[j].handle == reply->handle &&
                !bearer->runtime->tx[j].active)
                return 0;
    }
    return UINT32_MAX;
}

dbt_result_t dbt_sdp_bearer_clear(dbt_sdp_bearer_t *bearer) {
    if (!bearer || !bearer->runtime)
        return DBT_INVALID;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i)
        if (bearer->replies[i].active)
            return DBT_BUSY;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i)
        release_reply(bearer, &bearer->replies[i]);
    return DBT_OK;
}
