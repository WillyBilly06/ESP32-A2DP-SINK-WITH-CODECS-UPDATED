#include "probe_echo.h"
#include "probe_security.h"
#include "esp_heap_caps.h"

/* Encrypted diagnostic ERTM channel, not an audio profile. One echo at a time. */
static dbt_runtime_t *runtime;
static dbt_gap_t *gap;
typedef struct {
    uint8_t *bytes;
    uint32_t serial, generation;
    uint16_t handle, cid, size;
    bool used, active;
} echo_t;
static echo_t replies[DBT_CHANNEL_LIMIT];
static struct {
    uint32_t generation;
    uint16_t handle,size,offset;
    uint8_t sequence;
    bool active;
} wire_pattern;
static struct {
    uint32_t generation;
    uint16_t handle,size,offset;
    uint8_t sequence;
    bool active;
} receive_pattern;
static void check_pattern(uint8_t stage,const uint8_t *data,uint16_t size) {
    for(unsigned i=1;i<size;++i) {
        uint8_t expected=(uint8_t)(data[0]+i*29u);
        if(data[i]!=expected) {
            probe_security_trace(stage,size,(uint16_t)i,expected,data[i]);return;
        }
    }
}
static void clear(echo_t *reply) {
    if (reply->bytes)
        heap_caps_free(reply->bytes);
    reply->bytes = 0;
    reply->used = reply->active = false;
}
static void done(void *context, dbt_tx_outcome_t outcome) {
    (void)outcome;
    clear(context);
}
dbt_result_t probe_echo_init(dbt_runtime_t *host_runtime, dbt_gap_t *host_gap) {
    runtime = host_runtime;
    gap = host_gap;
    dbt_service_t service = {0x1001, 2048, 2048, DBT_SECURITY_ENCRYPTED};
    dbt_result_t result = dbt_channels_register(&runtime->channels, service);
    if (result != DBT_OK)
        return result;
    dbt_ertm_parameters_t ertm={.mtu=2048,.mps=256,.retransmission_ms=2000,.monitor_ms=12000,.window=8,.max_transmit=3};
    result=dbt_channels_require_ertm(&runtime->channels,0x1001,ertm);
    if(result!=DBT_OK)return result;
    /* Encrypted diagnostic administration: 0x01 forgets this bound peer and
     * disconnects after the NVS commit. No unsecured BLE reset endpoint. */
    dbt_service_t administration = {0x1003, 48, 48, DBT_SECURITY_ENCRYPTED};
    return dbt_channels_register(&runtime->channels, administration);
}
void probe_echo_event(void *context, dbt_channel_event_t event, const dbt_channel_t *channel,
                      dbt_channel_reason_t reason, uint16_t result) {
    (void)context;
    (void)reason;
    (void)result;
    if (event != DBT_CH_EVENT_CLOSED && event != DBT_CH_EVENT_FAILED)
        return;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i)
        if (replies[i].used && replies[i].serial == channel->serial &&
            replies[i].generation == channel->link_generation && !replies[i].active)
            clear(&replies[i]);
}
void probe_echo_data(void *context, const dbt_channel_t *channel, const uint8_t *data,
                     uint16_t size) {
    (void)context;
    if (channel->psm == 0x1003) {
        const dbt_link_t *link = dbt_acl_find(&runtime->acl, channel->handle);
        if (link && size == 1 && data[0] == 1 && probe_security_forget_peer(link) == DBT_OK)
            return;
        goto full;
    }
    if (channel->psm != 0x1001)
        return;
    check_pattern(1,data,size);
    echo_t *slot = 0;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        if (replies[i].used && replies[i].serial == channel->serial)
            goto full;
        if (!replies[i].used)
            slot = &replies[i];
    }
    if (!slot)
        goto full;
    uint8_t *copy = size ? heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) : 0;
    if (size && !copy)
        goto full;
    for (unsigned i = 0; i < size; ++i)
        copy[i] = data[i];
    slot->bytes = copy;
    slot->size = size;
    slot->handle = channel->handle;
    slot->cid = channel->local_cid;
    slot->generation = channel->link_generation;
    slot->serial = channel->serial;
    slot->active = false;
    slot->used = true;
    return;
full: {
    const dbt_link_t *link = dbt_acl_find(&runtime->acl, channel->handle);
    if (link)
        (void)dbt_gap_disconnect(gap, link, 0x13);
}
}
void probe_echo_poll(void) {
    if (!runtime)
        return;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        echo_t *reply = &replies[i];
        if (!reply->used || reply->active)
            continue;
        const dbt_channel_t *channel =
            dbt_channels_find(&runtime->channels, reply->handle, reply->cid);
        if (!channel || channel->serial != reply->serial ||
            channel->link_generation != reply->generation) {
            clear(reply);
            continue;
        }
        dbt_result_t result = dbt_runtime_send(runtime, reply->handle, reply->cid, reply->bytes,
                                               reply->size, done, reply);
        if (result == DBT_OK) {
            check_pattern(2,reply->bytes,reply->size);
            reply->active = true;
        }
    }
}
void probe_echo_transmit(const uint8_t *data,uint16_t size) {
    if(!runtime || size<5 || data[0]!=DBT_H4_ACL)return;
    uint16_t handle=dbt_read_le16(data+1)&0xfffu;
    const dbt_link_t *link=dbt_acl_find(&runtime->acl,handle);
    if(!link || link->kind!=DBT_LINK_CLASSIC)return;
    uint8_t pb=(data[2]>>4)&3u;
    uint16_t at=5;
    if(pb==0 || pb==2) {
        wire_pattern.active=false;
        if(size<10)return;
        uint16_t remote_cid=dbt_read_le16(data+7);
        bool echo=false;
        for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i) {
            const dbt_channel_t *channel=&runtime->channels.channels[i];
            if(channel->state==DBT_CH_OPEN && channel->handle==handle && channel->psm==0x1001 &&
                channel->remote_cid==remote_cid)echo=true;
        }
        if(!echo)return;
        wire_pattern.handle=handle;wire_pattern.generation=link->generation;
        wire_pattern.size=dbt_read_le16(data+5);wire_pattern.offset=0;
        wire_pattern.sequence=data[9];wire_pattern.active=true;at=9;
    } else if(pb!=1 || !wire_pattern.active || wire_pattern.handle!=handle ||
              wire_pattern.generation!=link->generation)return;
    bool recorded=false;
    for(unsigned i=at;i<size;++i) {
        uint8_t expected=(uint8_t)(wire_pattern.sequence+wire_pattern.offset*29u);
        if(data[i]!=expected && !recorded) {
            probe_security_trace(3,wire_pattern.size,wire_pattern.offset,expected,data[i]);recorded=true;
        }
        ++wire_pattern.offset;
    }
    if(wire_pattern.offset>=wire_pattern.size)wire_pattern.active=false;
}
void probe_echo_receive_packet(const dbt_hci_packet_t *packet,const probe_ingress_sample_t *sample) {
    if(!runtime || packet->type!=DBT_H4_ACL)return;
    const dbt_link_t *link=dbt_acl_find(&runtime->acl,packet->handle);
    if(!link || link->kind!=DBT_LINK_CLASSIC)return;
    uint16_t at=0;
    if(packet->pb==2) {
        receive_pattern.active=false;
        if(packet->size<5)return;
        const dbt_channel_t *channel=dbt_channels_find(&runtime->channels,packet->handle,
                                                       dbt_read_le16(packet->payload+2));
        if(!channel || channel->psm!=0x1001 || channel->state!=DBT_CH_OPEN)return;
        receive_pattern.handle=packet->handle;receive_pattern.generation=link->generation;
        receive_pattern.size=dbt_read_le16(packet->payload);receive_pattern.offset=0;
        receive_pattern.sequence=packet->payload[4];receive_pattern.active=true;at=4;
    } else if(packet->pb!=1 || !receive_pattern.active || receive_pattern.handle!=packet->handle ||
              receive_pattern.generation!=link->generation)return;
    if(packet->pb==1 && sample) {
        uint8_t expected=(uint8_t)(receive_pattern.sequence+receive_pattern.offset*29u);
        if(sample->before!=expected)probe_security_trace(6,receive_pattern.size,receive_pattern.offset,expected,sample->before);
        if(sample->before!=packet->payload[0])probe_security_trace(5,receive_pattern.size,receive_pattern.offset,sample->before,packet->payload[0]);
        if(sample->after_ready && sample->before!=sample->after)probe_security_trace(7,receive_pattern.size,receive_pattern.offset,sample->before,sample->after);
    }
    for(unsigned i=at;i<packet->size;++i) {
        uint8_t expected=(uint8_t)(receive_pattern.sequence+receive_pattern.offset*29u);
        if(packet->payload[i]!=expected) {
            probe_security_trace(4,receive_pattern.size,receive_pattern.offset,expected,packet->payload[i]);
            if(sample)probe_security_trace_hash(sample->before_hash,sample->after_hash,sample->owner_hash,
                                                sample->ordinal,packet->size,(uint16_t)i,sample->after_ready);
            receive_pattern.offset=(uint16_t)(receive_pattern.offset+packet->size-i);
            break;
        }
        ++receive_pattern.offset;
    }
    if(receive_pattern.offset>=receive_pattern.size)receive_pattern.active=false;
}
uint32_t probe_echo_wait_ms(void) {
    if (!runtime)
        return UINT32_MAX;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        const echo_t *reply = &replies[i];
        if (!reply->used || reply->active)
            continue;
        const dbt_link_t *link = dbt_acl_find(&runtime->acl, reply->handle);
        if (!link || link->generation != reply->generation)
            return 0;
        if (!probe_security_authorize(0, link, 0x1001, DBT_SECURITY_ENCRYPTED))
            continue;
        for (unsigned j = 0; j < DBT_MAX_LINKS; ++j)
            if (runtime->acl.links[j].used && runtime->acl.links[j].handle == reply->handle &&
                !runtime->tx[j].active)
                return 0;
    }
    return UINT32_MAX;
}
