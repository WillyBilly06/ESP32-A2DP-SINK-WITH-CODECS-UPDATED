#include "dbt_channels.h"

enum { TX_PLAIN, TX_REQUEST, TX_CONNECTION_ACCEPT, TX_CONFIG_ACCEPT, TX_SECURITY_PENDING, TX_LE_ACCEPT, TX_LE_GRANT };
enum { CONNECTION_REQUEST = 2, CONNECTION_RESPONSE = 3, CONFIG_REQUEST = 4,
       CONFIG_RESPONSE = 5, DISCONNECT_REQUEST = 6, DISCONNECT_RESPONSE = 7,
       LE_CONNECT_REQUEST=0x14,LE_CONNECT_RESPONSE=0x15,LE_CREDITS=0x16 };
struct dbt_le_channel {
    dbt_credit_t engine;
    dbt_channels_t *manager;
    dbt_channel_t *channel;
    uint16_t initial_credits;
    uint8_t minimum_key_size;
    bool faulted;
};

static void copy(void *destination, const void *source, unsigned size) {
    uint8_t *out = destination;
    const uint8_t *in = source;
    for (unsigned i = 0; i < size; ++i) out[i] = in[i];
}
static void zero(void *destination, unsigned size) {
    uint8_t *out = destination;
    for (unsigned i = 0; i < size; ++i) out[i] = 0;
}
static bool equal(const uint8_t *left, const uint8_t *right, unsigned size) {
    for (unsigned i = 0; i < size; ++i) if (left[i] != right[i]) return false;
    return true;
}
static void put16(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)value; data[1] = (uint8_t)(value >> 8);
}
static bool due(uint32_t now, uint32_t deadline) {
    return (uint32_t)(now - deadline) < 0x80000000u;
}
static bool psm_valid(uint16_t psm) { return (psm & 0x0101u) == 1u; }
static uint16_t smaller(uint16_t left, uint16_t right) { return left < right ? left : right; }
static uint8_t *le_allocate(void *context,uint16_t bytes){
    struct dbt_le_channel *le=context;dbt_channels_t *m=le->manager;
    return m->hooks.allocate(m->hooks.context,bytes);
}
static void le_release(void *context,uint8_t *bytes){
    struct dbt_le_channel *le=context;dbt_channels_t *m=le->manager;m->hooks.release(m->hooks.context,bytes);
}
static void le_sdu(void *context,const uint8_t *bytes,uint16_t size){
    struct dbt_le_channel *le=context;dbt_channels_t *m=le->manager;
    if(m->hooks.data){m->in_callback=true;m->hooks.data(m->hooks.context,le->channel,bytes,size);m->in_callback=false;}
}
static void le_dispose(dbt_channels_t *m,dbt_channel_t *c){
    if(c->le){dbt_credit_close(&c->le->engine);m->hooks.release(m->hooks.context,(uint8_t *)c->le);c->le=0;}
}

static dbt_channel_t *find(dbt_channels_t *manager, uint16_t handle, uint16_t cid) {
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        dbt_channel_t *channel = &manager->channels[i];
        if (channel->state != DBT_CH_FREE && channel->handle == handle && channel->local_cid == cid) return channel;
    }
    return 0;
}
const dbt_channel_t *dbt_channels_find(const dbt_channels_t *manager, uint16_t handle, uint16_t cid) {
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        const dbt_channel_t *channel = &manager->channels[i];
        if (channel->state != DBT_CH_FREE && channel->handle == handle && channel->local_cid == cid) return channel;
    }
    return 0;
}
static dbt_channel_t *by_request(dbt_channels_t *manager, uint16_t handle, uint32_t generation,
                                 uint8_t code, uint8_t id) {
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        dbt_channel_t *channel = &manager->channels[i];
        if (channel->state != DBT_CH_FREE && channel->handle == handle && channel->link_generation == generation &&
            channel->request_sent && channel->request_id == id && channel->request_code == code) return channel;
    }
    return 0;
}
static dbt_channel_t *by_remote(dbt_channels_t *manager, uint16_t handle, uint32_t generation, uint16_t remote_cid) {
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        dbt_channel_t *channel = &manager->channels[i];
        if (channel->state != DBT_CH_FREE && channel->handle == handle && channel->link_generation == generation &&
            channel->remote_cid == remote_cid) return channel;
    }
    return 0;
}
static void notify(dbt_channels_t *manager, dbt_channel_t *channel, dbt_channel_event_t event,
                     dbt_channel_reason_t reason, uint16_t result) {
    if (!manager->hooks.event) return;
    dbt_channel_t snapshot;
    copy(&snapshot, channel, sizeof(snapshot));
    manager->in_callback = true;
    manager->hooks.event(manager->hooks.context, event, &snapshot, reason, result);
    manager->in_callback = false;
}
static void finish(dbt_channels_t *manager, dbt_channel_t *channel, dbt_channel_reason_t reason, uint16_t result) {
    bool opened = channel->opened_once;
    /* Keep the snapshot useful to the event consumer, then make the slot free. */
    notify(manager, channel, opened ? DBT_CH_EVENT_CLOSED : DBT_CH_EVENT_FAILED, reason, result);
    le_dispose(manager,channel);
    zero(channel, sizeof(*channel));
}
static void maybe_open(dbt_channels_t *manager, dbt_channel_t *channel) {
    if (channel->state != DBT_CH_CONFIGURING || !channel->connection_ready ||
        !channel->local_configured || !channel->peer_configured || channel->request_code != 0) return;
    bool reconfigured = channel->reconfiguring;
    channel->state = DBT_CH_OPEN;
    channel->opened_once = true;
    channel->reconfiguring = false;
    notify(manager, channel, reconfigured ? DBT_CH_EVENT_RECONFIGURED : DBT_CH_EVENT_OPEN, DBT_CH_REASON_NORMAL, 0);
}
static bool live_link(const dbt_channels_t *manager, uint16_t handle, uint32_t generation) {
    const dbt_link_t *link = dbt_acl_find(manager->acl, handle);
    return link && link->generation == generation;
}
static bool tx_valid(const dbt_channels_t *manager, const dbt_signal_tx_t *tx) {
    if (!tx->valid || !live_link(manager, tx->handle, tx->generation)) return false;
    if (tx->serial == 0) return true;
    const dbt_channel_t *channel = dbt_channels_find(manager, tx->handle, tx->local_cid);
    if (!channel || channel->serial != tx->serial) return false;
    if (tx->effect == TX_REQUEST)
        return channel->request_code == tx->bytes[0] && channel->request_id == tx->bytes[1];
    return channel->state != DBT_CH_DISCONNECTING;
}
static void pop_tx(dbt_channels_t *manager) {
    manager->tx[manager->tx_head].valid = false;
    manager->tx_head = (uint8_t)((manager->tx_head + 1u) % DBT_SIGNAL_TX_SLOTS);
    --manager->tx_count;
    manager->tx_locked = false;
}
static void prune_tx(dbt_channels_t *manager) {
    while (!manager->tx_locked && manager->tx_count && !tx_valid(manager, &manager->tx[manager->tx_head])) pop_tx(manager);
}
static dbt_result_t enqueue(dbt_channels_t *manager, const dbt_link_t *link, uint16_t signaling_cid,
                             uint8_t code, uint8_t id, const uint8_t *body, uint16_t size,
                             dbt_channel_t *channel, uint8_t effect, dbt_signal_tx_t **result) {
    prune_tx(manager);
    if (size + 4u > DBT_SIGNAL_BYTES || id == 0 || (size && !body)) return DBT_INVALID;
    if (manager->tx_count == DBT_SIGNAL_TX_SLOTS) return DBT_BUSY;
    unsigned index = (manager->tx_head + manager->tx_count) % DBT_SIGNAL_TX_SLOTS;
    dbt_signal_tx_t *slot = &manager->tx[index];
    zero(slot, sizeof(*slot));
    slot->handle = link->handle; slot->generation = link->generation; slot->cid = signaling_cid;
    slot->effect = effect; slot->valid = true; slot->size = (uint8_t)(4u + size);
    if (channel) { slot->serial = channel->serial; slot->local_cid = channel->local_cid; }
    slot->bytes[0] = code; slot->bytes[1] = id; put16(slot->bytes + 2, size);
    if (size) copy(slot->bytes + 4, body, size);
    ++manager->tx_count;
    if (result) *result = slot;
    return DBT_OK;
}
static bool authorized(dbt_channels_t *manager, const dbt_link_t *link, uint16_t psm, dbt_security_t security) {
    if (security >= DBT_SECURITY_ENCRYPTED && !link->encrypted) return false;
    if (!manager->hooks.authorize) return security != DBT_SECURITY_AUTHENTICATED;
    manager->in_callback = true;
    bool allowed = manager->hooks.authorize(manager->hooks.context, link, psm, security);
    manager->in_callback = false;
    return allowed;
}
static dbt_result_t security_request(dbt_channels_t *manager,const dbt_link_t *link,uint16_t psm,dbt_security_t security,bool start) {
    if(!manager->hooks.security_request)return DBT_UNSUPPORTED;
    manager->in_callback=true;
    dbt_result_t result=manager->hooks.security_request(manager->hooks.context,link,psm,security,start);
    manager->in_callback=false;return result;
}
static dbt_channel_t *allocate_channel(dbt_channels_t *manager, const dbt_link_t *link, uint16_t psm,
                                       uint16_t mtu, uint16_t minimum, dbt_security_t security) {
    if (manager->serial == UINT32_MAX) return 0;
    dbt_channel_t *empty = 0;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) if (manager->channels[i].state == DBT_CH_FREE) { empty = &manager->channels[i]; break; }
    if (!empty) return 0;
    uint16_t cid = 0x0040;
    while (find(manager, link->handle, cid)) ++cid;
    if(link->kind==DBT_LINK_LE && cid>0x007f)return 0;
    zero(empty, sizeof(*empty));
    empty->serial = ++manager->serial;
    empty->handle = link->handle; empty->link_generation = link->generation;
    empty->local_cid = cid; empty->psm = psm; empty->rx_mtu = empty->rx_capacity = mtu;
    empty->minimum_mtu = minimum; empty->tx_mtu = smaller(672, manager->maximum_mtu);
    empty->security = security; empty->state = DBT_CH_CONNECTING;
    return empty;
}
static uint8_t request_id(dbt_channels_t *manager, const dbt_link_t *link) {
    dbt_signal_link_t *state = 0;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_signal_link_t *candidate = &manager->links[i];
        if (candidate->used && candidate->handle == link->handle && candidate->generation == link->generation) { state = candidate; break; }
        if (!candidate->used || !live_link(manager, candidate->handle, candidate->generation)) state = candidate;
    }
    if (!state) return 0;
    if (!state->used || state->handle != link->handle || state->generation != link->generation) {
        zero(state, sizeof(*state)); state->used = true; state->handle = link->handle; state->generation = link->generation;
    }
    for (unsigned attempt = 0; attempt < 255; ++attempt) {
        if (++state->next_id == 0) ++state->next_id;
        bool busy = false;
        for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
            dbt_channel_t *channel = &manager->channels[i];
            if (channel->state != DBT_CH_FREE && channel->handle == link->handle && channel->link_generation == link->generation &&
                channel->request_code && channel->request_id == state->next_id) busy = true;
        }
        if (!busy) return state->next_id;
    }
    return 0;
}
static dbt_result_t queue_request(dbt_channels_t *manager, dbt_channel_t *channel, uint8_t code, bool null_config) {
    const dbt_link_t *link = dbt_acl_find(manager->acl, channel->handle);
    if (!link || link->generation != channel->link_generation) return DBT_INVALID;
    uint8_t body[10]; uint16_t size = 4;
    if (code == CONNECTION_REQUEST) { put16(body, channel->psm); put16(body + 2, channel->local_cid); }
    else if (code == CONFIG_REQUEST) {
        put16(body, channel->remote_cid); put16(body + 2, 0);
        if (!null_config) { body[4] = 1; body[5] = 2; put16(body + 6, channel->rx_mtu); size = 8; }
    } else if(code==LE_CONNECT_REQUEST){
        if(!channel->le)return DBT_INVALID;
        put16(body,channel->psm);put16(body+2,channel->local_cid);put16(body+4,channel->rx_mtu);
        put16(body+6,channel->le->engine.local.mps);put16(body+8,channel->le->initial_credits);size=10;
    } else { put16(body, channel->remote_cid); put16(body + 2, channel->local_cid); }
    uint8_t id = request_id(manager, link);
    dbt_result_t result = enqueue(manager, link, link->kind==DBT_LINK_LE?5:1, code, id, body, size, channel, TX_REQUEST, 0);
    if (result == DBT_OK) {
        channel->request_code = code; channel->request_id = id;
        channel->request_sent = false; channel->pending_extended = false;
        channel->deadline = manager->now_ms + manager->rtx_ms;
    }
    return result;
}
static bool le_selection_valid(const dbt_channels_t *m,const dbt_le_service_t *s){
    return s && s->spsm && s->spsm<=255 && s->receive.mtu>=23 && s->receive.mtu<=m->maximum_mtu &&
        s->receive.mps>=23 && s->receive.mps<=65533u && s->receive.mps<=m->maximum_mtu &&
        s->security<=DBT_SECURITY_AUTHENTICATED &&
        (!s->minimum_key_size || (s->minimum_key_size>=7 && s->minimum_key_size<=16 && s->security>=DBT_SECURITY_ENCRYPTED));
}
static uint16_t le_security(dbt_channels_t *m,const dbt_link_t *link,const dbt_le_service_t *s){
    if(s->security>=DBT_SECURITY_ENCRYPTED && !link->encrypted)return 8;
    if(!m->hooks.le_security && (s->security==DBT_SECURITY_AUTHENTICATED || s->minimum_key_size))
        return s->security==DBT_SECURITY_AUTHENTICATED?5:7;
    if(m->hooks.le_security){
        m->in_callback=true;
        uint16_t result=m->hooks.le_security(m->hooks.context,link,s->spsm,s->security,s->minimum_key_size);
        m->in_callback=false;if(result)return result>=5 && result<=8?result:6;
    }
    return authorized(m,link,s->spsm,s->security)?0:6;
}
static bool le_construct(dbt_channels_t *m,dbt_channel_t *c,const dbt_le_service_t *s,dbt_credit_parameters_t peer){
    struct dbt_le_channel *le=(struct dbt_le_channel *)m->hooks.allocate(m->hooks.context,sizeof(*le));
    if(!le)return false;
    zero(le,sizeof(*le));le->manager=m;le->channel=c;le->initial_credits=s->receive.credits;le->minimum_key_size=s->minimum_key_size;
    dbt_credit_parameters_t local=s->receive;local.credits=0;
    dbt_credit_hooks_t hooks={le,le_allocate,le_release,le_sdu};
    if(dbt_credit_init(&le->engine,hooks,local,peer)!=DBT_OK){m->hooks.release(m->hooks.context,(uint8_t *)le);return false;}
    c->le=le;return true;
}
static void le_fault(dbt_channels_t *m,dbt_channel_t *c){
    if(!c->le || c->state==DBT_CH_DISCONNECTING)return;
    c->le->faulted=true;dbt_credit_close(&c->le->engine);
    c->request_code=0;c->request_sent=false;c->state=DBT_CH_DISCONNECTING;c->deadline=m->now_ms+m->rtx_ms;
    (void)queue_request(m,c,DISCONNECT_REQUEST,false);
}

dbt_result_t dbt_channels_register_le(dbt_channels_t *m,const dbt_le_service_t *services,uint8_t count){
    if(!m || m->in_callback || !services || !count || count>DBT_SERVICE_LIMIT || m->le_service_count ||
       !m->hooks.allocate || !m->hooks.release)return DBT_INVALID;
    for(unsigned i=0;i<count;++i){
        if(!le_selection_valid(m,&services[i]))return DBT_INVALID;
        for(unsigned j=0;j<i;++j)if(services[i].spsm==services[j].spsm)return DBT_INVALID;
    }
    m->le_services=services;m->le_service_count=count;return DBT_OK;
}
dbt_result_t dbt_channels_open_le(dbt_channels_t *m,uint16_t handle,const dbt_le_service_t *s,uint32_t now,uint16_t *cid){
    if(!m || m->in_callback || !cid || !le_selection_valid(m,s) || !m->hooks.allocate || !m->hooks.release)return DBT_INVALID;
    const dbt_link_t *link=dbt_acl_find(m->acl,handle);if(!link || link->kind!=DBT_LINK_LE)return DBT_UNSUPPORTED;
    if(le_security(m,link,s))return DBT_UNSUPPORTED;
    prune_tx(m);if(m->tx_count==DBT_SIGNAL_TX_SLOTS)return DBT_BUSY;
    dbt_channel_t *c=allocate_channel(m,link,s->spsm,s->receive.mtu,23,s->security);if(!c)return DBT_NO_SPACE;
    dbt_credit_parameters_t peer={23,23,0};
    if(!le_construct(m,c,s,peer)){zero(c,sizeof(*c));return DBT_NO_SPACE;}
    m->now_ms=now;dbt_result_t result=queue_request(m,c,LE_CONNECT_REQUEST,false);
    if(result!=DBT_OK){le_dispose(m,c);zero(c,sizeof(*c));return result;}
    *cid=c->local_cid;return DBT_OK;
}
dbt_result_t dbt_channels_grant_le(dbt_channels_t *m,uint16_t handle,uint16_t cid,uint16_t credits){
    if(!m || m->in_callback)return DBT_INVALID;
    dbt_channel_t *c=find(m,handle,cid);const dbt_link_t *link=dbt_acl_find(m->acl,handle);
    if(!c || !c->le || !link || c->state!=DBT_CH_OPEN || c->link_generation!=link->generation)return DBT_BUSY;
    dbt_result_t result=dbt_credit_grant_prepare(&c->le->engine,credits);if(result!=DBT_OK)return result;
    uint8_t body[4];put16(body,c->local_cid);put16(body+2,credits);
    result=enqueue(m,link,5,LE_CREDITS,request_id(m,link),body,4,c,TX_LE_GRANT,0);
    if(result!=DBT_OK)dbt_credit_grant_cancel(&c->le->engine);
    return result;
}
static bool le_data_allowed(dbt_channels_t *m,dbt_channel_t *c,const dbt_link_t *link){
    if(!c || !c->le || !link || c->link_generation!=link->generation || c->state!=DBT_CH_OPEN || !c->connection_ready)return false;
    dbt_le_service_t s={c->psm,c->le->engine.local,c->security,c->le->minimum_key_size};
    return le_security(m,link,&s)==0;
}
dbt_result_t dbt_channels_credit_begin(dbt_channels_t *m,uint16_t handle,uint16_t cid,const uint8_t *bytes,uint16_t size){
    if(!m || m->in_callback)return DBT_INVALID;
    dbt_channel_t *c=find(m,handle,cid);if(!le_data_allowed(m,c,dbt_acl_find(m->acl,handle)))return DBT_BUSY;
    return dbt_credit_tx_begin(&c->le->engine,bytes,size);
}
dbt_result_t dbt_channels_credit_frame(dbt_channels_t *m,uint16_t handle,uint16_t cid,dbt_l2cap_tx_t *frame){
    if(!m || m->in_callback || !frame)return DBT_INVALID;
    dbt_channel_t *c=find(m,handle,cid);const dbt_link_t *link=dbt_acl_find(m->acl,handle);
    if(!le_data_allowed(m,c,link))return DBT_BUSY;
    dbt_credit_frame_t view;dbt_result_t result=dbt_credit_tx_view(&c->le->engine,&view);if(result!=DBT_OK)return result;
    return dbt_l2cap_tx_begin_credit(frame,link,c->remote_cid,view.data,view.size,view.first,view.sdu_length);
}
dbt_result_t dbt_channels_credit_commit(dbt_channels_t *m,uint16_t handle,uint16_t cid,bool *complete){
    if(!m || m->in_callback || !complete)return DBT_INVALID;
    dbt_channel_t *c=find(m,handle,cid);if(!c || !c->le || c->state!=DBT_CH_OPEN)return DBT_INVALID;
    dbt_result_t result=dbt_credit_tx_commit(&c->le->engine);if(result==DBT_OK)*complete=!c->le->engine.tx_active;return result;
}
bool dbt_channels_credit_ready(const dbt_channels_t *m,uint16_t handle,uint16_t cid){
    if(!m)return false;
    const dbt_channel_t *c=dbt_channels_find(m,handle,cid);
    return c && c->le && c->state==DBT_CH_OPEN && c->le->engine.tx_active && c->le->engine.peer.credits;
}
void dbt_channels_clear(dbt_channels_t *m){
    if(!m || m->in_callback)return;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)if(m->channels[i].state!=DBT_CH_FREE)finish(m,&m->channels[i],DBT_CH_REASON_PROTOCOL,0);
    m->tx_count=m->rx_count=0;m->tx_locked=m->rx_deferred=false;
}
static dbt_result_t reject(dbt_channels_t *manager, const dbt_link_t *link, uint16_t cid,
                            uint8_t id, uint16_t reason, uint16_t local, uint16_t remote) {
    uint8_t data[6]; put16(data, reason);
    uint16_t size = 2;
    if (reason == 1) { put16(data + 2, link->kind == DBT_LINK_LE ? 23 : DBT_SIGNAL_BYTES); size = 4; }
    else if (reason == 2) { put16(data + 2, local); put16(data + 4, remote); size = 6; }
    return enqueue(manager, link, cid, 1, id, data, size, 0, TX_PLAIN, 0);
}

dbt_result_t dbt_channels_init(dbt_channels_t *manager, dbt_acl_t *acl, dbt_channel_hooks_t hooks,
                               uint16_t maximum_mtu, uint32_t rtx_ms, uint32_t ertx_ms) {
    if (!manager || !acl || maximum_mtu < 48 || rtx_ms < 1000 || rtx_ms > 60000 ||
        ertx_ms < 60000 || ertx_ms > 300000) return DBT_INVALID;
    zero(manager, sizeof(*manager));
    manager->acl = acl; manager->hooks = hooks; manager->maximum_mtu = maximum_mtu;
    manager->rtx_ms = rtx_ms; manager->ertx_ms = ertx_ms;
    manager->fixed_mask[DBT_LINK_CLASSIC] = (uint64_t)1 << 1;
    manager->fixed_mask[DBT_LINK_LE] = (uint64_t)1 << 5;
    return DBT_OK;
}
dbt_result_t dbt_channels_register_fixed(dbt_channels_t *manager, dbt_link_kind_t kind, uint16_t cid) {
    if (!manager || manager->in_callback || kind > DBT_LINK_LE || cid == 0 || cid >= 64 ||
        cid == 1 || cid == 5 || !manager->hooks.fixed_data) return DBT_INVALID;
    manager->fixed_mask[kind] |= (uint64_t)1 << cid;
    return DBT_OK;
}
dbt_result_t dbt_channels_register(dbt_channels_t *manager, dbt_service_t service) {
    if (!manager || manager->in_callback || !psm_valid(service.psm) || service.minimum_mtu < 48 ||
        service.rx_mtu < service.minimum_mtu || service.rx_mtu > manager->maximum_mtu ||
        service.security > DBT_SECURITY_AUTHENTICATED) return DBT_INVALID;
    dbt_service_t *empty = 0;
    for (unsigned i = 0; i < DBT_SERVICE_LIMIT; ++i) {
        if (manager->services[i].psm == service.psm) return DBT_INVALID;
        if (manager->services[i].psm == 0) empty = &manager->services[i];
    }
    if (!empty) return DBT_NO_SPACE;
    copy(empty, &service, sizeof(service));
    return DBT_OK;
}
dbt_result_t dbt_channels_open(dbt_channels_t *manager, uint16_t handle, uint16_t psm, uint16_t mtu,
                               uint16_t minimum, dbt_security_t security, uint32_t now, uint16_t *local_cid) {
    if (!manager || !local_cid || manager->in_callback || !psm_valid(psm) || minimum < 48 ||
        mtu < minimum || mtu > manager->maximum_mtu || security > DBT_SECURITY_AUTHENTICATED) return DBT_INVALID;
    const dbt_link_t *link = dbt_acl_find(manager->acl, handle);
    if (!link || link->kind != DBT_LINK_CLASSIC) return DBT_UNSUPPORTED;
    bool allowed=authorized(manager,link,psm,security);
    prune_tx(manager);
    if (manager->tx_count == DBT_SIGNAL_TX_SLOTS) return DBT_BUSY;
    dbt_channel_t *channel = allocate_channel(manager, link, psm, mtu, minimum, security);
    if (!channel) return DBT_NO_SPACE;
    manager->now_ms = now;
    if(!allowed) {
        dbt_result_t progress=security_request(manager,link,psm,security,true);
        if(progress!=DBT_OK && progress!=DBT_BUSY){zero(channel,sizeof(*channel));return DBT_UNSUPPORTED;}
        channel->state=DBT_CH_SECURING;channel->deadline=now+manager->ertx_ms;
        *local_cid=channel->local_cid;return DBT_OK;
    }
    dbt_result_t result = queue_request(manager, channel, CONNECTION_REQUEST, false);
    if (result != DBT_OK) { zero(channel, sizeof(*channel)); return result; }
    *local_cid = channel->local_cid;
    return DBT_OK;
}
dbt_result_t dbt_channels_close(dbt_channels_t *manager, uint16_t handle, uint16_t cid, uint32_t now) {
    if (!manager || manager->in_callback) return DBT_INVALID;
    dbt_channel_t *channel = find(manager, handle, cid);
    if (!channel) return DBT_INVALID;
    if (channel->state == DBT_CH_DISCONNECTING) return DBT_BUSY;
    if (!channel->connection_ready) { finish(manager, channel, DBT_CH_REASON_NORMAL, 0); return DBT_OK; }
    prune_tx(manager);
    if (manager->tx_count == DBT_SIGNAL_TX_SLOTS) return DBT_BUSY;
    channel->request_code = 0; manager->now_ms = now;
    dbt_result_t result = queue_request(manager, channel, DISCONNECT_REQUEST, false);
    if (result == DBT_OK){channel->state=DBT_CH_DISCONNECTING;if(channel->le)dbt_credit_close(&channel->le->engine);}
    return result;
}

static bool command_shape(const uint8_t *command, uint16_t size) {
    switch (command[0]) {
    case 1: {
        if (size < 6) return false;
        uint16_t reason = dbt_read_le16(command + 4);
        return reason <= 2 && size == (reason == 0 ? 6u : reason == 1 ? 8u : 10u);
    }
    case CONNECTION_REQUEST: return size == 8;
    case CONNECTION_RESPONSE: return size == 12;
    case CONFIG_REQUEST: return size >= 8;
    case CONFIG_RESPONSE: return size >= 10;
    case DISCONNECT_REQUEST: case DISCONNECT_RESPONSE: return size == 8;
    case 0x0a: return size == 6;
    case 0x0b: return size >= 8;
    case 0x12: return size == 12;
    case 0x13: return size == 6;
    case LE_CONNECT_REQUEST:case LE_CONNECT_RESPONSE:return size==14;
    case LE_CREDITS:return size==8;
    default: return true;
    }
}
static bool response_code(uint8_t code){
    switch(code){
    case 1:case 3:case 5:case 7:case 9:case 0x0b:case 0x13:case 0x15:case 0x18:case 0x1a:return true;
    default:return false;
    }
}
dbt_result_t dbt_channels_receive(dbt_channels_t *manager, uint16_t handle, uint32_t generation,
                                  uint16_t cid, const uint8_t *data, uint16_t size) {
    if (!manager || manager->in_callback || (size && !data)) return DBT_INVALID;
    const dbt_link_t *link = dbt_acl_find(manager->acl, handle);
    if (!link || link->generation != generation) return DBT_INVALID;
    if (cid != (link->kind == DBT_LINK_CLASSIC ? 1 : 5)) {
        if (cid >= 0x0040) {
            dbt_channel_t *channel = find(manager, handle, cid);
            /* Core Vol 3 A 6.1: CLOSED / WAIT_CONNECT / WAIT_DISCONNECT data
             * is ignored. A queued media PDU during Close is not a host fault.
             * Basic-mode data still flows during reconfiguration of an open
             * channel, using its current receive MTU and security policy. */
            if(!channel){++manager->ignored_dynamic_pdus;return DBT_OK;}
            if(channel->link_generation!=generation)return DBT_INVALID;
            bool receive_open=channel->state==DBT_CH_OPEN ||
                (channel->state==DBT_CH_CONFIGURING && channel->opened_once && channel->reconfiguring);
            if(!receive_open){++manager->ignored_dynamic_pdus;return DBT_OK;}
            if(channel->le){
                if(!le_data_allowed(manager,channel,link)){le_fault(manager,channel);return DBT_OK;}
                if(dbt_credit_receive(&channel->le->engine,data,size)!=DBT_OK)le_fault(manager,channel);
                return DBT_OK;
            }
            if(size>channel->rx_mtu){++manager->oversized_dynamic_pdus;return DBT_OK;}
            if (!authorized(manager,link,channel->psm,channel->security)) return DBT_UNSUPPORTED;
            if (manager->hooks.data) {
                manager->in_callback = true;
                manager->hooks.data(manager->hooks.context, channel, data, size);
                manager->in_callback = false;
            }
            return DBT_OK;
        }
        if (!manager->hooks.fixed_data || cid == 0 || cid >= 64 ||
            !(manager->fixed_mask[link->kind] & ((uint64_t)1 << cid))) return DBT_UNSUPPORTED;
        manager->in_callback = true;
        manager->hooks.fixed_data(manager->hooks.context, link, cid, data, size);
        manager->in_callback = false;
        return DBT_OK;
    }
    uint32_t cursor = 0, first_request = UINT32_MAX; unsigned commands = 0;
    while (cursor < size) {
        if (size - cursor < 4u) goto malformed;
        uint16_t length = dbt_read_le16(data + cursor + 2);
        uint32_t total = 4u + length;
        if (total > size - cursor || data[cursor + 1] == 0 ||
            !command_shape(data + cursor, (uint16_t)total)) goto malformed;
        if(first_request==UINT32_MAX && !response_code(data[cursor]))first_request=cursor;
        cursor += total; ++commands;
    }
    if (commands == 0 || (link->kind == DBT_LINK_LE && commands != 1)) goto malformed;
    bool oversized=size>(link->kind==DBT_LINK_LE?23u:DBT_SIGNAL_BYTES);
    /* Core 4.1: reject the first Request in an oversized C-frame. A frame
     * containing only recognized Responses is silently discarded. */
    if(oversized && first_request==UINT32_MAX){++manager->oversized_signaling_pdus;return DBT_OK;}
    if (manager->rx_count == DBT_SIGNAL_RX_SLOTS) return DBT_NO_SPACE;
    unsigned index = (manager->rx_head + manager->rx_count) % DBT_SIGNAL_RX_SLOTS;
    dbt_signal_rx_t *slot = &manager->rx[index];
    zero(slot, sizeof(*slot));
    slot->handle = handle; slot->generation = generation; slot->cid = cid;
    slot->oversized = oversized;
    slot->size = slot->oversized ? 4 : (uint8_t)size;
    copy(slot->bytes, data+(oversized?first_request:0u), slot->size); ++manager->rx_count;
    if(oversized)++manager->oversized_signaling_pdus;
    manager->rx_deferred = false;
    return DBT_OK;
malformed:
    ++manager->malformed_pdus;
    return DBT_INVALID;
}

static bool accept_incoming(dbt_channels_t *manager,const dbt_link_t *link,uint16_t psm){
    if(!manager->hooks.accept)return true;
    manager->in_callback=true;
    bool accepted=manager->hooks.accept(manager->hooks.context,link,psm);
    manager->in_callback=false;
    return accepted;
}
static dbt_result_t connection_request(dbt_channels_t *manager, const dbt_link_t *link, const uint8_t *command) {
    uint16_t psm = dbt_read_le16(command + 4), peer = dbt_read_le16(command + 6);
    dbt_channel_t *existing = by_remote(manager, link->handle, link->generation, peer);
    uint16_t result = 0; dbt_channel_t *channel = 0; dbt_service_t *service = 0;
    if (existing && existing->psm == psm && existing->incoming_connect_id == command[1]) channel = existing;
    else {
        for (unsigned i = 0; i < DBT_SERVICE_LIMIT; ++i) if (manager->services[i].psm == psm) service = &manager->services[i];
        if (!service || !psm_valid(psm)) result = 2;
        else if (peer < 0x0040) result = 6;
        else if (existing) result = 7;
        else if (!accept_incoming(manager,link,psm)) result = 4;
        else {
            channel = allocate_channel(manager, link, psm, service->rx_mtu, service->minimum_mtu, service->security);
            if (!channel) result = 4;
            else { channel->remote_cid = peer; channel->incoming_connect_id = command[1]; channel->state = DBT_CH_CONFIGURING;
                if(!authorized(manager,link,psm,service->security)) {
                    dbt_result_t progress=security_request(manager,link,psm,service->security,true);
                    if(progress==DBT_BUSY || progress==DBT_OK){channel->state=DBT_CH_SECURING;channel->security_incoming=true;channel->deadline=manager->now_ms+manager->ertx_ms;}
                    else{zero(channel,sizeof(*channel));channel=0;result=3;}
                }
            }
        }
    }
    uint8_t response[8];
    put16(response, channel ? channel->local_cid : 0); put16(response + 2, peer);
    bool securing=channel && channel->state==DBT_CH_SECURING;
    put16(response + 4, securing?1:result); put16(response + 6, securing?1:0);
    dbt_result_t queued = enqueue(manager, link, 1, CONNECTION_RESPONSE, command[1], response, 8,
                                  channel, securing?TX_SECURITY_PENDING:channel ? TX_CONNECTION_ACCEPT : TX_PLAIN, 0);
    if (channel && !channel->connection_ready && !securing) channel->deadline = manager->now_ms + manager->rtx_ms;
    return queued;
}
static void pending(dbt_channels_t *manager, dbt_channel_t *channel) {
    if (channel->pending_extended && due(manager->now_ms, channel->pending_limit)) {
        finish(manager, channel, DBT_CH_REASON_RESPONSE_TIMEOUT, 0);
        return;
    }
    if (!channel->pending_extended) {
        channel->pending_extended = true;
        channel->pending_limit = manager->now_ms + 300000u;
    }
    channel->deadline = manager->now_ms + manager->ertx_ms;
    if ((uint32_t)(channel->pending_limit - manager->now_ms) < manager->ertx_ms) channel->deadline = channel->pending_limit;
}
static void connection_response(dbt_channels_t *manager, const dbt_link_t *link, const uint8_t *command) {
    dbt_channel_t *channel = by_request(manager, link->handle, link->generation, CONNECTION_REQUEST, command[1]);
    if (!channel) { ++manager->unknown_responses; return; }
    uint16_t result = dbt_read_le16(command + 8);
    /* Core 4.3: both CIDs may be invalid on every non-success response,
     * including Pending. Correlate those by link generation and identifier. */
    if (result == 1) {
        if (dbt_read_le16(command + 10) > 2) finish(manager, channel, DBT_CH_REASON_PROTOCOL, result);
        else pending(manager, channel);
        return;
    }
    if (result != 0) { finish(manager, channel, DBT_CH_REASON_REFUSED, result); return; }
    uint16_t peer = dbt_read_le16(command + 4), local = dbt_read_le16(command + 6);
    if (local != channel->local_cid || peer < 0x0040 || by_remote(manager, link->handle, link->generation, peer)) {
        finish(manager, channel, DBT_CH_REASON_PROTOCOL, result); return;
    }
    channel->remote_cid = peer; channel->connection_ready = true;
    channel->request_code = 0; channel->request_sent = false;
    channel->state = DBT_CH_CONFIGURING; channel->deadline = manager->now_ms + manager->rtx_ms;
    channel->config_limit = manager->now_ms + 300000u;
}

typedef struct { uint16_t mtu, result; uint8_t options[38], size; bool mtu_seen; } options_t;
static bool parse_options(const uint8_t *data, uint16_t size, uint16_t minimum, uint16_t initial_mtu,
                           options_t *parsed, bool response) {
    zero(parsed, sizeof(*parsed)); parsed->mtu = initial_mtu;
    uint16_t cursor = 0;
    while (cursor < size) {
        if ((unsigned)(size - cursor) < 2u) return false;
        uint8_t raw_type = data[cursor], length = data[cursor + 1];
        uint16_t total = (uint16_t)(2u + length);
        if (total > size - cursor) return false;
        const uint8_t *value = data + cursor + 2;
        uint8_t type = raw_type & 0x7fu;
        uint16_t previous_mtu=parsed->mtu;bool previous_mtu_seen=parsed->mtu_seen;
        uint8_t replacement[24]; uint8_t replacement_size = 0; bool unknown = false;
        if (type == 1) {
            if (length != 2) return false;
            parsed->mtu = dbt_read_le16(value); parsed->mtu_seen = true;
            if (parsed->mtu < minimum) {
                replacement[0] = 1; replacement[1] = 2; put16(replacement + 2, minimum); replacement_size = 4;
            }
        } else if (type == 2) {
            if (length != 2) return false;
            if (dbt_read_le16(value) == 0) {
                replacement[0] = 2; replacement[1] = 2; put16(replacement + 2, 0xffff); replacement_size = 4;
            }
        } else if (type == 3) {
            if (length != 22) return false;
            if (value[1] != 1) { copy(replacement, data + cursor, total); replacement[3] = 1; replacement_size = (uint8_t)total; }
        } else if (type == 4) {
            if (length != 9) return false;
            if (value[0] != 0) {
                zero(replacement, 11); replacement[0] = 4; replacement[1] = 9; replacement_size = 11;
            }
        } else if (type == 5) {
            if (length != 1) return false; /* FCS is unused in Basic mode. */
        } else if (type == 7) {
            if (length != 2) return false; /* Extended window is unused in Basic mode. */
        } else unknown = true;
        if (!response && unknown && !(raw_type & 0x80u)) {
            if (parsed->result != 3) { parsed->result = 3; parsed->size = 0; }
            if (parsed->size + total > sizeof(parsed->options)) { parsed->result = 2; parsed->size = 0; return true; }
            copy(parsed->options + parsed->size, data + cursor, total); parsed->size += (uint8_t)total;
        } else if (!response && replacement_size && !(raw_type&0x80u) && parsed->result != 3) {
            parsed->result = 1;
            if (parsed->size + replacement_size > sizeof(parsed->options)) { parsed->result = 2; parsed->size = 0; return true; }
            copy(parsed->options + parsed->size, replacement, replacement_size); parsed->size += replacement_size;
        } else if (response && ((unknown && !(raw_type & 0x80u)) || replacement_size)) return false;
        /* Hints cannot be the sole cause of a rejected configuration (4.5).
         * Keep the previous MTU when a hinted value is below the minimum. */
        if(!response && (raw_type&0x80u) && type==1 && replacement_size){
            parsed->mtu=previous_mtu;parsed->mtu_seen=previous_mtu_seen;
        }
        cursor += total;
    }
    if (!response && !parsed->mtu_seen && parsed->mtu < minimum && parsed->result == 0) {
        parsed->result = 1; parsed->size = 4;
        parsed->options[0] = 1; parsed->options[1] = 2; put16(parsed->options + 2, minimum);
    }
    return true;
}
static dbt_result_t config_request(dbt_channels_t *manager, const dbt_link_t *link, const uint8_t *command, uint16_t size) {
    uint16_t local = dbt_read_le16(command + 4), flags = dbt_read_le16(command + 6);
    dbt_channel_t *channel = find(manager, link->handle, local);
    if (!channel || !channel->connection_ready || channel->state == DBT_CH_DISCONNECTING) return reject(manager, link, 1, command[1], 2, local, 0);
    if (channel->cached_request_size == size && equal(channel->cached_request, command, size)) {
        dbt_signal_tx_t *slot = 0;
        dbt_result_t result = enqueue(manager, link, 1, CONFIG_RESPONSE, command[1], channel->cached_reply + 4,
                                      (uint16_t)(channel->cached_reply_size - 4u), channel,
                                      channel->cached_reply[8] == 0 && channel->cached_reply[6] == 0 && !channel->incoming_failed ? TX_CONFIG_ACCEPT : TX_PLAIN, &slot);
        if (slot) { slot->config_epoch = channel->config_epoch; slot->peer_mtu = channel->peer_mtu_staged; }
        return result;
    }
    if (channel->state == DBT_CH_OPEN && manager->hooks.tx_idle) {
        manager->in_callback = true;
        bool idle = manager->hooks.tx_idle(manager->hooks.context, channel->handle, channel->local_cid);
        manager->in_callback = false;
        if (!idle) return DBT_BUSY;
    }
    options_t options;
    uint16_t initial = channel->incoming_more ? channel->peer_mtu_staged : channel->tx_mtu;
    bool valid = parse_options(command + 8, size - 8u, channel->minimum_mtu, initial, &options, false);
    if (flags & ~1u) valid = false;
    if (!valid) { zero(&options, sizeof(options)); options.result = 2; options.mtu = initial; }
    bool was_open = channel->state == DBT_CH_OPEN;
    if (!channel->incoming_more) { ++channel->config_epoch; channel->incoming_failed = false; }
    channel->incoming_failed |= options.result != 0;
    channel->peer_mtu_staged = smaller(options.mtu, manager->maximum_mtu);
    channel->incoming_more = (flags & 1u) != 0;
    uint8_t response[44]; put16(response, channel->remote_cid); put16(response + 2, flags & 1u); put16(response + 4, options.result);
    uint8_t option_size = options.size;
    if (options.result == 0 && !channel->incoming_more) {
        response[6] = 1; response[7] = 2; put16(response + 8, channel->peer_mtu_staged); option_size = 4;
    } else if (option_size) copy(response + 6, options.options, option_size);
    bool accepted = !channel->incoming_more && !channel->incoming_failed;
    dbt_signal_tx_t *slot = 0;
    dbt_result_t result = enqueue(manager, link, 1, CONFIG_RESPONSE, command[1], response, (uint16_t)(6u + option_size),
                                  channel, accepted ? TX_CONFIG_ACCEPT : TX_PLAIN, &slot);
    if (result != DBT_OK) return result;
    slot->config_epoch = channel->config_epoch; slot->peer_mtu = channel->peer_mtu_staged;
    channel->cached_request_size = (uint8_t)size; copy(channel->cached_request, command, size);
    channel->cached_reply_size = slot->size; copy(channel->cached_reply, slot->bytes, slot->size);
    if (was_open && options.result == 0) {
        channel->reconfiguring = true; channel->state = DBT_CH_CONFIGURING;
        channel->local_configured = false;
        channel->config_limit = manager->now_ms + 300000u;
    }
    if (channel->state == DBT_CH_CONFIGURING && options.result == 0) channel->peer_configured = false;
    if (channel->state != DBT_CH_OPEN && channel->request_code == 0) channel->deadline = manager->now_ms + manager->rtx_ms;
    return DBT_OK;
}
static void config_response(dbt_channels_t *manager, const dbt_link_t *link, const uint8_t *command, uint16_t size) {
    dbt_channel_t *channel = by_request(manager, link->handle, link->generation, CONFIG_REQUEST, command[1]);
    if (!channel || dbt_read_le16(command + 4) != channel->local_cid) { ++manager->unknown_responses; return; }
    uint16_t flags = dbt_read_le16(command + 6), result = dbt_read_le16(command + 8);
    options_t options;
    if ((flags & ~1u) || !parse_options(command + 10, size - 10u, channel->minimum_mtu, channel->rx_mtu, &options, true) ||
        (options.mtu_seen && (options.mtu < channel->minimum_mtu || options.mtu > channel->rx_mtu))) {
        finish(manager, channel, DBT_CH_REASON_PROTOCOL, result); return;
    }
    if (options.mtu_seen) channel->rx_mtu = options.mtu;
    if (result == 4) {
        if (flags & 1u) finish(manager, channel, DBT_CH_REASON_PROTOCOL, result);
        else pending(manager, channel);
        return;
    }
    channel->request_code = 0; channel->request_sent = false;
    if (result == 1 && options.mtu_seen && channel->attempts++ < 2) {
        channel->deadline = manager->now_ms + manager->rtx_ms;
        return;
    }
    channel->response_failed |= result != 0;
    if (flags & 1u) {
        channel->response_more = true;
        (void)queue_request(manager, channel, CONFIG_REQUEST, true);
        return;
    }
    channel->response_more = false;
    if (channel->response_failed || result > 5) { finish(manager, channel, DBT_CH_REASON_REFUSED, result); return; }
    channel->local_configured = true;
    channel->deadline = manager->now_ms + manager->rtx_ms;
    maybe_open(manager, channel);
}

static dbt_result_t le_connection_request(dbt_channels_t *m,const dbt_link_t *link,const uint8_t *command){
    uint16_t spsm=dbt_read_le16(command+4),peer_cid=dbt_read_le16(command+6),result=0;
    dbt_channel_t *c=by_remote(m,link->handle,link->generation,peer_cid);
    if(c && c->le && c->cached_request_size==14 && equal(c->cached_request,command,14))
        return enqueue(m,link,5,LE_CONNECT_RESPONSE,command[1],c->cached_reply+4,10,c,
            c->connection_ready?TX_PLAIN:TX_LE_ACCEPT,0);
    const dbt_le_service_t *service=0;
    for(unsigned i=0;i<m->le_service_count;++i)if(m->le_services[i].spsm==spsm)service=&m->le_services[i];
    dbt_credit_parameters_t peer={dbt_read_le16(command+8),dbt_read_le16(command+10),dbt_read_le16(command+12)};
    if(peer_cid<0x40 || peer_cid>0x7f)result=9;
    else if(c)result=10;
    else if(!service)result=2;
    else if(peer.mtu<23 || peer.mps<23 || peer.mps>65533u)result=11;
    else if((result=le_security(m,link,service))!=0){}
    else if(!accept_incoming(m,link,spsm))result=4;
    c=0;
    if(!result){
        c=allocate_channel(m,link,spsm,service->receive.mtu,23,service->security);
        if(!c)result=4;
        else if(!le_construct(m,c,service,peer)){zero(c,sizeof(*c));c=0;result=4;}
        else{c->remote_cid=peer_cid;c->tx_mtu=peer.mtu;c->incoming_connect_id=command[1];c->deadline=m->now_ms+m->rtx_ms;}
    }
    uint8_t body[10];zero(body,sizeof(body));put16(body+8,result);
    if(c){put16(body,c->local_cid);put16(body+2,c->rx_mtu);put16(body+4,c->le->engine.local.mps);put16(body+6,c->le->initial_credits);}
    dbt_signal_tx_t *slot=0;
    dbt_result_t queued=enqueue(m,link,5,LE_CONNECT_RESPONSE,command[1],body,10,c,c?TX_LE_ACCEPT:TX_PLAIN,&slot);
    if(queued!=DBT_OK){if(c){le_dispose(m,c);zero(c,sizeof(*c));}return queued;}
    if(c){c->cached_request_size=14;copy(c->cached_request,command,14);c->cached_reply_size=14;copy(c->cached_reply,slot->bytes,14);}
    return DBT_OK;
}
static void le_connection_response(dbt_channels_t *m,const dbt_link_t *link,const uint8_t *command){
    dbt_channel_t *c=by_request(m,link->handle,link->generation,LE_CONNECT_REQUEST,command[1]);
    if(!c || !c->le){++m->unknown_responses;return;}
    uint16_t result=dbt_read_le16(command+12);
    if(result){finish(m,c,DBT_CH_REASON_REFUSED,result);return;}
    uint16_t cid=dbt_read_le16(command+4);
    dbt_credit_parameters_t peer={dbt_read_le16(command+6),dbt_read_le16(command+8),dbt_read_le16(command+10)};
    if(cid<0x40 || cid>0x7f || by_remote(m,link->handle,link->generation,cid) || peer.mtu<23 || peer.mps<23 || peer.mps>65533u){
        finish(m,c,DBT_CH_REASON_PROTOCOL,0);return;
    }
    c->remote_cid=cid;c->tx_mtu=peer.mtu;c->le->engine.peer=peer;
    c->connection_ready=c->local_configured=c->peer_configured=true;
    c->request_code=0;c->request_sent=false;c->state=DBT_CH_OPEN;
    if(!le_data_allowed(m,c,link)){le_fault(m,c);return;}
    c->opened_once=true;notify(m,c,DBT_CH_EVENT_OPEN,DBT_CH_REASON_NORMAL,0);
}
static dbt_result_t process_command(dbt_channels_t *manager, const dbt_link_t *link, uint16_t cid,
                                     const uint8_t *command, uint16_t size) {
    uint8_t code = command[0], id = command[1];
    const uint8_t *body = command + 4;
    if (link->kind == DBT_LINK_LE && code != 1 && code != 6 && code != 7 && code != 0x12 && code != 0x13 &&
        code!=LE_CONNECT_REQUEST && code!=LE_CONNECT_RESPONSE && code!=LE_CREDITS)return reject(manager,link,cid,id,0,0,0);
    if (link->kind == DBT_LINK_CLASSIC && (code == 0x12 || code == 0x13 || code==LE_CONNECT_REQUEST || code==LE_CONNECT_RESPONSE || code==LE_CREDITS)) return reject(manager, link, cid, id, 0, 0, 0);
    switch (code) {
    case 1:
        for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
            dbt_channel_t *channel = &manager->channels[i];
            if (channel->state != DBT_CH_FREE && channel->handle == link->handle && channel->link_generation == link->generation &&
                channel->request_code && channel->request_sent && channel->request_id == id) {
                finish(manager, channel, DBT_CH_REASON_PROTOCOL, dbt_read_le16(body)); break;
            }
        }
        return DBT_OK;
    case CONNECTION_REQUEST: return connection_request(manager, link, command);
    case CONNECTION_RESPONSE: connection_response(manager, link, command); return DBT_OK;
    case CONFIG_REQUEST: return config_request(manager, link, command, size);
    case CONFIG_RESPONSE: config_response(manager, link, command, size); return DBT_OK;
    case LE_CONNECT_REQUEST:return le_connection_request(manager,link,command);
    case LE_CONNECT_RESPONSE:le_connection_response(manager,link,command);return DBT_OK;
    case LE_CREDITS:{
        dbt_channel_t *c=by_remote(manager,link->handle,link->generation,dbt_read_le16(body));
        if(!c || !c->le || c->state!=DBT_CH_OPEN)return DBT_OK;
        if(dbt_credit_add_peer_credits(&c->le->engine,dbt_read_le16(body+2))!=DBT_OK)le_fault(manager,c);
        return DBT_OK;
    }
    case DISCONNECT_REQUEST: {
        uint16_t local = dbt_read_le16(body), remote = dbt_read_le16(body + 2);
        dbt_channel_t *channel = find(manager, link->handle, local);
        /* CLOSED acknowledges a retransmission without recreating state. */
        if (!channel) return enqueue(manager, link, cid, DISCONNECT_RESPONSE, id, body, 4, 0, TX_PLAIN, 0);
        if (channel->remote_cid != remote) return DBT_OK;
        dbt_result_t result = enqueue(manager, link, cid, DISCONNECT_RESPONSE, id, body, 4, 0, TX_PLAIN, 0);
        if (result == DBT_OK) finish(manager, channel, DBT_CH_REASON_NORMAL, 0);
        return result;
    }
    case DISCONNECT_RESPONSE: {
        dbt_channel_t *channel = by_request(manager, link->handle, link->generation, DISCONNECT_REQUEST, id);
        if (channel && dbt_read_le16(body) == channel->remote_cid && dbt_read_le16(body + 2) == channel->local_cid)
            finish(manager, channel, DBT_CH_REASON_NORMAL, 0);
        else ++manager->unknown_responses;
        return DBT_OK;
    }
    case 8: return enqueue(manager, link, 1, 9, id, body, size - 4u, 0, TX_PLAIN, 0);
    case 9: case 0x0b: case 0x13: ++manager->unknown_responses; return DBT_OK;
    case 0x0a: {
        uint16_t type = dbt_read_le16(body); uint8_t response[12]; uint16_t length = 4;
        zero(response, sizeof(response)); put16(response, type);
        if (type == 2) { response[4] = 0x80; length = 8; } /* Fixed-channel discovery, no ERTM/streaming. */
        else if (type == 3) {
            for (unsigned i = 0; i < 8; ++i) response[4 + i] = (uint8_t)(manager->fixed_mask[DBT_LINK_CLASSIC] >> (8u * i));
            length = 12;
        }
        else put16(response + 2, 1);
        return enqueue(manager, link, 1, 0x0b, id, response, length, 0, TX_PLAIN, 0);
    }
    case 0x12: {
        if (link->role != 0) return reject(manager, link, cid, id, 0, 0, 0);
        uint16_t minimum = dbt_read_le16(body), maximum = dbt_read_le16(body + 2);
        uint16_t latency = dbt_read_le16(body + 4), timeout = dbt_read_le16(body + 6);
        bool valid = minimum >= 6 && maximum <= 3200 && minimum <= maximum && latency <= 499 &&
                     timeout >= 10 && timeout <= 3200 && (uint32_t)timeout * 8u > 2u * (latency + 1u) * maximum;
        bool accepted = false;
        if (valid && manager->hooks.le_parameters) {
            manager->in_callback = true;
            accepted = manager->hooks.le_parameters(manager->hooks.context, link, minimum, maximum, latency, timeout);
            manager->in_callback = false;
        }
        uint8_t response[2]; put16(response, accepted ? 0 : 1);
        return enqueue(manager, link, 5, 0x13, id, response, 2, 0, TX_PLAIN, 0);
    }
    default: return reject(manager, link, cid, id, 0, 0, 0);
    }
}

uint16_t dbt_channels_poll(dbt_channels_t *manager, uint32_t now, uint16_t budget) {
    if (!manager || manager->in_callback) return 0;
    manager->now_ms = now; prune_tx(manager);
    uint16_t processed = 0; unsigned deferred = 0;
    manager->rx_deferred = false;
    while (processed < budget && manager->rx_count && manager->tx_count < DBT_SIGNAL_TX_SLOTS) {
        dbt_signal_rx_t *job = &manager->rx[manager->rx_head];
        const dbt_link_t *link = dbt_acl_find(manager->acl, job->handle);
        if (!link || link->generation != job->generation) { ++manager->stale_pdus; job->cursor = job->size; }
        else if (job->oversized) { (void)reject(manager, link, job->cid, job->bytes[1], 1, 0, 0); job->cursor = job->size; }
        else {
            uint8_t *command = job->bytes + job->cursor;
            uint16_t size = (uint16_t)(4u + dbt_read_le16(command + 2));
            dbt_result_t result = process_command(manager, link, job->cid, command, size);
            if (result == DBT_BUSY) {
                if (++deferred >= manager->rx_count) { manager->rx_deferred = true; break; }
                dbt_signal_rx_t saved;
                copy(&saved, job, sizeof(saved));
                manager->rx_head = (uint8_t)((manager->rx_head + 1u) % DBT_SIGNAL_RX_SLOTS);
                unsigned tail = (manager->rx_head + manager->rx_count - 1u) % DBT_SIGNAL_RX_SLOTS;
                copy(&manager->rx[tail], &saved, sizeof(saved));
                continue;
            }
            if (result != DBT_OK) break;
            job->cursor += (uint8_t)size;
        }
        ++processed;
        deferred = 0;
        if (job->cursor == job->size) { manager->rx_head = (uint8_t)((manager->rx_head + 1u) % DBT_SIGNAL_RX_SLOTS); --manager->rx_count; }
    }
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        dbt_channel_t *channel = &manager->channels[i];
        if (channel->state == DBT_CH_FREE) continue;
        const dbt_link_t *link = dbt_acl_find(manager->acl, channel->handle);
        if (!link || link->generation != channel->link_generation) { finish(manager, channel, DBT_CH_REASON_LINK_LOST, 0); continue; }
        if(channel->le && channel->state==DBT_CH_OPEN && !le_data_allowed(manager,channel,link))le_fault(manager,channel);
        if(channel->le && channel->le->faulted && channel->state==DBT_CH_DISCONNECTING && !channel->request_code)
            (void)queue_request(manager,channel,DISCONNECT_REQUEST,false);
        if(channel->state==DBT_CH_SECURING) {
            bool allowed=authorized(manager,link,channel->psm,channel->security);
            dbt_result_t progress=allowed?DBT_OK:security_request(manager,link,channel->psm,channel->security,false);
            bool expired=due(now,channel->deadline);
            bool denied=!allowed && progress!=DBT_BUSY && progress!=DBT_OK;
            if(expired || denied) {
                if(channel->security_incoming) {
                    uint8_t body[8];put16(body,0);put16(body+2,channel->remote_cid);put16(body+4,3);put16(body+6,0);
                    if(enqueue(manager,link,1,CONNECTION_RESPONSE,channel->incoming_connect_id,body,8,0,TX_PLAIN,0)!=DBT_OK)continue;
                }
                finish(manager,channel,expired?DBT_CH_REASON_LOCAL_TIMEOUT:DBT_CH_REASON_SECURITY,3);
            } else if(allowed && (!channel->security_incoming || channel->security_pending_sent)) {
                if(channel->security_incoming) {
                    uint8_t body[8];put16(body,channel->local_cid);put16(body+2,channel->remote_cid);put16(body+4,0);put16(body+6,0);
                    if(enqueue(manager,link,1,CONNECTION_RESPONSE,channel->incoming_connect_id,body,8,channel,TX_CONNECTION_ACCEPT,0)!=DBT_OK)continue;
                    channel->state=DBT_CH_CONFIGURING;channel->deadline=now+manager->rtx_ms;
                } else if(queue_request(manager,channel,CONNECTION_REQUEST,false)==DBT_OK)channel->state=DBT_CH_CONNECTING;
            }
            continue;
        }
        if (!channel->le && channel->security >= DBT_SECURITY_ENCRYPTED && !link->encrypted) { finish(manager, channel, DBT_CH_REASON_SECURITY, 0); continue; }
        if (channel->state == DBT_CH_CONFIGURING && channel->connection_ready && due(now, channel->config_limit)) {
            finish(manager, channel, DBT_CH_REASON_RESPONSE_TIMEOUT, 0); continue;
        }
        if (channel->state != DBT_CH_OPEN && due(now, channel->deadline)) {
            finish(manager, channel, channel->request_sent ? DBT_CH_REASON_RESPONSE_TIMEOUT : DBT_CH_REASON_LOCAL_TIMEOUT, 0);
        }
    }
    prune_tx(manager);
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT && manager->tx_count < DBT_SIGNAL_TX_SLOTS; ++i) {
        dbt_channel_t *channel = &manager->channels[i];
        if (channel->state == DBT_CH_CONFIGURING && channel->connection_ready && !channel->local_configured && !channel->request_code)
            (void)queue_request(manager, channel, CONFIG_REQUEST, channel->response_more);
    }
    return processed;
}

bool dbt_channels_tx_pending(const dbt_channels_t *manager, uint16_t handle) {
    if (!manager) return false;
    for (unsigned i = 0; i < manager->tx_count; ++i) {
        const dbt_signal_tx_t *slot = &manager->tx[(manager->tx_head + i) % DBT_SIGNAL_TX_SLOTS];
        if (slot->handle == handle && tx_valid(manager, slot)) return true;
    }
    return false;
}
dbt_result_t dbt_channels_tx_select(dbt_channels_t *manager, uint16_t handle, dbt_signal_view_t *view) {
    if (!manager || !view || manager->in_callback) return DBT_INVALID;
    if (manager->tx_locked) return manager->tx[manager->tx_head].handle == handle ?
        dbt_channels_tx_peek(manager, view) : DBT_BUSY;
    prune_tx(manager);
    for (unsigned i = 0; i < manager->tx_count; ++i) {
        unsigned index = (manager->tx_head + i) % DBT_SIGNAL_TX_SLOTS;
        if (manager->tx[index].handle != handle || !tx_valid(manager, &manager->tx[index])) continue;
        dbt_signal_tx_t selected;
        copy(&selected, &manager->tx[index], sizeof(selected));
        for (unsigned j = i; j > 0; --j)
            copy(&manager->tx[(manager->tx_head + j) % DBT_SIGNAL_TX_SLOTS],
                 &manager->tx[(manager->tx_head + j - 1u) % DBT_SIGNAL_TX_SLOTS], sizeof(selected));
        copy(&manager->tx[manager->tx_head], &selected, sizeof(selected));
        return dbt_channels_tx_peek(manager, view);
    }
    return DBT_EMPTY;
}
dbt_result_t dbt_channels_tx_peek(dbt_channels_t *manager, dbt_signal_view_t *view) {
    if (!manager || !view || manager->in_callback) return DBT_INVALID;
    prune_tx(manager);
    if (!manager->tx_count) return DBT_EMPTY;
    dbt_signal_tx_t *slot = &manager->tx[manager->tx_head];
    manager->tx_locked = true;
    view->handle = slot->handle; view->generation = slot->generation; view->cid = slot->cid;
    view->size = slot->size; view->data = slot->bytes;
    return DBT_OK;
}
dbt_result_t dbt_channels_tx_commit(dbt_channels_t *manager, uint32_t now) {
    if (!manager || manager->in_callback || !manager->tx_count || !manager->tx_locked) return DBT_INVALID;
    dbt_signal_tx_t snapshot;
    copy(&snapshot, &manager->tx[manager->tx_head], sizeof(snapshot));
    pop_tx(manager); manager->now_ms = now;
    if (!tx_valid(manager, &snapshot)) return DBT_OK;
    dbt_channel_t *channel = snapshot.serial ? find(manager, snapshot.handle, snapshot.local_cid) : 0;
    if (!channel) return DBT_OK;
    if(snapshot.effect==TX_SECURITY_PENDING && channel->state==DBT_CH_SECURING) {
        channel->security_pending_sent=true;
    } else if (snapshot.effect == TX_REQUEST && channel->request_id == snapshot.bytes[1] && channel->request_code == snapshot.bytes[0]) {
        channel->request_sent = true; channel->deadline = now + manager->rtx_ms;
        if(channel->le && channel->request_code==LE_CONNECT_REQUEST)channel->le->engine.local.credits=channel->le->initial_credits;
    } else if(snapshot.effect==TX_LE_ACCEPT && channel->le && !channel->connection_ready){
        channel->le->engine.local.credits=channel->le->initial_credits;
        channel->connection_ready=channel->local_configured=channel->peer_configured=true;
        channel->state=DBT_CH_OPEN;
        const dbt_link_t *link=dbt_acl_find(manager->acl,channel->handle);
        if(le_data_allowed(manager,channel,link)){channel->opened_once=true;notify(manager,channel,DBT_CH_EVENT_OPEN,DBT_CH_REASON_NORMAL,0);}
        else le_fault(manager,channel);
    } else if(snapshot.effect==TX_LE_GRANT && channel->le && channel->state==DBT_CH_OPEN){
        if(dbt_credit_grant_commit(&channel->le->engine)!=DBT_OK)le_fault(manager,channel);
    } else if (snapshot.effect == TX_CONNECTION_ACCEPT && !channel->connection_ready) {
        channel->connection_ready = true; channel->deadline = now + manager->rtx_ms;
        channel->config_limit = now + 300000u;
        (void)queue_request(manager, channel, CONFIG_REQUEST, false);
    } else if (snapshot.effect == TX_CONFIG_ACCEPT && snapshot.config_epoch == channel->config_epoch) {
        channel->peer_configured = true; channel->tx_mtu = snapshot.peer_mtu;
        maybe_open(manager, channel);
    }
    return DBT_OK;
}
dbt_result_t dbt_channels_tx_cancel(dbt_channels_t *manager) {
    if (!manager || manager->in_callback || !manager->tx_count || !manager->tx_locked) return DBT_INVALID;
    dbt_signal_tx_t *slot = &manager->tx[manager->tx_head];
    dbt_channel_t *channel = slot->serial ? find(manager, slot->handle, slot->local_cid) : 0;
    if (channel && channel->serial == slot->serial) finish(manager, channel, DBT_CH_REASON_LOCAL_TIMEOUT, 0);
    pop_tx(manager);
    return DBT_OK;
}
void dbt_channels_link_change(dbt_channels_t *manager, const dbt_link_change_t *change) {
    if (!manager || manager->in_callback || !change || change->kind != DBT_LINK_REMOVED) return;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        dbt_channel_t *channel = &manager->channels[i];
        if (channel->state != DBT_CH_FREE && channel->handle == change->handle && channel->link_generation == change->generation)
            finish(manager, channel, DBT_CH_REASON_LINK_LOST, 0);
    }
    for (unsigned i = 0; i < DBT_SIGNAL_TX_SLOTS; ++i)
        if (manager->tx[i].handle == change->handle && manager->tx[i].generation == change->generation) manager->tx[i].valid = false;
    prune_tx(manager);
}
dbt_result_t dbt_channels_prepare_data(dbt_channels_t *manager, uint16_t handle, uint16_t cid,
                                       const uint8_t *data, uint16_t size, dbt_l2cap_tx_t *tx) {
    if (!manager || manager->in_callback) return DBT_INVALID;
    dbt_channel_t *channel = find(manager, handle, cid);
    const dbt_link_t *link = dbt_acl_find(manager->acl, handle);
    if (!channel || !link || link->generation != channel->link_generation)return DBT_BUSY;
    if(channel->le)return DBT_UNSUPPORTED;
    bool transmitting=channel->state==DBT_CH_OPEN ||
        (channel->state==DBT_CH_CONFIGURING && channel->opened_once && channel->reconfiguring);
    if(!transmitting)return DBT_BUSY;
    if(!authorized(manager,link,channel->psm,channel->security))return DBT_BUSY;
    if (size > channel->tx_mtu) return DBT_NO_SPACE;
    return dbt_l2cap_tx_begin(tx, link, channel->remote_cid, data, size);
}
uint32_t dbt_channels_wait_ms(const dbt_channels_t *manager, uint32_t now) {
    if (!manager) return UINT32_MAX;
    if (manager->rx_count && !manager->rx_deferred && manager->tx_count < DBT_SIGNAL_TX_SLOTS) return 0;
    uint32_t wait = UINT32_MAX;
    for (unsigned i = 0; i < DBT_CHANNEL_LIMIT; ++i) {
        const dbt_channel_t *channel = &manager->channels[i];
        if (channel->state == DBT_CH_FREE || channel->state == DBT_CH_OPEN) continue;
        if(channel->state==DBT_CH_SECURING && due(now,channel->deadline) && channel->security_incoming && manager->tx_count==DBT_SIGNAL_TX_SLOTS)continue;
        if (due(now, channel->deadline)) return 0;
        uint32_t remaining = channel->deadline - now;
        if (remaining < wait) wait = remaining;
        if (channel->state == DBT_CH_CONFIGURING && channel->connection_ready) {
            if (due(now, channel->config_limit)) return 0;
            remaining = channel->config_limit - now;
            if (remaining < wait) wait = remaining;
        }
    }
    return wait;
}
