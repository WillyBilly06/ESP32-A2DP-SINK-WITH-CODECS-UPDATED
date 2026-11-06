#include "dbt_avdtp.h"

static void zero(void *object, unsigned size) {
    uint8_t *p = object;
    for (unsigned i = 0; i < size; ++i) p[i] = 0;
}
static void copy(uint8_t *to, const uint8_t *from, uint16_t size) {
    for (uint16_t i = 0; i < size; ++i) to[i] = from[i];
}
static bool due(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }
static uint32_t until(uint32_t now, uint32_t deadline) {
    return due(now, deadline) ? 0 : deadline - now;
}
static void drop_rx(dbt_avdtp_t *s) {
    if (s->rx) s->hooks.release(s->hooks.context, s->rx);
    s->rx = 0;
    s->rx_remaining = 0;
    s->rx_size = s->rx_capacity = 0;
}
static void drop_packet(dbt_avdtp_t *s) {
    if (s->packet && s->packet != s->small_packet)
        s->hooks.release(s->hooks.context, s->packet);
    s->packet = 0;
    s->packet_size = 0;
}
static void drop_tx(dbt_avdtp_t *s, dbt_avdtp_tx_t *t) {
    if (t->data && t->data != t->small) s->hooks.release(s->hooks.context, t->data);
    zero(t, sizeof(*t));
}
static void complete(dbt_avdtp_t *s,dbt_avdtp_outcome_t outcome,const dbt_avdtp_message_t *message){
    const dbt_avdtp_completion_t *local=s->local_completion;
    s->local_completion=0; /* Detach before notification; never touch it afterward. */
    void (*notify)(void *,dbt_avdtp_outcome_t,const dbt_avdtp_message_t *)=
        local?local->complete:s->hooks.complete;
    void *context=local?local->context:s->hooks.context;
    if(notify){s->in_callback=true;notify(context,outcome,message);s->in_callback=false;}
}
static void finish(dbt_avdtp_t *s, dbt_avdtp_outcome_t outcome,
                   const dbt_avdtp_message_t *message) {
    drop_tx(s,&s->tx[0]);complete(s,outcome,message);
}
static bool reply_length(const dbt_avdtp_message_t *message);

dbt_result_t dbt_avdtp_init(dbt_avdtp_t *s, dbt_avdtp_hooks_t hooks,
    uint32_t binding, uint16_t rx_mtu, uint16_t tx_mtu, uint16_t maximum_message,
    uint32_t transmit_ms, uint32_t response_ms) {
    if (!s || !hooks.allocate || !hooks.release || !hooks.command || !binding ||
        rx_mtu < 48 || tx_mtu < 48 || !maximum_message ||
        !transmit_ms || transmit_ms > INT32_MAX || !response_ms || response_ms > INT32_MAX)
        return DBT_INVALID;
    zero(s, sizeof(*s));
    s->hooks = hooks;
    s->binding = binding;
    s->rx_mtu = rx_mtu;
    s->tx_mtu = tx_mtu;
    s->maximum_message = maximum_message;
    s->transmit_ms = transmit_ms;
    s->response_ms = response_ms;
    return DBT_OK;
}

static dbt_result_t queue(dbt_avdtp_t *s, unsigned slot, uint8_t label, uint8_t command,
    dbt_avdtp_type_t type, const uint8_t *body, uint16_t size, uint32_t now) {
    dbt_avdtp_tx_t *t = &s->tx[slot];
    if (t->used) return DBT_BUSY;
    if ((!body && size) || size > s->maximum_message) return DBT_INVALID;
    uint32_t count = 1;
    if ((uint32_t)size + 2 > s->tx_mtu) {
        uint32_t left = (uint32_t)size - (s->tx_mtu - 3u);
        count += (left + s->tx_mtu - 2u) / (s->tx_mtu - 1u);
    }
    if (count > 255) return DBT_NO_SPACE;
    uint8_t *bytes = size <= sizeof(t->small) ? t->small
        : s->hooks.allocate(s->hooks.context, size);
    if (!bytes) return DBT_NO_SPACE;
    copy(bytes, body, size);
    t->data = bytes;
    t->size = size;
    t->label = label;
    t->signal = command;
    t->type = (uint8_t)type;
    t->count = (uint8_t)count;
    t->deadline = now + s->transmit_ms;
    t->used = true;
    return DBT_OK;
}

static bool overlaps(const void *a,unsigned an,const void *b,unsigned bn){
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;return an && bn && (x<=y?y-x<an:x-y<bn);
}
static bool owns_storage(const dbt_avdtp_t *s,const void *p,unsigned n){
    return overlaps(s,sizeof(*s),p,n) || overlaps(s->rx,s->rx_capacity,p,n) ||
        overlaps(s->packet,s->packet_size,p,n) || overlaps(s->tx[0].data,s->tx[0].size,p,n) ||
        overlaps(s->tx[1].data,s->tx[1].size,p,n);
}
static dbt_result_t command(dbt_avdtp_t *s, uint8_t command,
    const uint8_t *body, uint16_t size, uint32_t now, uint8_t *label,
    const dbt_avdtp_completion_t *completion) {
    if (!s || !s->binding || s->failed || s->in_callback || !label ||
        command < 1 || command > DBT_AVDTP_DELAY_REPORT) return DBT_INVALID;
    if (dbt_avdtp_poll(s, now) != DBT_OK) return DBT_INVALID;
    uint8_t next = s->next_label;
    dbt_result_t result = queue(s, 0, next, command, DBT_AVDTP_COMMAND, body, size, now);
    if (result == DBT_OK) {
        s->local_completion=completion;
        *label = next;
        s->next_label = (uint8_t)((next + 1u) & 15u);
    }
    return result;
}

dbt_result_t dbt_avdtp_command(dbt_avdtp_t *s,uint8_t signal,const uint8_t *body,uint16_t size,
    uint32_t now,uint8_t *label){return command(s,signal,body,size,now,label,0);}
dbt_result_t dbt_avdtp_command_with_completion(dbt_avdtp_t *s,uint8_t signal,const uint8_t *body,uint16_t size,
    uint32_t now,uint8_t *label,const dbt_avdtp_completion_t *completion){
    if(!s || !completion || !completion->complete || !label ||
       owns_storage(s,completion,sizeof(*completion)) || overlaps(completion,sizeof(*completion),label,sizeof(*label)))return DBT_INVALID;
    return command(s,signal,body,size,now,label,completion);
}
dbt_result_t dbt_avdtp_command_prepare(dbt_avdtp_t *s,uint8_t signal,const uint8_t *body,uint16_t size,
    uint32_t now,uint8_t *label,const dbt_avdtp_completion_t *completion){
    dbt_result_t result=dbt_avdtp_command_with_completion(s,signal,body,size,now,label,completion);
    if(result==DBT_OK)s->tx[0].blocked=true;
    return result;
}
dbt_result_t dbt_avdtp_command_ready(dbt_avdtp_t *s,uint32_t binding,uint8_t label,
    const dbt_avdtp_completion_t *completion,uint32_t now){
    if(!s || !completion || s->in_callback || s->binding!=binding)return DBT_INVALID;
    if(dbt_avdtp_poll(s,now)!=DBT_OK)return DBT_INVALID;
    dbt_avdtp_tx_t *t=&s->tx[0];
    if(!t->used || !t->blocked || t->waiting || t->index || t->label!=label || s->local_completion!=completion)return DBT_EMPTY;
    t->blocked=false;t->deadline=now+s->transmit_ms;return DBT_OK;
}
dbt_result_t dbt_avdtp_command_cancel(dbt_avdtp_t *s,uint32_t binding,uint8_t label,
    const dbt_avdtp_completion_t *completion,uint32_t now){
    if(!s || !binding || !completion || s->in_callback)return DBT_INVALID;
    dbt_avdtp_tx_t *t=&s->tx[0];
    if(s->binding!=binding || !t->used || t->label!=label || s->local_completion!=completion)return DBT_EMPTY;
    if(s->failed || t->waiting || t->index || due(now,t->deadline) ||
       (s->pinned && s->selected==0))return DBT_PENDING;
    if(s->selected_valid && s->selected==0){drop_packet(s);s->selected_valid=false;}
    finish(s,DBT_AVDTP_CANCELLED,0);return DBT_OK;
}

dbt_result_t dbt_avdtp_respond(dbt_avdtp_t *s, uint8_t label, uint8_t command,
    dbt_avdtp_type_t type, const uint8_t *body, uint16_t size, uint32_t now) {
    if (!s || !s->binding || s->failed || !s->remote_pending ||
        label != s->remote_label || command != s->remote_signal ||
        type < DBT_AVDTP_GENERAL_REJECT || type > DBT_AVDTP_REJECT ||
        (type == DBT_AVDTP_GENERAL_REJECT && size)) return DBT_INVALID;
    dbt_avdtp_message_t message = {body, size, label, command, type};
    if (!reply_length(&message)) return DBT_INVALID;
    if (!s->in_callback && dbt_avdtp_poll(s, now) != DBT_OK) return DBT_INVALID;
    return queue(s, 1, label, command, type, body, size, now);
}

static bool reply_length(const dbt_avdtp_message_t *m) {
    if (m->type == DBT_AVDTP_GENERAL_REJECT) return m->size == 0;
    if (m->type == DBT_AVDTP_REJECT) {
        if (m->signal == DBT_AVDTP_ABORT) return false; /* no Abort Reject exists */
        bool detail = m->signal == DBT_AVDTP_SET_CONFIGURATION ||
            m->signal == DBT_AVDTP_RECONFIGURE || m->signal == DBT_AVDTP_START ||
            m->signal == DBT_AVDTP_SUSPEND;
        return m->size == (detail ? 2u : 1u);
    }
    switch (m->signal) {
    case DBT_AVDTP_DISCOVER: return m->size >= 2 && m->size <= 124 && !(m->size & 1u);
    case DBT_AVDTP_GET_CAPABILITIES: case DBT_AVDTP_GET_ALL_CAPABILITIES:
    case DBT_AVDTP_GET_CONFIGURATION: case DBT_AVDTP_SECURITY_CONTROL: return true;
    default: return m->size == 0;
    }
}
dbt_result_t dbt_avdtp_ignore_abort(dbt_avdtp_t *s,uint8_t label) {
    if(!s || !s->binding || s->failed || !s->remote_pending || s->remote_label!=label ||
       s->remote_signal!=DBT_AVDTP_ABORT || s->tx[1].used)return DBT_INVALID;
    s->remote_pending=false;
    return DBT_OK;
}
static dbt_result_t dispatch(dbt_avdtp_t *s, const dbt_avdtp_message_t *m, uint32_t now) {
    if (m->type != DBT_AVDTP_COMMAND) {
        const dbt_avdtp_tx_t *t = &s->tx[0];
        if (!t->used || !t->waiting || t->label != m->label || t->signal != m->signal) {
            ++s->discarded;
            return DBT_EMPTY;
        }
        if (!reply_length(m)) {
            ++s->discarded;
            return DBT_INVALID;
        }
        finish(s, DBT_AVDTP_RESPONSE, m);
        return DBT_OK;
    }
    if (s->remote_pending) {
        if(m->signal!=DBT_AVDTP_ABORT || s->tx[1].used || !s->hooks.interrupt)return DBT_BUSY;
        s->in_callback=true;
        bool accepted=s->hooks.interrupt(s->hooks.context,m);
        s->in_callback=false;
        if(!accepted)return DBT_BUSY;
        s->remote_pending=false;
    }
    s->remote_pending = true;
    s->remote_label = m->label;
    s->remote_signal = m->signal;
    s->remote_deadline = now + s->response_ms;
    if (m->signal > DBT_AVDTP_DELAY_REPORT)
        return dbt_avdtp_respond(s, m->label, m->signal, DBT_AVDTP_GENERAL_REJECT, 0, 0, now);
    s->in_callback = true;
    s->hooks.command(s->hooks.context, m);
    s->in_callback = false;
    return DBT_OK;
}

dbt_result_t dbt_avdtp_receive(dbt_avdtp_t *s, uint32_t binding,
    const uint8_t *p, uint16_t size, uint32_t now) {
    if (!s || !s->binding || s->failed || s->in_callback || !p || !size)
        return DBT_INVALID;
    if (binding != s->binding) return DBT_EMPTY;
    if (dbt_avdtp_poll(s, now) != DBT_OK) return DBT_INVALID;
    uint8_t kind = (p[0] >> 2) & 3u;
    uint8_t label = p[0] >> 4;
    dbt_avdtp_type_t type = (dbt_avdtp_type_t)(p[0] & 3u);
    if (size > s->rx_mtu) goto malformed;
    if (kind <= 1) {
        uint16_t header = kind ? 3 : 2;
        if (size < header) goto malformed;
        uint8_t command = p[header - 1] & 63u; /* RFA is ignored on reception. */
        if (!command) goto malformed;
        if (s->rx) {
            /* The sole permitted interruption is Abort for the same SEP. Its
             * complete body must be a single SEID. No partial command executes. */
            bool abort_same = !kind && type == DBT_AVDTP_COMMAND &&
                command == DBT_AVDTP_ABORT && size == 3 && s->rx_size &&
                (p[2] >> 2) == (s->rx[0] >> 2);
            if (!abort_same) goto malformed;
            drop_rx(s);
        }
        if (!kind) {
            if (size - 2u > s->maximum_message) goto malformed;
            dbt_avdtp_message_t message = {p + 2, (uint16_t)(size - 2), label, command, type};
            return dispatch(s, &message, now);
        }
        uint8_t count = p[1];
        if (count < 2) goto malformed;
        uint32_t lower = size - 3u + (uint32_t)(count - 2u) * (size - 1u);
        uint32_t upper = lower + size - 1u;
        if (lower > s->maximum_message) goto malformed;
        uint16_t capacity = (uint16_t)(upper < s->maximum_message ? upper : s->maximum_message);
        if (!capacity) goto malformed;
        /* A response already known to be unsolicited cannot consume heap. */
        if (type != DBT_AVDTP_COMMAND && (!s->tx[0].used || !s->tx[0].waiting ||
            s->tx[0].label != label || s->tx[0].signal != command)) {
            ++s->discarded;
            return DBT_EMPTY;
        }
        if (type == DBT_AVDTP_COMMAND && s->remote_pending) return DBT_BUSY;
        s->rx = s->hooks.allocate(s->hooks.context, capacity);
        if (!s->rx) return DBT_NO_SPACE;
        s->rx_capacity = capacity;
        s->rx_size = (uint16_t)(size - 3);
        copy(s->rx, p + 3, s->rx_size);
        s->rx_label = label;
        s->rx_signal = command;
        s->rx_type = (uint8_t)type;
        s->rx_remaining = (uint8_t)(count - 1);
        s->rx_packet_size = size;
        s->rx_deadline = now + s->response_ms;
        return DBT_OK;
    }
    if (!s->rx || label != s->rx_label || type != s->rx_type ||
        (kind == 2 && (s->rx_remaining <= 1 || size != s->rx_packet_size)) ||
        (kind == 3 && (s->rx_remaining != 1 || size > s->rx_packet_size)) ||
        size - 1u > (uint16_t)(s->rx_capacity - s->rx_size)) goto malformed;
    copy(s->rx + s->rx_size, p + 1, (uint16_t)(size - 1));
    s->rx_size = (uint16_t)(s->rx_size + size - 1);
    --s->rx_remaining;
    if (kind == 3) {
        dbt_avdtp_message_t message = {s->rx, s->rx_size, label, s->rx_signal, type};
        dbt_result_t result = dispatch(s, &message, now);
        drop_rx(s);
        return result;
    }
    return DBT_OK;
malformed:
    drop_rx(s);
    ++s->discarded;
    return DBT_INVALID;
}

dbt_result_t dbt_avdtp_tx_peek(dbt_avdtp_t *s, const uint8_t **packet, uint16_t *size) {
    if (!s || !s->binding || !packet || !size || s->failed || s->in_callback)
        return DBT_INVALID;
    if (s->pinned) return DBT_BUSY;
    if (!s->selected_valid) {
        if (s->tx[1].used) s->selected = 1;
        else if (s->tx[0].used && !s->tx[0].waiting && !s->tx[0].blocked) s->selected = 0;
        else return DBT_EMPTY;
        s->selected_valid = true;
    }
    dbt_avdtp_tx_t *t = &s->tx[s->selected];
    uint8_t kind = t->count == 1 ? 0 : t->index == 0 ? 1 :
        t->index == t->count - 1u ? 3 : 2;
    uint16_t header = kind == 1 ? 3 : kind == 0 ? 2 : 1;
    uint16_t chunk = (uint16_t)(t->size - t->offset);
    if (chunk > s->tx_mtu - header) chunk = (uint16_t)(s->tx_mtu - header);
    if (!s->packet) {
        uint16_t bytes = (uint16_t)(header + chunk);
        s->packet = bytes <= sizeof(s->small_packet) ? s->small_packet
            : s->hooks.allocate(s->hooks.context, bytes);
        if (!s->packet) return DBT_NO_SPACE;
        s->packet_size = bytes;
        s->packet[0] = (uint8_t)((t->label << 4) | (kind << 2) | t->type);
        if (kind == 1) { s->packet[1] = t->count; s->packet[2] = t->signal; }
        else if (!kind) s->packet[1] = t->signal;
        copy(s->packet + header, t->data + t->offset, chunk);
    }
    *packet = s->packet;
    *size = s->packet_size;
    s->pinned = true;
    return DBT_OK;
}

dbt_result_t dbt_avdtp_tx_commit(dbt_avdtp_t *s, uint32_t now) {
    if (!s || !s->pinned || s->in_callback) return DBT_INVALID;
    if (!s->failed) (void)dbt_avdtp_poll(s, now);
    dbt_avdtp_tx_t *t = &s->tx[s->selected];
    uint8_t kind = (s->packet[0] >> 2) & 3u;
    uint16_t header = kind == 1 ? 3 : kind == 0 ? 2 : 1;
    t->offset = (uint16_t)(t->offset + s->packet_size - header);
    ++t->index;
    s->pinned = false;
    drop_packet(s);
    if (s->failed) return DBT_INVALID; /* caller returned the last borrowed bytes */
    if (t->index == t->count) {
        s->selected_valid = false;
        if (!s->selected) {
            /* Request payload is no longer needed while waiting for its reply. */
            if (t->data != t->small) s->hooks.release(s->hooks.context, t->data);
            t->data = 0;
            t->waiting = true;
            t->deadline = now + s->response_ms;
        } else {
            uint8_t label = t->label, command = t->signal, type = t->type;
            drop_tx(s, t);
            s->remote_pending = false;
            if (s->hooks.responded) {
                s->in_callback = true;
                s->hooks.responded(s->hooks.context, label, command, (dbt_avdtp_type_t)type);
                s->in_callback = false;
            }
        }
    }
    return DBT_OK;
}

dbt_result_t dbt_avdtp_tx_cancel(dbt_avdtp_t *s) {
    if (!s || !s->pinned || s->in_callback) return DBT_INVALID;
    s->pinned = false;
    return DBT_OK;
}

dbt_result_t dbt_avdtp_poll(dbt_avdtp_t *s, uint32_t now) {
    if (!s || !s->binding || s->in_callback || s->failed) return DBT_INVALID;
    s->now_ms = now;
    /* An incomplete received message is a local integrity discard, not a
     * completed transaction or a reason to terminate unrelated media. A local
     * command still keeps its original response deadline below. */
    if (s->rx && due(now, s->rx_deadline)) {
        drop_rx(s);
        ++s->discarded;
    }
    if ((s->remote_pending && !s->tx[1].used && due(now, s->remote_deadline)) ||
        (s->tx[0].used && due(now, s->tx[0].deadline)) ||
        (s->tx[1].used && due(now, s->tx[1].deadline))) {
        s->failed = true;
        drop_rx(s);
        /* Preserve all TX storage until the runtime releases its borrow. The
         * notification is emitted once; clear later silently releases storage. */
        if(s->tx[0].used)complete(s,DBT_AVDTP_TIMEOUT,0);
        return DBT_INVALID;
    }
    return DBT_OK;
}

uint32_t dbt_avdtp_wait_ms(const dbt_avdtp_t *s, uint32_t now) {
    if (!s || !s->binding || s->failed) return UINT32_MAX;
    uint32_t wait = UINT32_MAX, next;
    if (s->rx) wait = until(now, s->rx_deadline);
    if (s->remote_pending && !s->tx[1].used) {
        next = until(now, s->remote_deadline);
        if (next < wait) wait = next;
    }
    for (unsigned i = 0; i < 2; ++i) if (s->tx[i].used) {
        next = until(now, s->tx[i].deadline);
        if (next < wait) wait = next;
    }
    /* Pending transmission wakeups depend on actual runtime credit/readiness;
     * the bearer combines this deadline with its transport wait, avoiding spin. */
    return wait;
}

dbt_result_t dbt_avdtp_clear(dbt_avdtp_t *s) {
    if (!s || !s->binding || s->in_callback) return DBT_INVALID;
    if (s->pinned) return DBT_BUSY;
    bool notify = s->tx[0].used && !s->failed;
    s->failed = true;
    drop_packet(s);
    drop_rx(s);
    drop_tx(s, &s->tx[1]);
    if (notify) finish(s, DBT_AVDTP_CANCELLED, 0);
    else {drop_tx(s,&s->tx[0]);s->local_completion=0;}
    s->remote_pending = s->selected_valid = false;
    s->binding = 0;
    return DBT_OK;
}

static uint32_t be32(const uint8_t *p) {
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
dbt_result_t dbt_avdtp_media_decode(const uint8_t *p, uint16_t size, dbt_avdtp_media_t *media) {
    if (!p || !media || size < 12 || (p[0] >> 6) != 2) return DBT_INVALID;
    uint32_t header = 12u + 4u * (p[0] & 15u);
    if (header > size) return DBT_INVALID;
    if (p[0] & 16u) {
        if (header + 4u > size) return DBT_INVALID;
        uint16_t words = (uint16_t)((uint16_t)p[header + 2] << 8 | p[header + 3]);
        header += 4u + 4u * words;
        if (header > size) return DBT_INVALID;
    }
    uint16_t end = size;
    if (p[0] & 32u) {
        uint8_t padding = p[size - 1u];
        if (!padding || padding > size - header) return DBT_INVALID;
        end = (uint16_t)(end - padding);
    }
    dbt_avdtp_media_t result = {p + header, be32(p + 4), be32(p + 8),
        (uint16_t)(end - header), (uint16_t)((uint16_t)p[2] << 8 | p[3]),
        (uint8_t)(p[1] & 127u), (p[1] & 128u) != 0};
    *media = result;
    return DBT_OK;
}
