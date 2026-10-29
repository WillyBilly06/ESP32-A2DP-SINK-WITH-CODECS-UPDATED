#include "dbt_classic_security.h"

static void copy(void *out, const void *in, unsigned size) {
    uint8_t *destination = out;
    const uint8_t *source = in;
    for (unsigned i = 0; i < size; ++i)
        destination[i] = source[i];
}
static void wipe(void *data, unsigned size) {
    volatile uint8_t *bytes = data;
    for (unsigned i = 0; i < size; ++i)
        bytes[i] = 0;
}
static bool equal(const uint8_t *a, const uint8_t *b) {
    uint8_t difference = 0;
    for (unsigned i = 0; i < 6; ++i)
        difference |= a[i] ^ b[i];
    return difference == 0;
}
static bool due(uint32_t now, uint32_t deadline) {
    return (uint32_t)(now - deadline) < 0x80000000u;
}
static uint32_t read32(const uint8_t *p) {
    return (uint32_t)dbt_read_le16(p) | ((uint32_t)dbt_read_le16(p + 2) << 16);
}
static void put16(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

bool dbt_classic_key_valid(const dbt_classic_key_t *key) {
    if (!key || key->pin_length > 16)
        return false;
    return key->type == 0 || key->type == 4 || key->type == 5 || key->type == 7 || key->type == 8;
}
bool dbt_classic_key_authenticated(const dbt_classic_key_t *key) {
    return dbt_classic_key_valid(key) && (key->type == 5 || key->type == 8);
}
bool dbt_classic_key_secure_connections(const dbt_classic_key_t *key) {
    return dbt_classic_key_valid(key) && (key->type == 7 || key->type == 8);
}
static dbt_classic_peer_t *peer_by_address(dbt_classic_security_t *security,
                                           const uint8_t address[6]) {
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (security->peers[i].used && equal(security->peers[i].address, address))
            return &security->peers[i];
    return 0;
}
static dbt_classic_peer_t *peer_by_serial(dbt_classic_security_t *security, uint32_t serial) {
    if (!serial)
        return 0;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (security->peers[i].used && security->peers[i].serial == serial)
            return &security->peers[i];
    return 0;
}
static const dbt_link_t *live_link(const dbt_classic_security_t *security,
                                   const dbt_classic_peer_t *peer) {
    if (!peer->connected)
        return 0;
    const dbt_link_t *link = dbt_acl_find(security->acl, peer->handle);
    return link && link->kind == DBT_LINK_CLASSIC && link->generation == peer->generation ? link
                                                                                          : 0;
}
static bool referenced(const dbt_classic_security_t *security, const dbt_classic_peer_t *peer) {
    if (security->command_active && security->active.peer_serial == peer->serial)
        return true;
    for (unsigned i = 0; i < security->count; ++i)
        if (security->commands[(security->head + i) % DBT_CLASSIC_SECURITY_COMMANDS].peer_serial ==
            peer->serial)
            return true;
    return false;
}
static dbt_classic_peer_t *new_peer(dbt_classic_security_t *security, const uint8_t address[6]) {
    dbt_classic_peer_t *found = peer_by_address(security, address);
    if (found)
        return found;
    if (security->serial == UINT32_MAX)
        return 0;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_classic_peer_t *peer = &security->peers[i];
        if (peer->used)
            continue;
        wipe(peer, sizeof(*peer));
        peer->used = true;
        peer->serial = ++security->serial;
        copy(peer->address, address, 6);
        peer->remote_io = peer->remote_auth = 0xff;
        peer->deadline = security->now_ms + security->config.procedure_timeout_ms;
        for (unsigned j = 0; j < DBT_MAX_LINKS; ++j) {
            const dbt_link_t *link = &security->acl->links[j];
            if (link->used && link->kind == DBT_LINK_CLASSIC && equal(link->peer, address)) {
                peer->connected = true;
                peer->handle = link->handle;
                peer->generation = link->generation;
                break;
            }
        }
        return peer;
    }
    return 0;
}
static void snapshot(const dbt_classic_peer_t *peer, dbt_classic_security_info_t *info) {
    wipe(info, sizeof(*info));
    copy(info->address, peer->address, 6);
    info->handle = peer->handle;
    info->generation = peer->generation;
    info->connected = peer->connected;
    info->key_type = peer->key.type;
    info->key_size = peer->key_size;
    info->authenticated = peer->key_known && dbt_classic_key_authenticated(&peer->key);
    info->secure_connections = peer->key_known && dbt_classic_key_secure_connections(&peer->key);
    info->bonded = peer->bonded;
    info->encrypted = peer->encryption_mode != 0;
}
static void notify(dbt_classic_security_t *security, dbt_classic_peer_t *peer,
                   dbt_classic_security_event_t event, dbt_classic_security_reason_t reason,
                   uint8_t status) {
    if (!security->hooks.event)
        return;
    dbt_classic_security_info_t info;
    snapshot(peer, &info);
    security->in_callback = true;
    security->hooks.event(security->hooks.context, event, &info, reason, status);
    security->in_callback = false;
}
static void fail_peer(dbt_classic_security_t *security, dbt_classic_peer_t *peer,
                      dbt_classic_security_reason_t reason, uint8_t status) {
    if (!peer || peer->failed)
        return;
    peer->failed = true;
    peer->ready = false;
    peer->key_size = 0;
    notify(security, peer, DBT_CLASSIC_SECURITY_FAILED, reason, status);
}
static uint32_t token(dbt_classic_security_t *security) {
    return security->token == UINT32_MAX ? 0 : ++security->token;
}
static bool pairing_allowed(dbt_classic_security_t *security, dbt_classic_peer_t *peer) {
    if (!security->pairable || !security->hooks.pairing_allowed || peer->deleting || peer->failed)
        return false;
    security->in_callback = true;
    bool allowed = security->hooks.pairing_allowed(security->hooks.context, peer->address);
    security->in_callback = false;
    return allowed;
}
static bool usable_key(const dbt_classic_security_t *security, const dbt_classic_peer_t *peer,
                       const dbt_classic_key_t *key) {
    if (!dbt_classic_key_valid(key) || (key->type == 0 && !security->config.allow_legacy))
        return false;
    if (security->config.require_secure_connections && !dbt_classic_key_secure_connections(key))
        return false;
    return !((security->config.authentication_requirements & 1u) ||
             peer->wanted == DBT_SECURITY_AUTHENTICATED) ||
           dbt_classic_key_authenticated(key);
}
static dbt_result_t load(dbt_classic_security_t *security, dbt_classic_peer_t *peer) {
    if (peer->key_known)
        return DBT_OK;
    if (!security->hooks.load)
        return DBT_EMPTY;
    dbt_classic_key_t key;
    wipe(&key, sizeof(key));
    security->in_callback = true;
    dbt_result_t result = security->hooks.load(security->hooks.context, peer->address, &key);
    security->in_callback = false;
    if (result == DBT_OK) {
        if (!dbt_classic_key_valid(&key))
            result = DBT_INVALID;
        else {
            copy(&peer->key, &key, sizeof(key));
            peer->key_known = true;
            peer->bonded = true;
            peer->key_epoch = 1;
        }
    }
    wipe(&key, sizeof(key));
    return result;
}
static dbt_result_t enqueue(dbt_classic_security_t *security, dbt_classic_peer_t *peer,
                            const uint8_t address[6], uint16_t opcode, const uint8_t *data,
                            uint8_t size, dbt_command_response_t response) {
    if (security->count == DBT_CLASSIC_SECURITY_COMMANDS)
        return DBT_NO_SPACE;
    if (size > 70)
        return DBT_INVALID;
    dbt_classic_security_command_t *command =
        &security->commands[(security->head + security->count) % DBT_CLASSIC_SECURITY_COMMANDS];
    wipe(command, sizeof(*command));
    uint8_t *out = command->parameters;
    if (size > DBT_CLASSIC_SECURITY_INLINE) {
        if (!security->hooks.allocate || !security->hooks.release)
            return DBT_UNSUPPORTED;
        command->extended = security->hooks.allocate(security->hooks.context, size);
        if (!command->extended)
            return DBT_NO_SPACE;
        out = command->extended;
    }
    command->opcode = opcode;
    command->size = size;
    command->response = response;
    copy(command->address, address, 6);
    if (peer) {
        command->peer_serial = peer->serial;
        command->handle = peer->handle;
        command->generation = peer->generation;
        command->key_epoch = peer->key_epoch;
        command->cipher_epoch = peer->cipher_epoch;
    }
    copy(out, data, size);
    ++security->count;
    return DBT_OK;
}
static void release_command(dbt_classic_security_t *security,
                            dbt_classic_security_command_t *command) {
    if (command->extended) {
        wipe(command->extended, command->size);
        security->hooks.release(security->hooks.context, command->extended);
    }
    wipe(command, sizeof(*command));
}
static dbt_result_t address_reply(dbt_classic_security_t *security, dbt_classic_peer_t *peer,
                                  const uint8_t address[6], uint16_t opcode) {
    return enqueue(security, peer, address, opcode, address, 6, DBT_COMMAND_COMPLETE);
}
static bool authorized_peer(const dbt_classic_security_t *security, const dbt_classic_peer_t *peer,
                            dbt_security_t level) {
    const dbt_link_t *link = live_link(security, peer);
    if (security->host->state != DBT_HOST_IDENTIFIED || !link || peer->failed || peer->deleting ||
        !peer->key_known || !link->encrypted || !peer->encryption_mode || !peer->key_epoch ||
        peer->encryption_key_epoch != peer->key_epoch ||
        peer->verified_cipher_epoch != peer->cipher_epoch ||
        peer->key_size < security->config.minimum_key_size ||
        (security->config.require_durable_bond && !peer->bonded))
        return false;
    if (security->config.require_secure_connections &&
        (peer->encryption_mode != 2 || !dbt_classic_key_secure_connections(&peer->key)))
        return false;
    return usable_key(security, peer, &peer->key) &&
           (level != DBT_SECURITY_AUTHENTICATED || dbt_classic_key_authenticated(&peer->key));
}
static void maybe_ready(dbt_classic_security_t *security, dbt_classic_peer_t *peer) {
    if (!peer->ready && peer->wanted != DBT_SECURITY_NONE &&
        authorized_peer(security, peer, peer->wanted)) {
        peer->ready = true;
        notify(security, peer, DBT_CLASSIC_SECURITY_READY, DBT_CLASSIC_REASON_NONE, 0);
    }
}
static void command_done(void *context, uint16_t opcode, dbt_command_outcome_t outcome,
                         uint8_t status, const uint8_t *returns, uint8_t size) {
    dbt_classic_security_t *security = context;
    uint32_t serial = security->active.peer_serial, generation = security->active.generation;
    uint32_t key_epoch = security->active.key_epoch, cipher_epoch = security->active.cipher_epoch;
    uint16_t handle = security->active.handle;
    uint8_t address[6];
    copy(address, security->active.address, 6);
    security->command_active = false;
    release_command(security, &security->active);
    dbt_classic_peer_t *peer = peer_by_serial(security, serial);
    if (peer && generation && (peer->generation != generation || !peer->connected)) {
        ++security->stale_completions;
        return;
    }
    if (outcome != DBT_COMMAND_SUCCESS) {
        ++security->rejected_commands;
        /* An incoming peer procedure can race a local request. Disallowed is
         * not success: keep waiting for actual auth/encryption events, bounded
         * by the existing procedure deadline, without resubmitting in a loop. */
        if (peer && outcome == DBT_COMMAND_REJECTED && status == 0x0c &&
            (opcode == 0x0411 || opcode == 0x0413 || opcode == 0x0c53))
            return;
        if (peer)
            fail_peer(security, peer,
                      outcome == DBT_COMMAND_TIMEOUT ? DBT_CLASSIC_REASON_TIMEOUT
                                                     : DBT_CLASSIC_REASON_CONTROLLER,
                      status);
        return;
    }
    bool valid;
    if (opcode == 0x0411 || opcode == 0x0413 || opcode == 0x0c53)
        valid = size == 1;
    else if (opcode == 0x1408)
        valid = size == 4 && dbt_read_le16(returns + 1) == handle;
    else
        valid = size == 7 && equal(returns + 1, address);
    if (!valid) {
        if (peer)
            fail_peer(security, peer, DBT_CLASSIC_REASON_CONTROLLER, 0);
        dbt_host_abort(security->host, DBT_FAULT_BAD_RESPONSE);
        return;
    }
    if (!peer)
        return;
    if (opcode == 0x1408) {
        const dbt_link_t *link = live_link(security, peer);
        if (!link || !link->encrypted || !peer->encryption_mode || peer->key_epoch != key_epoch ||
            peer->cipher_epoch != cipher_epoch || peer->encryption_key_epoch != key_epoch)
            return;
        uint8_t bytes = returns[3];
        if (bytes < security->config.minimum_key_size || bytes > 16) {
            fail_peer(security, peer, DBT_CLASSIC_REASON_KEY_SIZE, 0);
            return;
        }
        peer->key_size = bytes;
        peer->verified_cipher_epoch = cipher_epoch;
        maybe_ready(security, peer);
    }
}

dbt_result_t dbt_classic_security_init(dbt_classic_security_t *security, dbt_host_t *host,
                                       dbt_acl_t *acl, dbt_classic_security_config_t config,
                                       dbt_classic_security_hooks_t hooks) {
    if (!security || !host || !acl || config.io_capability > 3 ||
        config.authentication_requirements > 5 || config.minimum_key_size < 7 ||
        config.minimum_key_size > 16 || config.oob_present > 3 ||
        config.procedure_timeout_ms < 1000 || config.procedure_timeout_ms > 120000 ||
        config.store_timeout_ms < 100 || config.store_timeout_ms > 120000 ||
        (config.require_durable_bond && config.authentication_requirements < 2) ||
        (config.authentication_requirements >= 2 && (!hooks.store || !hooks.load)) ||
        (config.io_capability != 3 && !hooks.input && !hooks.display_passkey) ||
        ((config.oob_present || config.io_capability == 2) && !hooks.input) ||
        (config.oob_present && (!hooks.allocate || !hooks.release)) ||
        ((config.require_secure_connections || config.oob_present > 1) &&
         !config.controller_secure_connections_enabled) ||
        ((config.authentication_requirements & 1u) && config.io_capability == 3 &&
         !config.oob_present))
        return DBT_INVALID;
    wipe(security, sizeof(*security));
    security->host = host;
    security->acl = acl;
    copy(&security->config, &config, sizeof(config));
    copy(&security->hooks, &hooks, sizeof(hooks));
    return DBT_OK;
}
void dbt_classic_security_pairable(dbt_classic_security_t *security, bool enabled) {
    if (security && !security->in_callback)
        security->pairable = enabled;
}
dbt_result_t dbt_classic_security_link(dbt_classic_security_t *security,
                                       const dbt_link_change_t *change) {
    if (!security || security->in_callback || !change)
        return DBT_INVALID;
    if (change->kind == DBT_LINK_ADDED) {
        const dbt_link_t *link = dbt_acl_find(security->acl, change->handle);
        if (!link || link->generation != change->generation)
            return DBT_INVALID;
        if (link->kind != DBT_LINK_CLASSIC)
            return DBT_UNSUPPORTED;
        dbt_classic_peer_t *old = peer_by_address(security, link->peer);
        if (old && !old->connected && old->generation) {
            if (old->store_token || referenced(security, old))
                return DBT_BUSY;
            wipe(old, sizeof(*old));
        }
        dbt_classic_peer_t *peer = new_peer(security, link->peer);
        if (!peer)
            return DBT_NO_SPACE;
        if (peer->connected && peer->generation != link->generation)
            return DBT_BUSY;
        peer->connected = true;
        peer->handle = link->handle;
        peer->generation = link->generation;
        peer->deadline = security->now_ms + security->config.procedure_timeout_ms;
    } else if (change->kind == DBT_LINK_REMOVED) {
        for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
            dbt_classic_peer_t *peer = &security->peers[i];
            if (!peer->used || !peer->connected || peer->handle != change->handle ||
                peer->generation != change->generation)
                continue;
            peer->connected = false;
            peer->ready = false;
            peer->input_token = 0;
            peer->key_size = peer->encryption_mode = 0;
            notify(security, peer, DBT_CLASSIC_SECURITY_LINK_LOST, DBT_CLASSIC_REASON_NONE, 0);
            if (!peer->store_token && !referenced(security, peer))
                wipe(peer, sizeof(*peer));
        }
    }
    return DBT_OK;
}
static dbt_result_t interaction(dbt_classic_security_t *security, dbt_classic_peer_t *peer,
                                dbt_classic_input_t input, uint32_t value) {
    if (peer->input_token)
        return peer->input == input ? DBT_OK : DBT_BUSY;
    peer->input_token = token(security);
    if (!peer->input_token)
        return DBT_NO_SPACE;
    peer->input = input;
    peer->deadline = security->now_ms + security->config.procedure_timeout_ms;
    if (security->hooks.input) {
        security->in_callback = true;
        security->hooks.input(security->hooks.context, peer->address, input, peer->input_token,
                              value);
        security->in_callback = false;
        return DBT_OK;
    }
    return dbt_classic_security_reply(security, peer->input_token, false, 0, 0);
}
static uint16_t negative_opcode(dbt_classic_input_t input) {
    switch (input) {
    case DBT_CLASSIC_INPUT_PIN:
        return 0x040e;
    case DBT_CLASSIC_INPUT_CONFIRM:
        return 0x042d;
    case DBT_CLASSIC_INPUT_PASSKEY:
        return 0x042f;
    case DBT_CLASSIC_INPUT_OOB:
        return 0x0433;
    default:
        return 0;
    }
}
dbt_result_t dbt_classic_security_reply(dbt_classic_security_t *security, uint32_t input_token,
                                        bool accepted, const uint8_t *data, uint8_t size) {
    if (!security || security->in_callback || !input_token || (size && !data))
        return DBT_INVALID;
    dbt_classic_peer_t *peer = 0;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (security->peers[i].used && security->peers[i].input_token == input_token)
            peer = &security->peers[i];
    if (!peer || peer->failed)
        return DBT_INVALID;
    if (accepted && due(security->now_ms, peer->deadline))
        return DBT_INVALID;
    uint8_t parameters[70];
    wipe(parameters, sizeof(parameters));
    copy(parameters, peer->address, 6);
    uint8_t length = 6;
    uint16_t opcode = negative_opcode(peer->input);
    if (accepted) {
        switch (peer->input) {
        case DBT_CLASSIC_INPUT_PIN:
            if (!size || size > 16)
                return DBT_INVALID;
            opcode = 0x040d;
            parameters[6] = size;
            copy(parameters + 7, data, size);
            length = 23;
            break;
        case DBT_CLASSIC_INPUT_CONFIRM:
            if (size)
                return DBT_INVALID;
            opcode = 0x042c;
            break;
        case DBT_CLASSIC_INPUT_PASSKEY:
            if (size != 4 || read32(data) > 999999)
                return DBT_INVALID;
            opcode = 0x042e;
            copy(parameters + 6, data, 4);
            length = 10;
            break;
        case DBT_CLASSIC_INPUT_OOB:
            if (size != (security->config.oob_present == 3 ? 64 : 32))
                return DBT_INVALID;
            opcode = security->config.oob_present == 1 ? 0x0430 : 0x0445;
            length = opcode == 0x0430 ? 38 : 70;
            copy(parameters + (security->config.oob_present == 2 ? 38 : 6), data, size);
            break;
        default:
            return DBT_INVALID;
        }
    } else if (size)
        return DBT_INVALID;
    dbt_result_t result =
        enqueue(security, peer, peer->address, opcode, parameters, length, DBT_COMMAND_COMPLETE);
    wipe(parameters, sizeof(parameters));
    if (result == DBT_OK) {
        if (accepted && peer->input == DBT_CLASSIC_INPUT_PIN)
            peer->key.pin_length = size;
        peer->input_token = 0;
        peer->input = DBT_CLASSIC_INPUT_NONE;
        if (!accepted)
            fail_peer(security, peer, DBT_CLASSIC_REASON_POLICY, 0);
    }
    return result;
}
static dbt_result_t begin_store(dbt_classic_security_t *security, dbt_classic_peer_t *peer,
                                bool deleting) {
    uint32_t store_token = token(security);
    if (!store_token)
        return DBT_NO_SPACE;
    if (!security->hooks.store)
        return DBT_UNSUPPORTED;
    security->in_callback = true;
    bool queued = security->hooks.store(security->hooks.context, peer->address,
                                        deleting ? 0 : &peer->key, store_token);
    security->in_callback = false;
    if (!queued)
        return DBT_NO_SPACE;
    peer->store_token = store_token;
    peer->store_deadline = security->now_ms + security->config.store_timeout_ms;
    peer->store_expired = false;
    peer->deleting = deleting;
    return DBT_OK;
}
dbt_result_t dbt_classic_security_complete_store(dbt_classic_security_t *security,
                                                 uint32_t store_token, bool committed) {
    if (!security || security->in_callback || !store_token)
        return DBT_INVALID;
    dbt_classic_peer_t *peer = 0;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (security->peers[i].used && security->peers[i].store_token == store_token)
            peer = &security->peers[i];
    if (!peer) {
        ++security->stale_completions;
        return DBT_EMPTY;
    }
    peer->store_token = 0;
    if (!committed) {
        fail_peer(security, peer, DBT_CLASSIC_REASON_STORAGE, 0);
        return DBT_OK;
    }
    if (peer->deleting) {
        peer->deleting = false;
        peer->bonded = false;
        peer->key_known = false;
        peer->ready = false;
        wipe(&peer->key, sizeof(peer->key));
        notify(security, peer, DBT_CLASSIC_BOND_REMOVED, DBT_CLASSIC_REASON_NONE, 0);
        if (peer->connected)
            fail_peer(security, peer, DBT_CLASSIC_REASON_POLICY, 0);
    } else {
        peer->bonded = true;
        peer->dirty_key = false;
        wipe(peer->key.bytes, sizeof(peer->key.bytes));
        if (!peer->failed && !peer->store_expired) {
            notify(security, peer, DBT_CLASSIC_BOND_STORED, DBT_CLASSIC_REASON_NONE, 0);
            maybe_ready(security, peer);
        }
    }
    if (!peer->connected && !referenced(security, peer))
        wipe(peer, sizeof(*peer));
    return DBT_OK;
}
dbt_result_t dbt_classic_security_forget(dbt_classic_security_t *security, const uint8_t address[6],
                                         uint32_t now_ms) {
    if (!security || security->in_callback || !address)
        return DBT_INVALID;
    security->now_ms = now_ms;
    dbt_classic_peer_t *peer = new_peer(security, address);
    if (!peer)
        return DBT_NO_SPACE;
    if (peer->store_token)
        return DBT_BUSY;
    dbt_result_t result = begin_store(security, peer, true);
    if (result == DBT_OK) {
        peer->ready = false;
        peer->input_token = 0;
    }
    return result;
}
bool dbt_classic_security_authorize(const dbt_classic_security_t *security, const dbt_link_t *link,
                                    dbt_security_t level) {
    if (!security || !link || !link->used || link->kind != DBT_LINK_CLASSIC ||
        level > DBT_SECURITY_AUTHENTICATED)
        return false;
    const dbt_link_t *current = dbt_acl_find(security->acl, link->handle);
    if (!current || current->generation != link->generation || current->kind != DBT_LINK_CLASSIC ||
        !equal(current->peer, link->peer))
        return false;
    if (level == DBT_SECURITY_NONE)
        return true;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_classic_peer_t *peer = &security->peers[i];
        if (peer->used && peer->connected && peer->handle == link->handle &&
            peer->generation == link->generation)
            return authorized_peer(security, peer, level);
    }
    return false;
}
dbt_result_t dbt_classic_security_require(dbt_classic_security_t *security, const dbt_link_t *link,
                                          dbt_security_t level, uint32_t now_ms) {
    if (!security || security->in_callback || !link || !link->used ||
        link->kind != DBT_LINK_CLASSIC || level > DBT_SECURITY_AUTHENTICATED)
        return DBT_INVALID;
    const dbt_link_t *current = dbt_acl_find(security->acl, link->handle);
    if (!current || current->generation != link->generation || current->kind != DBT_LINK_CLASSIC ||
        !equal(current->peer, link->peer))
        return DBT_INVALID;
    if (level == DBT_SECURITY_NONE)
        return DBT_OK;
    security->now_ms = now_ms;
    dbt_classic_peer_t *peer = new_peer(security, link->peer);
    if (!peer)
        return DBT_NO_SPACE;
    if (peer->failed)return DBT_INVALID;
    if (peer->deleting || (peer->connected && peer->generation != link->generation))
        return DBT_BUSY;
    peer->connected = true;
    peer->handle = link->handle;
    peer->generation = link->generation;
    dbt_security_t previous_wanted = peer->wanted;
    if (level > peer->wanted)
        peer->wanted = level;
    if (dbt_classic_security_authorize(security, link, level))
        return DBT_OK;
    dbt_result_t result = load(security, peer);
    if (result != DBT_OK && result != DBT_EMPTY)
        return result;
    if (level == DBT_SECURITY_AUTHENTICATED && security->config.io_capability == 3 &&
        !security->config.oob_present &&
        (!peer->key_known || !dbt_classic_key_authenticated(&peer->key))) {
        peer->wanted = previous_wanted;
        return DBT_UNSUPPORTED;
    }
    if (peer->key_known && !usable_key(security, peer, &peer->key)) {
        if (!pairing_allowed(security, peer))
            return DBT_UNSUPPORTED;
        peer->force_pair = true;
    }
    bool previous_requirement=peer->locally_required;
    peer->locally_required=true;
    if(!peer->force_pair && ((peer->accepted_pairing && peer->io_replied &&
        (!peer->pairing_complete || !peer->key_known)) ||
       (peer->key_known && link->encrypted && peer->encryption_mode &&
        peer->encryption_key_epoch==peer->key_epoch)))return DBT_BUSY;
    if (!peer->auth_requested) {
        uint8_t parameters[2];
        put16(parameters, link->handle);
        result = enqueue(security, peer, peer->address, 0x0411, parameters, 2, DBT_COMMAND_STATUS);
        if (result != DBT_OK) {
            peer->locally_required=previous_requirement;
            peer->wanted=previous_wanted;
            return result;
        }
        peer->auth_requested = true;
        peer->deadline = now_ms + security->config.procedure_timeout_ms;
    }
    return DBT_BUSY;
}

dbt_result_t dbt_classic_security_receive(dbt_classic_security_t *security,
                                          const dbt_hci_packet_t *packet, uint32_t now_ms) {
    if (!security || security->in_callback || !packet || packet->type != DBT_H4_EVENT ||
        (packet->size && !packet->payload))
        return DBT_INVALID;
    security->now_ms = now_ms;
    const uint8_t *data = packet->payload;
    dbt_classic_peer_t *peer = 0;
    if (packet->event == 0x06 || packet->event == 0x08 || packet->event == 0x30) {
        if (packet->size != (packet->event == 0x08 ? 4 : 3))
            return DBT_INVALID;
        const dbt_link_t *link = dbt_acl_find(security->acl, dbt_read_le16(data + 1));
        if (!link || link->kind != DBT_LINK_CLASSIC)
            return DBT_UNSUPPORTED;
        peer = new_peer(security, link->peer);
        if (!peer)
            return DBT_NO_SPACE;
        if (peer->failed)
            return DBT_OK;
        if (!peer->key_known) {
            dbt_result_t loaded = load(security, peer);
            if (loaded != DBT_OK && loaded != DBT_EMPTY)
                return loaded;
        }
        if (data[0]) {
            fail_peer(security, peer,
                      packet->event == 0x06 ? DBT_CLASSIC_REASON_AUTHENTICATION
                                            : DBT_CLASSIC_REASON_ENCRYPTION,
                      data[0]);
            return DBT_OK;
        }
        if (packet->event == 0x06) {
            peer->auth_complete = true;
            if (peer->wanted == DBT_SECURITY_NONE)
                peer->wanted = DBT_SECURITY_ENCRYPTED;
            return DBT_OK;
        }
        uint8_t mode = packet->event == 0x08 ? data[3] : peer->encryption_mode;
        if (mode > 2)
            return DBT_INVALID;
        if (peer->cipher_epoch == UINT32_MAX) {
            fail_peer(security, peer, DBT_CLASSIC_REASON_RESOURCES, 0);
            return DBT_OK;
        }
        ++peer->cipher_epoch;
        peer->encryption_key_epoch = mode ? peer->key_epoch : 0;
        bool was_secure = peer->ready;
        peer->encryption_mode = mode;
        peer->key_size = 0;
        peer->size_requested = false;
        peer->ready = false;
        if (mode && peer->wanted == DBT_SECURITY_NONE)
            peer->wanted = DBT_SECURITY_ENCRYPTED;
        if (!mode && was_secure)
            fail_peer(security, peer, DBT_CLASSIC_REASON_ENCRYPTION, 0);
        if (mode && security->config.require_secure_connections && mode != 2)
            fail_peer(security, peer, DBT_CLASSIC_REASON_ENCRYPTION, 0);
        return DBT_OK;
    }
    uint16_t expected;
    switch (packet->event) {
    case 0x16:
    case 0x17:
    case 0x31:
    case 0x34:
    case 0x35:
        expected = 6;
        break;
    case 0x18:
        expected = 23;
        break;
    case 0x32:
        expected = 9;
        break;
    case 0x33:
    case 0x3b:
        expected = 10;
        break;
    case 0x36:
        expected = 7;
        break;
    default:
        return DBT_UNSUPPORTED;
    }
    if (packet->size != expected)
        return DBT_INVALID;
    const uint8_t *address = packet->event == 0x36 ? data + 1 : data;
    peer = new_peer(security, address);
    if (!peer)
        return DBT_NO_SPACE;
    if (packet->event == 0x17) {
        dbt_result_t result = load(security, peer);
        if (result != DBT_OK && result != DBT_EMPTY)
            return result;
        if (peer->key_known && !peer->force_pair && !peer->failed && !peer->deleting &&
            usable_key(security, peer, &peer->key)) {
            dbt_classic_key_t key;
            copy(&key, &peer->key, sizeof(key));
            if (!peer->dirty_key && security->hooks.load) {
                security->in_callback = true;
                result = security->hooks.load(security->hooks.context, address, &key);
                security->in_callback = false;
                if (result != DBT_OK || !usable_key(security, peer, &key)) {
                    wipe(&key, sizeof(key));
                    return result == DBT_OK ? DBT_INVALID : result;
                }
            }
            uint8_t parameters[22];
            copy(parameters, address, 6);
            copy(parameters + 6, key.bytes, 16);
            result = enqueue(security, peer, address, 0x040b, parameters, 22, DBT_COMMAND_COMPLETE);
            wipe(parameters, sizeof(parameters));
            wipe(&key, sizeof(key));
            return result;
        }
        return address_reply(security, peer, address, 0x040c);
    }
    if (packet->event == 0x32) {
        if (data[6] > 3 || data[7] > 3 || data[8] > 5)
            return DBT_INVALID;
        peer->remote_io = data[6];
        peer->remote_auth = data[8];
        return DBT_OK;
    }
    if (packet->event == 0x36) {
        if (data[0])
            fail_peer(security, peer, DBT_CLASSIC_REASON_AUTHENTICATION, data[0]);
        else
            peer->pairing_complete = true;
        return DBT_OK;
    }
    if (packet->event == 0x18) {
        if (peer->failed || peer->deleting)
            return DBT_OK;
        uint8_t type = data[22];
        if (type == 6) {
            if (!peer->key_known) {
                fail_peer(security, peer, DBT_CLASSIC_REASON_KEY_TYPE, 0);
                return DBT_OK;
            }
            type = peer->key.type;
        } else if (!peer->accepted_pairing) {
            fail_peer(security, peer, DBT_CLASSIC_REASON_POLICY, 0);
            return DBT_OK;
        }
        dbt_classic_key_t key;
        copy(key.bytes, data + 6, 16);
        key.type = type;
        key.pin_length = peer->key.pin_length;
        if (!usable_key(security, peer, &key) ||
            (peer->auto_confirmed && dbt_classic_key_authenticated(&key))) {
            wipe(&key, sizeof(key));
            fail_peer(security, peer, DBT_CLASSIC_REASON_KEY_TYPE, 0);
            return DBT_OK;
        }
        copy(&peer->key, &key, sizeof(key));
        wipe(&key, sizeof(key));
        if (peer->key_epoch == UINT32_MAX) {
            fail_peer(security, peer, DBT_CLASSIC_REASON_RESOURCES, 0);
            return DBT_OK;
        }
        ++peer->key_epoch;
        peer->key_known = true;
        peer->dirty_key = true;
        peer->bonded = false;
        peer->ready = false;
        peer->force_pair = false;
        peer->key_size = 0;
        peer->size_requested = peer->encryption_requested = false;
        peer->deadline = now_ms + security->config.procedure_timeout_ms;
        if (security->config.authentication_requirements >= 2) {
            dbt_result_t result = begin_store(security, peer, false);
            if (result != DBT_OK)
                fail_peer(security, peer, DBT_CLASSIC_REASON_STORAGE, 0);
        }
        return DBT_OK;
    }
    bool accepted = peer->accepted_pairing || pairing_allowed(security, peer);
    if (packet->event == 0x31) {
        if (!accepted) {
            uint8_t parameters[7];
            copy(parameters, address, 6);
            parameters[6] = 0x18;
            return enqueue(security, peer, address, 0x0434, parameters, 7, DBT_COMMAND_COMPLETE);
        }
        if (peer->io_replied)
            return DBT_OK;
        uint8_t parameters[9];
        copy(parameters, address, 6);
        parameters[6] = security->config.io_capability;
        parameters[7] = security->config.oob_present;
        parameters[8] = (uint8_t)(security->config.authentication_requirements |
                                  (peer->wanted == DBT_SECURITY_AUTHENTICATED ? 1u : 0u));
        dbt_result_t result =
            enqueue(security, peer, address, 0x042b, parameters, 9, DBT_COMMAND_COMPLETE);
        if (result == DBT_OK) {
            peer->accepted_pairing = true;
            peer->io_replied = true;
            if (peer->wanted == DBT_SECURITY_NONE)
                peer->wanted = DBT_SECURITY_ENCRYPTED;
            peer->deadline = now_ms + security->config.procedure_timeout_ms;
        }
        return result;
    }
    if (packet->event == 0x3b) {
        uint32_t value = read32(data + 6);
        if (value > 999999)
            return DBT_INVALID;
        if (accepted && security->hooks.display_passkey) {
            security->in_callback = true;
            security->hooks.display_passkey(security->hooks.context, address, value);
            security->in_callback = false;
        }
        return DBT_OK;
    }
    if (!accepted) {
        uint16_t opcode = packet->event == 0x16   ? 0x040e
                          : packet->event == 0x33 ? 0x042d
                          : packet->event == 0x34 ? 0x042f
                                                  : 0x0433;
        return address_reply(security, peer, address, opcode);
    }
    peer->accepted_pairing = true;
    if (peer->wanted == DBT_SECURITY_NONE)
        peer->wanted = DBT_SECURITY_ENCRYPTED;
    if (packet->event == 0x16) {
        if (!security->config.allow_legacy)
            return address_reply(security, peer, address, 0x040e);
        return interaction(security, peer, DBT_CLASSIC_INPUT_PIN, 0);
    }
    if (packet->event == 0x35) {
        if (!security->config.oob_present)
            return address_reply(security, peer, address, 0x0433);
        return interaction(security, peer, DBT_CLASSIC_INPUT_OOB, 0);
    }
    if (packet->event == 0x34)
        return interaction(security, peer, DBT_CLASSIC_INPUT_PASSKEY, 0);
    uint32_t value = read32(data + 6);
    if (value > 999999)
        return DBT_INVALID;
    bool local_mitm = (security->config.authentication_requirements & 1u) ||
                      peer->wanted == DBT_SECURITY_AUTHENTICATED;
    bool remote_mitm = peer->remote_auth != 0xff && (peer->remote_auth & 1u);
    bool just_works = (!local_mitm && !remote_mitm) || security->config.io_capability == 3 ||
                      peer->remote_io == 3 ||
                      (security->config.io_capability == 0 && peer->remote_io == 0);
    if (just_works && !local_mitm && security->config.allow_just_works) {
        dbt_result_t result = address_reply(security, peer, address, 0x042c);
        if (result == DBT_OK)
            peer->auto_confirmed = true;
        return result;
    }
    return interaction(security, peer, DBT_CLASSIC_INPUT_CONFIRM, value);
}

void dbt_classic_security_poll(dbt_classic_security_t *security, uint32_t now_ms) {
    if (!security || security->in_callback)
        return;
    security->now_ms = now_ms;
    if (security->host->state == DBT_HOST_FAILED) {
        for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
            if (security->peers[i].used)
                fail_peer(security, &security->peers[i], DBT_CLASSIC_REASON_CONTROLLER, 0);
        return;
    }
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_classic_peer_t *peer = &security->peers[i];
        if (!peer->used)
            continue;
        if (peer->store_token && !peer->store_expired && due(now_ms, peer->store_deadline)) {
            peer->store_expired = true;
            fail_peer(security, peer, DBT_CLASSIC_REASON_STORAGE, 0);
        }
        if (peer->failed)
            continue;
        if (peer->input_token && due(now_ms, peer->deadline)) {
            if (address_reply(security, peer, peer->address, negative_opcode(peer->input)) ==
                DBT_OK) {
                peer->input_token = 0;
                peer->input = DBT_CLASSIC_INPUT_NONE;
                fail_peer(security, peer, DBT_CLASSIC_REASON_TIMEOUT, 0);
                continue;
            }
        }
        const dbt_link_t *link = live_link(security, peer);
        if (link && peer->wanted != DBT_SECURITY_NONE && !peer->ready &&
            due(now_ms, peer->deadline)) {
            fail_peer(security, peer, DBT_CLASSIC_REASON_TIMEOUT, 0);
            continue;
        }
        if (link && peer->key_known && peer->encryption_mode && link->encrypted &&
            peer->encryption_key_epoch == peer->key_epoch && !peer->size_requested) {
            uint8_t parameters[2];
            put16(parameters, link->handle);
            if (enqueue(security, peer, peer->address, 0x1408, parameters, 2,
                        DBT_COMMAND_COMPLETE) == DBT_OK)
                peer->size_requested = true;
        }
        if (link && peer->key_known && peer->encryption_mode &&
            peer->encryption_key_epoch != peer->key_epoch && !peer->encryption_requested) {
            uint8_t parameters[2];
            put16(parameters, link->handle);
            if (enqueue(security, peer, peer->address, 0x0c53, parameters, 2, DBT_COMMAND_STATUS) ==
                DBT_OK)
                peer->encryption_requested = true;
        }
        /* An application request made during incoming SSP is deferred until
         * pairing/key notification completes. Do not start a competing
         * authentication procedure while IO/SSP is still in progress. */
        if(link && peer->locally_required && peer->key_known && !peer->encryption_mode &&
           !peer->auth_requested && peer->pairing_complete) {
            uint8_t parameters[2];put16(parameters,link->handle);
            if(enqueue(security,peer,peer->address,0x0411,parameters,2,DBT_COMMAND_STATUS)==DBT_OK)
                peer->auth_requested=true;
        }
        if (link && peer->key_known && !peer->encryption_mode && !peer->encryption_requested &&
            peer->auth_requested && peer->auth_complete) {
            /* Simple Pairing Complete is not ownership of the peer's pending
             * encryption procedure. Only drive encryption after our own
             * Authentication Requested procedure actually completes. */
            uint8_t parameters[3];
            put16(parameters, link->handle);
            parameters[2] = 1;
            if (enqueue(security, peer, peer->address, 0x0413, parameters, 3, DBT_COMMAND_STATUS) ==
                DBT_OK)
                peer->encryption_requested = true;
        }
        maybe_ready(security, peer);
    }
    if (!security->command_active && security->count) {
        dbt_classic_security_command_t *command = &security->commands[security->head];
        dbt_classic_peer_t *peer = peer_by_serial(security, command->peer_serial);
        bool stale = command->peer_serial &&
                     (!peer || (command->generation &&
                                (!peer->connected || peer->generation != command->generation)));
        bool negative = command->opcode == 0x040c || command->opcode == 0x040e ||
                        command->opcode == 0x042d || command->opcode == 0x042f ||
                        command->opcode == 0x0433 || command->opcode == 0x0434;
        if (peer && ((peer->failed && !negative) ||
                     (command->opcode == 0x1408 && (command->key_epoch != peer->key_epoch ||
                                                    command->cipher_epoch != peer->cipher_epoch))))
            stale = true;
        if (!stale) {
            dbt_result_t result =
                dbt_host_command(security->host, command->opcode,
                                 command->extended ? command->extended : command->parameters,
                                 command->size, command->response, command_done, security, now_ms);
            if (result != DBT_OK)
                return;
            copy(&security->active, command, sizeof(*command));
            security->active.extended = 0;
            wipe(security->active.parameters, sizeof(security->active.parameters));
            security->command_active = true;
        }
        release_command(security, command);
        security->head = (uint8_t)((security->head + 1u) % DBT_CLASSIC_SECURITY_COMMANDS);
        --security->count;
    }
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_classic_peer_t *peer = &security->peers[i];
        if (peer->used && !peer->connected && !peer->store_token && !peer->input_token &&
            !referenced(security, peer) && (peer->failed || due(now_ms, peer->deadline)))
            wipe(peer, sizeof(*peer));
    }
}
uint32_t dbt_classic_security_wait_ms(const dbt_classic_security_t *security, uint32_t now_ms) {
    if (!security || security->host->state == DBT_HOST_FAILED)
        return UINT32_MAX;
    if (!security->command_active && security->count &&
        security->host->command_count < DBT_COMMAND_SLOTS)
        return 0;
    uint32_t wait = UINT32_MAX;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_classic_peer_t *peer = &security->peers[i];
        if (!peer->used)
            continue;
        if (peer->store_token && !peer->store_expired) {
            uint32_t delta = due(now_ms, peer->store_deadline) ? 0 : peer->store_deadline - now_ms;
            if (delta < wait)
                wait = delta;
        }
        if (peer->failed)
            continue;
        if ((peer->input_token ||
             (peer->connected && peer->wanted != DBT_SECURITY_NONE && !peer->ready) ||
             (!peer->connected && !peer->store_token && !referenced(security, peer)))) {
            uint32_t delta = due(now_ms, peer->deadline) ? 0 : peer->deadline - now_ms;
            if (delta < wait)
                wait = delta;
        }
    }
    return wait;
}
dbt_result_t dbt_classic_security_clear(dbt_classic_security_t *security) {
    if (!security || security->in_callback)
        return DBT_INVALID;
    if (security->command_active)
        return DBT_BUSY;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (security->peers[i].store_token)
            return DBT_BUSY;
    for (unsigned i = 0; i < DBT_CLASSIC_SECURITY_COMMANDS; ++i)
        release_command(security, &security->commands[i]);
    wipe(security, sizeof(*security));
    return DBT_OK;
}
