#include "probe_security.h"
#include "freertos/queue.h"
#include "nvs.h"
#include <stdio.h>

#ifndef DBT_PROBE_PAIRING_PEER_BYTES
#error "The pairing probe requires an explicitly selected Linux adapter address"
#endif
#define BOND_LIMIT 8u
static const uint8_t allowed_peer[6] = {DBT_PROBE_PAIRING_PEER_BYTES};
static dbt_classic_security_t security;
static dbt_runtime_t *runtime;
static dbt_gap_t *gap;
static TaskHandle_t owner;
static nvs_handle_t storage;
static bool storage_open;
static QueueHandle_t work_queue, completion_queue, log_queue;
typedef struct {
    uint8_t address[6];
    dbt_classic_key_t key;
    bool used;
} cached_bond_t;
static cached_bond_t bonds[BOND_LIMIT];
typedef struct {
    uint32_t token;
    uint8_t address[6];
    dbt_classic_key_t key;
    bool deleting, committed;
} store_job_t;
typedef struct {
    union {
        dbt_classic_security_info_t info;
        struct {
            uint32_t before,after,owner,ordinal;
            uint16_t size,offset;
            bool after_ready;
        } hashes;
    };
    uint8_t event, reason, status;
} log_item_t;
static struct {
    uint16_t handle;
    uint32_t generation;
    bool used;
} close_requests[DBT_MAX_LINKS];
static void wipe(void *data, unsigned size) {
    volatile uint8_t *out = data;
    for (unsigned i = 0; i < size; ++i)
        out[i] = 0;
}
static void copy(void *out, const void *in, unsigned size) {
    uint8_t *a = out;
    const uint8_t *b = in;
    for (unsigned i = 0; i < size; ++i)
        a[i] = b[i];
}
static bool same(const uint8_t *a, const uint8_t *b) {
    for (unsigned i = 0; i < 6; ++i)
        if (a[i] != b[i])
            return false;
    return true;
}
static void storage_name(const uint8_t address[6], char key[15]) {
    static const char digits[] = "0123456789abcdef";
    key[0] = 'b';
    key[1] = 'r';
    for (unsigned i = 0; i < 6; ++i) {
        key[2 + i * 2] = digits[address[i] >> 4];
        key[3 + i * 2] = digits[address[i] & 15];
    }
    key[14] = 0;
}
static int cache_index(const uint8_t address[6]) {
    for (unsigned i = 0; i < BOND_LIMIT; ++i)
        if (bonds[i].used && same(bonds[i].address, address))
            return (int)i;
    return -1;
}
static int free_index(void) {
    for (unsigned i = 0; i < BOND_LIMIT; ++i)
        if (!bonds[i].used)
            return (int)i;
    return -1;
}
static bool decode(const uint8_t bytes[28], const char *key, cached_bond_t *bond) {
    if (bytes[0] != 1 || bytes[3] || bytes[26] || bytes[27])
        return false;
    copy(bond->address, bytes + 4, 6);
    bond->key.type = bytes[1];
    bond->key.pin_length = bytes[2];
    copy(bond->key.bytes, bytes + 10, 16);
    char expected[15];
    storage_name(bond->address, expected);
    for (unsigned i = 0; i < 15; ++i)
        if (expected[i] != key[i])
            return false;
    bond->used = dbt_classic_key_valid(&bond->key);
    return bond->used;
}
static void encode(uint8_t bytes[28], const store_job_t *job) {
    wipe(bytes, 28);
    bytes[0] = 1;
    bytes[1] = job->key.type;
    bytes[2] = job->key.pin_length;
    copy(bytes + 4, job->address, 6);
    copy(bytes + 10, job->key.bytes, 16);
}
static esp_err_t preload(void) {
    esp_err_t error = nvs_open("native_bonds", NVS_READWRITE, &storage);
    if (error != ESP_OK)
        return error;
    storage_open = true;
    nvs_iterator_t iterator = 0;
    error = nvs_entry_find_in_handle(storage, NVS_TYPE_BLOB, &iterator);
    if (error == ESP_ERR_NVS_NOT_FOUND)
        return ESP_OK;
    while (error == ESP_OK) {
        nvs_entry_info_t entry;
        error = nvs_entry_info(iterator, &entry);
        if (error != ESP_OK)
            break;
        int index = free_index();
        if (index < 0) {
            error = ESP_ERR_NVS_NOT_ENOUGH_SPACE;
            break;
        }
        uint8_t bytes[28];
        size_t size = sizeof(bytes);
        error = nvs_get_blob(storage, entry.key, bytes, &size);
        if (error != ESP_OK || size != sizeof(bytes) || !decode(bytes, entry.key, &bonds[index])) {
            wipe(bytes, sizeof(bytes));
            if (error == ESP_OK)
                error = ESP_ERR_INVALID_STATE;
            break;
        }
        wipe(bytes, sizeof(bytes));
        error = nvs_entry_next(&iterator);
    }
    nvs_release_iterator(iterator);
    return error == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : error;
}
static dbt_result_t load(void *context, const uint8_t address[6], dbt_classic_key_t *key) {
    (void)context;
    int index = cache_index(address);
    if (index < 0)
        return DBT_EMPTY;
    copy(key, &bonds[index].key, sizeof(*key));
    return DBT_OK;
}
static bool store(void *context, const uint8_t address[6], const dbt_classic_key_t *key,
                  uint32_t token) {
    (void)context;
    if (!same(address, allowed_peer))
        return false;
    if (key && cache_index(address) < 0 && free_index() < 0)
        return false;
    store_job_t job;
    wipe(&job, sizeof(job));
    copy(job.address, address, 6);
    job.token = token;
    job.deleting = key == 0;
    if (key)
        copy(&job.key, key, sizeof(*key));
    bool accepted = xQueueSend(work_queue, &job, 0) == pdTRUE;
    wipe(&job, sizeof(job));
    return accepted;
}
static void worker(void *context) {
    (void)context;
    for (;;) {
        store_job_t job;
        if (xQueueReceive(work_queue, &job, portMAX_DELAY) != pdTRUE)
            continue;
        char key[15];
        storage_name(job.address, key);
        esp_err_t error;
        if (job.deleting) {
            error = nvs_erase_key(storage, key);
            if (error == ESP_ERR_NVS_NOT_FOUND)
                error = ESP_OK;
        } else {
            uint8_t bytes[28];
            encode(bytes, &job);
            error = nvs_set_blob(storage, key, bytes, sizeof(bytes));
            wipe(bytes, sizeof(bytes));
        }
        if (error == ESP_OK)
            error = nvs_commit(storage);
        job.committed = error == ESP_OK;
        (void)xQueueSend(completion_queue, &job, portMAX_DELAY);
        wipe(&job, sizeof(job));
        xTaskNotifyGive(owner);
    }
}
static void logger(void *context) {
    (void)context;
    for (;;) {
        log_item_t item;
        if (xQueueReceive(log_queue, &item, portMAX_DELAY) != pdTRUE)
            continue;
        if(item.event==0x8b) {
            printf("{\"probe\":\"ingress_hash\",\"ordinal\":%lu,\"size\":%u,\"offset\":%u,"
                   "\"controller_before\":%lu,\"controller_after\":%lu,\"host_owner\":%lu,\"after_ready\":%s}\n",
                   (unsigned long)item.hashes.ordinal,item.hashes.size,item.hashes.offset,
                   (unsigned long)item.hashes.before,(unsigned long)item.hashes.after,
                   (unsigned long)item.hashes.owner,item.hashes.after_ready?"true":"false");
            continue;
        }
        if (item.event & 0x80u) {
            printf("{\"probe\":\"echo_pattern\",\"stage\":%u,\"size\":%u,\"offset\":%lu,"
                   "\"expected\":%u,\"actual\":%u}\n",item.event & 0x7fu,item.info.handle,
                   (unsigned long)item.info.generation,item.info.key_size,item.info.key_type);
            continue;
        }
        printf("{\"probe\":\"classic_security\",\"event\":%u,\"reason\":%u,\"status\":%u,"
               "\"connected\":%s,"
               "\"generation\":%lu,\"key_type\":%u,\"key_size\":%u,\"authenticated\":%s,"
               "\"encrypted\":%s,\"bonded\":%s,\"audio_ready\":false}\n",
               item.event, item.reason, item.status, item.info.connected ? "true" : "false",
               (unsigned long)item.info.generation, item.info.key_type, item.info.key_size,
               item.info.authenticated ? "true" : "false", item.info.encrypted ? "true" : "false",
               item.info.bonded ? "true" : "false");
    }
}
void probe_security_trace(uint8_t stage,uint16_t size,uint16_t offset,uint8_t expected,uint8_t actual) {
    static unsigned submitted;
    if (!log_queue || submitted >= 16) return;
    log_item_t item;
    wipe(&item,sizeof(item));item.event=(uint8_t)(0x80u | stage);
    item.info.handle=size;item.info.generation=offset;
    item.info.key_size=expected;item.info.key_type=actual;
    if(xQueueSend(log_queue,&item,0)==pdTRUE)++submitted;
}
void probe_security_trace_hash(uint32_t before,uint32_t after,uint32_t queued,uint32_t ordinal,
                               uint16_t size,uint16_t offset,bool after_ready) {
    static unsigned submitted;
    if(!log_queue || submitted>=4)return;
    log_item_t item;wipe(&item,sizeof(item));item.event=0x8b;
    item.hashes.before=before;item.hashes.after=after;item.hashes.owner=queued;
    item.hashes.ordinal=ordinal;item.hashes.size=size;item.hashes.offset=offset;
    item.hashes.after_ready=after_ready;
    if(xQueueSend(log_queue,&item,0)==pdTRUE)++submitted;
}
static void event(void *context, dbt_classic_security_event_t kind,
                  const dbt_classic_security_info_t *info, dbt_classic_security_reason_t reason,
                  uint8_t status) {
    (void)context;
    log_item_t item;
    wipe(&item, sizeof(item));
    copy(&item.info, info, sizeof(item.info));
    item.event = (uint8_t)kind;
    item.reason = (uint8_t)reason;
    item.status = status;
    (void)xQueueSend(log_queue, &item, 0);
    xTaskNotifyGive(owner);
    if (kind == DBT_CLASSIC_SECURITY_FAILED && info->connected) {
        for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
            if (!close_requests[i].used) {
                close_requests[i].handle = info->handle;
                close_requests[i].generation = info->generation;
                close_requests[i].used = true;
                break;
            }
    }
}
static bool pairing(void *context, const uint8_t address[6]) {
    (void)context;
    return same(address, allowed_peer);
}
bool probe_security_accept(void *context, const uint8_t address[6], const uint8_t device_class[3]) {
    (void)context;
    (void)device_class;
    return same(address, allowed_peer);
}
bool probe_security_authorize(void *context, const dbt_link_t *link, uint16_t psm,
                              dbt_security_t level) {
    (void)context;
    (void)psm;
    return dbt_classic_security_authorize(&security, link, level);
}
dbt_result_t probe_security_forget_peer(const dbt_link_t *link) {
    if (!link || !same(link->peer, allowed_peer) ||
        !dbt_classic_security_authorize(&security, link, DBT_SECURITY_ENCRYPTED))
        return DBT_INVALID;
    return dbt_classic_security_forget(&security, link->peer, runtime->now_ms);
}
dbt_result_t probe_security_request(void *context, const dbt_link_t *link, uint16_t psm,
                                    dbt_security_t level, bool start) {
    (void)context;
    (void)psm;
    if (dbt_classic_security_authorize(&security, link, level))
        return DBT_OK;
    if (start)
        return dbt_classic_security_require(&security, link, level, runtime->now_ms);
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_classic_peer_t *peer = &security.peers[i];
        if (peer->used && peer->connected && peer->handle == link->handle &&
            peer->generation == link->generation)
            return peer->failed ? DBT_INVALID : DBT_BUSY;
    }
    return DBT_UNSUPPORTED;
}
dbt_result_t probe_security_init(dbt_runtime_t *host_runtime, dbt_gap_t *host_gap,
                                 TaskHandle_t host_owner) {
    TaskHandle_t worker_handle = 0, logger_handle = 0;
    dbt_result_t result = DBT_NO_SPACE;
    runtime = host_runtime;
    gap = host_gap;
    owner = host_owner;
    if (preload() != ESP_OK) {
        if (storage_open)
            nvs_close(storage);
        storage_open = false;
        wipe(bonds, sizeof(bonds));
        return DBT_INVALID;
    }
    work_queue = xQueueCreate(4, sizeof(store_job_t));
    completion_queue = xQueueCreate(4, sizeof(store_job_t));
    log_queue = xQueueCreate(8, sizeof(log_item_t));
    if (!work_queue || !completion_queue || !log_queue)
        goto failed;
    dbt_classic_security_config_t config = {0};
    config.io_capability = 3;
    config.authentication_requirements = 4;
    config.minimum_key_size = 16;
    config.allow_just_works = true;
    config.allow_legacy = false;
    config.require_durable_bond = true;
    config.procedure_timeout_ms = 30000;
    config.store_timeout_ms = 5000;
    dbt_classic_security_hooks_t hooks = {0};
    hooks.pairing_allowed = pairing;
    hooks.load = load;
    hooks.store = store;
    hooks.event = event;
    result = dbt_classic_security_init(&security, &runtime->host, &runtime->acl, config, hooks);
    if (result != DBT_OK)
        goto failed;
    dbt_classic_security_pairable(&security, true);
    result = DBT_NO_SPACE;
    if (xTaskCreatePinnedToCore(worker, "native_bond_store", 3072, 0, 1, &worker_handle, 0) !=
        pdPASS)
        goto failed;
    if (xTaskCreatePinnedToCore(logger, "native_security_log", 2048, 0, 0, &logger_handle, 0) !=
        pdPASS)
        goto failed;
    return DBT_OK;
failed:
    if (logger_handle)
        vTaskDelete(logger_handle);
    if (worker_handle)
        vTaskDelete(worker_handle);
    if (work_queue)
        vQueueDelete(work_queue);
    if (completion_queue)
        vQueueDelete(completion_queue);
    if (log_queue)
        vQueueDelete(log_queue);
    work_queue = completion_queue = log_queue = 0;
    if (storage_open)
        nvs_close(storage);
    storage_open = false;
    wipe(bonds, sizeof(bonds));
    (void)dbt_classic_security_clear(&security);
    return result;
}
void probe_security_link(const dbt_link_change_t *change) {
    dbt_result_t result = dbt_classic_security_link(&security, change);
    if (result != DBT_OK && result != DBT_UNSUPPORTED) {
        const dbt_link_t *link = dbt_acl_find(&runtime->acl, change->handle);
        if (link && link->generation == change->generation)
            (void)dbt_gap_disconnect(gap, link, 0x13);
    }
}
dbt_result_t probe_security_receive(const dbt_hci_packet_t *packet, uint32_t now_ms) {
    return runtime->ready ? dbt_classic_security_receive(&security, packet, now_ms)
                          : DBT_UNSUPPORTED;
}
void probe_security_poll(uint32_t now_ms) {
    store_job_t job;
    while (xQueueReceive(completion_queue, &job, 0) == pdTRUE) {
        if (job.committed) {
            int index = cache_index(job.address);
            if (job.deleting) {
                if (index >= 0)
                    wipe(&bonds[index], sizeof(bonds[index]));
            } else {
                if (index < 0)
                    index = free_index();
                if (index >= 0) {
                    copy(bonds[index].address, job.address, 6);
                    copy(&bonds[index].key, &job.key, sizeof(job.key));
                    bonds[index].used = true;
                } else
                    job.committed = false;
            }
        }
        (void)dbt_classic_security_complete_store(&security, job.token, job.committed);
        wipe(&job, sizeof(job));
    }
    dbt_classic_security_poll(&security, now_ms);
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (close_requests[i].used) {
            const dbt_link_t *link = dbt_acl_find(&runtime->acl, close_requests[i].handle);
            if (!link || link->generation != close_requests[i].generation ||
                dbt_gap_disconnect(gap, link, 0x13) == DBT_OK)
                close_requests[i].used = false;
        }
}
uint32_t probe_security_wait_ms(uint32_t now_ms) {
    if (uxQueueMessagesWaiting(completion_queue))
        return 0;
    if (gap->state == DBT_GAP_READY && runtime->host.command_count < DBT_COMMAND_SLOTS)
        for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
            if (close_requests[i].used)
                return 0;
    return dbt_classic_security_wait_ms(&security, now_ms);
}
unsigned probe_security_bytes(void) {
    return sizeof(security) + sizeof(bonds) + sizeof(close_requests);
}
unsigned probe_security_loaded_bonds(void) {
    unsigned count = 0;
    for (unsigned i = 0; i < BOND_LIMIT; ++i)
        if (bonds[i].used)
            ++count;
    return count;
}
