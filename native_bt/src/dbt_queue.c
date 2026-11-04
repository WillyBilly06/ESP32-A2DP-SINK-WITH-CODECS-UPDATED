#include "dbt_queue.h"

_Static_assert(ATOMIC_INT_LOCK_FREE == 2 && sizeof(unsigned int) == 4,
               "Ingress needs lock-free 32-bit atomics");
_Static_assert((DBT_QUEUE_BYTES & (DBT_QUEUE_BYTES - 1u)) == 0,
               "Queue size must be a power of two");
_Static_assert(DBT_QUEUE_BYTES < UINT32_MAX / 2u,
               "Monotonic ring counters need an unambiguous distance");

static uint32_t slot(uint32_t position) {
    return position & (DBT_QUEUE_BYTES - 1u);
}

void dbt_queue_init(dbt_queue_t *queue) {
    atomic_init(&queue->written, 0);
    atomic_init(&queue->read, 0);
    atomic_init(&queue->rejected, 0);
    atomic_init(&queue->high_water, 0);
}

static dbt_result_t push(dbt_queue_t *queue, const uint8_t *data,
                         uint16_t size, bool reserve_control, bool sensitive) {
    if (data == 0 || size == 0 || size > DBT_MAX_H4_BYTES) return DBT_INVALID;
    uint32_t written = atomic_load_explicit(&queue->written, memory_order_relaxed);
    uint32_t read = atomic_load_explicit(&queue->read, memory_order_acquire);
    uint32_t used = written - read;
    uint32_t required = (uint32_t)size + 2u;
    uint32_t reserve = reserve_control ? DBT_CONTROL_RESERVE_BYTES : 0u;
    if (used > DBT_QUEUE_BYTES || required + reserve > DBT_QUEUE_BYTES - used) {
        atomic_fetch_add_explicit(&queue->rejected, 1, memory_order_relaxed);
        return DBT_NO_SPACE;
    }
    queue->bytes[slot(written)] = (uint8_t)size;
    queue->bytes[slot(written + 1u)] = (uint8_t)((size >> 8) | (sensitive ? 0x80u : 0u));
    for (uint32_t i = 0; i < size; ++i) {
        queue->bytes[slot(written + 2u + i)] = data[i];
    }
    uint32_t occupancy = used + required;
    if (occupancy > atomic_load_explicit(&queue->high_water, memory_order_relaxed)) {
        atomic_store_explicit(&queue->high_water, occupancy, memory_order_relaxed);
    }
    /* Publish a complete record, including its length, in one release. */
    atomic_store_explicit(&queue->written, written + required, memory_order_release);
    return DBT_OK;
}

dbt_result_t dbt_queue_push(dbt_queue_t *queue, const uint8_t *data,
                            uint16_t size, bool reserve_control) {
    return push(queue,data,size,reserve_control,false);
}
dbt_result_t dbt_queue_push_sensitive(dbt_queue_t *queue, const uint8_t *data,
                                      uint16_t size, bool reserve_control) {
    return push(queue,data,size,reserve_control,true);
}

dbt_result_t dbt_queue_pop(dbt_queue_t *queue, uint8_t *data,
                           uint16_t capacity, uint16_t *size) {
    if (data == 0 || size == 0) return DBT_INVALID;
    uint32_t read = atomic_load_explicit(&queue->read, memory_order_relaxed);
    uint32_t written = atomic_load_explicit(&queue->written, memory_order_acquire);
    uint32_t used = written - read;
    if (used == 0) return DBT_EMPTY;
    if (used < 2u || used > DBT_QUEUE_BYTES) return DBT_INVALID;
    uint16_t length = (uint16_t)(queue->bytes[slot(read)] |
                                ((uint16_t)queue->bytes[slot(read + 1u)] << 8));
    bool sensitive = (length & 0x8000u) != 0;
    length &= 0x7fffu;
    if (length == 0 || length > DBT_MAX_H4_BYTES || (uint32_t)length + 2u > used) {
        return DBT_INVALID;
    }
    *size = length;
    if (length > capacity) return DBT_NO_SPACE;
    for (uint32_t i = 0; i < length; ++i) {
        data[i] = queue->bytes[slot(read + 2u + i)];
    }
    if(sensitive) {
        volatile uint8_t *bytes = queue->bytes;
        for(uint32_t i=0;i<length;++i)bytes[slot(read+2u+i)]=0;
    }
    atomic_store_explicit(&queue->read, read + (uint32_t)length + 2u, memory_order_release);
    return DBT_OK;
}

uint32_t dbt_queue_used(const dbt_queue_t *queue) {
    uint32_t read = atomic_load_explicit(&queue->read, memory_order_relaxed);
    return atomic_load_explicit(&queue->written, memory_order_acquire) - read;
}
