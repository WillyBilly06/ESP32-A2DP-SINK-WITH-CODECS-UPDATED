#include "dbt_opus_scratch.h"
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

/* Process-wide ownership is deliberate: shared Opus call paths have no arena
 * parameter. No lock wait, allocation, or callback occurs on the failure path. */
static _Atomic(dbt_opus_scratch *) active;
static _Thread_local dbt_opus_scratch *local_active;

static dbt_opus_scratch *current(void) {
    dbt_opus_scratch *s = atomic_load_explicit(&active, memory_order_acquire);
    if (!s || local_active != s) abort(); /* unguarded entry or wrong task */
    return s;
}

int dbt_opus_scratch_init(dbt_opus_scratch *s, size_t capacity,
                          dbt_opus_alloc_fn allocate, dbt_opus_free_fn release) {
    if (!s || !capacity || !allocate || !release) return DBT_OPUS_SCRATCH_BAD_ARGUMENT;
    if (s->storage || s->capacity || s->release) return DBT_OPUS_SCRATCH_BAD_ARGUMENT;
    memset(s, 0, sizeof(*s));
    s->storage = allocate(capacity);
    if (!s->storage) return DBT_OPUS_SCRATCH_EXHAUSTED;
    s->capacity = capacity;
    s->release = release;
    return DBT_OPUS_SCRATCH_OK;
}

int dbt_opus_scratch_bind(dbt_opus_scratch *s, void *storage, size_t capacity) {
    if (!s || !storage || !capacity || s->storage || s->capacity || s->release)
        return DBT_OPUS_SCRATCH_BAD_ARGUMENT;
    s->storage = storage;
    s->capacity = capacity;
    return DBT_OPUS_SCRATCH_OK;
}

int dbt_opus_scratch_destroy(dbt_opus_scratch *s) {
    if (!s) return DBT_OPUS_SCRATCH_BAD_ARGUMENT;
    if (atomic_load_explicit(&active, memory_order_acquire) == s) return DBT_OPUS_SCRATCH_BUSY;
    if (s->storage && s->release) s->release(s->storage);
    memset(s, 0, sizeof(*s));
    return DBT_OPUS_SCRATCH_OK;
}

int dbt_opus_scratch_run(dbt_opus_scratch *s, dbt_opus_work_fn work, void *arg,
                         dbt_opus_reset_fn reset, void *reset_arg, int *result) {
    dbt_opus_scratch *expected = NULL;
    if (!s || !s->storage || !s->capacity || !work || !reset || !result)
        return DBT_OPUS_SCRATCH_BAD_ARGUMENT;
    if (!atomic_compare_exchange_strong_explicit(&active, &expected, s,
                                                  memory_order_acq_rel, memory_order_acquire))
        return DBT_OPUS_SCRATCH_BUSY;
    local_active = s;
    s->used = 0;
    if (setjmp(s->escape) != 0) {
        s->used = 0;
        local_active = NULL; /* reset must not allocate through the Opus arena */
        int reset_result = reset(reset_arg);
        atomic_store_explicit(&active, NULL, memory_order_release);
        return reset_result == 0 ? DBT_OPUS_SCRATCH_EXHAUSTED : DBT_OPUS_SCRATCH_RESET_FAILED;
    }
    *result = work(arg);
    s->used = 0;
    local_active = NULL;
    atomic_store_explicit(&active, NULL, memory_order_release);
    return DBT_OPUS_SCRATCH_OK;
}

size_t dbt_opus_scratch_mark(void) { return current()->used; }
void dbt_opus_scratch_restore(size_t mark) {
    dbt_opus_scratch *s = current();
    if (mark > s->used) longjmp(s->escape, 1);
    s->used = mark;
}
void *dbt_opus_scratch_push(size_t count, size_t size, size_t alignment) {
    dbt_opus_scratch *s = current();
    uintptr_t base = (uintptr_t)s->storage;
    size_t padding, bytes;
    if (!size || !alignment || (alignment & (alignment-1)) || count > SIZE_MAX / size)
        longjmp(s->escape, 1);
    bytes = count * size;
    padding = (size_t)(-(base + s->used) & (alignment - 1));
    if (padding > s->capacity - s->used || bytes > s->capacity - s->used - padding)
        longjmp(s->escape, 1);
    s->used += padding;
    void *p = s->storage + s->used;
    s->used += bytes;
    if (s->used > s->peak) s->peak = s->used;
    return p;
}
