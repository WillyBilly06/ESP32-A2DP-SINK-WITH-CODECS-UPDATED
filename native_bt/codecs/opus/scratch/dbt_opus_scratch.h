#ifndef DBT_OPUS_SCRATCH_H
#define DBT_OPUS_SCRATCH_H

#include <setjmp.h>
#include <stddef.h>
#include <stdint.h>

typedef void *(*dbt_opus_alloc_fn)(size_t);
typedef void (*dbt_opus_free_fn)(void *);
typedef int (*dbt_opus_work_fn)(void *);
typedef int (*dbt_opus_reset_fn)(void *);

typedef struct {
    unsigned char *storage;
    size_t capacity, used, peak;
    dbt_opus_free_fn release;
    jmp_buf escape;
} dbt_opus_scratch;

enum { DBT_OPUS_SCRATCH_OK = 0, DBT_OPUS_SCRATCH_BUSY = -100,
       DBT_OPUS_SCRATCH_EXHAUSTED = -101, DBT_OPUS_SCRATCH_RESET_FAILED = -102,
       DBT_OPUS_SCRATCH_BAD_ARGUMENT = -103 };

/* Arena must be zero initialized; owner serializes init/destroy with run. */
int dbt_opus_scratch_init(dbt_opus_scratch *, size_t, dbt_opus_alloc_fn, dbt_opus_free_fn);
/* Borrowed storage belongs to the containing active-codec allocation. */
int dbt_opus_scratch_bind(dbt_opus_scratch *, void *, size_t);
static inline size_t dbt_opus_scratch_capacity(unsigned channels) {
    return channels == 1 ? 12288u : channels == 2 ? 16384u : 0u;
}
int dbt_opus_scratch_destroy(dbt_opus_scratch *);
/* work_result is valid only on OK. reset must invalidate partial decoder output/state. */
int dbt_opus_scratch_run(dbt_opus_scratch *, dbt_opus_work_fn, void *,
                         dbt_opus_reset_fn, void *, int *work_result);
size_t dbt_opus_scratch_mark(void);
void dbt_opus_scratch_restore(size_t);
void *dbt_opus_scratch_push(size_t, size_t, size_t);

#endif
