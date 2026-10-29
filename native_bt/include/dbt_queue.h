#ifndef DBT_QUEUE_H
#define DBT_QUEUE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>

/* One controller-task producer, one host-task consumer. No borrowed RX data. */
#define DBT_QUEUE_BYTES 4096u
#define DBT_MAX_H4_BYTES 1029u /* 1024-byte host ACL payload + H4/ACL header. */
#define DBT_CONTROL_RESERVE_BYTES 520u

typedef enum {
    DBT_OK = 0,
    DBT_EMPTY,
    DBT_INVALID,
    DBT_NO_SPACE,
    DBT_UNSUPPORTED,
    DBT_BUSY
} dbt_result_t;

typedef struct {
    _Atomic uint32_t written;
    _Atomic uint32_t read;
    _Atomic uint32_t rejected;
    _Atomic uint32_t high_water;
    uint8_t bytes[DBT_QUEUE_BYTES];
} dbt_queue_t;

/* Initialize before registering callbacks. Never reset a live queue. */
void dbt_queue_init(dbt_queue_t *queue);
dbt_result_t dbt_queue_push(dbt_queue_t *queue, const uint8_t *data,
                            uint16_t size, bool reserve_control);
/* Clears consumed payload before publishing the read position. Normal media
 * records keep the original copy cost. The consumer output remains borrowed
 * storage owned by its caller, which must clear its own sensitive copies. */
dbt_result_t dbt_queue_push_sensitive(dbt_queue_t *queue, const uint8_t *data,
                                      uint16_t size, bool reserve_control);
dbt_result_t dbt_queue_pop(dbt_queue_t *queue, uint8_t *data,
                           uint16_t capacity, uint16_t *size);
/* Call only from the consumer; it is a snapshot, not a reservation. */
uint32_t dbt_queue_used(const dbt_queue_t *queue);

#endif
