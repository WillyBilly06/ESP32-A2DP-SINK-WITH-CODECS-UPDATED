#ifndef PROBE_PRODUCT_IO_OWNER_H
#define PROBE_PRODUCT_IO_OWNER_H
#include "probe_product_io.h"
#include "dbt_gap.h"
typedef struct {
    dbt_input_action_t input;bool have_input,led_pending,key_released;
    uint8_t key_operation;uint32_t key_due,led_serial,led_pending_token;
    dbt_ble_control_peer_t notice_peer;uint8_t notice[3],notice_size;
    bool have_notice,sound_enabled;uint32_t unsupported,failures;
    dbt_gap_t *gap;
    uint16_t pair_handle;uint32_t pair_generation,pair_token,pair_deadline;
    uint8_t pair_phase,pair_status;bool pair_gone;
} probe_product_io_owner_t;
/* Host owner only. IO provider must already be started; bounded per poll. */
void probe_product_io_owner_poll(probe_product_io_owner_t *,uint32_t now_ms);
uint32_t probe_product_io_owner_wait_ms(const probe_product_io_owner_t *,uint32_t now_ms);
/* Required on every runtime link change, including before IO startup. Cancels
 * an unsent disconnect synchronously before a reused handle can be submitted. */
void probe_product_io_owner_link(probe_product_io_owner_t *,const dbt_link_change_t *);
#endif
