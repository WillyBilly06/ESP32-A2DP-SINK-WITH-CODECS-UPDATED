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
    uint16_t audio_handle;uint32_t audio_generation,audio_connected_at,audio_disconnected_at;
    uint32_t connected_due,connected_rate,last_max_at;
    uint32_t sound_rate,sound_admitted,sound_attached,sound_cancelled,sound_dropped;
    uint8_t sound_type,inflight_type;bool audio_connected,had_disconnect,connected_pending;
    bool sound_pending,sound_exclusive,sound_inflight,have_max,pairing_active,io_ready;
    bool restart_preparing,restart_fault;
} probe_product_io_owner_t;
/* Host owner only. IO provider must already be started; bounded per poll. */
void probe_product_io_owner_poll(probe_product_io_owner_t *,uint32_t now_ms);
uint32_t probe_product_io_owner_wait_ms(const probe_product_io_owner_t *,uint32_t now_ms);
/* Required on every runtime link change, including before IO startup. Cancels
 * an unsent disconnect synchronously before a reused handle can be submitted. */
void probe_product_io_owner_link(probe_product_io_owner_t *,const dbt_link_change_t *);
/* Copied profile identity; event values match probe_audio_product_event_t. */
void probe_product_io_owner_audio(probe_product_io_owner_t *,unsigned event,uint16_t handle,uint32_t generation,uint32_t rate,uint32_t now_ms);
void probe_product_io_owner_volume(probe_product_io_owner_t *,uint8_t volume,uint32_t now_ms);
/* Missing file, muted sound, and disabled provider are optional at boot. */
bool probe_product_io_owner_startup_needed(bool sound_enabled,uint8_t sound_status);
/* Owner-only reversible restart input barrier. Begin pauses sampling, poll
 * drains accepted actions, cancel resumes. OK from poll means producer
 * paused, copied actions drained and all owner-held input applied. */
dbt_result_t probe_product_io_owner_restart_begin(probe_product_io_owner_t *);
dbt_result_t probe_product_io_owner_restart_poll(probe_product_io_owner_t *,uint32_t now_ms);
void probe_product_io_owner_restart_cancel(probe_product_io_owner_t *);
#endif
