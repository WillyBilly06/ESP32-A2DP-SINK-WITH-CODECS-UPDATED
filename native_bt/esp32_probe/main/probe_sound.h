#ifndef PROBE_SOUND_H
#define PROBE_SOUND_H
#include "dbt_sound.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
typedef struct {
    dbt_sound_t *provider;dbt_sound_play_t *play;TaskHandle_t worker;
    void (*wake_external)(void *);void *wake_context;
    uint32_t token,rate;uint8_t type;bool attached,stopping;
    dbt_result_t last_result;
    uint32_t ticket,phase_since,preferred_rate;
    uint8_t phase;
    bool start_pending,exclusive,cancel_sent,phase_timedout;
} probe_sound_t;
/* Initialize after output and provider initialization. All calls except
 * disconnect run on the single provider/sound worker, never on host or ISR. */
dbt_result_t probe_sound_init(probe_sound_t *,dbt_sound_t *,TaskHandle_t);
/* Alternative for a persistent semaphore/dispatcher. Manager must initially
 * be zero-initialized; repeat init is rejected. Wake must be nonblocking,
 * render-task-safe and remain valid until the manager and render are quiescent. */
dbt_result_t probe_sound_init_with_wake(probe_sound_t *,dbt_sound_t *,void (*)(void *),void *);
dbt_result_t probe_sound_play(probe_sound_t *,uint8_t,uint32_t,bool);
dbt_result_t probe_sound_poll(probe_sound_t *,uint32_t,dbt_sound_peer_t *);
void probe_sound_stop(probe_sound_t *);
dbt_result_t probe_sound_disconnect(probe_sound_t *,dbt_sound_peer_t);
static inline bool probe_sound_busy(const probe_sound_t *s){return s && (s->play || s->token || s->ticket || s->phase);}
static inline bool probe_sound_start_pending(const probe_sound_t *s){return s && s->start_pending;}
bool probe_sound_deadline(const probe_sound_t *,uint32_t *);
#endif
