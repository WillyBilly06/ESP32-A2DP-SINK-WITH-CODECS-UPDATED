#ifndef DBT_SOUND_H
#define DBT_SOUND_H
#include "dbt_queue.h"
#include <stddef.h>

/* All provider and storage calls are worker-only. Responses are complete BLE
 * payloads; the owner queues copied commands and checks peer generation before
 * delivering responses. The backend must never format or erase the filesystem. */
typedef struct { uint16_t handle; uint32_t generation; } dbt_sound_peer_t;
typedef struct {
    void *context;
    bool (*begin)(void *,uint8_t,uint32_t);
    bool (*write)(void *,uint32_t,const uint8_t *,uint16_t);
    bool (*seal)(void *);
    bool (*read_temp)(void *,uint32_t,uint8_t *,uint16_t);
    bool (*commit)(void *,uint8_t);
    void (*abort)(void *);
    dbt_result_t (*size)(void *,uint8_t,uint32_t *); /* OK, EMPTY, or error */
    bool (*read)(void *,uint8_t,uint32_t,uint8_t *,uint16_t);
    bool (*remove)(void *,uint8_t);
    bool (*set_mute)(void *,bool);
    uint32_t (*now_ms)(void *);
    bool (*peer_valid)(void *,dbt_sound_peer_t); /* synchronized owner view */
} dbt_sound_store_t;
typedef struct { uint8_t data[3],size; } dbt_sound_response_t;
typedef struct {
    dbt_sound_store_t store;
    dbt_sound_peer_t peer;
    uint32_t expected,written,crc,deadline;
    uint8_t type,sequence,exists,queued_type;
    uint32_t queued_rate;
    bool active,muted,playing,queued,queued_exclusive;
    _Atomic uint32_t upload_phase,owner_handle,owner_generation,identity_generation;
} dbt_sound_t;
typedef struct { uint32_t rate,data_offset,data_bytes,frames;uint16_t channels,bits,align; } dbt_sound_wav_t;

/* Owner serializes all calls; supplied peer generation must be nonzero. */
dbt_result_t dbt_sound_init(dbt_sound_t *,dbt_sound_store_t,bool initial_muted);
dbt_result_t dbt_sound_refresh(dbt_sound_t *);
uint8_t dbt_sound_status(const dbt_sound_t *);
dbt_result_t dbt_sound_command(dbt_sound_t *,dbt_sound_peer_t,uint8_t,const uint8_t *,uint16_t,uint32_t,dbt_sound_response_t *);
/* Host-safe, nonblocking cancel request. BUSY means commit already admitted;
 * its eventual completion must be suppressed for the disconnected generation. */
dbt_result_t dbt_sound_disconnect(dbt_sound_t *,dbt_sound_peer_t);
/* Worker-only; true reports an aborted timeout/cancellation and its owner.
 * The caller may send SOUND_FAILED(0x0b) only while that owner is still valid. */
bool dbt_sound_poll(dbt_sound_t *,uint32_t,dbt_sound_peer_t *);
dbt_result_t dbt_sound_wav(dbt_sound_t *,uint8_t,bool,uint32_t,dbt_sound_wav_t *);

/* Single producer worker, single consumer render. Start/destroy/rate-switch
 * require both owners quiescent, including absence of in-flight calls. */
typedef void *(*dbt_sound_alloc_fn)(void *,size_t);
typedef void (*dbt_sound_free_fn)(void *,void *);
typedef struct { void *context;bool (*start)(void *,bool);void (*stop)(void *); } dbt_sound_output_t;
typedef struct dbt_sound_play dbt_sound_play_t;
dbt_result_t dbt_sound_play_open(dbt_sound_t *,uint8_t,uint32_t,bool,dbt_sound_output_t,void *,dbt_sound_alloc_fn,dbt_sound_free_fn,dbt_sound_play_t **);
dbt_result_t dbt_sound_play_pump(dbt_sound_play_t *,uint16_t);
/* Publish to the render owner only after ready; short completed sounds qualify. */
bool dbt_sound_play_ready(const dbt_sound_play_t *);
/* Render-owner bound for standalone/virtual-base rendering. Never invent
 * samples while a live producer's ring is empty; EOF/stop permits fade/drain. */
uint16_t dbt_sound_play_mix_frames(const dbt_sound_play_t *,uint16_t);
/* Nonblocking cross-owner request. The worker pumps to acknowledge it and
 * the render owner mixes until drained; both owners then quiesce before close. */
void dbt_sound_play_request_stop(dbt_sound_play_t *);
/* No allocation, blocking, filesystem, decode, or logging in this hook. */
void dbt_sound_mix(dbt_sound_play_t *,int32_t *,uint16_t);
bool dbt_sound_play_drained(const dbt_sound_play_t *);
/* Stop/cancel and switch only after both worker and render have quiesced. */
void dbt_sound_play_close(dbt_sound_play_t *);
/* Worker-owned single pending slot. A later request replaces the older one. */
dbt_result_t dbt_sound_play_queue(dbt_sound_t *,uint8_t,uint32_t,bool);
bool dbt_sound_play_take_queued(dbt_sound_t *,uint8_t *,uint32_t *,bool *);
void dbt_sound_play_clear_queue(dbt_sound_t *);
#endif
