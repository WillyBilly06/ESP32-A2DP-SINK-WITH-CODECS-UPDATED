#ifndef DBT_CREDIT_H
#define DBT_CREDIT_H
#include "dbt_acl.h"

typedef struct {uint16_t mtu,mps,credits;} dbt_credit_parameters_t;
typedef struct {
    const uint8_t *data;
    uint16_t size,sdu_length;
    bool first;
} dbt_credit_frame_t;
typedef struct {
    void *context;
    uint8_t *(*allocate)(void *context,uint16_t bytes);
    void (*release)(void *context,uint8_t *bytes);
    /* Borrowed completed SDU, host-owner callback, no reentrant mutations. */
    void (*sdu)(void *context,const uint8_t *bytes,uint16_t size);
} dbt_credit_hooks_t;
typedef struct {
    dbt_credit_hooks_t hooks;
    dbt_credit_parameters_t local,peer;
    uint8_t *partial;
    const uint8_t *transmit;
    uint16_t expected,received,tx_size,tx_cursor,pending_bytes,pending_grant;
    uint32_t sdus_received,sdus_sent,frames_received,frames_sent;
    bool active,failed,in_callback,tx_active,tx_first,pending_frame;
} dbt_credit_t;

/* Single-owner K-frame data path, Core Vol 3 A 3.4 and 10.1. No signaling,
 * CID/security setup, HCI transport, or automatic credit policy here. Init
 * after channel establishment/admission only. local.credits must describe
 * grants already sent to the peer. Init allocates no payload storage.
 * Channel manager MUST disconnect this channel when failed becomes true. */
dbt_result_t dbt_credit_init(dbt_credit_t *channel,dbt_credit_hooks_t hooks,
    dbt_credit_parameters_t local,dbt_credit_parameters_t peer);
void dbt_credit_close(dbt_credit_t *channel);
/* Payload excludes Basic L2CAP header; includes first-frame SDU length. Only
 * fragmented SDUs allocate, exactly their declared size, released on delivery.
 * Credit/MPS/MTU/length violations and OOM latch failure and release partials. */
dbt_result_t dbt_credit_receive(dbt_credit_t *channel,const uint8_t *bytes,uint16_t size);
dbt_result_t dbt_credit_add_peer_credits(dbt_credit_t *channel,uint16_t credits);
/* Stage one local grant; commit AFTER full signaling indication admission.
 * Cancellation grants no credits. Returning credits is an explicit capacity
 * decision by the owning channel manager, never automatic on packet arrival. */
dbt_result_t dbt_credit_grant_prepare(dbt_credit_t *channel,uint16_t credits);
dbt_result_t dbt_credit_grant_commit(dbt_credit_t *channel);
void dbt_credit_grant_cancel(dbt_credit_t *channel);
/* SDU storage is borrowed until the final commit or close. Peek copies one
 * complete K-frame to caller-owned transport storage; repeatable under BUSY.
 * Commit only after that complete frame is admitted, never per ACL fragment.
 * Caller preserves prepared bytes until HCI returns its borrowed buffer. */
dbt_result_t dbt_credit_tx_begin(dbt_credit_t *channel,const uint8_t *bytes,uint16_t size);
dbt_result_t dbt_credit_tx_peek(dbt_credit_t *channel,uint8_t *output,uint16_t capacity,uint16_t *size);
/* Borrowed scatter view for direct HCI fragmentation without an MPS-sized copy.
 * The first frame's two-byte sdu_length prefix is separate from data/size. */
dbt_result_t dbt_credit_tx_view(dbt_credit_t *channel,dbt_credit_frame_t *frame);
dbt_result_t dbt_credit_tx_commit(dbt_credit_t *channel);
#endif
