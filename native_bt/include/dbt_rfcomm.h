#ifndef DBT_RFCOMM_H
#define DBT_RFCOMM_H
#include "dbt_queue.h"

/* RFCOMM frame codec. The caller supplies whether this DLC has negotiated
 * credit flow control; only a nonzero DLCI UIH frame with P/F set has a credit
 * octet. Decode borrows the input and never mutates output on failure. */
typedef enum { DBT_RFCOMM_SABM=0x2f, DBT_RFCOMM_UA=0x63,
    DBT_RFCOMM_DM=0x0f, DBT_RFCOMM_DISC=0x43, DBT_RFCOMM_UIH=0xef } dbt_rfcomm_type_t;
typedef struct {
    const uint8_t *data;
    uint16_t size;
    uint8_t dlci,credits;
    dbt_rfcomm_type_t type;
    bool cr,pf,has_credits;
} dbt_rfcomm_frame_t;
uint8_t dbt_rfcomm_fcs(const uint8_t *data,uint8_t size);
dbt_result_t dbt_rfcomm_encode(const dbt_rfcomm_frame_t *frame,bool credit_flow,
    uint8_t *output,uint16_t capacity,uint16_t *written);
dbt_result_t dbt_rfcomm_decode(const uint8_t *wire,uint16_t size,bool credit_flow,
    dbt_rfcomm_frame_t *frame);
/* Role-aware command/response bit, only for non-UIH frames. For UIH use
 * command=true: its frame address C/R denotes the originating mux role. */
bool dbt_rfcomm_cr(bool mux_initiator,bool command);

/* One MCC item, borrowed from the UIH DLCI 0 information field. Type is the
 * six-bit command code; command is the MCC C/R, independent of frame C/R.
 * This bounded parser handles a single-octet MCC length (up to 127). */
typedef struct { const uint8_t *data;uint16_t size;uint8_t type;bool command; } dbt_rfcomm_mcc_t;
dbt_result_t dbt_rfcomm_mcc_decode(const uint8_t *data,uint16_t size,dbt_rfcomm_mcc_t *message);
dbt_result_t dbt_rfcomm_mcc_encode(const dbt_rfcomm_mcc_t *message,uint8_t *output,uint16_t capacity,uint16_t *written);
#endif
