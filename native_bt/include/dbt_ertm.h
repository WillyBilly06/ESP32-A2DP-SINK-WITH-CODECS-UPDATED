#ifndef DBT_ERTM_H
#define DBT_ERTM_H
#include "dbt_queue.h"

#define DBT_ERTM_SEQUENCE_MODULUS 64u
#define DBT_ERTM_MAX_MPS 65529u
typedef enum {DBT_ERTM_RR,DBT_ERTM_REJ,DBT_ERTM_RNR,DBT_ERTM_SREJ} dbt_ertm_supervisory_t;
typedef enum {DBT_ERTM_UNSEGMENTED,DBT_ERTM_START,DBT_ERTM_END,DBT_ERTM_CONTINUE} dbt_ertm_sar_t;
typedef struct {
    const uint8_t *data;
    uint16_t size,sdu_length;
    uint8_t tx_seq,req_seq,sar,supervisor;
    bool supervisory,poll,final;
} dbt_ertm_frame_t;
typedef enum {DBT_ERTM_WIRE_OK,DBT_ERTM_WIRE_BAD_FCS,DBT_ERTM_WIRE_INVALID} dbt_ertm_wire_result_t;
uint16_t dbt_ertm_crc(uint16_t seed,const uint8_t *bytes,uint16_t size);
/* Enhanced 16-bit control only. Payload excludes Basic header; optional FCS
 * covers the inferred Basic header, control, SDU length and data. */
dbt_result_t dbt_ertm_frame_encode(uint16_t cid,bool fcs,const dbt_ertm_frame_t *frame,
    uint8_t *output,uint16_t capacity,uint16_t *written);
/* Preserve output on error/checksum failure. Negotiated MTU/MPS, sequence and
 * SAR session validation belong to the channel engine, not this wire parser. */
dbt_ertm_wire_result_t dbt_ertm_frame_decode(uint16_t cid,bool fcs,const uint8_t *bytes,
    uint16_t size,dbt_ertm_frame_t *frame);
#endif
