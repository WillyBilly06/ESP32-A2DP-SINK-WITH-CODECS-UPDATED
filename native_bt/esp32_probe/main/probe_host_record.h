#ifndef PROBE_HOST_RECORD_H
#define PROBE_HOST_RECORD_H
#include <stdint.h>
typedef enum { PROBE_HOST_CLASSIC_VISIBILITY=1,PROBE_HOST_GAP_STATE=2 } probe_host_record_kind_t;
/* Immutable owner snapshot. Values are copied, never borrowed or formatted on
 * the protocol task. These diagnostic kinds are not A2DP codec/wire flags. */
typedef struct {uint32_t kind,value[5];} probe_host_record_t;
/* One immutable failure snapshot retains all original failed/parser fields. */
typedef struct {
    uint32_t fault,controller_status,pending_opcode,rx_rejected,rx_flow_failed_opcode;
    uint32_t at_ms,origin,result,handle,generation,link_present,cid,bytes,declared;
    uint32_t packet_type,event,pb,bc,code,identifier;
    uint32_t host_stage,host_packet_bytes,host_type,host_event;
} probe_host_failure_t;
#endif
