#ifndef PROBE_HOST_RECORD_H
#define PROBE_HOST_RECORD_H
#include <stdint.h>
typedef enum { PROBE_HOST_CLASSIC_VISIBILITY=1,PROBE_HOST_GAP_STATE=2,PROBE_HOST_MEMORY_SERVICES=3,PROBE_HOST_A2DP_DELAY=4,
    PROBE_HOST_A2DP_JOB=5,PROBE_HOST_A2DP_STATE=6,PROBE_HOST_A2DP_HANDOFF=7,PROBE_HOST_CCCD=8,PROBE_HOST_SMP=9 } probe_host_record_kind_t;
/* Immutable producing-task snapshot. Values are copied, never borrowed or formatted on
 * the protocol task. These diagnostic kinds are not A2DP codec/wire flags. */
/* Seven values still fit the existing32-byte media trace payload. */
typedef struct {uint32_t kind,value[7];} probe_host_record_t;
/* One immutable failure snapshot retains all original failed/parser fields. */
typedef struct {
    uint32_t fault,controller_status,pending_opcode,rx_rejected,rx_flow_failed_opcode;
    uint32_t at_ms,origin,result,handle,generation,link_present,cid,bytes,declared;
    uint32_t packet_type,event,pb,bc,code,identifier;
    uint32_t host_stage,host_packet_bytes,host_type,host_event;
} probe_host_failure_t;
/* Copied controller properties; live heap values are collected by logger. */
typedef struct {
    uint32_t hci_version,manufacturer,host_bytes,runtime_bytes;
    uint32_t classic_acl_bytes,classic_acl_packets,le_acl_bytes,le_acl_packets,shared_acl_pool;
    uint8_t address[6],padding[2];
    uint32_t reserved[5];
} probe_host_controller_t;
#endif
