#ifndef PROBE_HOST_RECORD_H
#define PROBE_HOST_RECORD_H
#include <stdint.h>
typedef enum { PROBE_HOST_CLASSIC_VISIBILITY=1,PROBE_HOST_GAP_STATE=2 } probe_host_record_kind_t;
/* Immutable owner snapshot. Values are copied, never borrowed or formatted on
 * the protocol task. These diagnostic kinds are not A2DP codec/wire flags. */
typedef struct {uint32_t kind,value[5];} probe_host_record_t;
#endif
