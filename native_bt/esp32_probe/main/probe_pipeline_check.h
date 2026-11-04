#ifndef PROBE_PIPELINE_CHECK_H
#define PROBE_PIPELINE_CHECK_H
#include <stdbool.h>
/* Finite controller-off diagnostic. Caller must be the admitted core-0 return
 * owner. The core-1 task remains alive until reset to preserve wake contexts. */
bool probe_pipeline_check(void);
#endif
