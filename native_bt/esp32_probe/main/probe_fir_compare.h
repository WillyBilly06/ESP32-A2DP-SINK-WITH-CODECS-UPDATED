#ifndef PROBE_FIR_COMPARE_H
#define PROBE_FIR_COMPARE_H
#include <stdbool.h>
/* Finite task-context comparison after decoder/render retirement. Returns
 * numerical/state equivalence; speed is reported separately, never a pass. */
bool probe_fir_compare(void);
#endif
