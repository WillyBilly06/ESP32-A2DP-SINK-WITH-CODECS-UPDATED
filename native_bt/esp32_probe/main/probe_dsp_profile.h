#ifndef PROBE_DSP_PROFILE_H
#define PROBE_DSP_PROFILE_H
#include <stdint.h>
typedef struct {uint32_t cycles[5],frames;} probe_dsp_profile_t;
/* The single diagnostic owner resets, processes <=128 frames and snapshots.
 * No publication to a concurrent reader and no real-time product integration. */
void probe_dsp_profile_reset(void);
void probe_dsp_profile_snapshot(probe_dsp_profile_t *);
unsigned probe_dsp_profile_sum_enabled(void);
#endif
