#ifndef PROBE_DSP_COMPARE_H
#define PROBE_DSP_COMPARE_H
#include <stdbool.h>
/* Finite task-context diagnostic, after the renderer/decoder have retired.
 * No I2S/radio/storage operation. Returns output/state equivalence, not speed. */
bool probe_dsp_compare(void);
#endif
