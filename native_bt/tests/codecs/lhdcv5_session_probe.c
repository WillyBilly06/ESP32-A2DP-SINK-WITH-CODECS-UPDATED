#include "dbt_lhdcv5.h"
#include <stdlib.h>
#include <stdint.h>

#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

int g_nzc_formula=0;
int g_pkframe=0;
typedef union { struct { size_t bytes; } h; uint64_t align; } header_t;
static size_t live, peak, calls, fail_at;
static dbt_lhdcv5_session_t session, other;
void *dbt_lhdc_test_alloc(size_t bytes) {
    ++calls;
    if (calls == fail_at) return NULL;
    header_t *h = calloc(1, sizeof(*h) + bytes);
    if (!h) return NULL;
    h->h.bytes = bytes;
    live += bytes;
    if (live > peak) peak = live;
    return h + 1;
}
void dbt_lhdc_test_free(void *pointer) {
    header_t *h = (header_t *)pointer - 1;
    live -= h->h.bytes;
    free(h);
}
EXPORT size_t probe_live(void) { return live; }
EXPORT size_t probe_peak(void) { return peak; }
EXPORT size_t probe_calls(void) { return calls; }
EXPORT void probe_peak_reset(void) { peak = live; }
EXPORT void probe_fail_in(size_t index) { fail_at = index ? calls + index : 0; }
EXPORT int probe_open(unsigned rate, unsigned use_other) {
    lhdc_dec_config_t config = {(lhdc_dec_sample_rate_t)rate, LHDC_DEC_BITDEPTH_24,
                                LHDC_DEC_FRAME_5MS, 2, 1536, 0};
    return dbt_lhdcv5_open(use_other ? &other : &session, &config);
}
EXPORT void probe_close(unsigned use_other) { dbt_lhdcv5_close(use_other ? &other : &session); }
