/* Build beside the user's LHDC sources on Linux. Instrument allocation only;
 * the baseline and candidate execute their original float operations. */
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

typedef union { max_align_t alignment; size_t bytes; } allocation_header;
static size_t live_bytes, peak_bytes, allocation_calls;
static int fail_next;
static void *tracked_malloc(size_t bytes) {
    ++allocation_calls;
    if (fail_next) { fail_next = 0; return NULL; }
    allocation_header *header = malloc(sizeof(*header) + bytes);
    if (!header) return NULL;
    header->bytes = bytes;
    live_bytes += bytes;
    if (live_bytes > peak_bytes) peak_bytes = live_bytes;
    return header + 1;
}
static void tracked_free(void *pointer) {
    if (!pointer) return;
    allocation_header *header = (allocation_header *)pointer - 1;
    live_bytes -= header->bytes;
    free(header);
}
#define malloc tracked_malloc
#define free tracked_free
#define printf(...) ((void)0)
#include "lhdc_imdct.c"
#undef malloc
#undef free
#undef printf

void probe_release(void) {
    lhdc_imdct_free_480(); lhdc_imdct_free_960();
    lhdc_imdct_free_1920(); lhdc_imdct_free_cos();
    fail_next = 0;
}
int probe_reset(void) {
    probe_release();
    if (live_bytes != 0) return -1;
    peak_bytes = allocation_calls = 0;
    return 0;
}
int probe_ensure(int n) { return lhdc_cos_table_ensure(n); }
size_t probe_live(void) { return live_bytes; }
size_t probe_peak(void) { return peak_bytes; }
size_t probe_calls(void) { return allocation_calls; }
size_t probe_cos_allocation(void) {
    return s_cos_tab ? ((allocation_header *)s_cos_tab - 1)->bytes : 0;
}
void probe_fail_next(void) { fail_next = 1; }
void probe_reference(const float *in, float *out, int n) { lhdc_imdct_ref(in, out, n); }
void probe_fast(const float *in, float *out, int n) { lhdc_imdct_transform(in, out, n); }
int probe_init(int n) { return lhdc_imdct_init(n); }
void probe_force_reference(int enabled) { g_lhdc_diag_force_ref_imdct_960 = enabled; }

#ifdef DBT_SANITIZER_MAIN
int main(void) {
    static float coefficients[960], output[1920];
    const int sizes[] = {240, 480, 960, 1920, 480, 1920, 960, 240};
    for (unsigned k = 0; k < sizeof(sizes) / sizeof(sizes[0]); ++k) {
        int n = sizes[k];
        for (int i = 0; i < n / 2; ++i) coefficients[i] = (float)(i % 11 - 5);
        if (probe_ensure(n) != 0) return 1;
        probe_reference(coefficients, output, n);
    }
    probe_release();
    if (probe_live() != 0) return 2;
    for (int n = 480; n <= 1920; n *= 2) {
        if (probe_init(n) != 0) return 3;
        probe_fast(coefficients, output, n);
        probe_release();
        if (probe_live() != 0) return 4;
    }
    return 0;
}
#endif
