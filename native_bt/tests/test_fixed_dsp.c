#include "fixed_dsp.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(expression) do { if (!(expression)) return __LINE__; } while (0)
static fixed_dsp_t dsp;
static const uint32_t rates[] = {16000, 22050, 24000, 32000, 44100, 48000, 88200, 96000, 176400, 192000};

EXPORT int test_fixed_dsp_flat_and_integer_boundaries(void) {
    CHECK(!fixed_dsp_init(&dsp, 12345, 90, 500, 512));
    for (unsigned rate = 0; rate < sizeof(rates)/sizeof(rates[0]); ++rate) {
        CHECK(fixed_dsp_init(&dsp, rates[rate], 90, 500, 512));
        fixed_dsp_controls(&dsp, 0, 0, 0, 127, FDSP_BYPASS, true);
        const int32_t samples[] = {0, 16, -16, 1048576, -1048576, 1234567808, -1234567808};
        for (unsigned i = 0; i < sizeof(samples)/sizeof(samples[0]); ++i) {
            int32_t left = samples[i], right = samples[i];
            fixed_dsp_process(&dsp, &left, &right);
            CHECK(left == samples[i] && right == samples[i]);
        }
    }
    return 0;
}

EXPORT int test_fixed_dsp_eq_transition_continuity(void) {
    for (unsigned rate = 0; rate < sizeof(rates)/sizeof(rates[0]); ++rate) {
        CHECK(fixed_dsp_init(&dsp, rates[rate], 90, 500, 512));
        fixed_dsp_controls(&dsp, 0, 0, 0, 127, FDSP_BYPASS, true);
        int32_t previous = 1 << 24;
        for (unsigned update = 0; update < 8; ++update) {
            int8_t gain = update & 1 ? -12 : 12;
            fixed_dsp_controls(&dsp, gain, gain, gain, 127, FDSP_BYPASS, false);
            for (uint32_t frame = 0; frame < rates[rate] / 20u; ++frame) {
                int32_t left = 1 << 24, right = left;
                fixed_dsp_process(&dsp, &left, &right);
                int64_t jump = (int64_t)left - previous;
                CHECK(jump < 21474836 && jump > -21474836); /* <1% FS per sample on DC */
                CHECK(left == right);
                previous = left;
            }
        }
        for (unsigned band = 0; band < 5; ++band) CHECK(dsp.filters[band].remaining == 0);
    }
    return 0;
}

EXPORT int test_fixed_dsp_controls_and_full_scale(void) {
    CHECK(fixed_dsp_init(&dsp, 192000, 90, 500, 512));
    uint32_t random = 0x12345678;
    for (unsigned frame = 0; frame < 80000; ++frame) {
        if (frame % 777 == 0) {
            fixed_dsp_controls(&dsp, (int8_t)((int)(frame % 25) - 12), (int8_t)((int)((frame/3) % 25) - 12),
                                (int8_t)((int)((frame/7) % 25) - 12), (uint8_t)(frame % 128),
                                (uint8_t)(frame % 16), false);
        }
        random = random * 1664525u + 1013904223u;
        int32_t left = (int32_t)random;
        random = random * 1664525u + 1013904223u;
        int32_t right = (int32_t)random;
        fixed_dsp_process(&dsp, &left, &right);
        CHECK(left < 2147483632 && left > -2147483632 && right < 2147483632 && right > -2147483632);
    }
    CHECK(dsp.limited > 0 && dsp.peak_q27 > 114085069);
    fixed_dsp_controls(&dsp, 0, 0, 0, 0, FDSP_BYPASS, false);
    for (unsigned frame = 0; frame < 8000; ++frame) {
        int32_t left = INT32_MAX, right = INT32_MIN;
        fixed_dsp_process(&dsp, &left, &right);
    }
    int32_t left = INT32_MAX, right = INT32_MIN;
    fixed_dsp_process(&dsp, &left, &right);
    CHECK(left == 0 && right == 0);
    return 0;
}

EXPORT int test_fixed_dsp_meter_and_db(void) {
    CHECK(fixed_dsp_init(&dsp, 48000, 90, 500, 512));
    fixed_dsp_controls(&dsp, 0, 0, 0, 127, FDSP_BYPASS | FDSP_ANALYSIS, true);
    for (unsigned frame = 0; frame < 24000; ++frame) {
        int32_t left = 1 << 28, right = left;
        fixed_dsp_process(&dsp, &left, &right);
    }
    CHECK(dsp.meter_levels[0] > (1u << 27) && dsp.meter_levels[0] <= (1u << 28));
    CHECK(fixed_dsp_level_db_q8(0) == -120*256);
    CHECK(fixed_dsp_level_db_q8(1u << 31) == 0);
    CHECK(fixed_dsp_level_db_q8(1u << 30) < -1530 && fixed_dsp_level_db_q8(1u << 30) > -1550);
    fixed_dsp_zero_analysis(&dsp);
    CHECK(dsp.goertzel_levels[0] == 0 && dsp.meter_levels[0] == 0);
    return 0;
}

/* Bulk entrypoint for a Python spectral oracle and transition fixtures. */
EXPORT int dsp_test_buffer(uint32_t rate, int8_t bass, int8_t mid, int8_t treble,
                            const int32_t *input, int32_t *output, uint32_t frames) {
    if (!fixed_dsp_init(&dsp, rate, 90, 500, 512)) return -1;
    fixed_dsp_controls(&dsp, bass, mid, treble, 127, FDSP_BYPASS, true);
    for (uint32_t i = 0; i < frames; ++i) {
        int32_t left = input[i], right = input[i];
        fixed_dsp_process(&dsp, &left, &right);
        output[i] = left;
    }
    return 0;
}
EXPORT uint32_t dsp_test_state_bytes(void) { return sizeof(dsp); }
