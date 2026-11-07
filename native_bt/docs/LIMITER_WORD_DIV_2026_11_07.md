# Exact word-sized limiter division — 2026-11-07

The rational DSP soft limiter remains unchanged:

    output = T + floor(C * excess / (C + excess))
    T = 114085069; C = 134217728 - T

For every clipped magnitude this equals134217728 - ceil(C*C/(C+excess)). The numerator of the latter division is constant. The owned implementation uses normalized base-2^16 quotient estimates with a bounded one-or-two correction, followed by exact remainder-to-ceiling adjustment. All intermediate word wrapping is intentional unsigned C arithmetic. This follows the CC0/public-domain narrowing-division correction documented in [libdivide's divlu source](https://github.com/ridiculousfish/libdivide/blob/master/doc/divlu.c), specialized to the proved limiter bounds.

The normalized divisor has1..7 leading zero shifts, is nonzero and below2^31, and the quotient fits a word. No retry loop, lookup table, dynamic allocation, task, queue, coefficient, gain, threshold, filter state, buffer size or scheduling policy is added or changed. The full-width reference remains available for host comparisons through FIXED_DSP_FORCE_WIDE_LIMITER. The default firmware selects the exact word implementation.

On original ESP32, an ordinary C quotient followed by multiply/subtract was compiled into both QUOU and REMU. A small pure inline QUOU instruction prevents that redundant remainder division; the remaining multiply/subtract has the same unsigned word semantics. Other targets retain unsigned C division. The final linked helper has exactly2 QUOU, zero REMU, zero calls and no backward branch. The installed official esp32_rev300_rom.elf confirms the old applicable ROM __udivdi3 path uses2 QUOU +2 REMU. This is an instruction/implementation observation, not a cycle-speed guarantee.

## Correctness and footprint

Experiment: native_bt/_build/experiments/limiter-word-div-1794058558429.

- Exhaustive original rational-curve comparison over all2,033,398,579 clipped magnitudes, including magnitude2^31.
- 1,000,304 signed complete-state cases per run, including INT32_MIN, both threshold sides, sign symmetry, peak tracking and counter wrap; the quick run uses ASan/UBSan without signed-wrap assumptions.
- Full scalar/block comparison against the forced original limiter:8,806,258 PCM frames across13 rates, coefficient/control ramps, analysis boundaries and flags; complete DSP state matches.
- Independent full-width finalizer oracle:1,848,830 channel samples over16,384 blocks;2,001,042 signed64 word-quantization cases also pass.
- Complete ccache firmware build and offline admission pass. Candidate389514bc25464678621e0bd2e31b9699aa7acb70e55c9a560a1f76723c2a8e2a,1,338,560 bytes.
- Relative to8e85: instruction SRAM+112 bytes; data/BSS/flash text/rodata deltas are zero. There is no new persistent data state, but the instruction SRAM cost is real. The linked helper compiler frame is32 bytes; this is not a live stack-watermark measurement.

The full DSP comparison's reference explicitly retains the original wide divider. The independent finalizer oracle still computes the original rational expression rather than deriving expected values from the candidate.

## Pre-candidate live baseline

Two guarded native YouTube metronome runs on verified8e85 used LHDC192k/S24/stereo, selected fixed900, phone150/150 and restored90/150. Core0 helper elapsed cycles per frame were584.56 and573.24, including preemption, startup and drain. Refill unfilled totals were13,440 and6,144, also including startup/drain. The two runs differ, so one lower post-change sample is insufficient to establish a general speed or continuity improvement. Detailed per-stage CRC-bearing rows and quiet before/after helper counters are preserved.

The baseline still has shortages after the earlier decoder gain correction. This optimization preserves the limiter and does not assert it caused every shortage. At document creation the candidate is not flashed; device timing, LDAC regression, sustained coexistence and full maximum-bitrate qualification remain open.

Reproduction (host tests only):

    python native_bt/tools/test_dsp_limiter_word.py
    python native_bt/tools/test_dsp_block.py --word-ramp --volume-ramp

## Guarded candidate deployment and finite trace

Candidate389514bc2546 is now flashed onCOM6 at921600 with full image readback at115200. Owned VALID, no automatic Bluedroid restore, preserved original app, all37 cold cases pass. Cold stack low3464 bytes versus3608 on8e85; task allocation is unchanged and the difference is not attributed to one function.

One complete fixed900/192k/S24/max-volume native YouTube run records575.29 elapsed Core0 helper cycles/frame and15,296 unfilled frames including startup/drain. This lies within the earlier573.24–584.56 timing range; there is no demonstrated whole-pipeline speed gain or continuity fix. Its decoder has zero rejects/nonfinite/concealed frames; ordinary finite sample clipping remains. A second repeat is incomplete after the logging service stopped and is not a pass. Playback was independently verified paused and volume90/150. The logger was relaunched through its persistent launcher, is listening again and has fresh UART.
