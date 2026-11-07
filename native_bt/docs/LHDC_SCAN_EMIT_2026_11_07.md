# LHDC stereo S32 scan/emission fusion

The retained implementation measures about 1.8–1.9% lower whole-decoder elapsed
time in two short 192 kHz/S24/fixed900 YouTube runs. PCM and quality regressions
pass. This is a small decoder improvement; LHDC render underruns remain.

## Implementation and ownership

The previous frame path scanned scaled floats for quality, then traversed the
same inputs again to emit stereo S32 PCM. The new helper computes signed scaled
values once, collects quality magnitudes, and emits into existing private MDCT
scratch. That storage is disjoint from the left/right floating inputs and stays
unpublished until decode returns. No allocation or PCM ownership boundary moves.

The shortcut is restricted to stereo S32 with a positive finite scale, in the
decoder's normal floating-point environment. Mono, S16 and exceptional scales
retain the generic scan/output path. The original signed-product saturation
checks remain. Peak, clip, nonfinite and concealment decisions are reported;
concealment still overwrites the entire private PCM frame with silence and clears
both complete overlap buffers. Integer EQ/DSP, gain normalization, codec
negotiation, packet geometry, task priorities and core affinities are unchanged.

The target helper disables GCC jump threading locally, keeping the component's
other O3 options. This avoids duplicating rare rail/NaN branches. Standalone
Xtensa code comparison gives 554 bytes normally versus 230 without threading;
the actual full-image helper is 218 bytes and contains no function calls.

## Fidelity and build checks

- All 4,294,967,296 binary32 conversion patterns match the independent original
  saturation macro; an additional 1,000,000 patterns run under sanitizers.
- 36,000 actual frame/output/quality/overlap/canary cases pass in each test mode,
  including NaNs, infinities, rails, late concealment and exceptional scales.
- 48 independent C99-encoder/full-PCM comparisons pass across four rates, two
  depths, two amplitudes and bitrate/transition cases.
- 80 real-codec deferred cases preserve every PCM byte and timestamp, including
  all 37 existing configurations, backpressure, batching/fragmentation and the
  existing maximum LHDC fixture. No boot cases or expected PCM points change.
- All builds use ccache. Full IDF6.1 build, source stability, storage and image
  admission pass. Earlier failed IRAM/storage/environment receipts are retained.

The retained image is
`9283f7559e3333c7af43d5fe832060a813bfd2eb0a8595c4a53c3b9425101f2e`.
Relative to ec89367e, instruction RAM increases 92 bytes, flash text 288 bytes,
and data/BSS/rodata are unchanged. No task, queue or buffer is added.

COM6 upload 921600/full written-region readback 115200 match. Owned boot is
VALID, the original application is preserved, and automatic Bluedroid fallback
is false. All 37 cold codec cases pass: 173 callbacks, tracked peak 56899 bytes,
minimum free after allocation 102996 bytes and stack low-water 3504/8192 bytes.
Sampled runtime decoder stack low-water is 5904 bytes versus 5920 before. Initial
runtime internal free heap remains 59648 bytes; later codec-switch snapshots
differ slightly, so this is not a leak/fragmentation qualification.

## Physical measurements

Approved Redmagic, native YouTube 120 BPM metronome, 192 kHz/stereo/S24 in S32
containers, fixed900 setting, unchanged EQ 11/8/12. Phone 90 maps to observed
DSP 76; phone 150 maps to127. Each trace has an 11-second command interval and an
independent 14-second pause guard. Playback is paused and volume 90 restored.

| Run | Whole decoder mean per attempted unit | Unfilled PCM frames |
| --- | ---: | ---: |
| Baseline ec89367e, volume 90 | 2166.93us | 12672 |
| Rejected word-conversion fusion | 2228.29us | 49408 |
| Retained compact FPU fusion, volume 90 | 2125.76us | 12288 |
| Retained repeat, volume 90 | 2127.78us | 15488 |
| Retained maximum volume150 | 2104.99us | 11264 |
| Retained LHDC after LDAC switch | lifecycle check only | 16640 |

Whole-decoder means use elapsed-counter differences divided by actual LHDC
decode attempts, including preemption and wrapper/resume work. These are short
sequential observations, not sustained1Mbps/headroom or acoustic qualification.
Counts include startup/drain; remaining in-play starvation is independently
captured with decoded input waiting for processed output. No queue-full,
decode-reject, nonfinite/concealed frame, descriptor or refill-contract failure
was observed in the retained LHDC traces.

The raw profiling field `scale_scan` now contains fused S32 emission work;
`output_tail` contains the remainder. Its apparent reduction alone is not a
speed result. Their combined mean changes from 328.65 µs to 251.86/252.13 µs;
whole-decoder time above is the end-to-end comparison.

Sampled LHDC packets remain 566 bytes with one 960-sample unit: 562 coded bytes,
899200 coded bits/s. No live 1 Mbps claim is made. A finite LDAC96/S32 regression
after the changes has 1361152 source PCM frames and zero I2S shortages; sampled
LDAC frame counts still vary, so sustained fixed990 is not proven. Returning to
LHDC192/S24/fixed900 also succeeds.

## Rejected variants and retained evidence

The first fused helper overflowed IRAM by 496 bytes. A sign-restoration version
overflowed by 116 bytes and exposed out-of-line four-byte memcpy calls. A
call-free word converter fit but measured slower, so its arithmetic is removed.
Plain FPU fusion overflowed by 212 bytes; the local compiler-option change fits.
All failed variants, disassembly, source copies, host checks, raw UART traces,
volume restoration and physical deployment records remain under
`native_bt/_build/experiments/lhdc-scan-emit-1794071349508/`.

Retained physical archive:
`native_bt/_build/hardware/runs/1794073332240019000-9283f7559e33/`.
[Compact results](LHDC_SCAN_EMIT_RESULTS_2026_11_07.json) bind image, configuration,
memory, timing and regression receipts. Completed verified AMD64 compiler
intermediates and hash-verified, reproducible C99 tone inputs were reclaimed for
the local storage gate. The generator, reproduction arguments, input/PCM hashes,
source, executables, results, Xtensa objects and all physical captures remain
preserved.

## Remaining work

The actual compiled FreeRTOS notification path calls its yield predicate with
equal-priority yielding disabled (`tasks.c:5995`); the remote-core comparison
requires strictly higher priority (`tasks.c:853`). Host and Core 1 renderer are
both 22, with a 1 ms tick and 666 µs DMA period. Consequently, a notification alone
does not guarantee immediate renderer service ahead of an equal-priority host.
This is a verified scheduling rule and a plausible next lead, not proof of the
cause of every shortage. Audit supported wake/yield behavior and model this
equal-priority interleaving before changing the scheduling policy.

Full 1 Mbps LHDC, sustained BLE/Classic coexistence, all-feature/security/OTA,
connection/lifecycle stress, long-duration and acoustic qualification remain
open. This image is a development candidate, not a production release.
