# Exact upper-word quantization experiment

The existing stable block filter uses native signed 32x32 multiply low/high
instructions. This candidate targets only the output range/fraction calculation
after the complete signed64 five-product sum. It does not alter multiplication,
coefficients, feedback, filter order, sample precision or carried Q28 fraction.

For `sum >> 28` to fit signed32, the signed upper word of the original sum must
fit signed28. The candidate tests that range using unsigned offset arithmetic,
then assembles the output from the upper word and the top four lower-word bits.
It retains the low28 fraction exactly when the original output would not clamp;
otherwise it produces the same signed saturation and zero carried fraction.
Ramping prefixes and the conservative split-sum fallback remain canonical.

`FIXED_DSP_WORD_QUANT` is off for the renderer. Only the diagnostic wrapper
`probe_dsp_word.c` enables it. The fourth isolated comparison slot now uses
this candidate in place of the rejected product-state prototype; old target
receipts retain their exact source/image and variant names. Os, O3 and the
current block kernel remain separate controls with identical input/state.

Local ASan/UBSan validation passes 26,624 blocks / 1,691,971 PCM sample frames
in each of the proven and split-fallback modes across 13 rates. An independent
signed-division/remainder oracle additionally checks 2,001,042 signed64 sums,
including saturation edges and both int64 rails. The ordinary block path is
also rerun unchanged. The actual four-variant comparison passes all 27 groups /
108 rows, and actual CMake/image-unit guards pass. Host timings are not used
as ESP32 performance evidence. Target timing remains the next gate.

## Target measurement and optional renderer connection

[The exact target receipt](DSP_WORD_TARGET_RESULTS_2026_11_05.json) records 108
matching PCM/state rows and a clean baseline return. At 192 kHz, median steady
full-EQ time falls from 6.022 to 5.413 us per PCM sample frame for flags9 and
6.095 to 5.485 for flags13, reductions of 10.1% and 10.0%. Flat-EQ time falls
from 3.168 to 2.981 us, or 5.9%. Median ramp-containing costs range from 0.30%
lower to 0.04% higher across the measured groups. The new helper is 274 bytes /
112 static instructions versus 306 bytes / 124 instructions, with the same
native multiply forms. Static counts are not dynamic execution counts.

The separate preliminary pipeline in that diagnostic still uses the scalar
default 64 ms path. It completes only 13 of 96 required packets, so the combined
harness exits with a failing pipeline gate even though the kernel comparison
passes. That failure cannot be attributed to a word kernel it did not select.
No continuous-playback claim follows from the isolated improvement.

`-WordDsp -BlockDsp` now optionally selects the exact kernel for a finite
copied/loaned renderer experiment. The original block kernel remains selectable
by omitting `-WordDsp`; the scalar/default and released Bluedroid paths remain
available. Receipt/cache/actual compilation flags bind both core units, and
`native_i2s.word_dsp` plus the pipeline I2S record expose the selected mode.

Actual block-render tests pass all 108 stream pairs against the scalar copied
reference under ASan/UBSan, including formats, +/-500 ppm, wrap, rapid controls,
OOM/cancellation and poisoned retired loans. Production output-provider tests
pass at 100/1000 Hz with the word mode and priority-order mode together. Eight
CMake configurations, sixteen image-admission cases and one early CLI rejection
pass, as do the existing block/finite guards. Renderer/pipeline target timing
and maximum-rate RF continuity remain separate, incomplete gates.

The next integrated synthetic trial uses the shared 1800-byte x8 decoder queue and two-step worker budget from the current RF probe, plus the explicit compact copied/16ms/word-block options. Earlier diagnostics used a separate 628-byte x2 queue and one-step budget; their results retain that scope. The new `pipeline_decoder_input` record exposes the actual reservation. Controller, BLE and storage remain absent, so this is not a full-product headroom test.

That [integrated trial](WORD_PIPELINE_2026_11_05.md) has now run and fails the
complete stream gate. DMA starts, but only four packets complete before the
encoded queue saturates. The very short prefix is not a continuity pass.
The [complete product candidate](WORD_PRODUCT_BUILD_2026_11_05.json) also builds
with all codec/product options, the requested priority ordering and word DSP.
Its source-matched image and limited callback-placement checks pass. Exact
binary/ELF/build inputs are archived; this product candidate has not been
flashed or qualified under live Bluetooth load.
