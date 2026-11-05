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
