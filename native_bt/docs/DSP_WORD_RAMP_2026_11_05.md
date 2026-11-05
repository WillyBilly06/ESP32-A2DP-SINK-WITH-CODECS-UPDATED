# Exact word quantization in the scalar ramp path

The previous optimization covered steady block filtering only. The integrated
pipeline still spends most of its short run in the canonical scalar prefix
while EQ/compensation/boost coefficients and volume ramp. This candidate reuses
the same full-width quantizer in that scalar filter under a separate private
`FIXED_DSP_WORD_RAMP` macro. It does not change coefficient advance, ramp length,
filter order, precision, clipping policy or filter-history updates.

The shared helper is available before the scalar filter, but ordinary scalar,
original block and the existing playback `-WordDsp` paths keep their previous
selection. Only the fourth diagnostic wrapper currently enables the ramp macro.
Its target record name is `Os_word_ramp_block`; earlier `Os_word_block` target
receipts keep their exact source/image scope.

The block test explicitly undefines both word macros in its scalar reference
translation unit. Thus the candidate and oracle do not accidentally share the
new scalar calculation. Host ASan/UBSan checks pass 26,624 blocks / 1,691,971 PCM
sample frames in each proven/fallback mode across13 rates, plus2,001,042 sums
against the independent division/remainder oracle. The actual four-way runner
passes27 groups /108 rows; the previous steady-only word mode also passes.

These are correctness observations, not a speed claim. On-device ramp timing
and any playback integration remain the next gates. The complete maximum-LHDC
pipeline and simultaneous Bluetooth/BLE workload remain unqualified.
