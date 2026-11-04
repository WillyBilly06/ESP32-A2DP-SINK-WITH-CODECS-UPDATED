# Exact zero-phase FIR folding candidate

The fractional coefficient-delta optimization is measured separately in
[its report](FIR_DELTA_2026_11_04.md). The initial full-output deadline failure
also occurs with zero fractional phase, so it needs a separate optimization.

## Generated-table invariant

For each existing 24/48/96-tap bank, row zero has coefficient zero at index 0
and the final two indices. Around middle `N/2-1`, it has exact integer symmetry:
`c[k] == c[N-2-k]` for `1 <= k < N/2-1`. The generator verifies this property
and emits a flag. Coefficient values are unchanged. An absent/false flag or the
explicit unfolded/wide reference switches retain the scalar path.

Only zero phase with complete history and a supported tap count uses folding.
Paired history samples are summed in signed int64, retaining all 33 possible
bits. Their common coefficient is multiplied without intermediate rounding.
The center tap is included once. The original generated absolute-sum bound
also bounds each regrouped partial sum, so no int64 overflow is introduced.
Final Q30 shift and saturation remain identical. No taps or precision are
removed; their algebraically identical contributions are regrouped.

The helper is separate from the general loop to limit changes to fractional
code generation. Priming, history publication, phase advance and latency are
unchanged. There are no new allocations, task/queue resources or persistent
state fields. Stack/code costs and timing remain subject to target validation.

## Verification so far

The actual folded, checked-unfolded and original wide paths each pass 150000
128-bit-integer output/full-state oracle cases under ASan/UBSan, including
complete/random history, ring wrap, rails and phase boundaries. Table checks
verify the symmetry flag against every bank. The five-way runner also passes
27 case groups against both prior references, with zero/+500/-500 ppm and all
samples/state compared. A zero-initialized-state regression prevents unsupported
tap counts from entering the folded helper.

The target comparison contains O3 wide reference, checked Os/O2/O3 and O3 folded
zero-phase variants. Work order rotates per block/trial. The finite benchmark
deadline is 15 seconds, its owner deadline 23 seconds; serial collection allows
35 seconds/128 KiB for isolated codec diagnostics and still ends on verified
baseline boot. RF capture limits are unchanged. These are bounded diagnostic
capture allowances, not relaxed audio performance gates.

## Target rejection

The candidate passed all 27 five-way target equivalence groups, including
complete PCM/state equality. It was nevertheless slower at zero phase:
approximately 43% at 48 kHz, 37% at 96 kHz and 30% at 192 kHz across three
paired trials. Fractional-phase differences were small and inconsistent.
The wide pair sums did not produce efficient code on this target.

Folding is therefore disabled in the product path. Only the explicit diagnostic
wrapper defines `DBT_RESAMPLER_ENABLE_FOLDED_ZERO`. The measured checked-delta O3
implementation remains selected. [The rejected result](FIR_ZERO_FOLD_RESULTS_2026_11_04.json)
preserves the 135 target records and failed full-pipeline outcome. The released
baseline returned without fatal markers. Full DSP/I2S, maximum-LHDC headroom and
the complete feature/stress gates remain open.
