# Matched-feedback arithmetic measurement — 2026-11-06

This exact prototype has no measured speed benefit and is disabled in the subsequent buffer experiments. It is retained only as an opt-in diagnostic, not a selected performance fix.

For a stable section with c1 == c3, c1*x1 - c3*y1 can be combined. Same-sign int32 histories have a difference representable in int32; opposite signs retain the original two full-width products. Coefficients, saturation, all history words and carried Q28 fractions remain exact. Unmatched sections and unproven/custom coefficient tables retain the original path. The generated tables contain312 matching and 624 ordinary nonidentity sections; only the peaking band qualifies, not all three EQ bands.

The first signed33 C expression generated extra Xtensa multiply/sign work. A checked-overflow variant and a cheaper same-sign variant were inspected separately; source, assembly and failed receipts are retained privately. One test initially assumed every EQ table matched; the production dispatch already tested equality, and the overbroad test assumption was corrected.

Selected, ABBA and no-volume-reuse modes pass AddressSanitizer/UndefinedBehaviorSanitizer: 67040 blocks and 3833465 complete PCM/state frames per proven and conservative-fallback variant, thirteen rates, all routing flags, rapid controls and rails. An actual-helper/scalar comparison covers 1916928 histories across 936 nonidentity tables, including 638976 matching-section cases; 2001042 independent signed64 quantizer cases also pass. Existing volume-comparison regression passes. Actual build admission passes9 CMake, 19 image and 1 early CLI checks. These are host correctness checks, not target speed evidence.

## Live result

Owned image7b6438b6a124f9a5ad7c32e47d5009fdc980c752dfb2d4cd1367f379dd7e6a8d: native YouTube, LHDC192k/S24/stereo/fixed900, EQ2/8/12, sampled volume68, flags12. Commanded play12.157s with independent 14 s pause guard; post-pause collection retains source drain/tail. Live blocks alternate ABBA; each executes once. Only full128-frame steady, nonclipped blocks are timed. Clipped calls are separate; timing includes interrupts/preemption. CRC-verified timing rows are printed after playback stops.

| Core | Reference calls | Candidate calls | Five-product mean cycles | Candidate mean cycles | Observed change |
|---|---:|---:|---:|---:|---:|
|0|5225|5225|109276.92|109901.52|+0.57%|
|1|4209|4210|179487.56|182259.63|+1.54%|

Core0 minimum cycles104603/104109; Core1 minima106560/106648. Neighboring audio differs, and the elapsed measurements do not isolate arithmetic CPU time. There is no demonstrated benefit, so neither candidate nor recorder is enabled in the subsequent buffer image. Full-stream unfilled DMA frames remain444928, with zero recorded encoded drops, decode rejects, descriptor faults or input sequence breaks; this is a failing continuity observation.

The diagnostic adds160 BSS bytes,1304 flash-text bytes and384 flash-rodata bytes relative to7a1c4cbfc441; DRAM data and IRAM are unchanged. App/selector readbacks match, the original app is preserved, the owned VALID boot passes, and 37 existing cold codec cases pass. The first target compile failed because the new diagnostic had not selected GNU C11 for the SDK cycle helper; that diagnostic-only flag was added and the failed receipt retained.

No floating-point transform conversion, reduced quality, codec removal or main release follows from this experiment. The complete master goal remains open.
