# DSP filter-inlining experiment

The current Os product kernel calls the same exact filter repeatedly. This
isolated experiment compares it with an Os build forcing that filter inline
inside the existing IRAM processing function. Arithmetic, coefficient ramps,
history/fraction carry, analysis, crossover/boost, limiter and flags are identical.
Inlining can change both call overhead and code/IRAM placement, so any timing
gain cannot be attributed to only one of those effects.

`-DspInlineCheck -AudioStageProfile -PipelineDiagnostic` selects the comparison.
It excludes nested stage/FIR comparisons and requires the existing isolated
entry, with controller/storage/product providers disabled and automatic return
to the pinned baseline. Receipt, cache and actual compiler definitions are
checked by image admission. The inline candidate is explicitly compiled at Os.
The product DSP policy remains unchanged; the target decision is recorded below.

The actual host runner passes 18 controlled reference/candidate PCM/full-state
pairs under ASan/UBSan. The prior compiler and internal-stage comparison modes
also pass after the extension. Actual CMake accepts the isolated configuration
and rejects five incompatible ones; Windows CLI rejects invalid combinations
before build/device access. These are not target timing or audio results.

A proposed broad peaking-filter simplification is not applied: among the 975
current EQ coefficient rows, 624 do not satisfy the assumed c1==c3 equality
(bass/treble use shelf designs). A generic rewrite based on that assumption
would change valid settings. No coefficients or filter topology are altered.

## Target decision

The [actual target result](DSP_INLINE_RESULTS_2026_11_04.json) passed all 18
reference/candidate pairs (36 records) with full PCM/state equality. Steady
work improved only about 1.2%; ramp-containing blocks were slightly slower.
The processing function grew from 1376 to 2625 bytes in IRAM, an additional
1249 bytes. This is not a useful enough trade for the current bottleneck, so
forced inlining remains diagnostic-only and the product Os kernel is unchanged.

The pipeline portion still failed before DMA playback and does not use the
forced-inline kernel. The baseline returned cleanly. No full output or
headroom pass follows from this experiment.
