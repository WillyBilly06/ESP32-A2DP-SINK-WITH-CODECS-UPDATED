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
The product DSP policy remains unchanged while hardware evidence is pending.

The actual host runner passes 18 controlled reference/candidate PCM/full-state
pairs under ASan/UBSan. The prior compiler and internal-stage comparison modes
also pass after the extension. Actual CMake accepts the isolated configuration
and rejects five incompatible ones; Windows CLI rejects invalid combinations
before build/device access. These are not target timing or audio results.

A proposed broad peaking-filter simplification is not applied: among the 975
current EQ coefficient rows, 624 do not satisfy the assumed c1==c3 equality
(bass/treble use shelf designs). A generic rewrite based on that assumption
would change valid settings. No coefficients or filter topology are altered.
