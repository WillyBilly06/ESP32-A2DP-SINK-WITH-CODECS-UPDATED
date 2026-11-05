# Exact word division for DSP analysis — 2026-11-05

## Source comparison

The complete owned fixed_dsp.c body, public contract, source generator, generated tables and native component build file were reviewed. Baseline sections read: dsp_processor.h:590–622,628–809; complete biquad.h; goertzel.h:125–174; config/app_config.h:54–57. The already pinned BTstack resampling/ESP32-output comparison has no equivalent full product EQ/limiter/analysis implementation. The working baseline supplies the feature/formula reference here; no BTstack implementation is integrated.

The baseline converts Q31 to float, performs EQ/volume compensation, optional full/split crossover/boost and soft clipping, and box-averages analysis toward 11025 Hz. The integer path retains these control roles with generated Q28 targets, full-width products, fractional carry, 20 ms ramps/crossfades and integer Goertzel/meter output. This mapping is source evidence, not acoustic/feature-parity qualification.

The actual bass-boost multiplier in baseline config is 1.0; no additional missing gain was established. EQ input scaling (bass 0.5, mid/treble 0.7), 150/1000/6000 Hz targets, 100 Hz volume compensation and +2 dB/150 Hz boost map to the generator. The generator clamps frequencies to 0.45*rate for low rates, while the baseline biquad formulas do not; this existing boundary difference remains explicit rather than claiming identical float/quantized output.

## Exact averaging proof

After d decimation samples, S is a sum of d signed Q31 mono samples. Init bounds d to a nonzero uint16 value; current rates use 1,2,3,4,8,9,16,17. Thus N=abs(S) <= d*2^31 <2^47. N's upper base-65536 part U=floor(N/65536) fits uint32.

Compute U=q_hi*d+r_hi, with 0<=r_hi<d. Then N=q_hi*d*65536+(r_hi*65536+low16(N)). The parenthesized second numerator is <d*65536<2^32. Therefore floor(N/d)=q_hi*65536+floor((r_hi*65536+low16(N))/d), using exactly two word divisions. The quotient is <=2^31. Restoring a negative sign in int64 preserves defined INT32_MIN behavior and the original truncation toward zero. The d=1 shortcut preserves S directly. Analysis sums/count/reset, output samples, analysis cadence and every coefficient/state remain unchanged.

No reciprocal approximation, floating-point arithmetic, table, persistent field or target-side allocation is added. This helper relies on the existing initialized/complete-window state contract; it is not a generic arbitrary-int64 divider.

## Generated data and build policy

The generator consistency check reproduces all thirteen rates and every table. Structural checks cover 975 EQ, 1664 compensation, 13 boost and 26 crossover biquads. Component maxima sum to 3170260148, matching FDSP_COEFF_BOX_BOUND and fitting the signed-64 partial-sum argument. Minimum generated endpoint Schur margin is 2152 Q28 units. These checks establish endpoint/accumulator bounds; interrupted ramp stability and acoustic behavior remain separate incomplete qualification. Tables are unchanged.

The component deliberately selects -Os for fixed_dsp.c; filter_steady_block has its own -O3 attribute. Existing comments record controlled target evidence for that split policy. FIR/audio conversion retain -O3. No blanket DSP -O3 change is made.

## Evidence and remaining costs

Strict C11 Clang analysis passes for scalar, word-block and conservative split-sum variants. ESP-IDF6.1 ccache compilation and source stability pass. Xtensa analysis now uses two QUOU instructions instead of the per-decimated-sample __divdi3 call. The separate Goertzel-window normalization and high-level soft-limiter __udivdi3 paths remain; this is not removal of every wide division.

Analysis text changes 751→801 bytes; its compiler frame remains 48. Whole text changes 866919→866971, entirely in flash. IRAM 122255, DRAM data 23893, aggregate data 396577 and BSS 50300 remain unchanged. No codec/MTU/buffering/affinity settings change. No elapsed-time, CPU utilization, waveform or underrun improvement is established by these instruction/size results.

Flash/readback and ordinary startup evidence are recorded separately. Built-in codec checks do not qualify this analysis path, difficult-signal/rapid-EQ behavior or sustained maximum-LHDC headroom. No unit, playback, timing, pairing, app or packet-injection campaign is invoked under the user's hold. Full DSP/audio/stack qualification remains unfinished.
