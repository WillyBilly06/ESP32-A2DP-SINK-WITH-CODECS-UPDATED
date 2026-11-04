# Exact DSP accumulation optimization — 2026-11-04

Nested profiling identified the biquads as the largest DSP cost. An opt-in
-DspStageProfile mode compares an uninstrumented Os reference with an Os kernel
using five CCOUNT stage markers. Compiler memory barriers bracket the reads;
marker overhead is included. A single diagnostic task owns all counters.
Instrumentation is disabled in product builds and adds no DSP state fields.

The CPU-only comparison runs after the pipeline retires its buffers, on core 1,
with the controller, I2S, filesystem and NVS inactive. Both variants start muted
like audio-open, apply the same EQ 3/-2/2, volume 50, flags 9 or 13 and receive
identical 5120-frame full-width/rail/S24-aligned signals. Every sample and the
complete DSP state are compared per 128-frame block. There are three trials at
48/96/192 kHz. These are kernel timings, not concurrent render-core guarantees.

## Arithmetic proof and compatibility

The old filter splits five products into integer/fractional contributions to
handle arbitrary coefficients without overflowing int64. The generated default
coefficient tables admit a cheaper exact accumulation. Across all rows and the
identity vector, the component-wise absolute bounds are:

[660040179, 1176472423, 530590634, 535795108, 267361804].
Their sum is 3170260148. Coefficient ramps are component-wise monotone between
current values and generated targets; interrupting/restarting a ramp does not
leave this global component box. For arbitrary int32 input/history and a prior
fraction below 2^28, the absolute bound on every partial sum is:

3170260148 * 2^31 + (2^28 - 1) = 6808081828004495359 < INT64_MAX.

The generator emits this bound with the table. The filter uses one int64 sum,
then the same floor shift, low-bit fraction carry and saturation/reset policy.
There is no coefficient, sample or output precision reduction, and no filter,
ramp, limiter, analysis or crossover/boost stage is omitted. The bound relies
on the existing init/controls coefficient invariant, not arbitrary mutation of
internal coefficient arrays. Unproven/custom tables, or a bound above the safe
threshold, compile the original general path. The diagnostic reference forces
that original path even when the default table carries the proof.

## Results

All 18 target reference/optimized case pairs passed full PCM/state equality.
The instrumented optimized stages at 192 kHz/flags 9 show the following mean
elapsed microseconds per frame, including marker overhead, after the ramp:

| Stage | Original split sum | Proven sum |
|---|---:|---:|
| Analysis | 0.388 | 0.385 |
| Ramps/input scale | 0.529 | 0.531 |
| EQ/compensation | 4.532 | 3.192 |
| Crossover/boost/makeup | 2.949 | 2.109 |
| Mix/volume/limiter/output | 0.814 | 0.811 |

The two filter groups improved from about 7.48 to 5.30 us/frame combined (29%).
The filter body compiled to 186 bytes versus 283 for the reference; no persistent
DSP RAM was added. The stage profile includes interruption/observer costs;
three finite case repetitions do not prove long-duration tail latency.

Original profiled image: 180a0ba088da566f21aad0c3a0a2ce0d347944baa2a17001523aa96df9b5b557
(source 0bb61c1c). Optimized image:
780b2eddd39ff81d29b1c8666e12f71ea0f8c73427742735cc6254ddb77fe37e
(source 501bf082). Exact tested binaries, input/build receipts, flags and serial
captures remain archived locally under the JSON run identifiers.

The full-quality pipeline still **fails**: 15 packets supplied, 13 retired,
12992 PCM frames delivered and 4864 rendered in 80060 us. Maximum active
128-frame render span was 1973 us (previously 2325), against a ~667 us budget.
DMA playback never started. No complete 96-packet oracle hash or continuity
result was obtained. Decoder spans also include diagnostic hashing, not only
production decoding. This is progress toward the maximum-rate requirement,
not a real-time guarantee.

The optimized run had 14328 bytes minimum internal free heap; task/render stack
remaining was 5364/4208 bytes. Buffers were reclaimed, no fatal boot marker was
observed, and released 1.0.4-idf6.1 ota0 reached System ready. Only COM6 was used.
The all-codec/product/security/RFCOMM image also compiles and passes offline
image/source/layout admission; it was not flashed in this checkpoint.

## Verification

- 200000 bounded biquad cases compare optimized, general-reference and an
  independent 128-bit oracle, including full int32 input/history.
- 100000 arbitrary-coefficient cases verify the unproven-table fallback against
  the reference and wide oracle.
- 106496 stereo frames at all 13 rates exercise rapid interrupted controls,
  all flags and full state/output equality. ASan/UBSan run without signed-
  overflow wrapping. Generated bounds are independently checked against data.
- All 179 portable tests, spectral DSP checks and O3 clock/resampler checks pass.
  Host results are numerical/model evidence, not hardware timing or audible-pop
  qualification.

```powershell
python native_bt/tools/generate_fixed_dsp_tables.py --check
python native_bt/tools/test_dsp_accumulation.py
python native_bt/tools/test_dsp_variants.py --profile-stages
powershell -File native_bt/tools/build_probe.ps1 -PipelineDiagnostic -AudioStageProfile -DspStageProfile -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir '<LHDC checkout>'
python native_bt/tools/deploy_audio_trial.py --pipeline-only --baud 230400
```

Next: reduce the remaining filter/state traffic and FIR integration cost using
exact reference comparisons, then rerun the unchanged maximum-quality fixture.
Do not mask the throughput failure with larger queues or reduced audio quality.
Controller/BLE/storage coexistence, complete feature/security lifecycles, long
fractional-clock/DMA operation and an equivalent-workload SRAM advantage remain
unqualified. The released fallback remains selected; the master goal is incomplete.

[Image-bound raw measurements and proof checks](IDF61_DSP_INTERNAL_RESULTS.json).
