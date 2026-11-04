# Render profiling and exact optimization — 2026-11-04

The previous maximum-rate fixture could construct all buffers but missed the
192 kHz render budget. This checkpoint adds an opt-in `-AudioStageProfile`
(requires `-PipelineDiagnostic`) and measures conversion/resampling, integer
DSP, controls and output bookkeeping separately with CCOUNT. Each render unit
remains bounded to 128 frames. Counters publish once per block, not per sample;
final totals are read after renderer ownership is retired and include any
stop/drain processing. They are elapsed core cycles, including interruptions.

The pipeline's hot failure check now reads only the failure atomics. Full stats
and a stack scan are no longer performed on every decoder work unit. The source
receipt/validator checks profiling flags in both main and the actual audio-core
compile command. SDK CPU helpers require GNU asm, so only the diagnostic audio
translation unit uses GNU C11; the portable core otherwise retains strict C11.

## Device evidence

Both builds use the same true 192 kHz / S24 / stereo / 1001600-bit/s fixture,
ring capacities, gains (3/-2/2), volume 50, control flags and CPU affinities.
Neither run initialized the controller, filesystem, NVS or product providers.

| Elapsed stage cost per rendered frame | Original Os | O3 + exact zero-remainder path |
|---|---:|---:|
| Conversion/resampling | 10.67 us | 5.05 us |
| Integer DSP | 12.47 us | 13.20 us |
| Profiled frames, including stop/drain | 2432 | 3328 |
| Maximum active 128-frame render span | 3126 us | 2478 us |

The optimized image is SHA-256
7f4c6fd94fb724bfc431d78894a966d4c46de7b61f059172115f42e606232593,
source checkpoint 4d0a002a. The initial profiled image is
 a39038cb03bbe8dc0b3c3f7d63385dbd9fef07e12fef88662319e3d5a8a580be,
source checkpoint 3f8503e4. Exact images, source/build manifests, flags and serial
receipts remain archived locally under the run identifiers in the JSON record.

These are observed finite startup prefixes, not repeated steady-state
comparisons. The data demonstrates a conversion-path improvement, and does not
establish a DSP speedup or isolate compiler flags from the arithmetic change.
The next comparison must run Os and O3 DSP on identical input/state, separating
initial coefficient/volume ramps from steady processing, before selecting flags.

The optimized run still **fails**: 13 packets supplied, 11 retired, 11328 PCM
frames delivered and 3200 rendered in 70062 us. One encoded queue drop occurred;
there were no worker faults or PCM timeouts. The 128-frame budget is about
667 us, so 2478 us remains too slow. DMA playback never started. Zero DMA
underruns in these runs are not playback/continuity evidence, and the full
96-packet PCM hash was not obtained.

Minimum internal heap was 14352 bytes; largest block after construction 10240.
Core-1/render stack remaining was 5380/4208 bytes. Decoder/audio buffers were
reclaimed and released 1.0.4-idf6.1 ota0 reached System ready without a fatal boot
marker. The full product candidate was built/admitted offline, not flashed.

## Exactness and regression checks

The FIR change skips only a mathematically zero coefficient interpolation when
its remainder is zero. It retains every FIR tap and the original filter
response. The three integer render files use targeted O3; other SDK settings
and task priorities/affinities remain unchanged.

- 150000 cases compare the actual O3 FIR output and complete state with an
  independent 128-bit integer convolution/interpolation oracle under ASan and
  UBSan. Every tap length is exercised with zero, bank-aligned, maximum and
  random phases, full-width inputs and varied priming/history states.
- O3 paged-ring boundary/mixer/comparison checks pass under ASan/UBSan, including
  a run without signed-overflow wrapping. Full-scale DSP, EQ transition,
  integer boundary, controls and meter checks also pass at O3 with strict
  overflow handling. These are host digital checks, not acoustic evidence.
- All 179 portable tests pass. O3 clock/resampler/spectral tests pass with the
  previous reported gain/phase/drift results. All-codec/product/security/RFCOMM
  compilation and offline image/layout/source admission pass.

```powershell
python native_bt/tools/test_resampler_exact.py
python native_bt/tools/test_audio_pages.py --opt-level 3
python native_bt/tools/test_audio_clock.py --opt-level 3
powershell -File native_bt/tools/build_probe.ps1 -PipelineDiagnostic -AudioStageProfile -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir '<LHDC checkout>'
python native_bt/tools/deploy_audio_trial.py --pipeline-only --baud 230400
```

[Image-bound results](IDF61_RENDER_PROFILE_RESULTS.json) preserve both measured
runs and validation scope. Maximum-rate real-time headroom, DMA continuity,
longer fractional-clock operation, RF/BLE/storage coexistence, complete feature
parity and a full-product SRAM advantage remain unqualified. The released
fallback remains selected. The master objective is incomplete.
