# Controlled DSP compiler comparison — 2026-11-04

The prior full-quality 192 kHz pipeline still failed before DMA playback. DSP
was its largest measured stage. This diagnostic builds the same integer DSP
source twice, with renamed public symbols and explicit final Os/O3 flags.
The image/source validator checks both compile commands.

After the pipeline retires its decoder and audio buffers, a core-1 task runs
18 controlled case pairs: 48/96/192 kHz, flags 9 and 13, three repetitions.
Each pair receives identical controls and 5120 stereo frames, including
full-width deterministic noise, S24-aligned samples and rail alternation.
Execution order alternates per 128-frame block. Every output sample and the
complete DSP state are compared after each block. Timer/idle sleeps, hashing,
comparison and logging are outside measured spans. Workspace is allocated once
after pipeline retirement and freed at completion; no live audio callback
allocates or waits. Speed is not an equivalence pass gate.

Both variants start at fixed_dsp_init defaults (volume 127/analysis enabled),
then ramp to EQ 3/-2/2, volume 50 and flags 9 (boost/crossover/analysis) or 13
(boost/full-range/analysis). The real pipeline starts muted; this benchmark is
a controlled compiler comparison, not an equivalent whole-pipeline startup
workload. Ramp-containing blocks round the 20 ms ramp upward to 128 frames;
steady sections start after that boundary. Timing is elapsed core cycles,
including interruptions. It is not concurrent core-0 render/decoder timing.

## Measured result

All 18 case pairs passed full PCM/state equality. Hashes repeated identically
across the three repetitions. Os was faster in every compared workload.
Values below are mean elapsed microseconds per frame at 240 MHz:

| Rate | Flags | Ramp Os | Ramp O3 | Steady Os | Steady O3 |
|---:|---:|---:|---:|---:|---:|
| 48000 | 9 | 11.79 | 12.26 | 9.41 | 10.20 |
| 48000 | 13 | 12.12 | 12.71 | 9.49 | 10.24 |
| 96000 | 9 | 11.68 | 12.14 | 9.12 | 9.92 |
| 96000 | 13 | 12.03 | 12.62 | 9.20 | 9.96 |
| 192000 | 9 | 11.53 | 12.07 | 9.02 | 9.83 |
| 192000 | 13 | 11.86 | 12.53 | 9.09 | 9.86 |

At 192 kHz/flags 9, Os used about 8.2% fewer steady cycles than O3. The Os/O3
process function sizes in this build were 1416/6832 bytes. The native build now
selects Os for fixed_dsp.c and retains O3 for conversion and the FIR resampler.
The selected-policy all-codec/product-I/O/security/RFCOMM image compiles and
passes offline source/image/layout admission; it was not flashed. This is a
flag selection supported by the declared paired workload; a complete
pipeline rerun remains required. Even the isolated Os DSP cost is above the
5.208 us total frame budget at 192 kHz, so compiler flags alone do not solve it.

Image SHA-256:
97fb9a5901bfa5e22844b825deb3f8ab9d0a6d89e98c1d4a1e3c04239f7602ec.
Source checkpoint 21b6ba2a. The exact image, build manifests, compiler commands
and serial capture are archived at the run identifier in the JSON record.
The diagnostic comparison used 4624 bytes of temporary workspace; its task
stack low-water mark was 5364 bytes. The original pipeline still failed its
throughput gate. Released 1.0.4-idf6.1 ota0 was selected and reached System ready
without a fatal boot marker. No physical sound/continuity or RF result is
claimed by the kernel comparison.

## Verification and next step

The actual comparison runner also passes host ASan/UBSan at Os/O3, with full
state/output checks and strict overflow handling. Host clocks/RTOS/heap are
stubs, and its timing fields are explicitly excluded as target evidence.

```powershell
python native_bt/tools/test_dsp_variants.py
powershell -File native_bt/tools/build_probe.ps1 -PipelineDiagnostic -AudioStageProfile -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir '<LHDC checkout>'
python native_bt/tools/deploy_audio_trial.py --pipeline-only --baud 230400
```

The next gate is profiling DSP internals on identical state/input, then testing
an exact integer optimization against this reference. Preserve coefficient
ramps, filter history/fraction carry, crossover/boost, limiter and analysis.
The released fallback remains selected. Full feature/security lifecycles,
maximum-rate real-time headroom, fractional-clock/sustained DMA playback,
controller/BLE/storage coexistence and an equivalent-workload SRAM advantage
remain unqualified. The master objective is incomplete.

[Image-bound raw results and comparisons](IDF61_DSP_VARIANT_RESULTS.json).
