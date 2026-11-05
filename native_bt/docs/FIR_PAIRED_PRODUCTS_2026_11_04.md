# Diagnostic paired-product FIR experiment

The new comparison variant schedules stereo low products before stereo high
products using Xtensa `MULL` and signed `MULSH`. General registers are the only
assembly operands; coefficient interpolation and signed 64-bit accumulation
retain their existing arithmetic. The full product is reconstructed as signed
high word times 2^32 plus unsigned low word. Four early-clobber outputs keep
results separate from both samples and the coefficient.

`DBT_RESAMPLER_PAIRED_PRODUCTS` is defined only by `probe_fir_pair.c`, which is
part of the finite `-FirCompilerCheck` diagnostic. The normal renderer retains
the C products. This is not the rejected symmetric zero-phase folding change,
and changes no coefficient, tap count, precision or resampler state.

The actual comparison runner now contains six variants and rotates execution
order across 27 rate/drift/trial groups. Every sample, source consumption,
phase and full state are compared. Host ASan/UBSan passes all groups and
600000 independent wide-integer oracle cases across four arithmetic modes.
The host executes the paired variant's portable C fallback; it does not prove
the assembly's result or speed. The actual target compile contains the
requested two `MULL` instructions followed by two `MULSH` instructions.

The image validator requires the paired wrapper and final O3 compilation along
with the five existing reference variants. Actual-validator/mode tests pass.
The target image builds and passes offline source/layout admission. COM6 then
completed all 27 groups (162 variant rows), with exact PCM, phase, consumption
and state equality. The paired assembly is rejected for production: every
comparison is slower than current O3 C, by 23.59–31.33% in steady cycles.
At 192 kHz, the median additional cycles are 24.69% at zero drift, 23.66% at
+500 ppm and 28.04% at -500 ppm. Normal product selection remains unchanged.
These controller-off kernel timings do not establish RF or playback headroom.

[Source-bound results](FIR_PAIRED_PRODUCTS_RESULTS_2026_11_04.json) identify the
exact image, ELF and archived source receipt. The separate pipeline attempt
still fails after accepting 512 source frames; DMA never starts. Its accepted
prefix agrees with the complete target decoder reference, but that is not a
full pipeline correctness or continuity pass. The helper verified return to
the released baseline with no fatal boot marker.

```powershell
python native_bt/tools/test_fir_variants.py
python native_bt/tools/test_resampler_exact.py
powershell -File native_bt/tools/build_probe.ps1 -PcmLoans -FirCompilerCheck -AudioStageProfile -PipelineDiagnostic -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir '<LHDC checkout>'
```
