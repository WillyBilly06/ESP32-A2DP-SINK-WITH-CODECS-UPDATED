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
The target image builds and passes offline source/layout admission. Hardware
equality and timing comparisons remain pending; no speedup or product selection
is claimed by this checkpoint.

```powershell
python native_bt/tools/test_fir_variants.py
python native_bt/tools/test_resampler_exact.py
powershell -File native_bt/tools/build_probe.ps1 -PcmLoans -FirCompilerCheck -AudioStageProfile -PipelineDiagnostic -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir '<LHDC checkout>'
```
