# LHDC maximum-rate profiling — 2026-11-04

The first COM6 maximum-rate checkpoint passed PCM checks but missed 35/84
isolated deadlines (4,838 us mean; 5,190 us maximum against a 5,000 us budget).
That measurement includes the self-test's complete FNV/peak/reference scans.
It cannot identify exclusive decoder cost or prove product/coexistence headroom.

The next finite diagnostic preserves that conservative deadline gate, adds
callback-subtracted estimates, and repeats the same 96 ordered encoded frames
three times with decoder close/reopen between trials. Each trial excludes the
first 12 frames from timing statistics and retains all 56 PCM reference checks.
Complete PCM FNV hashes are recorded for repeatability and later optimization
comparisons. A hash comparison supplements the independent PCM checks.

An opt-in generated-port hook records elapsed CCOUNT cycles for six existing
decoder channel stages: preamble, entropy, mantissa, dequantization/SNS, IMDCT,
and overlap. The single decoder task owns fixed-size counters; no allocation,
logging or locks occur in the hook. Statistics and serial output are produced
after decoding. Casts of timestamp differences to uint32_t preserve a single
counter wrap; each stage must be far shorter than a 32-bit CCOUNT period.

The decoder component uses GNU C11 to include the public ESP-IDF `esp_cpu.h`
inline-assembly APIs. No private hardware registers or direct assembly are added.
The normal port still removes stage timestamps unless explicitly enabled.

Counts and percentile scratch storage are capped at 96 samples. Tests execute
the actual collector/statistics code for inactive windows, reset/finish,
64-bit accumulation, bounds, canaries, percentile calculations and the exact
deadline boundary. Separate host tests retain startup admission, callback,
notification/timeout and core-0 baseline-return checks. Host tests are not
ESP32 timing evidence.

```powershell
./native_bt/tools/build_probe.ps1 -CodecDiagnosticOnly -LhdcMaximumCheck -LhdcStageProfile -WithLhdc -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir <sibling-decoder-repository>
python native_bt/tools/test_codec_diagnostic.py
python native_bt/tools/test_probe_codec_check_target.py
```

Actual deployment remains separately admitted on COM6. The helper now archives
each tested image, source manifest, build/configuration receipts, source commit
and raw boot/result capture under ignored `_build/hardware/runs/`, so later
builds cannot silently replace the evidence for a measured image.

Stage timing includes interrupts/preemption and profiler overhead. Subtracting
validation time still includes media-wrapper work and excludes the real
DSP/I2S/provider workload. No pass here establishes maximum-rate coexistence.
The unchanged 192 kHz/stereo/S24/1000 kbps configuration must eventually satisfy
the complete pipeline, control responsiveness and sustained-stress gates.

## Measured profile before the FAC-reader change

COM6 ran image
`8657d143491942e2a281a789d40f231012a79934d699b99ca3993089fde7beaa`
from source checkpoint `9ed5bd22`. All 37 existing configurations passed again.
Each of three maximum-rate trials passed 56 reference PCM checkpoints and
produced the same full-output FNV-1a hash, `4008775619` (92,160 stereo frames).

| Timing across the three trials | Mean per frame | Maximum per frame |
|---|---:|---:|
| Media including validation | 4,777–4,778 us | 5,101–5,113 us |
| Callback-subtracted estimate | 3,804 us | 4,119–4,131 us |
| Validation callback | 973 us | 985 us |

The conservative gate still failed, with 15, 11 and 14 deadline misses out of
84 measurements per trial. These are stable measurements, not a complete
pipeline result. Each trial generated 168 measured channel-stage records.

| Stage | Mean elapsed us per stereo frame at 240 MHz |
|---|---:|
| Preamble | 53.1 |
| Entropy | 1,597.8 |
| Mantissa | 606.2 |
| Dequantization/SNS | 195.8 |
| IMDCT | 835.6 |
| Overlap | 96.2 |

Entropy is the largest measured stage. Its initial FAC copy performs two
out-of-line bit-reader calls and variable 64-bit cache manipulation for each
byte. The candidate change retains the same scratch capacity and copies exact
bytes using a 32-bit residual carry after draining prefetched bytes. It changes
no codec arithmetic, framing, bitrate, sample rate, depth or security checks.

The actual unmodified bit reader and the new helper were compared over
3,565,921 cases: capacities and input lengths, every cache alignment, prefetched
states, complete residual-carry/input-byte transitions, reader-state equality
and output canaries. The same suite passed AddressSanitizer and
UndefinedBehaviorSanitizer. These are host correctness checks; target speed and
whole-decoder regression evidence must be gathered separately.

The profiled run returned cleanly to released Bluedroid firmware. Its complete
local artifact/receipts are retained in
`_build/hardware/runs/1793787196186963900-8657d1434919/`.
