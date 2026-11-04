# ESP-IDF 6.1 target codec checkpoint — 2026-11-04

This page retains the initial target results. The later
[stage-profile/optimization checkpoint](IDF61_LHDC_PROFILE_2026_11_04.md)
supersedes its maximum-rate isolated timings for the explicitly identified new
images; the full master objective remains incomplete.

This is isolated algorithm and control-flow evidence, not a production host,
RF/coexistence, full audio-chain or acoustic qualification. The released
Bluedroid application remains the default and is preserved in `ota_0`.

## Physical admission and restoration

The user released COM6 for flashing/testing in this conversation. Fresh reads
confirmed ESP32-D0WD-V3 revision 3.1, 8 MiB flash, the canonical partition table,
and released `1.0.4-idf6.1` in `ota_0` (ELF SHA-256
`9bb5c05b24b7d76530755ffd71c815a3d59f3d44852ab9b2f783f2c0881c938d`).
A fresh full-flash backup is retained locally, SHA-256
`15dbc29ae3581432d704f810e31262b8093ad5b8a01effa46ee30971a3819c40`.
The backup contains private persistent data and is not committed.

The physical bootloader passed checksum/digest checks. Every loaded segment byte
matches the rollback-enabled released build except its documented 24-byte
compile date/time descriptor field. This was verified rather than assuming that
the bootloader produced by a later local build was already flashed.

The codec-only entry point initializes no Bluetooth controller, NVS, filesystem,
product providers or I2S. It requires the pinned baseline identity and a pending
trial in `ota_1`. It never marks the diagnostic valid. A core-0 task performs
the bounded startup wait and selects the baseline on codec success or failure;
bootloader rollback remains available after an unexpected reset.

Only `ota_1` and the older selector sector were written. Full candidate readback
and byte-for-byte preservation of `ota_0` passed. The first run produced a clean
return marker followed by an observed baseline boot and `System ready`.

## 37-configuration target result

All 37 selected configurations passed on the ESP32 with the 6.1/GCC 15 build:
16 Opus configurations plus SBC, AAC mono/stereo, aptX/HD/LL variants, LDAC and
LHDC rate/depth cases. These tests check actual decoder output, frame counts,
format, reference samples or exact hashes as appropriate, allocation retirement
and stack floor. The historical 19/37 target failure does not describe this image.

- 173 PCM callbacks; all selected configurations completed.
- Maximum tracked decoder allocation: 63,672 bytes including allocation headers.
- Minimum internal free heap observed after tracked allocations: 220,740 bytes.
- Codec-task stack: 8,192 bytes; lowest measured remaining space: 3,684 bytes.
- Existing 192 kHz/S24 LHDC fixture: 41,899 us total for 60 ms of audio;
  maximum media call 14,552 us; zero per-packet deadline misses in this short run.

That LHDC fixture carries about 403 kbps of encoded data (404 kbps including the
adapter media prefix). It does **not** establish the 1,000 kbps requirement.
The timing includes media-wrapper and PCM-validation work and scheduling effects;
CCOUNT spans are elapsed core cycles, not exclusive decoder CPU accounting.
The small isolated run establishes neither sustained headroom nor a runtime RAM
advantage against an equivalent product workload.

Raw local records are under `native_bt/_build/hardware/codec-37/`.

## Maximum-rate fixture

`prepare_lhdc_max_fixture.py` checks the separate test encoder's exact 17-file
source pin (`3f9d1980fa9c57cdfb78792f9455a7dbac50edda`), compiles through ccache,
and generates 96 ordered 192 kHz/stereo/S24 frames at quality index 8.
The measured encoded payload is 1,001,600 bps including codec framing.
The unmodified sibling decoder provides 56 sparse PCM reference checkpoints.
The existing bounded cross-target sample tolerance is retained; this is not a
bit-exact floating-point or perceptual-fidelity claim.

The target benchmark excludes the first 12 frames from timing statistics and
reports 84 per-frame wall-time measurements against the 5,000 us audio budget.
Between fixtures it sleeps for one tick, outside the measured processing span.
It reports correctness and isolated deadline results separately. DSP, DMA,
controller/ACL work, BLE, flash commits and product providers are absent.

The COM6 maximum-rate run passed all 96 frame/format checks and all 56 reference
points (maximum absolute error 256 S32 units), but **failed the isolated deadline
gate**:

| Measured media processing | Microseconds |
|---|---:|
| Mean | 4,838 |
| P50 | 4,696 |
| P95 | 5,112 |
| P99 / maximum | 5,190 |
| One frame's playback budget | 5,000 |

35 of 84 measured frames took at least the 5 ms budget. Tracked peak allocation
was 63,672 bytes; stack low-water was 3,668 bytes. The 37 existing codec cases
also completed again, but the overall diagnostic correctly reported failure
because the maximum-rate performance gate failed. It selected the released
baseline, which booted cleanly to `System ready`.

This includes validation callback work. It does not establish that exclusive
decoder arithmetic alone misses its deadline, and it establishes no simultaneous
BLE/DSP/I2S headroom. The next smallest defensible test is separate callback and
decoder-stage timing with repeated identical inputs, followed by the real DSP,
PCM/DMA and coexistence workload. No fidelity/bitrate/rate reduction was used.
The tested image SHA-256 is
`e3f18e1ec31b1ec5060ef0be54c9f8c04df126c73f156530d34b785e1f0277a3`.

Reproduce fixture preparation and build:

```powershell
python native_bt/tools/prepare_lhdc_max_fixture.py --encoder <pinned-encoder-checkout> --decoder <sibling-decoder-directory>
./native_bt/tools/build_probe.ps1 -CodecDiagnosticOnly -LhdcMaximumCheck -WithLhdc -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir <sibling-decoder-repository>
python native_bt/tools/test_codec_diagnostic.py
python native_bt/tools/test_probe_codec_check_target.py
```

Deployment is a separate explicit COM6 operation with fresh identity, physical
backup, source/config/image and partition admission. Generic SDK flashing remains
disabled. Compilation or host mocks cannot substitute for the target result.

## Still required

Maximum-rate deadline/headroom correction, repeated measurements and stage
profiling; an equivalent baseline; complete DSP/I2S and simultaneous BLE/AVRCP/storage tests;
protocol/application parity; actual security/RF/lifecycle/fault and sustained
stress qualification. The master objective remains active and incomplete.

The full product probe's separate OTA/restart path also still invokes
`esp_restart()` directly from its core-1 owner (`probe_main.c`); the new core-0
diagnostic return does not fix that product path. Port the already measured
released-firmware core-0 restart behavior and requalify OTA before product use.
