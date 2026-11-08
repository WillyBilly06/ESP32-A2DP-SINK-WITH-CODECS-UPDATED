# LHDC S32 classification with direct Xtensa register moves

The retained path preserves signed floating multiplication and hardware
truncation. It classifies the binary32 magnitude/NaN/rail bits and updates peak
and clipping counts with integer comparisons. A NaN selects sticky positive
infinity without incrementing the clipped count. Infinity and finite rails keep
the original rounded 2^31 comparison. Only representable finite values reach a
cast. Gain calibration, concealment/overlap retirement and private PCM ownership
keep their existing behavior. The real-time EQ/DSP remains integer-only.

The initial union-only candidate was correct but slower. Its full matching
window measured 314.5 us for scale/output versus 248.0 us in the control, and
2,381.2 us per whole decoder frame versus 2,310.8 us. Linked opcode inspection
showed additional load/store sites. That implementation is rejected and retained
only as evidence; it is not the current firmware.

For original ESP32, the retained helper uses an explicit RFR instruction to move
bits from the FP register file to an address register. The instruction is
present in the [pinned Espressif GCC 15.2 machine description](https://raw.githubusercontent.com/espressif/gcc/esp-15.2.0_20251204/gcc/config/xtensa/xtensa.md),
whose SFmode move supports FR-to-AR transfer. The configured ESP32 core enables
its FP option. The compiler's earlier choice of memory for the union view is
consistent with the different integer/FP move constraints; that explanation is
an inference. The extra memory operations and measured slowdown are observed.

The target-specific branch is guarded by the original ESP32 configuration and
checks its FP option. Host/other builds keep the C representation access. The
scan handles one channel at a time and shares classification results with PCM
emission. The full linked helper has two RFR, two MAXU and two TRUNC.S sites,
zero floating comparison sites and zero calls. Baseline has 12 floating
comparison sites, two truncations and zero calls. These are static opcode-site
counts, not a cycle model. The retained image has 32 fewer IRAM bytes and
unchanged linked data/BSS/rodata/flash-text sizes relative to d7710746.
No audio buffer, task, allocation, core or priority is added or enlarged.

## Numerical and real-codec validation

- All 4,294,967,296 binary32 scalar conversion patterns match the independent
  original saturation macro. A further 1,000,000 run with ASan/UBSan and
  float-cast-overflow checks.
- 36,000 actual frame/output/quality/overlap/canary cases pass in each mode,
  including signed zero, NaNs, infinities, exact rails, late concealment,
  exceptional scales, mono and stereo, and the generic S16 path. These prove
  value/output behavior in the normal decoder FP environment; they do not
  promise preservation of diagnostic floating exception flags.
- All 80 real-codec deferred cases pass with every PCM byte and timestamp equal
  to the references, including all 37 configurations, fragmentation/batching,
  backpressure and the existing maximum LHDC fixture. Source-built codecs use
  ASan; owned adapters/worker also use UBSan. Host runs do not execute RFR.
- Actual configured ESP32 object and full-image disassembly verify RFR and the
  lack of calls. Existing 37 cold configurations / 173 callbacks pass on COM6.
  No new boot cases or expected PCM points were added.

Both host validators now keep incomplete receipts until their required gates
finish. Passing typed compiler intermediates are retired with hashes; symbols
are archived with on-disk gzip round-trip verification before the size gate.
The real-codec receipt stayed passed=false/functional_passed=true during a
recorded post-test quota failure. Failed build/test receipts, programs, source
variants and raw hardware captures are preserved. The initial target compiler
rejected an ambiguous adjacent-if line; it was split without changing tokens
or weakening warnings. Historical IDF stdout/stderr logs were archived
losslessly, leaving the 512 MiB cap unchanged.

## Physical timing

Approved Redmagic, native YouTube Get Lucky at the same 30-second URL offset,
LHDC V5 / 192 kHz / S24 / stereo / fixed 900 kbps UI setting / volume 150.
Each owned excerpt has an independent 14-second pause guard and remains within
15 seconds of playback. UART and its identity-checked monitor use 921600 baud.

| Path / complete interval | Published frames | Channel envelope us | Scale/output us | Whole frame us |
| --- | ---: | ---: | ---: | ---: |
| Baseline d7710746 | 2640 | 2020.2 | 248.0 | 2310.8 |
| Rejected union-only | 2640 | 2024.6 | 314.5 | 2381.2 |
| Retained direct-register repeat | 2640 | 2022.9 | 224.0 | 2289.8 |
| Retained direct-register repeat 2 | 2656 | 2016.8 | 223.7 | 2282.8 |

Scale/output elapsed time is about 9.7–9.8 percent lower; whole-frame elapsed
time is about 0.9–1.2 percent lower in these complete intervals. Counters are
compared within one generation with explicit uint32 wrap handling. They include
preemption and instrumentation. The first candidate run lacked a matching
before-generation; its shorter in-trace interval is recorded separately and is
not used to claim whole-frame speedup. These are short sequential observations,
not a randomized crossover or sustained throughput/headroom qualification.

The retained image is
**ccc99154495138cba3e5ff6c572796820dbeb430a64b8feb5e1f2151532a60d6**.
ccache target build, compiler-input recheck, 10,417-entry source stability,
workspace/image admission and actual-ELF ISR-root checks pass. Rebuildable output
is 504.685 MiB. Complete readback matches the 921600 upload; original app is
preserved, owned selection is VALID and automatic Bluedroid restore is disabled.
Cold tracked peak remains 56,899 B; minimum free after allocation is 106,500 B;
decoder stack low-water is 3,612 / 8,192 B. After four finite playback windows,
stopped free/minimum/largest internal heap is 63,360 / 30,580 / 59,392 B;
selected LHDC residency is 60,704 B; decoder/render/logger stack low waters are
5,912 / 3,772 / 1,404 B. These histories are not a matched Bluedroid RAM audit.

## Continuity remains open

Retained music windows have 1,152 / 0 / 1,088 finalized unfilled PCM frames.
The metronome regression has 5,568. No continuity improvement is inferred from
the computation measurements. Current windows have zero queue/decode/reset/PCM
timeout/sequence, ingress, descriptor or refill-contract faults. Their checked
UART records have valid CRC; legacy records remain explicitly Unverified.
All 10,657 published ACL metadata entries retire with status/lost zero. A fresh
paused six-second capture has 190 CRC-Verified records. Monitor PID 34852 / source 101
is live at 921600; native YouTube is PAUSED and I2S stopped at completion.

The metronome's first finalized gap has empty PCM, a decoder waiting for encoded
input, one unretired ACL metadata entry, and a new callback only 314 us earlier.
The owner has just marked its loop; cached pending completions are zero and the
transmit fence is clear. Decoder quality has zero peak concealments/nonfinite
frames. This is distinct from a quality-limiter mute and from earlier gaps with
raw PCM already waiting for DSP. Clock/pacing, reserve depletion, host delivery
and remaining consumer deadlines still need investigation. No actual clock-drift
cause is established here. IMDCT and entropy remain the largest measured decoder
stages. Full 1 Mbps, sustained BLE/control headroom, long-duration/acoustic and
complete feature/security/OTA qualification remain unproven. This stays on the
development branch; main and stable releases are not promoted.

Reproduce host checks (use the actual configured sibling decoder path):

~~~powershell
python native_bt/tools/test_lhdc_s32_output.py --decoder '<LHDC>/decoder' --baseline '<independent-preoptimization-lhdc_dec.c>'
python native_bt/tools/test_probe_codec_check.py --deferred --sanitize --lhdc-esp32-paths --lhdc-max-fixture --lhdc-source '<LHDC>'
~~~

Evidence: native_bt/_build/experiments/lhdc-word-classify-1794135844858/
and its register-variant/ directory. Retained hardware archive:
native_bt/_build/hardware/runs/1794138429775606000-ccc991544951/traces/.
