# LX6 Rice cursor and overflow hardening — 2026-11-07

The ESP32-D0WD-V3 uses dual Xtensa LX6 cores, not LX4 ([Espressif datasheet](https://documentation.espressif.com/esp32_datasheet_en.html)). This change reduces the entropy reader's generated LX6 footprint while preserving valid PCM and codec configuration. It does not establish the decoder's performance limit or resolve all underruns.

Image `3918eeddefc56aeef529262cf665499e82fd3427a9a71c7a638344719db80bc5` is flashed and read back on COM6. The owned image is VALID, the original application is preserved, and automatic Bluedroid restoration is disabled. Evidence is under `native_bt/_build/hardware/runs/1794090287699918000-3918eeddefc5`; reproducible experiment outputs are under `native_bt/_build/experiments/lhdc-rice-cursor-1794089188355`.

## Implementation

The Rice quotient reader repeatedly queried the same already decoded symbol. The new private `dbt_lhdc_rice.h` caches that symbol locally. It preserves the lazy FAC cursor, range/byte lookahead, probability model, complete sliding history, exhaustion behavior, and the short-three return without lookahead. FAC/model functions are unchanged. The source-pinned `prepare_port.py` applies the integration; the sibling decoder repository is untouched.

Malformed escape sequences previously allowed oversized shifts or coefficients outside signed 32-bit range. A stack-local error flag and checked head/tail arithmetic reject those magnitudes. The public entropy entry returns `LHDC_DEC_BITSTREAM_ERROR` before publishing an invalid byte position or final range. Adjacent valid boundary values remain accepted.

No codec rate, depth, gain, bitrate, precision, queue, allocation, affinity, or priority changes. No wait, spin, task, persistent buffer, or per-symbol allocation is added.

## Generated LX6 code

| Measurement | Baseline 19aae | Candidate 3918 |
|---|---:|---:|
| Rice symbol size, bytes | 2672 | 2171 |
| Rice entry frame, bytes | 80 | 64 |
| Entropy caller entry frame, bytes | 112 | 128 |

Total linked instruction RAM decreases **496 bytes**. Data, BSS, flash text, and rodata sizes are unchanged. The caller's larger local record offsets the Rice frame reduction; no whole-task stack saving or additional audio heap is claimed. These numbers come from the Xtensa ELF/disassembly.

## Correctness

The original and new readers pass **2,594,352 valid quotient comparisons** under ASan/UBSan, including complete FAC/model state, history/canaries, cursor and range equality. The corpus covers exhaustive ternary tapes of lengths 0–10 and random arithmetic input across 12 history-window choices, exhaustion, cursor wrap, and caps. Instrumented logical peek counts decrease from 9,123,380 to 3,747,989; this is not a 59% decoder speedup claim.

Seven private boundary cases check short-three lookahead, shift overflow, and signed coefficient boundaries. Five public entropy cases use the pinned C99 arithmetic encoder to construct exact FAC streams. Invalid head/tail/escape magnitudes produce checked errors; adjacent valid INT32_MAX results succeed. Canaries remain intact and error outputs preserve the byte-position/final-range sentinels. Initial harness failures involving an insufficient hard cap and missing mantissa lookahead are retained with their corrections.

The **48 tone and 48 dense-noise cases** preserve every PCM byte from the calibrated reference across four sample rates, 16/24-bit input, 400/1000 kbps, and forced bitrate transitions: 9,216 codec units. Original tone gain/residual gates remain unchanged. Noise uses exact reference equality and the original error/clipping gates. **80 actual-codec deferred cases** pass with the selected sanitizer instrumentation, covering all 37 configurations and the maximum LHDC fixture. No boot cases or PCM checkpoints changed.

A separate 48-case half-scale noise stress corpus also matches every reference PCM byte, but the unchanged reference clips in two 192 kHz/S16 cases: fixed 400 kbps has 33 clipped frames/41 samples; transitions have 15/19. Its failed quality gate and encoded inputs are retained. The clean noise corpus uses peaks 0.1/0.25; `--noise-peak 0.5` reproduces the stress failure. This is neither a clean fidelity pass nor a new regression attributed to this optimization.

## Device evidence

The full ccache build, source-stability, storage, and image-admission gates pass. All **37 existing target codec cases** pass, with 173 callbacks, 56,899 bytes tracked peak allocation, and 3,568/8,192 bytes cold stack low water. A final rebuild after adding host boundary tests produces the same binary SHA as the flashed image.

The same cold LHDC packet fixtures show the following one-before/one-after observation. Timings include PCM validation, use approximately 400 kbps fixtures, and are normalized per codec unit. They are not repeated sustained-stream speed guarantees.

| Configuration | Units | Baseline µs/unit | Candidate µs/unit | Change |
|---|---:|---:|---:|---:|
| 44.1 kHz/S16 | 9 | 1343.23 | 1337.01 | −0.464% |
| 48 kHz/S16 | 12 | 1302.43 | 1296.43 | −0.461% |
| 96 kHz/S24 | 12 | 1837.33 | 1827.17 | −0.553% |
| 192 kHz/S24 | 12 | 3139.45 | 3128.04 | −0.363% |

A guarded native YouTube music excerpt uses verified 192 kHz/S24/stereo/fixed 900 kbps at volume 150. The independent 14-second pause guard completes and the after-state confirms output stopped. There are 2,846 decode attempts with zero rejection, clipping, concealment, or nonfinite frames. All 2,731,712 accepted source PCM frames reach DMA, with zero descriptor/refill-contract failures. **4,928 refill-unfilled frames remain**, so continuity is not qualified.

Coherent stage counters cover 2,832 units: mean decoder body 2,436.45 µs, entropy 711.56 µs, and IMDCT 860.66 µs. Elapsed cycles include preemption and exclude wrapper/resume work outside those markers. Passive baseline music density changed substantially (one interval measured 1,555.98 µs body/54.83 µs entropy versus an earlier 2,661.01/882.86); unmatched content is excluded as a speed A/B. The source remained native YouTube.

## Remaining work

The output-level correction and 8 KiB input-ring saving from the [integrated checkpoint](INTEGRATED_AUDIO_2026_11_07.md) remain. Android's reconnect default was restored from 48 kHz to 192 kHz/S24 before the controlled check. The observed fixed 900 kbps workload does not qualify live 1 Mbps operation.

Verify actual startup/DMA-enable timing and reservoir occupancy across source-empty/render handoffs. Profile further entropy, mantissa, and transform candidates on representative content; inspect the existing FAC zero-run helper/tests before adding another version. Freed IRAM may permit a bounded implementation, but benefit is not assumed.

Full 1 Mbps headroom, encrypted BLE coexistence, rapid live EQ, long duration, security, product/OTA/lifecycle parity, acoustic behavior, and comparative whole-host RAM gates remain open. This is a development host, not a production release.
