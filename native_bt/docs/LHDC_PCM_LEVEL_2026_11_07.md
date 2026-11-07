# LHDC V5 PCM level normalization — 2026-11-07

Later real-phone measurements exposed a further24-bit level mismatch that the
C99 self-roundtrip did not prove. The [integrated checkpoint](INTEGRATED_AUDIO_2026_11_07.md)
adds a measured-3.0103dB product trim and verifies matching LDAC at48k/192k.
The15/16-only results below describe the earlier implementation.

The native decoder inherited subjective per-rate/depth output trims from the sibling decoder. The 192 kHz/S24 trim adds +0.79 dB; its untrimmed encoder/decoder round trip already has about +0.5606 dB gain. This makes the selected mode about +1.3506 dB too loud before DSP or phone volume, and causes irreversible saturation before a downstream volume reduction can help. The S16 trim instead removes 5.60 dB, introducing a separate depth-dependent level mismatch.

The owned source-pinned port now replaces those trims with the exactly representable constant 15/16 (0.9375), the inverse of the measured 16/15 reconstruction gain. This is constant normalization before the existing frame-safety scan and PCM conversion. It adds no frame-adaptive gain, buffers, tasks, allocations, waits or reduced-quality codec mode. Corruption/nonfinite protection, overlap handling, PCM saturation, integer DSP and the DSP soft limiter remain enabled. The unmodified sibling repository is unchanged. A host-only legacy comparison define is rejected in firmware builds.

## Independent level evidence

Experiment: native_bt/_build/experiments/lhdc-level-1794057006786.

The test signal is generated from known PCM by the pinned C99 encoder at commit 3f9d1980fa9c57cdfb78792f9455a7dbac50edda. This tests actual input/output amplitude, rather than only comparing two copies of decoder output. Independent original-decoder PCM with its host calibration hook set to 15/16 additionally matches every native PCM byte.

| Measurement | Previous calibration | Corrected normalization |
| --- | --- | --- |
| 192k/S24 quiet-tone gain | about +1.3506 dB | near 0 dB |
| 192k/S24 95%-peak tone, fixed400/fixed1000 | all96 frames clipped in each case | zero clipped frames |
| S16 quiet-tone gain, four rates | about -5.044 dB | near 0 dB |

The expanded host gate has48 cases: 44.1/48/96/192 kHz, S16/S24, 10%/95% input peaks, fixed400, fixed1000, and forced400↔1000 kbps transitions every16 encoder frames. Each case decodes96 five-millisecond codec units. The independent signal-fit gate requires absolute gain error below0.02 dB, residual below-45 dB,96 emitted units, and zero rejects/channel failures/concealed/nonfinite/clipped frames. These generated transitions exercise changing frame sizes; they do not qualify the phone's real adaptive-bitrate algorithm or RF behavior. The encoder does not supply32-bit source material, so this is not S32 source qualification.

Existing four LHDC cold-check checkpoint arrays and the56-point maximum fixture were refreshed from the independent normalized reference. Encoded packets, other codec cases, checkpoint indices, comparison tolerances and the total37 boot configurations are unchanged. This changes expected amplitude deliberately; it does not relax a PCM comparison to accommodate errors.

Commands:

    python native_bt/tools/test_lhdcv5_pcm_levels.py --encoder D:/owned61-lhdc-encoder --decoder "D:/ESP32 Bluetooth/LHDC-V5-Decoder-main/decoder" --out native_bt/_build/experiments/<run>
    python native_bt/tools/test_probe_codec_check.py --lhdc-source "D:/ESP32 Bluetooth/LHDC-V5-Decoder-main" --deferred --sanitize --lhdc-esp32-paths --lhdc-max-fixture

The second command passes80 actual-codec deferred cases under AddressSanitizer and the selected UndefinedBehaviorSanitizer instrumentation, including all37 configurations and the maximum LHDC fixture. It verifies PCM/timestamps, fragmentation, batching and backpressure on the host. It is not target execution.

## Device evidence and scope

Before this correction was flashed, fresh COM6 UART showed roughly65,000 LHDC decoder attempts with zero decoder clipping/concealment, while I2S refill shortages remained. The sampled DSP policy had +11/+8/+12 dB EQ, volume76/127 and flags12. These are separate stages: a level bug is confirmed by the independent generated input, but that does not prove the currently observed DMA shortages originate in decoder clipping. The existing full-frame concealment threshold is twice full scale and acts before downstream phone/DSP volume.

The complete ccache firmware build and offline image admission pass. Initial candidate SHA-256: 8e85a4aa3eb5b0f089831624e93d4a6a5195488bf9e01fbd635e869d93c08c0b (1,338,448 bytes). It includes the separately documented owner-memory diagnostic deferral. Compared with the preceding2ade local image, IRAM/data deltas are zero, BSS is8 bytes lower, flash text+348 and rodata+656. Those are combined-image deltas, not isolated LHDC speed measurements.

At document creation this candidate is not flashed. No acoustic, max-volume phone auto-bitrate continuity, sustained coexistence or production-readiness pass is claimed. The independent level gate proves the specified generated signals; it does not establish that all lossy reconstruction overshoots, EQ clipping or software scheduling shortages have been eliminated.

## Subsequent guarded COM6 deployment

The8e85 image was uploaded at921600 and its entire written application region read back at115200 with an identical SHA-256. The original application remains preserved, owned boot is VALID and automatic Bluedroid restore is disabled. All37 existing cold target configurations pass (173 callbacks, tracked peak56899 bytes, minimum free after allocation103292, decoder-task stack low3608); this is a cold check, not RF playback qualification. Archive: runs/1794057819937023100-8e85a4aa3eb5.

Two guarded native YouTube metronome checks used maximum phone volume150/150; sampled DSP volume is127. Android initially reverted to48k after reconnect, so that run is labelled48k. The second run explicitly negotiates LHDC192k/S24/stereo with automatic bitrate (CIE00ff3a050000354c0102110000). Its play/pause command interval is11.236s; the independently scheduled14s pause guard also completed. Native YouTube can start when its intent opens, so command interval is not acoustic duration.

The192k run adds24 decoder-clipped frames/541 channel samples, with zero whole-frame concealments or decoder rejections. Its last offending reconstructed peak is only slightly over full scale; this is not evidence that all remaining clipping has been removed. The sampled EQ is+11/+8/+12 dB and DSP volume127; the DSP limiter remains active. Refill unfilled total3584 includes startup and drain, so this is not an all-zero continuity pass. Source payload506 bytes carries two units, corresponding to about400kbps coded audio: automatic bitrate did not exercise the1Mbps target. Detailed CRC-bearing stage records and the first aborted volume-readback attempt are preserved in the experiment.

Playback is paused, the prior90/150 phone volume is read back after restoration, and the live UART dashboard is listening again. The measured excess-gain bug is corrected; residual reconstruction overshoots, startup/drain shortages and maximum-bitrate continuity remain open.
