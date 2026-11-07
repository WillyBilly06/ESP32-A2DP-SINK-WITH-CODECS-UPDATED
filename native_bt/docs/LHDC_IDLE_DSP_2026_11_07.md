# LHDC assistance while the decoder waits for input

This remains a development image. LHDC underruns are reduced in short sequential
comparisons, but uninterrupted playback and the full master goal are not proven.

## Change and ownership

`probe_audio_dsp_assist_ready()` previously admitted the existing Core0 DSP
worker only while the LHDC decoder waited for PCM space. It now also admits it
while the decoder waits for encoded input. Earlier physical starvation evidence
showed decoded PCM present, an input-starved decoder, and an idle helper.

The admission is a scheduling hint. The existing renderer still checks real
input, output space, format, generation and its single borrowed job before
dispatch. PCM-loan, control, renderer-handoff and awake states remain excluded.
Decoder priority21 continues to preempt helper priority20 on Core0. The
six-block/768-frame maximum, per-block publication, Core1 completion signal,
local progress after retirement, cancellation and sound-mixing paths remain
unchanged. There are no new tasks, buffers, allocations, waits or DSP arithmetic.

## Offline and deployment evidence

- `python native_bt/tools/test_dsp_assist_readiness.py`: the actual source
  function passes4096 cases across LHDC enabled/disabled, all256 codec IDs and
  idle/control/loan/handoff/awake states, with ASan and UBSan. Compilation uses
  ccache.
- `python native_bt/tools/test_dsp_assist_output.py --fair-share --early-input
  --rebalance --assist-blocks 6`: actual renderer/audio/DSP/sound code passes
  two mocked-core early-publication, held-worker, queued stop/cancel, completion
  ordering, bounded-batch and fairness regressions. This does not model ESP32
  execution deadlines.
- Full ccache-enabled IDF6.1 build, source stability and image admission pass.
  Image `ec89367eeef0acaaba286802a53d7aaced2e5b3b9991fa6d49fa9152f219c79f`
  adds16 bytes of flash text; data, BSS and instruction RAM are unchanged.
- COM6 guarded upload at921600 and complete written-region readback at115200
  match. The original application, bootloader and partition table remain
  preserved. The owned image is VALID; automatic Bluedroid fallback is false.
- All37 existing cold codec cases pass:173 callbacks, tracked peak56899 bytes,
  minimum free after allocation102996 bytes, stack low-water3584/8192 bytes.

## Native YouTube measurements

Redmagic `9125299103B7`, native YouTube120 BPM metronome, PCM5102A, no encoders.
Each command interval is11 seconds with an independent14-second pause guard.
Raw UART and structured events are retained. No volume keys are sent during
these traces. EQ remains11/8/12; limiter, meters, sound cues and application
features remain enabled. Phone90/150 maps to observed DSP76/127; phone150 maps
to DSP127. Volume90 is independently restored afterward.

LHDC configuration is192kHz, stereo,24-bit in S32 containers. Sampled payloads
remain566 bytes with the existing one960-sample-unit framing:562 coded bytes,
899200 coded bits/s. This is the phone's fixed900 setting, not a live1Mbps test.

| Image / workload | DMA source PCM frames | Unfilled PCM frames | Unfilled per million source frames |
| --- | ---: | ---: | ---: |
| Previous17320650, constant90 | 2623680 | 19328 | 7367 |
| Candidate, constant90 | 2729088 | 9984 | 3658 |
| Candidate repeat, constant90 | 2730752 | 12032 | 4406 |
| Candidate, constant150 | 2729792 | 8064 | 2954 |
| Candidate, constant90 after LDAC switches | 2723520 | 13568 | 4982 |

These counters include startup and postpause drain. In-play counter increases
and fresh first-starvation records independently demonstrate remaining genuine
render shortages. The baseline and candidate are sequential, with a reboot and
reconnection between them; this is an observed reduction, not a randomized or
reversed A/B proof or a new decoder-speed claim. No queue-full, decode-reject,
PCM failure, descriptor error or refill-contract error was observed.

The unchanged baseline already underruns without in-stream volume changes.
Its first gap has1920 raw frames and zero processed frames. Candidate first gaps
still have1792–2048 raw frames and no processed output, including both local
rendering and a running helper. Largest reported local render spans remain
3.282–4.648ms. Changing this admission alone does not resolve Core1 scheduling
or all helper publication deadlines.

## LDAC and codec-switch limits

Two finite LDAC96kHz/S32/stereo runs after LHDC each report zero initial
shortages and zero unfilled frames, with1357312 and1356800 source frames.
Returning to LHDC192k/S24/fixed900 also works, although its underruns remain.

The LDAC quality dialog has blank labels on this phone. Both the second radio
and the first radio were explicitly selected and independently read back.
Neither run proves fixed990: sampled packets include661 bytes/two frames,
661 bytes/three frames and657 bytes/four frames. Sparse high-rate packet samples
must not be presented as a sustained990kbps pass. Variable frame counts also
mean the RTP packet-clock step alone is insufficient for a bitrate claim.

Primary Android references were inspected to identify selector order and
configuration values: [quality preference](https://android.googlesource.com/platform/packages/apps/Settings/+/refs/heads/main/src/com/android/settings/development/bluetooth/BluetoothQualityDialogPreference.java),
[quality controller](https://android.googlesource.com/platform/packages/apps/Settings/+/refs/heads/main/src/com/android/settings/development/bluetooth/BluetoothQualityDialogPreferenceController.java),
and [LDAC encoder](https://android.googlesource.com/platform/packages/modules/Bluetooth/+/refs/heads/main/system/stack/a2dp/a2dp_vendor_ldac_encoder.cc).
These are upstream references, not proof of the Redmagic vendor implementation.
Why the observed source remains adaptive after these UI requests is unresolved.

## Retained evidence and remaining work

Local raw captures, failures, source tests, build log, deployment record, actual
ELF section deltas, verified volume restoration and comparison JSON are under
`native_bt/_build/experiments/lhdc-idle-assist-1794068029744/`.
Physical archive: `native_bt/_build/hardware/runs/1794068539911754100-ec89367eeef0/`.
Compact results accompany this document. A failed storage-budget check and the
hash manifest for removed rebuildable host PDBs are retained; source, test
results, binaries and all physical evidence were preserved.

Next investigate remaining local render/handoff latency and verify the phone's
actual fixed-quality LDAC source behavior. Live1Mbps LHDC, sustained
BLE/Classic coexistence, every codec and feature, security, OTA, lifecycle,
long-duration and acoustic qualification remain open. This image is not a
production release and does not replace the retained original firmware.
