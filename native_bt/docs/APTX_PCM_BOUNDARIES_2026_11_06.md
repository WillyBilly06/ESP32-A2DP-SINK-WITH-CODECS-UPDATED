# aptX HD PCM boundary capture - 2026-11-06

The user reports HD becomes noisy after two or three seconds and stays noisy after reconnecting. **This issue remains open.** Clean transport counters are not an acoustic verdict.

## Implementation

The explicit -AptxSnapshot option captures one normal HD decode call after 2,500 ms of decoder activity and one subsequent, independent 128-frame DSP block. Cold checks and resynchronization calls are excluded. Codec, input producer and renderer publish immutable diagnostic storage independently. A pending decoder scratch address is retired on reset/close. No new task, allocation, interrupt payload work, wait, or hot-path UART call is added.

Eight sections retain pre-decode context, the entire remaining encoded view (capacity 1,800), decoded PCM, accepted input-ring bytes, pre-DSP state, DSP input/output, and output after the optional mix. The DSP block is independent of the decoder view; this does not trace one sample identity through DMA. Export occurs after I2S stops, at most two 64-byte chunks per logger iteration. Existing baseline tracing otherwise remains enabled.

Default OFF. Temporary measured BSS cost: **12,760 bytes** relative to dd734e28b62e. No production queue or codec-quality setting changes. This is a diagnostic cost, not a RAM optimization.

## Verified evidence

Focused offline checks cover explicit arming, wrap-safe eligibility, full 1,800-byte bounds, immutable context copies, origin matching, two-span input-ring copies, 128-frame DSP capture, one-shot behavior and address retirement. The first harness used unsupported crossover values; initialization was corrected to the existing 90/500 Hz table contract before these checks passed. No additional boot cases were added.

The ccache build and source/artifact checks passed. COM6 was written at 921600 and read back at 115200. App and selector hashes matched; original application was preserved; owned VALID boot, no automatic Bluedroid restore, and the existing 37 codec cold checks were observed.

Image SHA-256: ecea383940fa28d2b12223e6b1b42226db607a1eeb7eeb2c90b3615963478be2. Hardware archive: native_bt/_build/hardware/runs/1793994225958937600-ecea383940fa.

One native YouTube Redmagic trace negotiated HD, 48 kHz, stereo, 24 encoded bits, S32 PCM. Play/pause dispatches were 12.197 seconds apart, with an independent 14-second pause guard. Data exported after pausing. UART source ID 2 remained unchanged. No microphone/acoustic recording was taken.

Snapshot sizes: context 4,404; remaining input/consumed 84; PCM and copied ring 448 each; DSP state 1,128; independent DSP input/output/final blocks 1,024 each. The selected decode call was the final portion of a 660-byte payload, not a truncated remaining view. Context and DSP size/field-offset fingerprints match the little-endian 32-bit desktop ABI; replay refuses mismatches.

| Boundary | Result |
| --- | --- |
| Target decoder vs desktop replay, identical pre-state/input | 448 bytes equal; consumed/written lengths equal |
| Decoder vs accepted input ring | 448 bytes equal |
| Target DSP vs desktop replay, identical pre-state/input | 1,024 bytes equal |
| DSP vs final output | 1,024 bytes equal; no mix hook |

Decoder replay uses the **same SDK freeaptx implementation** on another architecture. It verifies target execution/handoff consistency, not independent codec fidelity. It does not validate earlier state, every later call, or DMA/I2S output.

Zero new queue-full drops, decoder rejects, RTP sequence breaks, DMA shortages and descriptor errors were recorded. Worker detail-trace loss was 3,966, so this is not a full call-by-call scheduler history. Snapshot chunks were complete/contiguous with blob hashes. Copy spans in core cycles (including any preemption): decode 4,682; ring 321; DSP 3,557; final 676. This is not a general performance benchmark.

## Unresolved timing discrepancy

All 1,648 media payloads were 660 bytes. HD yields 440 frames per payload: 1,648 x 440 minus the initial 90-frame decoder delay equals the observed 725,030 PCM frames. Wire RTP timestamps instead advanced by 336 per packet (last 553,392). Its source and acoustic significance remain unproven.

Our parser already honors CSRCs, extensions and RTP padding. Do not trim 156 bytes, force output to 336 frames, alter the rate, or blame the phone on this evidence alone. Next compare actual source/controller framing and the downstream DMA path. The historical [AOSP HD encoder](https://android.googlesource.com/platform/system/bt/+/1f118853c585488577a1d929014973a6a9a04064/stack/a2dp/a2dp_vendor_aptx_hd_encoder.cc) derives timestamps from encoded PCM sample count; it is a reference, not evidence of the Redmagic vendor implementation.

## Reproduce

Use the usual full stay-native product build, with -AptxSnapshot plus -WithAptx -PcmCopied -BlockDsp -WordDsp -CodecAwareRender. Both build entry points reject incomplete configurations. Retain ccache, guarded deployment and the authorized short YouTube trace.

Extract preserved monitor JSONL with native_bt/tools/extract_pcm_snapshot.py INPUT.ndjson.gz --out NEW_DIRECTORY. It rejects partial/mixed-source captures, overlaps, gaps, lengths and unsupported schema, then writes blobs, hashes and structured replay_arguments.

Compile native_bt/tests/codecs/aptx_pcm_snapshot_replay.c, native_bt/codecs/aptx/dbt_aptx_backend.c, native_bt/src/dbt_pcm_snapshot.c and bt_audio_sink/main/dsp/fixed_dsp.c as little-endian **32-bit** C11 with -fwrapv -DDBT_APTX_SNAPSHOT=1 -DFIXED_DSP_BLOCK_API=1 -DFIXED_DSP_WORD_QUANT=1. Include native_bt/include, the DSP directory and SDK libfreeaptx/main. Do not compile freeaptx.c separately: the backend includes it. Run with replay_arguments. Nonzero means mismatch/invalid fixture. The focused lifetime harness is native_bt/tests/codecs/pcm_snapshot_capture.c.

Private evidence: native_bt/_build/experiments/aptx-pcm-boundaries-1793993219491. Preserve failed receipts. Do not publish audio bytes or private device backups as release assets.
