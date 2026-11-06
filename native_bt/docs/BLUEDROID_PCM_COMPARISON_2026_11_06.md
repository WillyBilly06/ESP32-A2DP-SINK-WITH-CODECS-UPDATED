# Bluedroid PCM baseline and core sharing — 2026-11-06

## Owning source comparison

Working released Bluedroid checkout: D:/bt-idf61 at 0d85a975ec079d954f1bdd3e94d2fed70848c252. AudioPipeline source SHA-256: b24a43ad91f523f020fe7c211d8327c8c3176934adc05877b70d3a984afc6b2a. The original dirty workspace has additional changes (SHA-256 fd2cf8593c0f4757c4a90c01f7ee3fb67178628dd0881af8d14ac5a1479fdade); its same PCM loop and disabled drift drop/insert were also inspected directly. It is not treated as a clean historical commit.

The released bt_audio_sink/main/audio/audio_pipeline.h:262–270 explicitly disables drift insertion/removal. Lines283–296 consume each Q1.31 PCM frame once, run DSP, then publish the unchanged frame count to I2S. There is no all-frame FIR in that Bluetooth audio path. The callback applies bounded backpressure before writing its PCM ring (116–173). main.cpp:2135–2142 places audio_render on Core0, with A2DP task on Core1 (2297). DSPProcessor converts Q31 to float internally; the owned DSP remains integer as separately required.

The owned product instead selected adaptive_clock=true in probe_output.c, so dbt_audio_render ran dbt_resampler_filter for every stereo frame even at zero correction. The filter uses96 taps at ≤48k,48 at ≤96k,24 above96k. This is added processing rather than a codec decoding requirement. It also changes the nominal PCM signal and group delay relative to the working product.

## Corrected product path

Stay-native now selects adaptive_clock=false. Existing initialization sets fraction and step_adjust to zero, clock updates return without changing them, and each output frame consumes exactly one source frame. Nominal PCM conversion retains S16/S24/S32 precision and all integer DSP, analysis and sound mixing. The product neither drops/duplicates source frames for clock correction nor runs the FIR. The optional FIR module and finite diagnostic mode remain available. Delay reporting no longer includes an unused FIR delay.

This restores the working product's PCM policy; it does not lower negotiated codec rate, bitrate or depth. It is not proof of indefinite independent-clock stability. Long-duration drift, burst tolerance and simultaneous BLE qualification remain open. Do not replace that missing evidence with a claim of continuous resampling support in this mode.

## Core policy

The user superseded their previous Core0 decode-only restriction and authorized measured DSP sharing. The opt-in -CodecAwareRender mode keeps Bluetooth/controller on Core1 and decoder priority21 on Core0. SBC/AAC/aptX/LDAC use a priority20 Core0 render owner. LHDC/Opus retain priority20 Core1 rendering. Only a closed channel transfers ownership; I2S allocation and destruction occur on its selected owner core. One existing ticket retains each accepted lifecycle request, and only one owner mutates DSP.

The Core0 render task is lazy, has a6144-byte stack plus TCB, and remains dormant when not selected. Keeping both handles valid prevents notification lifetime races. Allocation failure is explicit, with no silent host or core fallback. Separate UART records expose ownership transfers, stack allocation and failures. The encoded queue stays1800 bytes ×8.

## Earlier measured candidates

All records below are approved short Redmagic/COM6 playback traces; elapsed stage cycles include preemption. They are not controlled repeatability or all-feature proofs.

| Candidate | LDAC96k mean elapsed per render call | Result |
|---|---:|---|
| 5d9c7929, all-frame FIR, renderer Core1 |2.515ms|Above1.333ms/128 deadline; EOF recovery exercised; underruns remain|
| Packed FIR delta candidate, Core1 |2.274ms|Above deadline; not a repeatable isolated speedup|
| Packed FIR, whole render Core0 |1.305ms|Core0 saturated;2 IDLE0 watchdog events;403 encoded queue drops; not acceptable as a fix|

The same Core0/FIR candidate's short SBC44.1k run added no encoded queue drops or watchdog events. Its128 terminal underrun frames require timing interpretation; this is not a zero-gap qualification. These observations motivated checking the actual Bluedroid PCM behavior rather than further FIR tuning.

The original disabled FIR-fold/assembly variants remain disabled. The subsequent one-to-one measurement is below.

## Current build and hardware evidence

Image6c4e683dc2aca9f188b4f3c53431be48b5b942025edb5c0a15bca71ab5587bba is deployed to COM6. Both readbacks match (write921600, verify230400), owned host kept active without automatic Bluedroid fallback; original application region unchanged. Archive: native_bt/_build/hardware/runs/1793963645914257000-6c4e683dc2ac. Source manifest SHA-256:8f88d0a83f3983b8886041876d5f7194b471bf56ea7a23088ec87b8b3d64d938. Build uses ccache,10349 source hashes stable.37 existing boot codec checks pass; these are cold RF=false checks.

Targeted LDAC96k stereo/S32 trace: native_bt/_build/experiments/ldac96-unity-pcm-core0-1793963888920. Approved play command interval10s, always paused afterward. Final flushed counters cover the complete stream including setup/tail, not exactly the command interval: accepted=rendered=DMA-source1234688 frames,0 underrun frames,0 descriptor errors,0 queue drops,0 decode rejects/resets/PCM faults,0 RTP sequence breaks,0 watchdog events. Queue high-water4/8. Render9646 calls,4190016us total,0.434ms mean;1.333ms per128-frame deadline at96k. Worst call2.352ms includes preemption and exceeds one-block time; buffering absorbed this observed delay. This is short-run evidence, not worst-case or long-duration proof. Conversion/FIR-labelled counter now measures PCM conversion only; actual FIR is off and the separate policy row reports that explicitly. Worker trace ring lost4501 diagnostic events while aggregate snapshots remained intact; no claim of complete per-packet tracing.

Light-codec renderer and I2S callback both ran on Core0 below decoder priority. One ownership transfer,6144-byte auxiliary stack,4132-byte renderer stack low water, no helper allocation failures. After stop: heap98976/min64252/largest81920 with LDAC configured. These are current-mode values, not a whole-device RAM saving comparison.

A following short AAC trace did not crash but rejected473 LATM payloads before producing PCM. Observed prefix47fc uses audioMuxVersion1; the owned parser only supports version0, unlike the working Bluedroid parser. That independent compatibility defect is still open in this commit. Full codec, BLE/security, app, overlay/EQ, OTA, maximum LHDC and long-duration release gates remain incomplete. Broader campaigns remain held; the user authorized targeted ≤15-second playback diagnosis until the shared low-bitrate issue is fixed.
