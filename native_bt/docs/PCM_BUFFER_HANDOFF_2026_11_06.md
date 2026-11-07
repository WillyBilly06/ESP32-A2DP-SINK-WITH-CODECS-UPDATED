# PCM buffering and early input release — 2026-11-06

High-rate LHDC continuity is still incomplete. A zero decoder-drop count does not imply uninterrupted decoding: deferred PCM can remain blocked by the next stage, retain its packet/decoded view and apply backpressure without dropping either.

## Sizes and units

Observed workload: LHDC V5,192000 Hz,24 valid bits in32-bit stereo containers, one5-ms frame per received packet. One decoded frame is960 stereo sample frames, or7680 bytes. Decoder transfers are bounded to256 frames (2048 bytes), not128;128 is the renderer/DMA block limit. The earlier statement conflating those quanta was incorrect. Encoded 1800-byte x8 queue slots and decoded PCM storage are different units; an observed566-byte media payload does not represent566 decoded PCM bytes.

| Storage | Previous owned layout | Rebalance candidate |
|---|---:|---:|
| Decoded-input ring |1024 frames / 8192 bytes / 5.333 ms|2048 frames / 16384 bytes / 10.667 ms|
| Processed-output capacity |3584 frames / 28672 bytes / 18.667 ms|2560 frames / 20480 bytes / 13.333 ms|
| Processed-output target |3072 frames / 16 ms|2048 frames / 10.667 ms|
| DMA bridge |3 × 128 frames / 3072 bytes / 2 ms|unchanged|
| Ring + DMA requested payload |39936 bytes|39936 bytes|

The configured -PcmTargetMs16 value remains visible as configuration. The separate raw pcm_buffer_geometry record reports actual target/capacity in frames; it must not be interpreted as a16-ms processed target in the rebalanced mode.

The released Bluedroid reference at 0d85a975 has CONFIG_AUDIO_POOL_COUNT=16 and CONFIG_AUDIO_POOL_BUF_SIZE=1024: a 16384-byte Q31 stereo ring, 2048 physical/2047 usable frames, producer watermark 1024 and renderer setpoint 1023. Its DSP output chunk is 256 frames; its actual I2S driver settings are 3 descriptors x128 frames despite a stale 3×256 comment. The owned DMA geometry already matches that driver setting; decoded-ring distribution differs. The reference processes its ring chunk before advancing m_readIdx, so early copied-input release is an owned-pipeline improvement, not a claim that the reference already used it.

## Input ownership change

The old owned native copy path copies up to 128 frames from decoded input into a distinct unpublished output span, runs DSP, then releases input and output together. Holding those copied input slots prevents producer reuse throughout DSP, despite no remaining source reference.

The new owner-only dbt_audio_render_with_input_wake entry releases copied S32/stereo/unity input immediately after the bounded copy and notifies the decoder. Processed output remains unpublished until DSP and mixing complete. The ordinary entry retains its original timing; fractional/other-format paths notify after their normal final publication. No callback pointer, PCM buffer or other persistent field is added to dbt_audio_t. The callback may only notify the producer; no wait or reentrant render/lifecycle/DSP work is allowed.

-EarlyInputRelease selects this entry for local Core1 rendering. Same-core Core0 rendering retains its admission/notification timing, and prepared helper loans retain their existing fences. WAV producer notification remains after mixing. The encoded queue, codec fidelity, priorities and core assignment are unchanged.

ASan/UBSan passes192 complete stream pairs /787776 frames, including poisoned released input slots, all DSP/resampler state, mixer, formats, phase fallbacks and page/ring/counter wraps.624 native-copy notifications are observed before output publication. The actual two-core mocked provider holds the notification before DSP and then refills the complete input ring through the producer API while output remains unpublished. Queued stop/cancel, held BUSY helper, publication-before-DONE and default-mode regression pass. These are ownership and arithmetic checks, not target throughput or acoustics.

## Early-release observation

Image f8c4d5b7c9331621eb90fdb55549e8f6854ef0963fbf75b2774345a848544489 built with ccache and all codecs/product/security/RFCOMM. The arithmetic candidate and ABBA recorder are OFF. App/selector full readbacks match; original application is preserved; owned VALID boot and 37 existing cold codec checks pass.

Native YouTube commanded play 12.200 s, independent 14 s pause guard, 192k/S24/stereo/fixed900 UI/wire configuration; actual sampled DSP controls are volume68/EQ2/8/12/flags12. Accepted, rendered and DMA source frames all 2516672; initial shortage 559104, late recovery 167424, unfilled 391680, descriptor errors 0. PCM busy 6307, encoded drops/rejects/sequence breaks 0; helper frames 1414464. Raw detail loss 2824 is separate from zero aggregate snapshot loss. The prior nondebug fair-sharing segment had 318848 unfilled frames and 9908 PCM busy events, with different volume, audio/startup/tail windows. Early release is correct but this observation does not establish a continuity or throughput improvement.

## Fixed-memory redistribution

-HighRatePcmRebalance explicitly moves 1024 S32 stereo frames (8192 bytes) from the processed reserve into the decoded ring only for192k/S32/stereo. It requires the complete native early-release product and configured16ms reserve, excludes whole-Core0 LHDC and comparison/pacing modes, and defaults OFF. Total ring/DMA payload remains39936 bytes; storage is redistributed rather than increased. Two960-frame decoded units now fit in the input ring.

Actual mocked allocations verify the2048-frame input,2048-frame target and39936-byte payload; five other rate/format allocations remain unchanged. Full-ring reuse before local DSP and existing stop/cancel tests pass. Actual admission checks pass6 CMake/14 image cases; early-release admission separately passes8 CMake/18 image/1 CLI cases. The actual output target is cached atomically for telemetry and inactive delay reporting, avoiding a cross-task read of mutable audio configuration.

Candidate ae24c89e42eb77270ef14c6c98f14a22821166922fa9fda9dc2d6295f6c6f7cc passes source/image validation with10369 stable source inputs. Compared with the early-release image, DRAM data/BSS and IRAM are unchanged, flash text grows100 bytes and rodata32 bytes. Deployment/playback results are pending. No new boot test cases were added. All-codec streaming, full1000-kbps LHDC, encrypted BLE coexistence, long-duration stress, acoustic continuity and complete feature parity remain open.

## Rebalanced hardware result

COM6 app and selector full readbacks match image ae24c89e42eb77270ef14c6c98f14a22821166922fa9fda9dc2d6295f6c6f7cc. Owned VALID boot and all 37 existing cold codec cases pass; the original application is preserved. Raw geometry verifies 2048 input / 2560 processed-output frames, an actual 2048-frame output target, unchanged DMA and 39936 requested payload bytes.

Native YouTube, commanded playback 12.117 seconds plus the independent 14-second pause guard and post-pause capture. Sampled controls: DSP volume 68, EQ 2/8/12, flags 12; LHDC 192k/S24/stereo/fixed900 is restored and the received maximum payload remains 566 bytes.

- Decoder attempts/received packets: 2604; encoded payload bytes: 1473864; no decode reject, conceal, nonfinite or clipped frame.
- Accepted, rendered and DMA source frames: 2499840 each, exactly 2604 x 960. This count conservation supports correct decoded-block sizing; it is not a complete per-sample identity/acoustic proof on hardware.
- Initial DMA shortage: 547840; late recovery: 153984; unfilled: 393856; descriptor faults: zero.
- PCM busy: 6221; encoded drops/oversize rejects/sequence breaks: zero. Worker detail loss: 2776; aggregate snapshot loss: zero.
- Helper processes 1419776 frames; no helper failure.

The early-only segment recorded 391680 unfilled frames and 6307 PCM busy events. Different music segments and timing windows prevent an isolated speedup comparison. The rebalanced layout does not materially improve this failing observation. It does not establish that further buffer growth would solve the throughput problem. Both options remain explicit/default OFF; this is an unqualified development selection.

In a central 6.6-second window, 1kHz interrupted-task samples show Core0 decoder/render-helper/idle counts 3705/2161/773. Core1 host/renderer/controller/logger/other/idle counts are 1482/2657/1431/558/265/242. These are sampled roles, not exact CPU-time percentages. Decoder wake_owner calls are already limited to low-water packet retirement and control/fault receipts; the source does not wake the host on every PCM chunk. Further work needs measured consumer/decoder scheduling and computation headroom.

## Representative metronome regression

The native YouTube player loaded [120 BPM - Metronome by Drumset Fundamentals](https://www.youtube.com/watch?v=xpiVrEET-YQ), verified through MediaSession metadata. This is a representative clip, not identified as the user's exact original video. Direct native-app selection succeeded after earlier search snapshots exposed no video results. Initial selection autoplay stayed within a separate 12-second pause guard; its first UI dump failed and the same guard was retained.

LDAC 96k/stereo with the phone's 32-bit input setting; 12.208-second play/pause command interval, independent 14-second pause guard, eight real volume-down keys then eight up. Independent Android reads verify 80 -> 0 -> 80 and volume is restored. Accepted/rendered/DMA source counts all equal 1449472, with zero descriptor faults or recorded in-play shortages. The first shortage occurs 15.132 seconds after the first captured sequence-2 packet, after the 12.208-second command interval: decoded input/output are empty and the decoder waits for encoded input with queue depth zero. Its 640 unfilled frames are retained as a source-empty post-pause tail.

Periodic DSP snapshots show volume 68; they do not independently qualify remote DSP gain propagation or audible pop elimination. AudioService reports AVRCP absolute-volume support, but command-origin/acceptance timing remains a follow-up. All-codec, maximum-LDAC-bitrate, full 1000-kbps LHDC, encrypted coexistence, long-duration and acoustic qualification remain open.
