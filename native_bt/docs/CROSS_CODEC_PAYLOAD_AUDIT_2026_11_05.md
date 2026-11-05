# Cross-codec transport and PCM corrections

The user reported poor audio/delivery across LDAC, LHDC, AAC, aptX and SBC. Functional tests remain stopped at the user's request. This pass uses source review, existing captures, compilation and deployment integrity checks. It does not establish stable playback or identify every reported crash.

## Actual payload boundaries

The shared AVDTP layer strips RTP through dbt_avdtp_media_decode and passes the remaining bytes unchanged into the encoded worker. It does not insert a generic SBC media header.

| Codec | Bytes after RTP | Finding |
|---|---|---|
| SBC | SBC one-octet frame/fragment header, then coded frames | SBC semantics belong here. |
| AAC-LC | LATM AudioMuxElement | No SBC byte is removed. Complete LATM units must not wait solely for RTP marker; incomplete units remain bounded and assembled. |
| aptX / HD / LL | Raw 4-byte / 6-byte coded quanta | No SBC byte is removed. |
| LDAC | Its specified one-octet media header | Similar count bits are codec-defined; do not remove the legitimate header. |
| Android Opus / Opus05 | Their specified one-octet count/fragment header | AOSP and PipeWire explicitly use these fields. Similarity to SBC is not evidence of a wrong extra byte. |
| LHDC V5 | Two octets: frame count shifted left two, then sequence | The old SBC F/S/L interpretation was wrong. Parse count from bits 7:2, validate it against every self-delimiting coded frame, and keep the source packet immutable until PCM retirement. The unsupported SBC-style codec fragment assembler is removed; standard ACL/L2CAP reassembly remains. |

BTstack reference is pinned to 9edb5d82ea6ec4466a393415d65030929330acdf, especially src/l2cap.c, src/classic/avdtp.c and src/classic/a2dp_sink.c. Its media delivery and codec-specific parsing are separate boundaries. No BTstack implementation is integrated.

Google sources use packages/modules/Bluetooth commit c77db469de80f86660bc053bec4dee0c5d4b947c: system/stack/a2dp/a2dp_vendor_lhdcv5_encoder.cc, a2dp_vendor_lhdcv5.cc, a2dp_vendor_opus.cc and a2dp_aac_decoder.cc. PipeWire's a2dp-codec-opus.c independently confirms the Opus05 fragmentation fields. AAC's reference decoder feeds available transport bytes and lets decoder framing determine completion; it does not make the media marker an exclusive decode permission.

## Shared PCM liveness correction

The copied PCM ring could be 256 frames at 44.1/48 kHz while the worker submits complete 256-frame chunks. Positive clock correction can require a two-source-frame advance. If one source frame is retained at that boundary, only 255 frames are free and another complete chunk cannot be admitted. The minimum copied capacity is now the next power of two above the transfer size: 512 frames. This adds 1024 bytes for stereo S16 or 2048 for stereo S32 at these low rates. The 512-frame 96 kHz and 1024-frame 192 kHz capacities are unchanged. This is a capacity/liveness fix, not a claim that all reported crashes have one cause.

## Packet storage and diagnostics

The encoded queue's flexible payload began at offset 14 even though sizeof its metadata was 16. It now begins at offset 16 and every slot stride is rounded to eight bytes. The requested 1800 x 8 configuration remains 1816 bytes per slot and 14528 bytes total storage; no extra on-wire tag is inserted. Two diagnostic prefix octets occupy the existing metadata padding and remain outside the decoder payload pointer.

Failure traces now capture actual codec identity, RTP marker and the first two payload octets. The historical sbc_pcm record name is retained for old readers but carries codec_name and diagnostic_scope=all_codecs, making its meaning explicit.

The 108 bundled LHDC packet headers are corrected from zero placeholders to the real shifted frame count and sequence. Encoded bytes and PCM oracle values are unchanged. Existing fixture writers are updated without running encoder/decoder tests. Historical SBC-style LHDC fragment results are not proof of real V5 fragmentation support.

## Evidence limits and remaining work

Existing capture counters show queue saturation and PCM backpressure with zero size-overflow rejects. A watchdog record in retained replay data must not be attributed to the current live image. The retained live capture did not supply a fresh panic backtrace for the user's latest crash report. Maximum-rate sustained throughput remains unresolved; these corrections do not prove sufficient DSP/resampler headroom.

The flasher now verifies the entire preserved original app using the SDK stub's whole-region MD5 before and after programming, avoiding two multi-megabyte UART copies. Bootloader/table checks and every newly written region's full byte readback remain. This is a deployment speed correction, not Bluetooth functional qualification.
