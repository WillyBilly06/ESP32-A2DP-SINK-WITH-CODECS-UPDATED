# LHDC V5 packet and buffer limits

Source analysis only; no encoder, playback or functional test was run in this pass.

Google reference: packages/modules/Bluetooth commit c77db469de80f86660bc053bec4dee0c5d4b947c, system/audio/codecs/lhdcv5 and system/stack/a2dp/a2dp_vendor_lhdcv5_encoder.cc. User reference: D:/LHDCV5-Encoder/src/lhdcv5_api.c, lhdcv5_process.c, lhdcv5_header.c and include/lhdcv5_enc.h. Source hashes are recorded separately. These are limits of the reviewed lossy implementation; they are not a universal guarantee for every proprietary LHDC encoder.

## Source-derived sizes

The lossy core accepts up to 1,000,000 bit/s, although its public quality enum contains higher indexes. A coded frame is a two-byte frame header plus two channel payloads. The payload byte count per channel is floor(bitrate * 50 / 160000) at 48/96/192 kHz, or floor(bitrate * 50 / 147000) at 44.1 kHz. The encoder supports 10 ms and 20 ms aggregation intervals, normally at most two or four frames per batch. The API accepts MTU parameters from 300 through 8192 bytes; 8192 is the parameter ceiling, not the size of every emitted packet.

| Mode at 1000 kbps target | One coded frame | Four-frame codec batch, large MTU | With 2-byte LHDC media header and ordinary 12-byte RTP header |
|---|---:|---:|---:|
| 48 / 96 / 192 kHz stereo | 626 bytes | 2504 bytes | 2518 bytes |
| 44.1 kHz stereo | 682 bytes | 2728 bytes | 2742 bytes |

These totals assume no RTP extensions/CSRCs or SCMS-T. The AOSP transport subtracts its LHDC media-header overhead from the available peer/stack MTU and programs the encoder accordingly. More than 1000 bytes in one media packet is valid because a packet can contain multiple coded frames.

Our L2CAP media MTU is 1800 bytes. With the ordinary headers above, 1774 bytes remain for coded frames. At 192 kHz / 24-bit / 1000 kbps, two frames fit: 1252 codec bytes, 1254 bytes in the decoder-input payload including the LHDC media header, and 1266 bytes in the complete RTP media SDU. Four frames require a larger negotiated MTU; they do not fit our 1800-byte SDU. At 900 kbps and 48/96/192 kHz, three 564-byte frames fit: 1706 bytes with these headers.

The codec frame header has a ten-bit per-channel byte count. Its representable stereo payload is at most 2046 bytes, or 2048 bytes including the coded-frame header. This wire-format bound is separate from the much smaller frames emitted at the reviewed encoder's 1000 kbps ceiling.

## Current buffering chain

| Stage | Capacity / ownership |
|---|---|
| VHCI ingress | 4096-byte bounded SPSC byte ring; maximum H4 packet 1029 bytes including H4/ACL framing. |
| Controller ACL | Actual Classic controller ACL payload 1021 bytes in recorded boots. A larger logical media SDU uses ACL fragmentation/reassembly. |
| Host receive credits | Derived from ingress capacity minus its control reserve; three in the recorded image. Credits return after host packet dispatch, before decoder consumption. This protects ingress, not sustained decoder throughput. |
| L2CAP media | Negotiated maximum SDU 1800 bytes. Reassembly is independent of codec frame boundaries. |
| Encoded decoder input | Eight 1800-byte slots: 14400 payload bytes, 14528 bytes including slot metadata, 14536 bytes including the tracked allocation header. The active slot stays immutable until decoder/PCM continuation retires. |
| LHDC coded-frame payload | Previous port and adapter cap: 1536 bytes. The corrected port reserves 2048 payload bytes and the adapter accepts the 2046-byte wire payload bound. Matching pinned private-header overlays keep workspace and adapter structure layout consistent. Active-LHDC storage increases by approximately 512 bytes; the sibling decoder is untouched. |
| LHDC codec fragment assembly | Existing bounded 3072-byte allocation ceiling retained. This is distinct from standard HCI/L2CAP fragmentation and is not proof of all vendor-specific fragment formats. |
| Decoded LHDC PCM at 192 kHz | 960 stereo frames per coded 5 ms frame; 7680 bytes in S32 containers for 24-bit audio. Retained decoder workspace is borrowed until PCM retirement. |
| Copied PCM input at 192 kHz | 1024 stereo frames, 8192 bytes. The 4 ms sizing request rounds up to the next power of two, so actual capacity is 5.33 ms. |
| Processed PCM output at 192 kHz | 3072-frame / 16 ms target; 3584-frame / 28672-byte allocated output capacity in seven 512-frame pages. |
| I2S DMA | Three 128-frame stereo S32 descriptors: 384 frames / 3072 bytes, or 2 ms at 192 kHz. |

The 5 ms coded-frame interval at 192 kHz contains 960 stereo sample frames. The average processing budget is 5.208 microseconds per stereo sample frame, not 5.208 microseconds per LHDC coded frame.

## Changes and limits of the finding

- Keep the requested 1800 x 8 packet queue and matching L2CAP MTU. Raising every slot to 8192 bytes is not required by the reviewed negotiated packetization and would spend substantially more SRAM.
- Correct both the adapter and private decoder payload bounds together. Generated private headers are source-pinned, include paths select the overlay consistently, and compile-time checks reject an undersized decoder layout.
- Replace ten variable-modulus packet-ring indexes with bit masks. Initialization already requires a nonzero power-of-two depth. No speedup percentage is claimed.
- The original 1008-byte input cap could reject valid aggregated packets above 1 KB. The 1536-byte single-frame cap was not the cause of the earlier recorded 628-byte packets being dropped: those contained one 626-byte frame and a two-byte LHDC media header.
- A full queue and an oversized packet are different failures. Previous RF captures recorded queue-full/PCM/DMA scheduling problems with 628-byte input and zero oversize rejects. Correct buffer lengths do not establish sustainable maximum-LHDC processing headroom.
