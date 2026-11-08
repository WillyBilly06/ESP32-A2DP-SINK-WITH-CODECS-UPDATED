# Packed encoded-media storage — 2026-11-08

The candidate packs small encoded packets inside the existing physical pool. Production selection is limited to LHDC V5 at 192 kHz; other codec queue policies retain their fixed layout. This is a development checkpoint, not a completed continuity or feature-parity qualification.

## Ownership and capacity

The selected LHDC pool remains eight 1,816-byte physical slabs: 14,528 bytes, or 14,536 bytes including its existing allocation header. Every packet still supports the complete 1,800-byte payload maximum. There is no per-packet allocation, additional task, decoder precision change, core/priority change, startup-guard change or controller-window increase.

A record retains its existing 16-byte metadata header and aligns payload/storage to eight bytes. An observed 566-byte packet consumes 584 bytes, so three records fit in one slab. Raw pool capacity for that size rises from eight to 24 records. A full 1,800-byte packet still consumes exactly one slab. This improves utilization of the existing allocation; it does not reduce that allocation or prove a decoder-speed improvement.

Logical media counters still count packets. Separate physical page counters and single-owner offsets protect slab ownership. The producer seals a short page tail only after space exists for the next page; the sentinel and next record become visible through the existing release publication of media_written. The consumer skips at most one sealed tail when resolving a published record, then releases the previous physical slab. A short tail without room for a header is skipped without reading it.

A borrowed decoder input pointer remains contiguous and pinned through progress/resume, pending PCM, asynchronous loan completion and cancellation. It is never flattened into a temporary stack buffer. Start/suspend/reset/reconfiguration reclaim page positions only while admission is closed and existing PCM-loan retirement requirements have been met. Offset state distinguishes an unused page from a page-counter wrap through zero.

Receive-credit decisions count completely free physical slabs, not logical packets or unused bytes in an open slab. Existing outside-window and partial-PDU allowances remain. Therefore a small packet never earns credit on the assumption that the next packet will also be small. The controller window stays three ACL packets. With uniform 566-byte records and no outside permits, the conservative one-slab reserve can allow approximately 21 records; 24 is the raw pool capacity, not a promise that every controller state admits 24.

queue_layout now documents bit7 as packed storage and retains the physical count in bits0..6. All consumers were audited. Startup diagnostics bound logical packet counts by the minimum aligned record size, and codec_queue_memory explicitly reports packed_records. Duration and decoder/drop counters remain independent of physical page counts.

## Source and host verification

- Actual packed-worker fixture: 24 x 566-byte records in the unchanged pool, eight untruncated 1,800-byte records, oversize rejection, mixed lengths, logical/physical counter wrap, exact byte integrity through resumable borrows, copied-PCM backpressure, lifecycle teardown and fixed-layout restoration.
- Four existing asynchronous PCM-loan groups additionally execute with packed input, covering acknowledgments, cancellation before teardown, deadlines, stale retirement and failed completion.
- Two host threads exchange 20,000 mixed-size records through the real worker under AddressSanitizer/UndefinedBehaviorSanitizer. This is host concurrency evidence, not ESP32 timing evidence.
- Removing the sealed-tail marker in a generated negative source causes the sanitizer to reject interpreting uninitialized tail metadata as a record. The production candidate passes the same harness.
- Portable core: 191/191 checks pass, including the existing worker/loan and concurrent core tests.
- Actual extracted credit policy: 252 fixed/packed capacity/maximality/wrap/LE boundary cases, plus two 50,000-step existing mixed Classic/LE/fragment/deferred-PDU models and retirement-notification checks. The synthetic packed boundaries deliberately separate a large logical count from physical occupancy. These models are not RF proof.
- All 80 real-codec deferred cases pass in both fixed and packed mode. Every PCM byte and sample timestamp matches the synchronous reference, and paired full-output hashes/counts match across the two layouts. This includes all existing cold configurations, fragmentation/batching and the maximum LHDC fixture, with ESP32 source conditionals executed on the host and sanitizers enabled.
- Six actual app/profile worker variants pass: SBC, multi-codec, all-codec, output, asynchronous copied output and asynchronous PCM-loan output. Output/RTOS remain mocked in these cases.
- Frozen trace/formatter and its negative controls, nine CMake cases and 15 image-admission cases pass on the final source.

The earlier credit test extracted unrelated duration code and failed before execution; its extraction boundary was corrected to the actual credit functions. Two functional test runs were initially held by the existing 512 MiB rebuildable-artifact gate. Completed binaries/logs were archived losslessly with hash/round-trip checks, then the affected runners completed with the same gate unchanged. Failed/held receipts and generated negative sources remain available.

## Target build and comparison

Candidate image: 3b246522d523a36e108cac856a6b76eb5910ed43eabede6f939de2daadd3e3b3.

The ccache target build, 10,424-input source check, offline image admission and audio ISR audit pass. Relative to image2492e98a66c3, linked BSS rises16 bytes; DRAM data and IRAM are unchanged. The encoded pool remains unchanged. The admission and audit do not establish runtime continuity.

Two fresh fixed-layout controls on image2492e98a66c3 used native YouTube metronome, LHDC192k/S24/stereo/fixed900, ten-second requested intervals and independent fourteen-second pause guards. Their complete totals were 2,880 and0 final unfilled PCM frames. The first control's first finalized gap was bounded8.986–9.324 seconds before the pause request, with both PCM queues empty. The second had no latched shortage. This intermittent baseline prevents treating one clean candidate excerpt as proof of a fix.

### Device result

COM6 was written at921600 and verified by readback. The original application is preserved and automatic Bluedroid restoration remains disabled. Its immutable archive is runs/1794165938472508900-3b246522d523. The existing37 cold configurations /173 PCM callbacks passed, with56,899B tracked peak,106,144B minimum free heap during those allocations,3,580B decoder-stack low-water and12,652us maximum cold decode. Boot health showed no watchdog/backtrace/idle-starvation markers.

| Candidate workload | Initial shortage | Recovered | Final unfilled | First finalized gap relative to pause request |
| --- | ---: | ---: | ---: | --- |
| Metronome1 | 3,264 | 0 | 3,264 | +2.856 to+3.114s, after acknowledgment |
| Metronome2 | 2,496 | 192 | 2,304 | +2.866 to+3.137s, after acknowledgment |
| Music1 | 3,520 | 2,368 | 1,152 | +1.450 to+1.712s, after acknowledgment |

Each requested interval was10s, with its independent14s pause guard and a verified I2S span below15s. One immutable first-gap identity was observed per stream. No pre-pause finalized gap was recorded in these three finite candidate windows; all totals still include the complete stop/tail. The variable fixed baseline and limited duration do not establish a causal continuity fix or sustained reliability.

Queue high-water reached10, then11 packets, compared with7 on the two fixed controls. Live geometry verified packed_records:true, eight physical1,816B slabs,1,800B maximum payload and14,536B accounted allocation. The existing50ms guard measured50,206us in the first candidate, with eight encoded packets at enable. Thus the additional storage capacity was actually used on target without enlarging the pool or startup guard.

All five control/candidate windows retained zero queue/decode rejects, worker/helper/descriptor/refill-contract faults, host/worker trace losses and snapshot/UART losses. Accepted/rendered/DMA-source totals matched. CRC-verified counts were148/134/256/222/248, with1/1/1/1/3 unverified legacy records and no invalid records.

Phone volume was150/150. Valid active DSP snapshots in every window showed matching gains11/8/12, volume127 and flags12, with nonzero rendered frames and signal peaks. Stopped or incoherent DSP snapshots contain zeros and were explicitly excluded from configuration evidence. These digital observations do not constitute acoustic validation.

After candidate playback, tracked codec/queue resident bytes remained60,704, with63,004B free internal heap,29,356B minimum free heap and59,392B largest block in the recorded snapshot. These runtime figures are distinct from the cold codec-allocation minimum. The16B global-state increase is disclosed separately.

Full1Mbps streaming, sustained BLE/control headroom, acoustic quality and complete protocol/security/OTA parity remain open.

## Artifacts

native_bt/_build/experiments/packed-media-1794164796155 contains the design/before-source snapshots, paired real-codec receipts and full-output hash comparison, final credit/core receipts, build/flash logs, memory.json, fixed controls and binary/log archive manifests. Final packed ownership fixture receipt: native_bt/_build/packed_media/1794165640088453700/results.json. The mutated padding source is preserved separately from production. No hardware evidence is inferred from the host fixtures.
