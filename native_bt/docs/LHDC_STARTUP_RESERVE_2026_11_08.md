# LHDC startup reserve — 2026-11-08

The high-rate LHDC startup floor is provisionally **80 ms**, using the existing encoded and PCM pools. This adds 30 ms to the nominal startup hold. Lower-rate LHDC remains at 40 ms; other codec policies are unchanged. The implementation uses the existing event/deadline hold rather than a polling loop or a blocked callback.

## Measured reason

The [stream-boundary investigation](A2DP_STREAM_BOUNDARY_2026_11_08.md) found an active music gap on the 50 ms image, 165.6 ms after Streaming and 13.07 seconds before accepted Suspend. Approximately 123.4 ms of host time carried 75 ms of RTP audio in the early interval, leaving about 48.4 ms of cadence deficit before decode/refill time. That observation uses host timestamps and does not determine the RF/controller/source cause. The startup capture had only four encoded packets at enable.

The existing decode profile covered 2,624 frames and averaged about 2.29 ms per 5 ms audio unit across the instrumented frame stages. It includes preemption and the complete pause/tail, and is not pure CPU utilization, a worst-case bound or 1 Mbps evidence. The channel envelope already contains the inner channel stages; it was not counted twice. A static inspection also confirms existing Xtensa multiply-add/subtract instructions in the compiled IMDCT. No decoder arithmetic was changed in this stage.

The supplied local Bluedroid configuration pins its host to Core 1. This trial retains all current affinities and priorities.

## Exact target image

Image: 57850f2d36b9a7ea9e660a584d130fbfa39779946d238f0771de54460457c221.

COM6 was written at 921600 baud and verified by readback. The original application is preserved and automatic Bluedroid restoration remains disabled. Archive: runs/1794169443766106200-57850f2d36b9. The ccache build, 10,424-input source stability check, offline admission and audio ISR audit pass. Linked SRAM, IRAM and image size match the 50 ms diagnostic baseline.

The existing 37 cold codec configurations / 173 PCM callbacks pass: 56,899 B tracked peak, 106,144 B minimum free heap during those allocations, 3,468 B decoder-stack low-water and 12,575 us maximum cold decode. No boot watchdog, backtrace or idle-starvation markers appeared. These cold checks do not qualify RF playback.

## Guarded YouTube comparison

Native Redmagic YouTube, LHDC V5 / 192 kHz / 24-bit stereo / fixed 900 kbps. Each requested interval was 10 seconds with an independent 14-second phone pause guard; verified I2S spans stayed below 15 seconds.

| 80 ms excerpt | Observed hold | Encoded packets at enable | Initial shortage | Recovered | Final unfilled |
| --- | ---: | ---: | ---: | ---: | ---: |
| Music 1 | 80,429 us | 14 | 0 | 0 | 0 |
| Music 2 | 79,693 us | 14 | 192 | 0 | 192 |
| Metronome 1 | 79,876 us | 13 | 0 | 0 | 0 |
| Metronome 2 | 81,397 us | 14 | 0 | 0 | 0 |

No finalized pre-pause gap was recorded in these four finite windows. Full counts include the complete stop/tail. The music-2 event was bounded 2.840–3.126 seconds after the pause request and after its acknowledgment. The accepted renderer stop handoff was observed 923 us before that final gap; successful provider poll followed 23,613 us after it. Source inspection shows the renderer begins its 20 ms fade only when it handles the stop request, followed by silent DMA traversal. This identifies a stop-boundary event; it does not justify deleting its counter or claiming an acoustic result.

The encoded pool remains eight 1,816-byte slabs: 14,528 B, or 14,536 B including the allocation header. Maximum payload remains 1,800 B. Cumulative queue high-water reached 16 packets. All four windows retain zero queue/decode rejects, worker/helper/descriptor/refill-contract faults and host/worker/snapshot/UART losses. Accepted, rendered and DMA-source totals match. Verified CRC counts are 180 / 192 / 218 / 180, with 1 / 1 / 3 / 1 unverified legacy records and no invalid records. Valid active DSP snapshots and phone volume are preserved in all-window-checks.json.

The 50 ms baseline had a clean metronome and an active 960-frame music event. The variable baseline and short duration limit the comparison: this is a useful startup margin, not sustained continuity qualification. Fixed 900 kbps is the verified phone setting; the full 1 Mbps plus BLE/control target remains open.

## Source and regression evidence

Source review verifies that only the high-rate LHDC guard literal and its explanatory comment changed from commit 25030c16. No codec math, framing, queue capacity, precision, credit policy or task layout changed. Six final source-matched actual app/profile worker variants pass, including copied and immutable-loan async output. Frozen trace/formatter tests and negative controls pass, with nine CMake and 15 image-admission cases. The broader protocol/codec qualification remains scoped to its own dated evidence.

Artifacts are under native_bt/_build/experiments/startup-reserve80-1794169264156: source-review.json, memory.json, app-gates.json, all-window-checks.json, complete trace JSON/NDJSON and build/flash logs. Earlier preflight/finalization holds were caused by the unchanged 512 MiB artifact gate. Closed source ZIPs and copied test runtimes were consolidated byte-for-byte with manifests. Raw PDB data was consolidated exactly into completed-symbol-data.tar.xz; prior gzip wrapper hashes are recorded, while those wrappers themselves were replaced. Failed receipts and negative controls remain available.

Remaining work includes the stop-boundary transition, sustained maximum-rate/BLE/control headroom, acoustic validation and complete protocol/security/product/OTA parity. The full goal remains active and incomplete.
