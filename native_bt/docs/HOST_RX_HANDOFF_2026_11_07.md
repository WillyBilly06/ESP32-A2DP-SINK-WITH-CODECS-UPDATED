# Host RX handoff and deadline work — 2026-11-07

Current development image: `708940335ae512da0a470217381ae5ba7e07551da3bc8baab016d4c24335a7f9`. COM6 upload and full readback match; owned boot selection is VALID, automatic Bluedroid restore is disabled, and the original application is preserved. Runtime UART and monitor are both **921600 baud**. Offline admission confirms the current source inputs match the completed build. This remains unqualified development firmware.

## Retained changes

- AVRCP bearer wait calculation collects the AVCTP deadlines once rather than twice. A separate local bearer query preserves admission, stale-binding and failure work; standalone AVCTP users retain the full wait API. Mask scans stop after the highest active label, bounded by 16 labels. Timeout and wrap policy are unchanged. The deadline-only image uses 12 fewer flash-text bytes with no data, BSS or IRAM change.
- Sensitive HCI scratch erasure uses aligned volatile 32-bit stores with byte edges and an alias-safe type. Every ACL packet retains the existing sensitive classification, including possible SMP continuation fragments. Actual LX6 assembly uses MEMW/S32I.N in the word loop. A 587-byte H4 packet requires 149 fenced stores instead of 587; this is a store-count result, not a measured decoder speedup. No persistent erase state or extra buffer is added.
- An optional runtime checkpoint runs after a nonempty bounded host RX batch has returned and all borrowed RX callbacks and scratch erasure have completed. Normal HCI command pumping may already have run. The checkpoint precedes the runtime post-batch RX-credit reports and outbound ACL pumping; it does not retain packet views or wait for work. The existing runtime recursion/abort fence remains active. In the RenderFairShare build it calls the existing useful-pressure renderer yield predicate. No yielding was added inside VHCI send/readiness, ISR or borrowed protocol callbacks.

The last two changes together add **8 bytes linked BSS and 128 bytes flash text**, with no IRAM/data increase, compared with the deadline-only image. Core affinities, priorities, codec queues and PCM capacities are unchanged. These are bounded host-work changes; the radio measurements do **not** establish a continuity improvement.

## Host verification

Current-source hash checks pass for every input recorded by the following tests:

- 2,897,152 actual/reference deadline comparisons under ASan/UBSan: exhaustive 16-bit masks, randomized states, expiry/wrap, continuation/replies, stale/failed/busy bindings, unchanged queried state and two-to-one AVCTP scan count.
- 32,840 exact old/new erases under ASan/UBSan, with offsets 0–7, lengths 0–4096 and larger boundary cases. Guard bytes and sensitive-packet classification are unchanged.
- All 24 AVRCP wire/fragmentation/browsing/borrow regression cases.
- Four paired real-runtime cases under ASan/UBSan: interleaved credits, disconnect/reuse, competing loans, and RX-checkpoint ordering/recursion/abort. The checkpoint test also verifies the callback has retired, scratch is erased, each budgeted ACL is acknowledged exactly once, no RX invokes no checkpoint, and abort in the hook sends no stale credit. Compiling the old runtime passes the original three cases and fails the new ordering case as expected.

The final runtime test program passed all four cases, then its wrapper storage gate failed at 513.213 MiB. This failure is retained in the audit. Passing host objects were retired with hashes, debug symbols were losslessly compressed and verified, and the storage gate was rerun successfully at 511.796 MiB. Tested binaries, failed negative controls and raw device captures remain preserved. This is not an erased test failure.

The existing cold target check passes all **37 configurations / 173 callbacks**, tracked peak 56,899 bytes, minimum free after allocation 102,528 bytes and stack low-water 3,576/8,192 bytes. Its maximum decode time is a multi-unit packet measurement, not an individual 5 ms frame deadline. No boot selftests were added.

## Matched phone observations

The approved Redmagic used its native YouTube app, the same public 120 BPM metronome excerpt, LHDC V5 / 192 kHz / stereo / S24 / explicitly selected 900 kbps / volume 150. Each owned playback window had an independent phone-side 14-second pause guard. No 1 Mbps claim is made.

| Image / run | Initial shortage | Recovered | Final unfilled PCM sample frames | Accepted = rendered = DMA |
| --- | ---: | ---: | ---: | ---: |
| Earlier 2e47 baseline 1 | — | — | 4,608 | See raw trace |
| Earlier 2e47 baseline 2 | — | — | 2,624 | See raw trace |
| Deadline-only df4d run 1 | — | — | 3,072 | See raw trace |
| Deadline-only df4d run 2 | — | — | 11,776 | See raw trace |
| Current 7089 run 1 | 8,576 | 4,160 | **4,416** | 2,497,920 |
| Current 7089 run 2 | 14,592 | 5,568 | **9,024** | 2,525,760 |

Current descriptor and refill-contract errors are zero. Initial shortages that recovered are kept separate from finalized unfilled sample frames. The first baseline started near 1.996 s rather than the requested 1 s; the other recorded positions were near 1 s. Variable results and overlapping per-query timings prevent a speedup claim.

At the first finalized gap in each current trace, both PCM input and output were empty, the decoder had begun work about 1 ms earlier and one encoded packet remained queued. This identifies those events as PCM starvation while decoding was underway. It does not establish RF loss or explain every later gap. Identity-correlated controller-to-media timing is still needed.

One valid active runtime interval on the current image reports Core 1 host 42.56%, renderer 34.58%, controller 14.89%, logger 2.85% and idle 2.23%; Core 0 decoder 54.27%, DSP helper 27.47% and idle 18.07%. These elapsed task counters include interrupted-task ISR time and exclude the running tail. Snapshots are not simultaneous; they are not pure instruction costs or proof that a priority change will help.

## UART and remaining work

A new passive **20.134-second** live-source window receives **344,361 bytes / 842 consecutive records**, all CRC verified, with zero invalid/malformed records, UART writes or monitor-requested resets. This checks the current workload at 921600, not maximum link saturation. The monitor remains live on COM6. UART formatting/transmission remains background work and costs CPU time. Configuration-on-change and heartbeat behavior from the [UART checkpoint](UART_921600_2026_11_07.md) is retained.

LHDC continuity, measured 1 Mbps headroom, encrypted BLE coexistence, live-control acoustics, long-duration reliability, security/OTA/product/lifecycle parity and the complete comparative RAM qualification remain open. This work must not be published as a qualified main-branch replacement.

Evidence: `_build/experiments/avrcp-deadline-1794097781529` and `_build/hardware/runs/1794102182584413400-708940335ae5/traces`. Earlier comparison images and raw traces are retained in the 2e47 and df4d run archives. The compact [result index](HOST_RX_HANDOFF_RESULTS_2026_11_07.json) identifies exact current-source test receipts and current trace files.
