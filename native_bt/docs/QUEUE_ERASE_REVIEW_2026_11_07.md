# Queue erasure review and input timing — 2026-11-07

The word-erase queue candidate is **rejected**. Host correctness checks passed, but phone windows did not establish a playback benefit. The original queue implementation is restored byte for byte; no firmware optimization from this experiment is retained.

Current COM6 image: **b587e3dd083ac4b421ec234ce5f3c46a5d946010af8e8143369ddad63cf97030**. Guarded upload and full readback pass, owned boot selection is VALID, automatic Bluedroid restore is disabled, and the original application is preserved. Runtime UART and its identity-checked live viewer remain at **921600 baud**.

Restored IRAM vectors/text, flash text/read-only data and initialized DRAM sections are **byte-identical** to the earlier 708940335ae5 image. Exact section hashes are preserved. Complete images differ in application metadata; their whole-file hashes do not match.

## Candidate and retained verification

Every ACL packet follows the existing sensitive-ingress policy. Consumed payload is cleared before the read cursor is published to the producer. The experiment replaced per-byte masked erasure with at most two contiguous spans and aligned, alias-safe volatile word stores. All edge bytes and ownership checks were retained.

Actual LX6 linking added 80 bytes flash text and no BSS/data/IRAM. Assembly used MEMW/S32I.N and a hardware word loop. These are code-generation facts, not proof of improved audio headroom.

The new host regression is retained:

~~~powershell
python native_bt/tools/test_queue_wipe.py --out native_bt/_build/queue_wipe
~~~

Each object compiles through ccache; programs run under ASan/UBSan. The reference is actual queue source pinned to commit 528a27ee, with canonical line endings and a checked SHA256. The optional --candidate argument permits replaying an archived implementation without replacing production source.

Both the experimental and restored implementations pass:

- **674,408 exact queue operations**: all 4,096 ring positions, 25 selected lengths including 587-byte media H4 packets, normal/sensitive records, output guards, short output capacity, 32-bit cursor wrap and randomized queue/control-reserve saturation.
- Five existing sensitive-host cases: ACL/events, borrowed callbacks, abort, late packets, malformed/error paths and control reserve.
- Existing concurrent **20,000-packet SPSC** host test, with ordered content and wiped ring/scratch storage.
- An intentionally incorrect erase is rejected. Its expected nonzero exit, executable, source and negative object remain preserved.

The retained test receipt matches all 13 current source/generated inputs. The original experimental driver and exact rejected queue source are archived with their recorded hashes. These are host checks, not RF/deadline qualification.

The restored cold target check passes **37 configurations / 173 callbacks**, tracked peak 56,899 bytes, minimum free after allocation 102,528 bytes and stack low-water 3,464/8,192 bytes. No boot cases were added.

## Phone observations and rejection

Approved Redmagic/native YouTube, the same 120 BPM metronome excerpt, LHDC V5 / 192 kHz / S24 / stereo / explicitly selected **900 kbps** / volume 150. Each window had an independent phone-side 14-second pause guard. No 1 Mbps qualification is claimed.

| Sequence | Image | Final unfilled PCM sample frames |
| --- | --- | ---: |
| Before candidate, run 1 | 7089 | 3,008 |
| Before candidate, run 2 | 7089 | 1,536 |
| Word-erase candidate, run 1 | 0001 | 62,720 |
| Word-erase candidate, run 2 | 0001 | 12,288 |
| Restored implementation, run 1 | b587 | 1,920 |
| Restored implementation, run 2 | b587 | 4,736 |

All six captures have zero descriptor/refill-contract errors and zero invalid/malformed UART records. Accepted, rendered and DMA frame counts agree at each stopped snapshot. That accounting does not establish continuous playback.

The large first candidate result is confounded by a **pre-decode input timestamp discontinuity**. RTP sequence stays consecutive while the maximum adjacent timestamp step is 27,840 ticks at 192 kHz: **145 ms**, versus the ordinary 960-tick/5-ms step. Its timestamp range is 2,526,720 ticks across 2,576 received packets. The ordinary 960-tick-per-packet model covers 2,472,960 ticks, a 53,760-tick/280-ms excess. This is a presentation-timeline observation, not a radio-loss counter or a valid quantity to subtract from underruns.

The other five windows record a maximum adjacent step of 960 ticks. Candidate 2 still has more unfilled frames than either preceding window and either reverse control. These finite observations justify declining the optimization; they do not prove a pure CPU regression or one universal cause for every gap.

## Separate gap classes

At candidate 1's first finalized gap, **896 decoded PCM frames** wait with no output ready. The decoder has entered wait_encoded_input. The renderer enters at 143,470,354 us, **32 us after** the 143,470,322-us refill deadline; the helper is READY.

At candidate 2's first finalized gap, PCM input/output are empty and the decoder has waited for encoded input about 3.6 ms. Its last media admission was about 9.1 ms earlier. The preceding baseline first gaps instead capture decoding in progress, one encoded packet queued and empty PCM. Initial recoverable shortages and finalized unfilled frames remain separate.

Current telemetry records the latest ACL arrival from any link, not an identity-bound arrival for each media packet. It cannot prove whether a specific late admission was delayed in the controller, over the radio or in the host. Small bounded arrival-to-admission correlation is the next discriminating measurement. Existing optional probe_ingress performs full-payload corruption fingerprinting; enabling it would not supply this timing.

One candidate interval reports Core 1 host 38.83%, renderer 36.94%, controller 14.39%, logger 3.31%, idle 3.37%; Core 0 decoder 54.85%, helper 25.35%, idle 19.80%. Counters include interrupted-task ISR time and non-simultaneous snapshots. No isolated cycle gain or priority change is justified. The one-to-one PCM profile reports adaptive_clock=0; long-duration drift remains unqualified.

## Controller ownership review

Pinned [BTstack ESP32 VHCI transport](https://github.com/bluekitchen/btstack/blob/9edb5d82ea6ec4466a393415d65030929330acdf/port/esp32/components/btstack/hci_transport_esp32_vhci.c) forwards SDK send-available notification as a transport packet-sent event. Its [HCI implementation](https://github.com/bluekitchen/btstack/blob/9edb5d82ea6ec4466a393415d65030929330acdf/src/hci.c) gates ordinary submission on packet-buffer reservation and releases the reservation on that event. This supports retaining the owned host's conservative completion fence. It does not establish that availability queries alone permit early buffer reuse.

No reference source was integrated. Owned VHCI readiness checks, completion fence, mandatory credits, security classification, queue capacities, affinities and priorities are unchanged.

## Build and evidence

Two restored builds compiled but were held by the 512-MiB storage gate at 512.016/512.019 MiB. Neither unadmitted output was flashed. Debug symbols were losslessly compressed with byte-for-byte decompression checks and SHA manifests. Final full ccache/compiler-input recheck, source-stability and storage gate pass at 511.294 MiB.

A stale expected hash was rejected before COM6 opened. The final request was bound to the freshly verified b587 image. Admission and full physical readback pass. Raw failed receipts, candidate ELF/binary, original queue source, test binaries and all captures are preserved.

Evidence root: native_bt/_build/experiments/queue-wipe-1794104179053.

Physical archives under native_bt/_build/hardware/runs:

- Baseline: 1794102182584413400-708940335ae5
- Rejected: 1794104534109868100-0001355e0ccf
- Restored: 1794105402090891100-b587e3dd083a

The [compact result index](QUEUE_ERASE_RESULTS_2026_11_07.json) records exact captures, stage-specific first-gap snapshots, source bindings and retained image. Phone paused, I2S stopped, UART viewer live at source 67.

LHDC continuity and all full-master gates remain open: 1 Mbps/headroom, encrypted BLE coexistence, live-control acoustics, long-duration streaming, security/OTA/product/lifecycle parity and comparative whole-host RAM. This remains development firmware.
