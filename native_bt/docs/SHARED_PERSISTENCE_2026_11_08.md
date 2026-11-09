# Shared BLE persistence worker — 2026-11-08

Development image: **fb87d978320c1958a777479b67cef06c1ae1987513dcf4b274495fb4da458584**. Uploaded to COM6 at 921600 baud and verified by full readback at 115200. Firmware and the passive raw-UART monitor use **921600 baud**. The original application/layout remain preserved, the owned image stays selected, and automatic Bluedroid restoration is disabled. This checkpoint is not a release or full feature qualification.

## Scheduling and ownership

CCCD restoration/persistence now shares the existing EQ/control-persistence FIFO and worker. Previously CCCDs used the general provider worker while EQ used a separate worker. The persistence worker runs at priority 4 on Core 0, below decoder priority 21 and DSP helper priority 20. General provider/OTA work remains priority 4 on Core 1. Controller priority23, host22, renderer22, decoder21, helper20 and logger1 retain their existing priorities/cores. Reconnect preparation retains its previous Core 0/priority 1 policy.

No task, stack or audio buffer is added. The existing persistence queue grows from 4 to 5 pointer entries, preserving four ordinary control admissions plus the independently reserved single CCCD obligation. General and completion queues retain their capacities. The worker performs one admitted job at a time; queue-full and allocation failure remain explicit. Worker creation failure retains queued persistence obligations and their retry deadline rather than ACKing an uncommitted value. Completion admission can block only the background worker; the host never waits for it.

Actual ATT completion still requires successful storage, the matching live connection generation/token and the same committed bond incarnation. Pre-pair changes are merged before bonded attach; late changes after a copied save require another durable save. Stale completions cannot ACK a reused handle. EQ operations remain ordered and are not coalesced. OTA/provider hooks and lifecycle progress stay in the general lane. Existing bounded DSP publication handles cross-core producers; the audio/DMA owner applies changes at its established safe boundary.

## Separate timing records

A new individually checksummed **gatt_cccd_work** record reports the exact handle, generation, token, job kind, executed/core-valid flags, actual worker core and three timestamps. It separates queue wait, storage-call wall time and time from storage return to host-owner observation. The last interval includes completion publication/scheduling, not solely owner instructions. Wall times include preemption, SDK locks and IPC; they are not CPU-cycle costs or exact flash-program durations.

The timestamps reuse the CCCD job's otherwise unused command-reply storage through a byte-aligned union. Size/alignment assertions preserve the previous allocation size. Internal timestamp bytes are copied with memcpy, avoiding unaligned uint32 access. Worker/host ownership crosses the existing work/done queues. The background worker never writes into the host's single-producer trace ring; the owner copies the completed timing snapshot into the existing bounded ring. No keys or storage payloads are logged. Formatting and UART waits remain in the priority-1 logger. There is one extra record per CCCD storage job, with no per-media-packet print.

## Source and target checks

The actual adapter/protocol/NVS-shim host suite passes, including seven deferred-arrival and nine failure/retry cases, the five-entry saturation case, failed worker creation with all five obligations retained, ordered EQ commits, stale generations, authorization, OTA completion fencing and retirement. Both-core timing flags, unexecuted results and uint32 timer wrap pass. The source receipt binds 74 inputs. These are deterministic shims, not RF or real-flash stress evidence.

The actual trace/formatter/ring fixture passes ASan/UBSan with the new records, immutable copies, wrapped timing subtraction, invalid-clock flags, full/drop behavior and a stalled UART consumer. Its source receipt binds 11 inputs. The first BLE-suite invocation passed its functional assertions but exceeded the existing 512 MiB rebuildable-output gate by about 38 KiB. Its failed receipt remains. Completed passed test binaries and the completed compiler log were archived losslessly with raw SHA256 checks; rerunning the suite passes the unchanged gate. The limit was not raised and failed evidence was not discarded.

The ccache target build retained 10,425 unchanged source inputs. Exact-image admission and the ISR placement/reference audit pass, with optional task/tick/cycle profilers OFF. Relative to 7d131bf9e47d, linked IRAM/data/BSS sizes are unchanged; flash text increases 264 B and rodata 336 B. The additional queue payload is 4 B. The existing cold checks pass 37 configurations / 173 callbacks, tracked peak 56,899 B, minimum free 106,128 B and stack low-water 3468 B. No boot watchdog/backtrace/idle-starvation markers were found. The cold minimum precedes the live persistence overlap and does not prove unchanged streaming peak RAM.

## Live comparison

Only COM6 and the approved Redmagic were used. Both candidate excerpts used native YouTube at LHDC V5 / 192 kHz / 24-bit stereo /phone-selected fixed 900 kbps, with an independent 14 s phone pause guard and an actual I2S upper bound below 15 s. Source/format/pause/image checks passed. **This is not 1 Mbps or acoustic qualification.** The unchanged-EQ roundtrip proves protocol response/readback, not an audible EQ-transition test.

| Image/workload | Phone subscription-enable wait | Controls independently observed before pause | Initial / final unfilled PCM frames | Descriptor / new watchdog events |
|---|---:|---|---:|---:|
| Previous7d131bf9e47d music |1257ms|No; completed after pause|1152 /1152, stop tail|0 /0|
| Previous7d131bf9e47d metronome |779ms|Yes|0 /0|0 /0|
| Candidate music,13.248s I2S upper bound |268ms|Yes: ACK/readback/original CCCD restoration|0 /0|0 /0|
| Candidate metronome,13.296s upper bound |268ms|Yes: ACK/readback/original CCCD restoration|2688 /2688, stop tail|0 /0|

The candidate metronome's first finalized gap was bounded 2.838–3.111 s after the requested pause, after its acknowledgement, with raw/ready queues empty. All counts are retained. The music capture had 182 CRC-verified/15 legacy-unverified records; metronome had 168/15. Neither had malformed records. Final host/worker trace loss, snapshot/UART loss, media-arrival loss and SMP trace loss were zero.

All six storage timing records independently report actual Core 0 execution. Queue waits were 2,932–5,228 us; storage-call wall times 867–32,017 us; post-work-to-owner delays 607–4,220 us. Total queued-to-owner intervals were 8,361–36,315 us. Previous owner-stage intervals were about 0.44 s for music restoration and 0.54–1.44 s for saves; those older records did not separate queue wait from storage execution. The phone callback includes additional radio/Android scheduling, so its 268 ms is not equated with the firmware's 8–36 ms.

One previous music attempt took 6.694 s from Android's connect request to its connected callback, while firmware key resolution took 53.897 ms. That earlier connection delay is a separate unresolved behavior; it must not all be attributed to NVS execution. Candidate key-resolution intervals were 29.341 and 32.024 ms.

## RAM result and remaining work

Before the first candidate excerpt, free heap was 69,260 B versus 69,264 B before the previous normal-image excerpt. After both candidate excerpts, stopped free heap was 62,084 B versus 62,088 B previously, consistent with the extra 4 B queue entry. The largest stopped block was 59,392 B and decoder stack low-water 5912 B. These endpoints do not prove leak freedom.

The candidate's minimum free heap was **11,836B**, versus 16,108 B in the previous first music trace and 16,088 B after its second trace. The 4,252 B lower two-trace minimum is an unresolved transient RAM cost, despite unchanged job/stack sizes and near-identical endpoints. The faster persistence worker overlaps SMP retirement in the event timeline; that is an investigation lead, not proven allocation-level causation. This change is retained for measured control latency, **not presented as a RAM improvement**. Worker/allocator overlap needs direct measurement before claiming minimum-RAM behavior.

Earlier active PCM gaps and CPU1 starvation remain unresolved; the two finite candidate excerpts do not establish sustained continuity. Full 1 Mbps headroom, long coexistence runs, live regression of all codecs/features, security/lifecycle/OTA stress, acoustics and an equivalent Bluedroid RAM/CPU comparison remain open. These finite checks do not qualify a stable release. Main later received development source at the user's explicit request; see the [reconnect stack follow-up](RECONNECT_STACK_2026_11_08.md).

Evidence: native_bt/_build/experiments/cccd-persistence-lane-1794186314835 (baseline, source/test bindings, linked sizes, retained quota failure and lossless archives); hardware/runs/1794186688218720800-fb87d978320c/traces/shared-persistence (raw traces, client observers, phone format and summaries). Previous normal-image traces remain under runs/1794185326684129200-7d131bf9e47d/traces/reconnect-core0-normal.
