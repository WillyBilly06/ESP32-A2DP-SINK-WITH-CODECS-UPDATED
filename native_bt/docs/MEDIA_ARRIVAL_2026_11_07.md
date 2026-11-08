# HCI arrival timing and ready-owner deadlines — 2026-11-07

Current development image: **ff357cd8e33ed79c8193d90bd1cc4984668ca1d68f824f3a282cf7057d683180**. COM6 upload/full readback and owned VALID selection pass; automatic Bluedroid restore is disabled and the original application is preserved. UART and its live identity-checked viewer remain at **921600 baud**.

This checkpoint adds packet-bound timing and removes redundant deadline queries. **LHDC continuity and full feature qualification remain incomplete.**

## Packet-bound timing

The diagnostic profile has an eight-slot, 112-byte SPSC metadata FIFO. The controller callback records only the real HCI queue cursor, callback timestamp, wire size and ACL flags before the real queue publishes the packet. It performs no payload copy/hash, allocation, formatting or waiting. Rejected/non-enqueued packets and FIFO overflow explicitly invalidate the diagnostic without changing transport behavior.

The owner matches the actual post-pop cursor and header. Media association additionally requires a complete current Basic L2CAP/RTP packet, matching sequence/timestamp/SSRC and a media payload view inside the current HCI buffer. Fragmented, deferred and raw-codec media report unavailable timing. No borrowed payload pointer is retained.

Arrival fields share the existing media-version fence and immutable first-starvation/first-finalized-gap latches. An incoherent snapshot clears arrival validity. A bounded IRAM getter records published, unretired ACL metadata at a gap; a fault or concurrent retirement returns UINT32_MAX/unknown. This count includes an ACL being synchronously dispatched and is not solely the real queue occupancy.

The getter's actual LX6 assembly is in IRAM, reads only internal atomic words and contains no call, retry loop or flash-data dependency. Arrival timestamps describe the **VHCI callback**, not RF reception. A zero pending count does not identify the sender, radio or closed controller as the underlying cause.

This profile adds **168 bytes persistent data, 220 bytes IRAM text, 1,404 bytes flash text and 400 bytes flash constants** versus b587. It adds no audio capacity, heap allocation or task. It is compiled only with scheduler/audio diagnostics.

## Equivalent deadline short-circuit

The owner already polls every protocol and application subsystem before calculating its sleep. Previously it calculated fourteen additional unsigned deadlines even when the initial runtime query had returned zero. Their renderer checkpoints also ran. No later unsigned deadline can reduce that zero minimum.

The owner now skips those fourteen read-only queries only when the initial runtime result is zero. Protocol polling, final ready-task yield, event notifications, positive-deadline correction, affinities and priorities are preserved. When the initial runtime result is nonzero, the remaining collection still runs. A new packet arriving midway through that collection is not covered by this optimization.

The observed-zero mask now refers to queried values; omitted queries are not evidence that a subsystem has no pending work. The owner_wait_reasons record explicitly declares the short-circuit mode. The runtime bit reflects the runtime query rather than a subsequent BLE-parameter query.

This scheduling change adds **no RAM/IRAM**, removes 36 bytes flash text and adds 48 bytes diagnostic constant text, relative to the timing-only image.

## Host and target checks

All recorded current-source hashes match:

- ASan/UBSan: **36,864** cursor/header cases across all 4,096 ring positions, selected H4 sizes, zero/wrapped timestamps, nonmutating lookup failures, explicit overflow/rejection/missing-record handling, complete RTP identity and invalid fragment/deferred/raw/truncated views.
- ASan/UBSan: **20,000 actual HCI SPSC packets**, intervening events, a three-credit fixture, cursor/time wrap and repeated media lookup before retirement. Removing the cursor identity check is detected by the negative control.
- Actual output latches, partial-prefix refill/order/expiry/cancel, startup/wrap/coherence and bit-exact split/full integer DSP with EQ/volume changes pass at 100- and 1000-Hz simulated ticks. These are host checks, not acoustic proof.
- ASan/UBSan: **1,148,576** actual original/current owner collection cases. An independent minimum-deadline oracle covers all 32,768 zero masks, 32 profile/restart guard combinations, elapsed time, wrap and random full-width waits. The result is unchanged, and fourteen queries are omitted only for an initially runnable runtime. Forcing the old collection fails the query-count negative control. This tests control flow with mocked read-only getters, not complete protocol lifecycle behavior.
- Full ccache/compiler-input recheck, source-stability, workspace and offline image admission pass. Current cold target check: **37 configurations / 173 callbacks**, tracked peak 56,899 bytes, minimum free after allocation 102,036 bytes and stack low-water 3,600/8,192 bytes. No boot cases were added.

Reproduction:

~~~powershell
python native_bt/tools/test_media_arrival.py --out native_bt/_build/arrival_check
python native_bt/tools/test_owner_ready_wait.py --out native_bt/_build/owner_wait_check
python native_bt/tools/test_probe_output_format.py --compact-copy --block-dsp --priority-order --word-dsp --starvation-trace
~~~

The first two test programs preserve their intentional negative-control binaries and sources. Symbols are retained through verified lossless compression where necessary.

## Physical observations

Approved Redmagic/native YouTube, LHDC V5 / 192 kHz / S24 / stereo / explicitly selected **900 kbps**, volume 150. Each owned excerpt had an independent phone-side 14-second pause guard. Music metadata confirms the [official Get Lucky audio](https://www.youtube.com/watch?v=5NV6Rdv1a3I); the other fixture is the public 120 BPM metronome. No live 1 Mbps result is claimed.

| Image / fixture | Final unfilled PCM frames | First-gap callback-to-admission | Unretired ACL at first gap |
| --- | ---: | ---: | ---: |
| Timing-only 5b38 / metronome | 3,200 | 932 us | 0 |
| Timing-only 5b38 / music | 4,928 | 2,930 us | 1 |
| Ready-owner ff35 / metronome | 5,696 | 1,549 us | 0 |
| Ready-owner ff35 / music | 3,200 | 925 us | 0 |
| Ready-owner ff35 / music repeat | **0** | No finalized gap | No finalized gap |

The timing-only music packet arrived 2.93 ms before admission, leaving only 469 us before the finalized gap. This proves a host-side delay for that identified packet. The first timing-only metronome gap instead captures decoding still underway after admission.

The later metronome gap has empty PCM, the decoder waiting for encoded input about 11 ms, no unretired ACL metadata, and the last observed ACL callback about **32.8 ms earlier**. These observations do not support treating every gap as slow decoding or a host queue backlog. Flow-credit/controller/source delivery must be examined separately.

All five windows have FIFO status/lost zero, no invalid/malformed UART rows, zero descriptor/refill-contract errors, and accepted/rendered/DMA totals agreeing after stop. Their recorded maximum adjacent RTP step is 960 ticks. Those facts do not establish RF losslessness or continuous sound.

Measured query counts verify the intended work reduction:

| Ready-owner window | Runtime queries | Each of 14 optional queries | Queries omitted |
| --- | ---: | ---: | ---: |
| Metronome | 2,666 | 1,554 | **15,568** |
| Music | 2,440 | 1,666 | **10,836** |
| Music repeat | 2,131 | 1,689 | **6,188** |

Before the change each optional getter ran as often as the runtime getter. Observed average whole-collection elapsed time changed from 855/837 us before to 564/618/738 us afterward. These elapsed cycle counts include preemption, IRQ time and renderer service; they are not isolated CPU instruction costs.

Gap totals are mixed, and the windows have different RF/runtime histories. The ready image had an unobserved YouTube playback interval before the controlled excerpts; it was paused before comparison. Minimum heap and other since-boot counters include that history and must not be treated as equivalent-duration RAM comparisons. Stopped free heap is 58,688 bytes; observed minimum/largest free block are 21,776/30,720 bytes, and decoder/logger stack low waters are 5,904/1,384 bytes. Decoder residency remains 60,704 bytes.

The cleanup is retained as an equivalent, measured work reduction. **It is not established as an underrun fix.** One clean finite repeat does not complete continuity, coexistence or headroom qualification.

## Remaining transport investigation

The installed controller send path was inspected read-only. The valid-packet path copies the complete caller packet into controller-owned storage before posting its controller interrupt. The public SDK header describes the send-available callback as readiness notification and requires checking actual availability before each send.

This supports further review of the owned extra in-flight readiness fence, but no fence or buffer-lifetime policy has been changed here. An observation of SDK availability versus that fence is still needed before attributing a delivery stall to it. Vendor addresses are evidence only; no private entry point or register is used.

A source comparison also confirms that the original Bluedroid application remains Classic-connectable/non-discoverable while connected and restarts BLE advertising. Disabling those operations would not reproduce its policy. Renderer steps already yield to equal-priority host work, the actual tick is 1000 Hz, and the final notification wait preserves pending wakeups. No duplicate delay/yield or speculative priority change was added.

## Evidence and open scope

Experiment root: native_bt/_build/experiments/media-arrival-1794106377362.

Timing-only archive: native_bt/_build/hardware/runs/1794107357583631000-5b38eac1784a.
Ready-owner archive: native_bt/_build/hardware/runs/1794108421924184200-ff357cd8e33e.

The [compact result index](MEDIA_ARRIVAL_RESULTS_2026_11_07.json) binds exact captures, tests, source hashes and images. Raw/failed receipts, negative controls, ELF/binaries and traces are preserved. The phone is paused, I2S is stopped and UART source 71 is live at 921600.

Continuous 192-kHz/S24/LHDC-1-Mbps playback with measured headroom, encrypted BLE/GATT coexistence, live-control acoustics, long-duration clock behavior, security/OTA/product/lifecycle parity and comparative whole-host RAM remain open. The one-to-one PCM profile still has adaptive clock disabled. This remains unqualified development firmware.
