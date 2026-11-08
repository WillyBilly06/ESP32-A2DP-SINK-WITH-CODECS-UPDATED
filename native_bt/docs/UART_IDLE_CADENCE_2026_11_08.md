# UART logging cadence — 2026-11-08

Firmware, build defaults and the verified COM6 raw-UART web monitor use **921600 baud**. The previous image and running monitor already matched that setting; this change reduces redundant idle diagnostic work.

## Runtime behavior

The existing priority-1 Core1 logger owns formatting and UART record writes. Its UART driver sleeps on transmit interrupts when space is unavailable. It can wait; this is not a zero-cost or universally non-blocking UART write. Bluetooth/decoder producers use the existing bounded event storage instead of waiting on UART output.

Normal operation now distinguishes these background activities:

- Existing bounded event and first-gap trace draining stays above the idle filter at its nominal 100 ms cadence.
- AVRCP metadata change logging keeps its nominal one-second cadence, outside the idle filter.
- During I2S playback, detailed counters retain the nominal one-second cadence. These are scheduler-dependent cadences, not deadlines or guaranteed sample rates.
- While output is stopped and the observed activity tuple is unchanged, full detail is emitted every fifth counter cycle. Other idle cycles emit a small liveness record so a passive reader can distinguish a quiet device from a disconnected UART.
- Changes in profile state, codec revision, PCM generation, profile errors, worker faults/drops or ignored/oversized dynamic packets reopen detail on the next counter cycle. The tuple is an activity trigger, not an atomic cross-subsystem snapshot. Other cumulative diagnostics remain available at the idle heartbeat.

All emitted records remain individually available in the existing raw-UART dashboard. No decoder precision, packet capacity, startup guard, codec policy, controller credits, priorities, audio buffers or core assignment changes are included. There is no additional task or heap allocation. The small activity tuple uses the existing logger stack; linked DRAM data/BSS and IRAM are unchanged.

A UART record takes more than one CPU cycle. At 921600 baud with 8N1 framing, 100 bytes occupy approximately 1.085 ms on the wire, before formatting and interrupt overhead. Moving UART waits off audio-critical producers and avoiding repeated formatting are the relevant protections. This change does not establish that logging caused the outstanding LHDC gaps.

## Build and source checks

The ccache firmware build completed with 10,423 source inputs stable. Offline image admission and the audio ISR audit passed. Actual frozen-trace/formatter checks and their two negative controls passed, including 9 CMake and 15 image-admission cases. The existing actual-output fixture still matches the media-tuple accessor and its concurrency/interference tests from the startup comparison stage.

Candidate image: 2492e98a66c3903110900fb2abda2aa42e08fd14826c97720f887256fa4fe165.

## Exact-image device checks

The image was written to COM6 at 921600 and verified by readback. The original application, bootloader/table and owned selection were preserved; automatic Bluedroid restore remains disabled. The existing cold check passed 37 codec configurations / 173 PCM callbacks, with a 56,899-byte tracked peak and 106,160-byte minimum free heap during those allocations. No boot watchdog, interrupt-watchdog, backtrace or idle-starvation markers were found.

A 15.441-second cold-idle capture had 3 full detail cycles and 12 liveness records: 114/114 CRC-verified records, 39,528 received UART bytes. After actual LHDC playback, a 15.325-second idle capture again had 3 detail cycles and 12 liveness records: 126/126 CRC-verified records and 46,715 received bytes, approximately 3.05 KB/s. The earlier idle capture with LHDC counters retained carried 135,609 bytes over approximately 9.58 seconds, about 14.15 KB/s. That is approximately 79% less measured idle UART traffic, consistent with suppressing four of five unchanged detail batches. This is a finite passive measurement, not a measured decoder-speed improvement. Logger stack low-water remained 1,324 bytes.

The first native YouTube attempt did not reach COM6 because the phone route was disconnected following the board reset. Its failed receipt is retained; zero decoded frames was not counted as success. The paired speaker route was restored and independently read back as active. The phone's settings then verified LHDC V5 / 192 kHz / 24-bit stereo / fixed 900 kbps.

A routed 10-second metronome excerpt passed the format, image, pause-guard and UART-source checks; actual I2S span was bounded below 15 seconds. There were 147 CRC-verified records, one unverified legacy record and no invalid records. Queue-full/decode rejects, helper/descriptor faults, host/worker trace loss and snapshot/UART losses were zero. Expected PCM backpressure remained counted without individual normal-mode records.

The total initial shortage was 576 frames, of which 320 were recovered and 256 finalized unfilled. The first finalized gap was bounded 0.750–1.008 seconds after the pause request, after its acknowledgment; counts are retained without removing the stop/tail. This is not a continuity qualification. Background detail delivery remained sparse under load: only one running-I2S sample was received in this excerpt. The source policy retains its nominal active cadence, but the capture does not prove one-second scheduling or freedom from all logging-related interference.

Artifacts: native_bt/_build/experiments/logging-921600-1794163845409, including before-idle.json, after-idle.json, after-playback-idle.json, failed and routed trace receipts/raw events, route/format readback and memory.json. The immutable deployment archive is runs/1794164043678472700-2492e98a66c3. Source-matched frozen-trace tests are under native_bt/_build/dispatch_trace/1794164015130623500/results.json.

The broader custom-stack goal, maximum LHDC 1 Mbps continuity, acoustic performance and sustained BLE/control/security/OTA qualification remain open. This checkpoint is development firmware.
