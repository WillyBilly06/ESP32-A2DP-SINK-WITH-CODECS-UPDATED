# Completed flash slice and PCM reserve, 2026-11-09

COM6 runs readback-verified independent-host image `b566da3f908f64c48d674e0869d66a41cc5c0c0605d223afacd6675e600de9cb`. This fixes observer token serialization and measures a completed cache-disabled slice. **LHDC continuity remains unresolved; this is development firmware and the master goal remains active.**

## What changed

The SDK acquires its flash-operation mutex inside spi_flash_disable_interrupts_caches_and_other_cpu. The previous observer allocated its token before calling that function, so concurrent callers could reuse a token or replace preparing metadata before obtaining the SDK lock. Token publication now happens only after the original function returns, inside the serialized cache-disabled interval. The original SDK disable and enable functions are still called exactly once. No SDK guard, cache register, scheduler policy or flash behavior is replaced.

The end wrapper timestamps the end of the confirmed slice before invoking SDK restore. While still serialized, it retains the longest completed token, begin and elapsed time. The existing live token also serves as the publication sequence: the background getter accepts a peak only between unchanged finished tokens. Reads make one attempt; an in-progress guard makes the observation unavailable. Early unarmed operation never reads the timer, and diagnostic token exhaustion remains unavailable rather than blocking flash.

Only the longest completed **confirmed cache-disabled slice since arm** is retained. It excludes SDK lock/IPC preparation, restoration after entry to the enable wrapper and the end-wrapper bookkeeping after its timestamp. It is neither a complete hardware cache/scheduler transition measurement nor a worst-case duration bound. It does not identify program versus erase or reconstruct lost storage jobs. The logger emits a separate CRC-protected flash_guard_peak record only when that retained peak changes. No UART, heap, task, wait or retry is added to the guard.

This behavior agrees with Espressif's [ESP32 SPI0/1 concurrency constraints](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_flash/spi_flash_concurrency.html): SPI1 flash operations suspend ordinary tasks and leave only IRAM-safe interrupt handling available. The exact local SDK source and linked observer dependencies are bound in the results.

## Confirmed playback obstruction

The native YouTube metronome excerpt uses LHDC V5, 192 kHz, 24-bit, stereo, phone-fixed900 kbps, verified in fresh Android settings. It has an independent 14-second phone pause process and a verified MCU I2S duration upper bound of 12.647748 seconds. BLE connects before playback; this is not a connection-during-audio test or 1Mbps qualification.

All 32 descriptor writes receive ACKs and independent readbacks. Unchanged EQ receives its ACK and settings readback; original CCCD restoration receives a write ACK plus independent descriptor readback. Controls finish before pause and the nonce-bound helper reports passed, then closes GATT.

The first finalized miss at 256590794 us has 1024 decoded raw frames waiting, zero processed output and decoder intent wait_pcm_space. Its frozen flash token 2946/serial 368 is active from Core 0 with scheduler state SUSPENDED. The completed peak has **the same token and begin time**, 256582901 us, and ends at 256600284 us: **17,383 us**. The miss falls inside that interval.

The completed-peak record is CRC-verified and was read by the PC at 1794221965.644535 seconds, before its pause request at 1794221966.8264754 seconds. This gives a pre-pause ordering witness even though the first-unfilled record was delivered later. The corrected SDK-lock serialization prevents token reuse in this association. These observations confirm a playback-time task-rendering obstruction, rather than inferring it from overlapping NVS job timestamps.

| Observation | Result |
|---|---:|
| Initial / recovered / final unfilled PCM frames |2880 /384 /2496|
| Descriptor errors / media drops / decoder faults / sequence breaks |0 /0 /0 /0|
| Host diagnostic events lost during excerpt |92|
| Worker / snapshot / UART record losses |0 /0 /0|
| Stopped internal free heap / boot-to-date minimum / largest block |61372 /19480 /34816 B|
| Decoder stack margin |5912 B|

The exact CCCD save at the long slice is absent from the finite event ring. Retained saves have wall intervals 21,820 and 24,321 us earlier in the window; those include preemption and are not substituted for the matched flash duration or assigned to its token. The slice could involve program or erase; that distinction is not established. The 2496 total unfilled frames are a window total, not all attributed to this one interval. This is not an A/B audio-improvement claim.

## Why pre-filling alone cannot cover it

The linked 192 kHz/S32/stereo policy allocates 1024 raw input frames, 2560 processed frames and two 192-frame DMA descriptors. Total PCM/DMA payload remains 31744 B. At 192000 frames/s, even completely full processed output plus DMA holds only:

`(2560 + 2*192) / 192000 = 15.333333 ms`

The 17.383 ms confirmed slice exceeds that generous capacity bound by 2.049667 ms. At least 394 additional processed frames (3152 B) would be needed just for this measured interval, before lock/IPC/restore overhead, dispatch time or safety margin. This is a mathematical lower bound, **not a proposed safe buffer size**. Normal steady-state target is only 2048 processed frames, so actual available reserve can be smaller. Raw input and encoded packets cannot replenish processed output while the task renderer is suspended.

The runtime geometry snapshot in this excerpt was taken after stop and reports zero capacities; it is not used as allocation or occupancy evidence. The calculation above is explicitly bound to the admitted linked-source policy and rate, not that stopped snapshot. The finding rules out encoded-queue growth and unchanged-capacity prefill as complete remedies for this event.

The next implementation must preserve processed PCM through legitimate durable storage using explicit admission and buffer ownership, with enough measured coverage and minimum SRAM. A temporary reserve is one candidate; its allocation, overlap, teardown and low-memory behavior need testing before it can be adopted. Reducing avoidable storage work remains useful, but does not by itself cover a long mandatory operation. Durable ACKs, security checks and PCM precision remain requirements.

## Source, build and regression evidence

Actual-source ASan/UBSan output tests pass at 100 and 1000 Hz simulated ticks. They check that the mocked SDK entry sees the old token, then verify active/finished state, retained maximum, smaller subsequent slices, timer wrap, invalid publication phases, exhaustion and the existing two frozen IRQ latches/refill behavior. These deterministic cases do not prove physical timing or arbitrary multithread execution. The app/profile/real-codec worker fixture also passes with async output/Core 0/priority order; its recorded sources match the current tree.

The ccache build rechecks 10440 inputs and passes source/image/partition admission and the linked owned-ISR audit. Relative to 297aae153578, IRAM grows 80 B and DRAM data 16 B, with BSS unchanged; flash text grows 196 B. Only 65 B of the linked IRAM region remains. No audio buffer, task, affinity, priority, codec arithmetic or quality setting changes.

Upload uses 921600 baud; full readback at 115200 matches both written regions. Original ota0, bootloader and partition layout are preserved, owned ota1 remains VALID and automatic Bluedroid return is disabled. The default host still uses Espressif's BTDM controller through VHCI with Bluedroid/NimBLE disabled.

Existing cold target checks pass 37 configurations / 173 callbacks, stack margin 3580 B, tracked peak 56899 B, minimum free after allocation 105568 B and maximum cold decode 13929 us. These are not live real-time or feature-parity proof. Boot watchdog, idle-starvation and backtrace counts are zero. All 134 snapshot records pass CRC; 52 other boot records are outside that audit.

[Structured results and digests](FLASH_SLICE_DURATION_RESULTS_2026_11_09.json) bind source, image, tests, memory, matched interval and handback. Exact source/image/ELF, raw UART, UI, APK/source and image-pinned harness remain locally under native_bt/_build/hardware/runs/1794221550234303300-b566da3f908f. Large hardware artifacts are not published in Git. Completed host symbols were archived losslessly with hashes and roundtrip verification; the 512 MiB rebuildable limit remains.

BDK Audio is restored to the foreground, YouTube PAUSED, I2S stopped and API-confirmed coexistence bits cleared. Identity-verified COM6 UART remains 921600 baud; COM7 is untouched. Sustained 192k/S24/1Mbps plus BLE, full live codec/security/lifecycle/OTA/acoustic qualification and equivalent-feature RAM/CPU comparison remain open.
