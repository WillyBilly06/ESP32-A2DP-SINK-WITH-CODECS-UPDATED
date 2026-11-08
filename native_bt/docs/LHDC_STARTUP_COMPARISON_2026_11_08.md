# LHDC startup-guard comparison — 2026-11-08

The shorter startup guard was rejected. The normal high-rate LHDC guard remains **50 ms**, and optional task-dispatch tracing is disabled in the normal image. LHDC continuity and the full 1 Mbps plus BLE/control target remain unqualified.

## Same-image comparison

Redmagic native YouTube metronome excerpts used LHDC V5, 192 kHz, 24-bit stereo and the phone's fixed-900 setting. Each requested playback interval was 10 seconds with an independent 14-second pause guard; actual I2S playback remained below 15 seconds. Codec precision, queues, core affinities and priorities were unchanged.

| Order | Requested guard | Observed guard | Encoded slots at enable | Final unfilled PCM frames |
| --- | ---: | ---: | ---: | ---: |
| A1 | 50 ms | 49,738 us | 7 | 512 |
| B1 | 30 ms | 29,773 us | 4 | 6,208 |
| B2 | 30 ms | 30,541 us | 4 | 7,872 |
| A2 | 50 ms | 50,065 us | 7 | 1,600 |

Both 30 ms windows had finalized shortages during active playback, well before the phone pause. A1's first finalized shortage followed the pause acknowledgment; A2 straddled the pause timing bound. Counts retain the complete stop/tail and are not adjusted to remove gaps. Only the first shortage and first finalized gap have immutable timing latches.

The comparison image was 6d29a9e8d3389752e7bf2ecab994650be867af7633b171e6f0f17e6ceb7422df. Its temporary alternating selector was removed after preserving its complete source and host fixtures in the experiment archive.

The restored normal image was 1670826c46423a8bd4a1f420fb04c359fced6520667ce1b2b0f36c9ed9cbadee. Two restored windows contained 0 and 1,216 final unfilled frames; in the latter, the first finalized gap was bounded 2.313–2.571 seconds after the pause request and after its acknowledgment. These finite windows do not establish sustained continuity.

## Timing interpretation

A roughly 50 ms host/RTP elapsed-time excess at the first empty-buffer event does not prove that the 50 ms startup guard caused a source stall. Selecting the first buffer-empty event conditions the observation on exhausting the initial reserve. The optional admission-clock samples had sparse active-playback coverage, including one window with no matched pre-pause samples; they cannot establish the proposed feedback mechanism.

A retained one-attempt accessor copies the existing versioned media tuple without retry, allocation or additional producer writes. Null, odd-version and mid-read mutation cases are covered by the actual-output sanitizer fixture. The optional background admission-clock record requires dispatch tracing. With tracing disabled, its log code and unused accessor are absent from the linked normal image. HCI callback time is not RF arrival time.

## Supplied Bluedroid queue review

The actual current local reference is esp-idf/components/bt/host/bluedroid/btc/profile/std/a2dp/btc_a2dp_sink.c, SHA-256 79142cc01bfbecdc40af99d4a276dd3632045f8705722fb73149266d4536fa0d. It contains local modifications and is not an unmodified ESP-IDF 5.5.2 golden baseline.

Its applied queue-limit function selects 18 packets for AAC/SBC/default, including LHDC, and 20 for LDAC. The separate 24-entry macro does not describe the applied LHDC cap. Enqueue transfers ownership of an existing BT_HDR allocation rather than allocating a fresh uniform 1,800-byte slot per packet.

The owned LHDC queue reserves eight 1,816-byte slots, including metadata, for 14,528 pool bytes; payload capacity remains 1,800 bytes per packet. The observed fixed-900 payload was 566 bytes. The difference in packet capacity/storage efficiency is a concrete follow-up, not a proven cause of the gaps. Any packed-storage candidate must preserve borrowed decoder-input lifetime, worst-case 1,800-byte admission, controller-credit safety and bounded ownership across cancellation/reset.

The actual retirement path already wakes the host as partial credits become available in its pressure region. Borrowed DSP waits already sleep on notification, and the renderer excludes its one-sample tail from runnable work. These inspected paths do not support the earlier polling-spin hypothesis.

## Evidence

All artifacts are under native_bt/_build/experiments/startup-credit-1794159778694:

- abba-results.json and all-window-checks.json bind the seven baseline/comparison/restoration windows and complete UART captures. Queue/decode/helper/descriptor/host-log/worker-log/UART-loss counters were zero in those windows; CRC-invalid records were zero.
- comparison-sources/manifest.json preserves the removed experiment and its source hashes.
- normal-image-admission.json, normal-audio-isr.json and final-passive.json bind the restored image and passive UART observation.
- reference-queue-review.json binds the exact supplied reference file.

The comparison and restored images passed offline admission, readback, preserved original-app checks and the existing 37 cold codec configurations / 173 PCM callbacks. The normal image reclaimed 488 B IRAM and 360 B DRAM data/BSS relative to the diagnostic comparison image; its linked SRAM/IRAM matched the preceding normal baseline. A formatting warning was caught during an off-board build and corrected before deployment; the failed log remains archived.

Portable host and actual-output evidence is distinct from target timing, acoustic quality, sustained BLE/control operation, OTA/security parity and full 1 Mbps qualification. Those requirements remain open.
