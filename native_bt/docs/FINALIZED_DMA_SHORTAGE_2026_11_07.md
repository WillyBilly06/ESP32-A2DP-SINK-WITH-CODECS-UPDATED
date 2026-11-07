# Finalized DMA shortage diagnosis — 2026-11-07

Image `587bc22da238bb2b4cccdbb5908f97e2cd495d314e475d4035fa64ca9446b5d4` is flashed/read back on COM6, owned VALID, with the original application preserved and automatic Bluedroid fallback disabled. All 37 cold codec cases pass. This change adds separate first-initial and first-finalized shortage snapshots plus actual DMA-start timestamps; it does not yet claim a continuity fix.

The initial shortage may still be recovered. The new finalized latch is written when an older refill window closes with missing frames, before newer output is consumed. An ineligible window is captured immediately with deadline zero. Recovered and canceled windows do not populate it. Each stage retains its own UART record and a shared stream/event identity. Counts are adjacent atomic observations; decoder/media/render versions report consistency without retries. The final timestamp is the closure observation, typically about 64 microseconds after the guarded copy cutoff, not a simultaneous snapshot at that cutoff.

Startup observations record first PCM, successful I2S enable, elapsed time, input occupancy, preload occupancy, and guard state. A one-attempt version check marks torn observations invalid. There is no new task, queue, buffer allocation, waiting, spin, or IRQ logging. Linked data increases 168 bytes; instruction RAM decreases 208 bytes in this build. The entire SRAM footprint is not claimed smaller from these diagnostic figures.

Actual output/audio/DSP code passes host ASan/UBSan at 100 and 1000 Hz for recovery versus finalization, cancellation, reset, immutable first capture, deadline identity, closure before newer consumption, startup guard/enable, consistency, and microsecond wrap. The existing busy-helper publication/refill test also passes. Metadata scheduler stubs do not measure target time. Initial harness/header failures are preserved. Full ccache build, source-stability, resource and image-admission gates pass; rebuildable storage was 500.378 MiB.

## Native YouTube observation

Redmagic settings are independently read back as LHDC V5, 192 kHz, stereo, 24-bit, fixed 900 kbps, volume 150. One short existing-song trace uses an independent 14-second pause guard and ends with I2S stopped. Fixed 900 kbps does not qualify live 1 Mbps.

- DMA enables **40,413 microseconds** after first PCM, with **2,432 rendered frames** before preload and **960 input frames**. The 40 ms guard is effective in this observed lifecycle.
- First initial shortage: timestamp 120368000, 64 of 128 frames available, input/output both empty.
- First finalized shortage: timestamp 120368667, the same descriptor still has 64 valid frames, and the copy cutoff was 120368602. There are now **832 input frames** and zero published output frames.
- Renderer is inside local processing, begun at 120368294; helper state is IDLE. Decoder has started another unit at 120368579. This supports investigating local publication latency at this gap; it does not establish an all-gap cause.
- Final counters: **12,672 initial shortages − 4,800 recovered = 7,872 unfilled sample frames**, with zero descriptor/refill-contract errors. All 2,714,176 accepted source frames reach DMA. Continuity remains unqualified.

The raw event capture initially includes retained UART history. Analysis selects exact source ID 46 and the controlled action time, yielding 277 rows; the 9,849 raw rows remain unchanged and the filter manifest is saved. Final state records are from the fresh source. The trace collector's starting cursor must be taken from the current state revision, which shares the log-ID counter, rather than an unqualified first /events page (limited to 400 rows).

A bounded next candidate is to publish a smaller urgent DSP span and allow safe partial copies into the same pending descriptor. The current code waits for all missing output before copying and normally processes a 128-frame block, even when only 64 frames are missing. Any candidate must retain exact PCM order, guarded deadlines, stop/cancel semantics, full DSP precision and all codec behavior; it needs deterministic tests and a new target trace before being retained.

Evidence: `native_bt/_build/experiments/final-refill-evidence-1794092367615` and `native_bt/_build/hardware/runs/1794092729771300300-587bc22da238/traces/final-refill-192-v150-1794092998113`. The full master goal remains active, including continuous maximum-rate audio and complete feature, coexistence, security, OTA, lifecycle and stress qualification.
