# Urgent PCM prefix delivery — 2026-11-07

The output path can now publish a smaller urgent DSP span and copy an available prefix into the most recent pending DMA descriptor. It keeps the original copy cutoff, ordering fence, sample precision, queues, task priorities and buffer sizes. There is no added task, audio buffer or wait.

Previously, a refill waited until every missing frame was available. A local 128-frame DSP block could also remain unpublished while a descriptor needed only 64 frames. The retained candidate limits local rendering to at most 64 frames **only while an eligible refill is pending**, and permits bounded partial copies before the unchanged deadline. Copy receipts validate the requested prefix, and the next EOF closes the older window before consuming newer PCM.

Actual output/audio/DSP code passes ASan/UBSan at 100 and 1000 Hz for prefix ordering, partial expiry, cancellation, startup/coherence/wrap and exact PCM equality between full and split DSP spans at 48/192 kHz with EQ/volume changes. The existing busy-helper publication/refill checks pass. No boot cases were added.

## Physical observations

These adjacent native YouTube excerpts use LHDC V5, 192 kHz, stereo, 24-bit, fixed 900 kbps, volume 150, an approximately 11-second command interval and an independent 14-second pause guard. They are not matched-content speed or continuity A/B qualification.

| Image/checkpoint | Accepted source frames | Final unfilled sample frames |
|---|---:|---:|
| 587b finalized-shortage diagnostic | 2,714,176 | 7,872 |
| 6968 first pending-window prefix candidate | 2,725,248 | 2,560 |
| 9b60 fully rechecked source build | 2,706,240 | 4,224 |
| 2e47 current 921600-UART build | 2,709,376 | 6,080 |

Each completed listed trace delivers all accepted source frames to DMA and has zero descriptor/refill-contract errors. Remaining inserted gaps mean continuity is unqualified. These counts are PCM sample frames, not LHDC access units. The current 6,080 frames equal about 31.67 ms cumulatively at 192 kHz.

A proactive extension that forced local 64-frame work at low output water, even without a pending refill, was **rejected and removed**. UART became stale under load; after stopping, the device reported 124,864 unfilled frames. Its shorter failed workload is not compared as a percentage with the completed traces. The failed image/source, raw UART, pause guard and post-stop state remain archived.

The [finalized-shortage report](FINALIZED_DMA_SHORTAGE_2026_11_07.md) distinguishes a recoverable initial shortage from an unfilled descriptor. Current startup is actually observed at 40,877 µs after first PCM, with 2,432 rendered frames before preload and 960 input frames. At the current first finalized miss, 512 decoded frames were waiting, the renderer was outside local DSP, and the helper was idle. This is evidence for further handoff/scheduling work, not proof that every gap shares that cause.

## Stale-build defect and correction

Windows source restoration preserved an older timestamp. Ninja therefore skipped the restored audio source, even though the old before/after source manifest was stable. The resulting 7c90 binary still contained the rejected extension. One redundant deployment also proceeded after the separate hash check failed; that orchestration error is recorded and corrected.

The misbound archive `runs/1794094293361918500-7c90d5ae4032` now has `STALE_BUILD_INVALIDATED.json`. Its readback identifies the binary but does not prove correspondence with its saved source manifest. No raw evidence was erased.

`invalidate_compile_outputs.py` now invalidates existing compiler-output timestamps inside the admitted build tree before compilation through ccache. It modifies neither object contents nor sources and deletes no objects. The real Ninja regression reproduces backdated source/header failures, then proves the new results, an actual ccache hit, unchanged object bytes during invalidation, and rejection of out-of-tree outputs.

The build receipt uses schema 2 and records the compiler-input recheck; image admission rejects the old proof. The complete recheck covered 1,145 outputs. Deployment also supports `--expected-sha256`, checked before opening COM6; its wrong-hash negative control passes. The restored source was fully recompiled and flashed as 9b60, not falsely claimed byte-identical to 6968. The current 2e47 image then adds the [verified fast-UART changes](UART_921600_2026_11_07.md).

Evidence directories: `native_bt/_build/experiments/final-refill-evidence-1794092367615` and its `urgent` subdirectory. Physical archives and failed receipts remain intact. These are development changes; the entire master goal, including continuous 1 Mbps, coexistence, security, OTA, lifecycle, stress, acoustic and comparative whole-host RAM gates, remains open.
