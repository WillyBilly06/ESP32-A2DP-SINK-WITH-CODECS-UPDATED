# DSP completion notification order — 2026-11-06

The full Bluetooth/audio goal remains active and incomplete. This follows the retained BUSY-helper DMA refill correction. It changes two existing post-DONE notifications, preserving their asynchronous completion contract and all codec/audio features.

## Verified kernel and application path

probe_audio.c installs wake_worker as the output PCM-progress callback; it calls xTaskNotifyGive on the decoder. The pinned ESP-IDF6.1 FreeRTOS-Kernel/tasks.c xTaskGenericNotify (line5898) unblocks a notification waiter and requests a yield when taskIS_YIELD_REQUIRED says so. Its dual-core prvIsYieldRequiredSMP (line815) returns a current-core yield for a higher-priority compatible target, and signals the other core separately for a remote compatible target. No SDK source is edited.

The retained product has helper20/Core0, decoder21/Core0 and owner22/Core1. dsp_assist_work previously published DONE, notified the producer, then notified the owner. A blocked decoder becoming ready can therefore preempt the helper before its Core1 notification. Core1 might discover DONE through a later DMA notification, but the completion signal itself was delayed by unrelated same-core execution.

The candidate loads/signals the permanent Core1 owner first, then calls the unchanged PCM-progress notification. Both follow release-published DONE; neither waits for an acknowledgement. The producer remains independently notified without depending on Core1 retirement. No borrowed DSP, PCM or mutable job metadata is read after DONE. No task, stack, buffer, allocation, lock, ISR operation or priority is added or changed. It does not guarantee Core1 executes before the decoder; it makes its completion eligible earlier.

## Source regression

The actual two-thread output fixture now suspends the helper continuation at the PCM-progress callback, modeling priority21 producer execution after notifying it. No DMA EOF or periodic pending-job polling is generated. Valid old code reports0 owner completion signals at this point; the negative control fails. The reordered code reports1, and Core1 retires the job and closes its stream while the helper continuation remains held. It then resumes safely, with ASan/UBSan enabled. This is a controlled interleaving/model, not target scheduling timing.

Both ordinary and fair-share/early-input/rebalanced product modes pass. Existing four-block/single-block, stop/cancel, no early free, live/idle sound notifications,640-ready-frame retirement and quiescent-restart checks remain. The separate published-PCM/BUSY-refill regression still passes at100Hz/1000Hz modeled clocks under ASan/UBSan. Fixed-point arithmetic retains the established wrapping option. No new boot test cases are added.

## Physical comparison in progress

Fresh baseline on retained image6e4c4932cf282e5beb56c313f0b2e40301dd55be08c31e7405bb894ec458da85, archive runs/1794036556549434000-6e4c4932cf28: Redmagic native YouTube metronome12.295s, independent14s pause guard, UI LHDC192k/24/stereo/fixed900, phone volume80/150. Active UART controls confirm DSP volume68/EQ2/8/12/flags12. Accepted/rendered/DMA source frames2693312; initial shortage502016, recovered269824, unfilled232192; descriptor/contract errors0. No new queue/decode/PCM/sequence rejection occurs; detail loss3502 remains separate from0 aggregate losses. The helper1941us retirement maximum is since boot, including earlier streams, not a fresh per-run latency distribution.

Candidate a9427d8cafde6e89ee4826435e392fe7eef1653ba7e7b8873a60ff01963fdb57 builds with ccache and passes exact source/image admission. All codecs/product/security/RFCOMM remain enabled, temporary task-cycle hooks OFF. Relative to6e4c4932, flash text+16 bytes; IRAM/data/BSS/rodata unchanged. Guarded COM6 write921600/readback115200 passes both full readbacks, owned VALID selection, original preservation and37 existing cold codec checks (archive runs/1794037693865568200-a9427d8cafde). No automatic fallback.

Raw kernel snippets, preserved sources, negative/positive sanitizer receipts, footprint and traces are under native_bt/_build/experiments/assist-wake-1794037437554. Full1000kbps, sustained LHDC continuity, acoustics, encrypted BLE/audio/OTA/control stress and complete feature parity remain open.

## Short hardware observations

All LHDC runs use restored UI192k/24/stereo/fixed900 and the same sampled DSP volume68/EQ2/8/12/flags12. Every play command has its independent14s pause guard. These are successive metronome segments, not identical replay or sustained qualification. Whole-stream counters include startup/drain; they cannot label every shortage as an in-play audible gap.

| Image/run | Command interval | Accepted/rendered/DMA source frames | Initial shortage | Recovered | Unfilled | Copy/mask max us |
|---|---:|---:|---:|---:|---:|---:|
|6e4c4932 baseline|12.295s|2693312|502016|269824|232192|6/9|
|a9427d8c candidate1|12.070s|2684672|465664|258816|206848|5/8|
|a9427d8c candidate2|12.207s|2727360|440832|260864|179968|5/9|

No descriptor, refill-contract, helper failure or new queue/decode/PCM/sequence rejection occurs. Detail losses3502/3416/3482 are retained separately from0 aggregate losses. The new counts are consistent with improved service, but overlap earlier run variation and do not isolate a throughput gain from notification order. The retained reason is removing the demonstrated preemption dependency, with no fault observed in the bounded hardware runs. LHDC still fails continuity. Since-boot helper retirement maxima1941/1615/1774us have different history windows and must not be treated as comparable latency distributions.

After switching to LDAC96k/stereo and32-bit phone input, optimized-quality selection produces661-byte media samples with a1-byte header and512 RTP-tick steps at96k, consistent with990000 compressedbps. Native YouTube12.248s accepts/renders/delivers1451264 source frames with0 initial shortages/unfilled,0 descriptor/contract errors and no new queue/decode/PCM/sequence rejection. Detail loss5526 remains explicit. This is finite digital evidence, not acoustics or all-feature qualification.

Active LHDC candidate snapshots report60704 resident codec bytes,19604 free internal heap,18432 largest block and5960 decoder stack low-water bytes in both runs. The older baseline has prior codec-switch history and reports19592 free/19212 minimum/16384 largest; the candidate does not add static or requested PCM memory. These snapshots do not prove leak/fragmentation resilience. The LDAC active sample has28196 resident bytes,68524 free and57344 largest.

The normal a9427d8c image remains selected, temporary task-cycle hooks/storage OFF, raw UART dashboard listening and native YouTube paused. [Structured results](ASSIST_COMPLETION_WAKE_RESULTS_2026_11_06.json) preserve image/run binding and limitations.

The subsequent [five-block scheduling comparison](ASSIST_FIVE_BLOCKS_2026_11_07.md) keeps this completion ordering and the guarded BUSY-refill path. Its explicit development cap5 improves repeated short LHDC observations, while4 stays the source/build default and full continuity remains open.
