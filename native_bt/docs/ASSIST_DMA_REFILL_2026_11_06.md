# DMA refill while DSP helper is busy — 2026-11-06

The master Bluetooth/audio goal remains active and incomplete. This change addresses a specific consumer handoff found after validating decoded LHDC sizes and correcting host CPU attribution. It does not change codec precision, buffers, task affinity or priorities.

## Defect and ownership

The Core0 helper processes at most four128-frame blocks per lease and release-publishes each completed PCM span. The Core1 owner previously returned from render_step immediately for a READY/BUSY lease, before service_refill_window. The helper notified the owner only after the complete lease reached DONE. An already-published block could therefore wait while the current conservative descriptor refill window expired.

The processed ring has a producer-owned write cursor and atomic release/acquire publication. dbt_audio_take consumes only published output and updates the consumer cursor; it does not inspect DSP coefficients, mutable helper state, unpublished PCM or lifecycle resources. Existing refill code already restricts this consumption to the IRQ core and protects it against that same-core ISR with a bounded local mask. The fix permits this one operation while a helper lease remains borrowed. DSP/meter/control/lifecycle work still waits for DONE.

After each successful helper publication, a valid pending-descriptor index triggers one task notification to the permanent owner. There is no wait or retry, at most four checks/notifications per bounded lease, and no notification for this purpose when no descriptor is pending. The existing final producer/owner completion signals remain. Deadline, copy/mask budget, stop, descriptor history, next-EOF expiration and ordering checks are unchanged.

## Actual-source regression

The new sanitizer fixture compiles actual output/audio/DSP/job code. It supplies valid pinned-owner/IRQ identities, opens a real processed ring, grants the existing post-EOF refill lease, and interleaves the owner after the first helper publication while the lane remains BUSY. It checks128 correct frames copied, DSP state and allocations unchanged, no early retirement,384 later output frames retained and512 total helper frames.

A valid negative control with the old runtime source reports owns=1,callback_core=1,ready=128 but recovered=0 and zero owner notifications during BUSY. Fixed code reports recovered=128 and exactly one such wake while preserving the borrow through the remaining blocks. Both100Hz and1000Hz modeled clocks pass ASan/UBSan; fixed-point signed arithmetic keeps its established wrapping option. This is deterministic interleaving evidence, not ESP32 timing or a multithread proof.

The separate actual two-thread Core0/Core1 suite passes full-ring early reuse, unchanged39936-byte payload and five other layouts, live/idle sound signals, held-worker queued stop/cancel, publication-before-DONE,640-ready-frame retirement handoff, no early free, no timed pending polling and quiescent restart under ASan/UBSan. Existing refill ordering/deadline/wrap/stop/budget checks pass at both tick rates. The asynchronous sound overlay/queue/takeover/cancellation suite passes UBSan and compiles target objects; no sound feature is disabled.

A host-symbol quota failure after otherwise passing fixture execution is retained. Two aged rebuildable PDBs were hashed then removed; no hardware archive was removed. The complete sanitized rerun and quota check pass.

## Physical baseline and candidate

Baseline ea790bbaaa24f6d04d0e7e7c35c5c71b930d50cd568bcb24ac0393af661e15a6, archive runs/1794035670765910300-ea790bbaaa24, full owned host. Fresh Redmagic native YouTube metronome playback12.241s under an independent14s pause guard, with UI LHDC192k/24/stereo/fixed900 and actual phone volume80/150. Active device records show DSP volume68, EQ2/8/12, flags12 and2048 raw /2560 processed /2048 target frames, DMA3x128.

Baseline source/DMA frames2669760; initial shortage356800, recovered119296, unfilled237504; descriptor/contract errors0. PCM busy8227, no encoded queue/decode/PCM failure or input-sequence/oversize loss. Worker detail loss3588 remains separate from zero aggregate/UART losses.

Candidate6e4c4932cf282e5beb56c313f0b2e40301dd55be08c31e7405bb894ec458da85 builds with ccache and passes exact source/image admission, all codecs/product/security/RFCOMM present, TaskCycleAccounting OFF. Relative to baseline, flash text+68 bytes; IRAM, data, BSS and rodata unchanged. No task, stack, heap or PCM allocation is added. Guarded COM6 write921600/readback115200 passes both full readbacks, owned VALID selection, original preservation and37 existing cold codec cases (archive runs/1794036556549434000-6e4c4932cf28). No new boot cases or automatic fallback.

Exact sources, valid negative and positive receipts, failed setup/quota receipts, separate traces and footprint: native_bt/_build/experiments/assist-refill-1794036169410. Full1000kbps, sustained LHDC continuity, all-codec acoustics, encrypted coexistence, controls/OTA/security and all-feature qualification remain open.

## Physical comparison

All three LHDC command intervals use the same native metronome app, independent14s pause guard, UI192k/24/stereo/fixed900 and volume80. Active UART controls remain DSP volume68/EQ2/8/12/flags12. These are short successive metronome segments, not identical replay or sustained qualification. Counters include startup/drain; no claim that every whole-window shortage is an in-play audible gap.

| Exact image/run | Command interval | Source/DMA frames | Initial shortage | Late recovered | Unfilled | Copy/mask max us |
|---|---:|---:|---:|---:|---:|---:|
| ea790bba baseline |12.241s|2669760|356800|119296|237504|7/9|
|6e4c4932 candidate1|12.217s|2721856|395648|213248|182400|5/9|
|6e4c4932 candidate2|12.200s|2713920|456704|256384|200320|5/11|

Observed unfilled totals decrease23.2%/15.7%, and both candidate runs recover more already-published PCM. That is a repeated bounded observation, not a demonstrated full CPU throughput increase or a continuity pass. Helper leases remain4x128, with no reported helper, descriptor or refill-contract fault. No new encoded queue/decode/PCM failure, input sequence or oversize loss is recorded. Worker detail losses are3588/3508/3516; aggregate/UART losses remain0. Timing/CPU and raw record coverage limits remain explicit.

Switching to LDAC96k/stereo with32-bit phone input and its optimized-quality radio selection succeeds. The native YouTube12.168s trace accepts/renders/delivers1443584 source frames with0 initial shortages,0 unfilled and0 descriptor/contract errors. Wire samples have661 media bytes with1-byte header and512 RTP-tick steps at96k, consistent with990000 compressedbps. This is a sampled inference, not a full wire-bit ledger. No new queue/decode/PCM/sequence errors occur; worker detail loss5512 is retained. This digital regression does not establish acoustic or all-feature qualification. The phone is paused and the normal candidate remains selected; temporary task-cycle hooks are OFF.

[Structured results](ASSIST_DMA_REFILL_RESULTS_2026_11_06.json) bind the normal image and each scoped observation.

Active LHDC baseline and both candidates each report60704 codec-resident bytes,19604 free internal heap,18432 largest block and5960 decoder stack low-water bytes in the sampled record. These are finite snapshots, not leak/stress qualification. The LDAC active sample reports28196 resident bytes,68524 free and57344 largest block. Detailed heap/stack samples are retained beside each trace.

The later [completion wake-order correction](ASSIST_COMPLETION_WAKE_2026_11_06.md) signals the owner before notifying the higher-priority producer after DONE. It preserves this BUSY-helper refill path and all descriptor ownership limits.
