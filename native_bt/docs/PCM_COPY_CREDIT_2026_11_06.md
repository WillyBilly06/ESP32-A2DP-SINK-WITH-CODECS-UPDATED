# PCM copy credits and rejected shorter helper lease — 2026-11-06

The full Bluetooth/audio goal remains active and incomplete. No codec, feature, fidelity setting, buffer size, priority or core affinity is removed. This follows the observation that decoded input remains available while processed output is empty at the first gap.

## Rejected shorter lease

Image 383bc1a326880b5a183ebd8ea1a23fdd9e3ef8adef01227e584e3639aa88b6ab bounds a helper lease to two128-frame blocks instead of four. The pressure/wait predicate and ownership state machine are unchanged. Sanitized tests enforce the 256-frame ceiling with 512 input frames available, before-DONE publication/lifetime, stop/cancel and the bounded local block before yielding. Static sections and binary size are unchanged. Full app/selector readbacks, owned VALID boot and all37 existing cold codec configurations pass.

A12.208-second guarded native YouTube metronome trace at the restored192k/24/stereo/fixed900 configuration delivers2328768 source frames and leaves571520 unfilled. The four-block baseline has359552–360064 unfilled and about2.55 million delivered frames in similar command intervals. Helper work falls to47.56% of frames, Core0 idle charge grows to27.8–28.4%, and Core1 renderer charge grows to47.5–48.3%. No queue/decode/ingress reject, descriptor error or watchdog is recorded. Smaller leases increase handoff pressure and do not improve this workload. The candidate is rejected; its source, exact image/boot receipts and raw trace are preserved. Four-block source is restored before the copy-credit candidate.

The measured baseline DMA rate is191992.05 and192016.97 frames/s over4.81/4.52-second snapshot intervals. Finite snapshot/EOF quantization remains, but a grossly fast I2S clock does not explain the shortage. Its nominal rate is192000.

## Capacity-aware copied PCM

The existing worker copies up to256 frames, while the renderer can release128 input frames at a time. A provider with128 frames free previously rejects the entire256-frame request. The new optional worker hook queries the copy provider's current credit, up to that unchanged256-frame ceiling, and submits only the available span. It borrows no view and accepts no samples during the query.

NULL keeps the existing fixed-ceiling behavior. Asynchronous PCM loans ignore the hook and retain their acknowledgement/lifetime contract. A zero credit follows the ordinary one-attempt BUSY/notification/deadline path without invoking the PCM provider; an out-of-range credit is a provider-contract failure. Offsets, delivered-frame accounting and per-sample timestamps advance only after the actual copy succeeds. BUSY consumes nothing. Full views and encoded packets remain borrowed until every partial copy and codec continuation retires.

The producer-only output query observes the existing input ring with one bounded attempt. Only that producer advances written; the consumer can only increase free space before the subsequent submit. The existing stream receipt protects immutable input geometry from teardown. Priming has valid writable input before I2S runs, so the query follows stream ownership rather than the running-DMA flag. Stopped/failed state preserves normal submission error reporting. No wait, retry loop, allocation, new task, PCM storage or format conversion is added.

The portable worker gains one optional callback pointer. Linked BSS increases8 bytes from the padded worker layout; IRAM and DRAM data/rodata are unchanged, flash text increases164 bytes. Candidate image8a5197d0a3ca249c4f5e7017b011d8e1ed145f8e327776902fb9b8290a1cca7b passes the full ccache build and exact source/image admission. Four-block assistance and the exact analysis optimization remain enabled.

## Verification

Twelve actual worker groups pass ASan/UBSan: zero credit without a provider call, partial and odd spans, exact bytes/timestamps through wrap, invalid credit, zero-credit timeout, cancellation, asynchronous stream receipts and immutable loans. Actual two-core renderer tests pass empty/partial/full credit in priming, early input reuse, fixed-memory geometry, lifecycle fences and one local block after helper retirement. The first priming test caught an incorrect running-DMA check; it is corrected before hardware deployment. A briefly misapplied admission-check edit is restored, with the precise query corrected. No new boot case is added.

The maintained real-codec harness passes80 sanitizer-backed cases across all37 configurations plus the maximum LHDC fixture, matching every PCM byte and sample timestamp with128-frame copy credits, simulated backpressure, batching and lifecycle transitions. Its old LHDC fragment fixture incorrectly applied SBC F/S/L flags and a one-byte prefix. It now checks valid LHDC two-octet multi-frame packets and rejects count-mismatched SBC-header substitutions; actual SBC/AAC fragmentation cases remain. Runtime codec framing is not weakened to accept the invalid fixture. Host sanitizer timing and RTOS values are not target evidence.

An older profile/worker harness was attempted but is stale: it lacks the current core-ID/primed-output declarations and newer protocol linkage. Temporary declaration/shim edits were reverted. It is not counted as a passing gate. The maintained actual-codec/worker and actual-output suites above provide the scoped replacement checks; full profile/app/feature qualification remains open.

Two aged host PDB debug-symbol files are removed only after resolved-path/hash receipts to satisfy the512MiB rebuildable-output quota. Executables, sources and all protected hardware evidence are retained. Failed test/build/observation receipts remain in native_bt/_build/experiments/dsp-lease-1794026333302. Physical copy-credit results follow after guarded deployment.

## Copy-credit physical result and rejection

Guarded image8a5197d0a3ca deployment passes full application/selector readbacks, owned VALID boot, original preservation and37 cold cases. A12.220-second native YouTube metronome trace restores the same actual192k/24/stereo/fixed900, phone volume80/150 and sampled DSP68/EQ2,8,12/flags12. It accepts/renders/sends to DMA2519552 frames through12159 accepted views (207.22 frames/view on average). The first gap still has2048 decoded input frames and zero processed output.

Initial shortages545408, late recoveries164352 and unfilled381056 remain separate; descriptor/contract errors are zero. PCM backpressure8396 and detail-ring loss4049 are explicit. Queue/decode/PCM failure/timeout, sequence break, ingress reject, watchdog, formatting loss and UART-write failures are zero. Core0 helper handles61.73% of frames at843.31 elapsed cycles/frame; local DSP has964224 frames at1537.16 elapsed cycles/frame. Preemption remains included.

This preserves bytes but does not establish a playback improvement over the359552–360064-frame four-block baseline. The extra callback path and8-byte padded static cost are therefore not retained. Source is restored to the four-block/fixed256-frame transfer baseline, including the existing exact analysis optimization. The candidate source/API/tests, all80 passing real-codec comparisons, failed fixture receipts and physical evidence remain privately reproducible. Only the valid LHDC regression-fixture correction is retained in source; restored baseline worker and80 real-codec checks pass again.

The paused device will be replaced through the guarded owned-image workflow. This is not Bluedroid fallback. The existing opt-in instruction-RAM DSP placement is being remeasured under current fair sharing, logging and helper scheduling; the old placement trial used a materially different pipeline and gave no established speed result. No placement benefit is claimed until current measurements complete.
