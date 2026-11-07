# Sound-space progress notifications during music — 2026-11-06

The complete Bluetooth/audio goal remains active and incomplete. This change removes a verified unnecessary kernel operation from music-only PCM progress. It does not disable sounds or change DSP, codec, packet/PCM buffering, affinity or priority.

## Problem and ownership contract

The output provider installs a permanent sound-wake callback. A completed local block or assisted batch called that callback even without a reserved sound lease. The callback reaches the persistent GATT control-worker binary semaphore. A signal can be coalesced; this is not a claim that every callback schedules a task or directly wakes the high-priority host. The semaphore/kernel work is still unnecessary when no sound-space consumer exists.

Only data-space progress now checks the existing atomic sound token. A reserved/attached/standalone sound keeps receiving refill/progress signals. Reserve/attach/release results, failures, cancellation and token retirement retain their unconditional lifecycle wakeups, including rejection before a token exists. The token is published before attach; mixing is detached before token clearing. The helper's post-DONE path reads only that atomic token and permanent callback, not a potentially retired sound object.

The negative actual-output test fails on idle data signals before the change. Afterward, music-only rendering has no sound-space callbacks; a live lease receives progress; failed exclusive reserve and successful release still signal results with no live token. Existing held-helper stop/cancel, before-DONE publication/lifetime, early input reuse, fixed-memory geometry and local-block-before-yield checks pass under ASan/UBSan. The full actual async sound/client suite passes UBSan for overlays, queues, codec takeover, cancellation, deadline and late retirement with mocked I2S. Host compilation now uses ccache per translation unit. These are not acoustic or complete app/OTA/stress tests.

## Diagnostic evidence

The new optional task-cycle counter and its bounded/no-call IRAM hooks are documented in RENDER_TASK_CYCLES_2026_11_06.md. Initial live accounting has zero hook faults and invalid observations. Local-render marked work averages 1031.79 charged and 1531.00 wall cycles per PCM frame; about 32.61% is switched-away/selection time. Charged time includes IRQ and marker costs, so it is not pure DSP arithmetic. This narrows the remaining investigation without claiming that preemption alone causes every gap.

A guarded same-instrumentation comparison retains actual LHDC V5 192 kHz/24-bit/stereo/fixed900, phone volume 80/150 and sampled DSP68/EQ2,8,12/flags12. Native YouTube plays the selected 120 BPM metronome. Each command interval is below 15 seconds with an independent 14-second pause guard and post-pause collection.

| Diagnostic mode | Before | Wake fix |
|---|---:|---:|
| Commanded interval (s) |12.228|12.234|
| Source accepted/rendered/DMA frames |2579072|2629248|
| Initial shortage frames |497920|435456|
| Late recovered frames |164992|147072|
| Unfilled frames |332928|288384|
| Invalid task-cycle observations / hook faults |0 / 0|0 / 0|

The fixed diagnostic records 1096960 local frames through8570 calls; charge/wall totals are1139893253/1655877708 cycles. This is about1039.14/1509.52 cycles/frame and still substantial switched-away time. The comparable diagnostic run improves, but different source positions, startup/drain intervals and instrumentation preclude an isolated overall speedup claim.

## Retained normal image

Normal image6e7afd71e8a0534dfe90571241f4b1cb12bef816329a09891b35a4ca9f22f172 explicitly disables TaskCycleAccounting. Admission verifies no accounting unit or forced kernel header; the 276 instruction-SRAM bytes and152 static-data bytes are removed. Relative to the retained d2676c584056 baseline, the normal wake fix adds48 flash-text bytes with unchanged IRAM, DRAM data/BSS and rodata. No dynamic allocation, task, queue or new state is added.

Full app/selector readbacks at921600/115200, owned VALID boot, original-app preservation and the existing37 cold codec cases pass. No automatic Bluedroid restore occurs. Exact normal archive: runs/1794031776350538600-6e7afd71e8a0.

The normal LHDC metronome run commands12.229 seconds and eventually accepts/renders/delivers2651520 source frames. Initial shortage381056, late recovered118400 and unfilled262656 remain separate; descriptor/contract errors are zero. Queue/decode/PCM failure/timeout, sequence breaks, ingress rejects, watchdogs, formatted-record losses and UART-write failures remain zero. Detail-ring loss3444 is retained separately. No task-cycle record is emitted by this normal image.

The 262656 unfilled frames lie within the previous instruction-RAM baseline range (254464/269824), so a normal-build playback improvement is not established. The unnecessary signaling defect is corrected, but LHDC continuity remains unresolved. The measured helper/local elapsed costs are832.40/1365.26 cycles per respective source frame, including preemption. All controls and wire samples remain at the requested192k/24/fixed900 configuration (566 media bytes/one5 ms frame, consistent with899200 compressed bits/s); live1000 kbps remains unqualified.

## LDAC regression and final state

On the same normal image, Android selects LDAC96 kHz/32-bit phone input/stereo. A12.157-second guarded native YouTube metronome run accepts/renders/delivers1444096 source frames with zero initial shortages, late-recovery demand, unfilled frames and descriptor errors. Sampled DSP stays68/EQ2,8,12/flags12. Consecutive sampled packets have661 media bytes, a one-byte header and RTP increments512 at96000 Hz, consistent with990000 compressed bits/s. This is a finite digital regression, not an acoustic/all-feature pass.

The phone is left paused on the metronome with LDAC96 selected and volume80/150. The full UART dashboard remains listening. Future LHDC testing must restore both rate and quality explicitly. Exact test sources/results, raw UART, failures, images and source bindings are under native_bt/_build/experiments/task-charge-1794029417140 and protected hardware archives. All remaining master-goal, maximum-LHDC, BLE/control/storage coexistence, volume acoustics and full feature/lifecycle/stress gates remain open.
