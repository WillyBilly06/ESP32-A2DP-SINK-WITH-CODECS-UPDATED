# Renderer task-switch cycle accounting — 2026-11-06

The full Bluetooth/audio goal remains active and incomplete. The retained d2676c584056 image improves LHDC but still misses deadlines. This optional diagnostic separates local renderer charged cycles from time switched away without changing sample precision, codec features, buffers, priorities or core affinity.

## Measurement contract

-TaskCycleAccounting defaults OFF and requires the existing native TaskRuntimeStats/SchedulerTrace asynchronous product. It installs standard FreeRTOS task-switch macros only in the pinned kernel's C compilation. No SDK source file is edited. Other trace backends, Amazon SMP, a non-dual-core configuration, power management or a non-240MHz clock reject compilation. Ordinary builds have no override or accounting unit.

The hooks compare opaque permanent pinned-render handles and update per-core CCOUNT/pointer words. They do not dereference a TCB or call a kernel, allocator, timer, peripheral, logging or application callback. ELF disassembly confirms the two hooks call no functions and reside in IRAM. Their counter state is in DRAM. Registration happens before measured work; enable follows successful scheduler registration.

An own-core task mark takes one bounded local interrupt-mask span, preserving the prior level. It combines retired charged cycles with the unfinished current quantum. It rejects a different core, inactive/wrong task, missing handle or switch-order fault. It cannot read another core's live CCOUNT tail. Matched owner/core marks bound local audio-render calls; raw uint32 subtraction handles short-window cycle wrap. The counters include interrupted-task IRQ time and context restoration, while excluding switched-away execution and the OUT-to-IN kernel-selection gap. They are scheduler-charged cycles, not ISR-free instruction counts.

The existing maximum wall-delay records remain. Separate per-stream local-render counters report measured calls/frames, invalid observations, wall/charged totals, paired charge at maximum wall span and maximum charged span. Each logger snapshot makes one sequence-check attempt. Cross-core helper-retirement markers are deliberately excluded from own-task accounting. Marker work at the measured boundary remains included; no pure-math claim is made from these records alone.

## Local checks and footprint

Actual hook/marker/aggregation code passes ASan/UBSan tests for running tails, preemption/selection gaps, both cores, raw timer/total wrap, invalid identity/order/core, local-mask restoration, stream reset, non-maximal observations, sequence wrap and invalid/cross-core exclusion. Existing ordinary and task-runtime sampler suites still pass. Admission exercises6 CMake,18 actual image,7 compiled SDK-header and1 earlyCLI rejection cases. Private SDK/runtime option transition tests still pass.

The complete ccache product build and exact source/image admission pass for diagnostic0bbaa6fc2ea0ebac8ebfb7fe41e7c1ec25b7370284b24f09f833df67899a2b06. Relative to d2676c584056: IRAM text+276 bytes, DRAM data+64, BSS+88, flash text+1164 and rodata+432. No task, stack, queue or dynamic allocation is added. The276 instruction-SRAM bytes and152 static-data bytes are diagnostic overhead, not a claimed optimization.

Guarded COM6 deployment, zero-fault/valid-marker confirmation and native YouTube LHDC192/24/fixed900 diagnosis are recorded below. Preserve write921600/read115200, owned VALID selection with no automatic host fallback, original app/table/bootloader, the existing37 cold codec cases and independent14-second pause guards for each <=15-second play command interval. All master-goal and full-feature qualification gates remain open.

Exact sources, host checks, actual hook disassembly, footprint and later raw captures are under native_bt/_build/experiments/task-charge-1794029417140 and protected hardware run archives.

## First live measurement

Guarded diagnostic deployment passes full app/selector readbacks, owned VALID boot, original preservation and the existing37 cold codec cases. Native YouTube metronome at restored LHDC192/24/stereo/fixed900 runs for12.228 seconds under its independent14-second guard. All recorded local-render marks are coherent with zero hook faults and zero invalid observations.

The local path measures1046784 frames in8178 calls,1602625461 wall cycles and1080065537 charged cycles:1531.00 wall and1031.79 charged cycles/frame, or4.299 charged microseconds/frame. About32.61% of those measured intervals is switched-away/selection time. The paired worst wall span is3914.78us with1433.80us charged to the renderer. IRQ and marker costs remain included; this does not establish pure DSP arithmetic time.

The stream eventually accepts/renders/delivers2579072 frames but has332928 unfilled frames (497920 initial shortage,164992 recovered) and no descriptor, queue/decode/PCM, input sequence, ingress, watchdog, formatting or UART-write fault. Worker detail loss3682 is separate from zero aggregate-record losses. The first gap has1920 decoded input frames and no processed output. This is diagnostic evidence, not a performance pass; instrumentation can perturb the timing.

The source audit also finds music-only PCM completion signaling the sound provider on every block/batch, even with no live sound lease. The installed callback reaches probe_gatt_control_wake, which gives the persistent control-worker semaphore. It is not a direct high-priority host wake, and the binary semaphore may coalesce signals. The kernel operation and possible background work are nevertheless unnecessary without a sound-space consumer.

A bounded fix gates only PCM-space progress on the existing atomic sound token. Request completion, failures, cancellation and token retirement retain their unconditional lifecycle notifications. Token publication precedes attach; mix removal precedes token clearing. No new state, allocation, lock or wait is added. A negative regression reproduces idle data signals before the fix; positive cases preserve live-lease progress and no-token failed-reserve/release acknowledgements. The actual two-core/fixed-memory/early-release tests pass under ASan/UBSan, and the full async sound overlay/queue/takeover/cancellation suite passes UBSan. Host build compilation now uses ccache per translation unit. A post-test quota failure was resolved by hashing and removing two aged rebuildable PDBs; the clean rerun passes.

Same-diagnostic candidate50ea6288b709659343355a281ed7cdd04dec1ee07a09b7b8175a254a38305fc2 passes ccache/source/image admission and adds36 flash-text bytes with no IRAM/data/BSS/rodata delta. Its physical comparison is recorded in SOUND_PROGRESS_WAKE_2026_11_06.md. No sound feature is disabled, and the full goal remains open.

The retained normal image explicitly disables this temporary diagnostic; image admission verifies that the forced kernel header and accounting unit are absent. The optional source remains available through -TaskCycleAccounting for future bounded diagnosis. No diagnostic throughput number is substituted for the retained normal image result.

The subsequent [host-phase charge investigation](HOST_DEADLINE_CLOCK_2026_11_06.md) adds a separate permanent host slot under the same opt-in flag. It also corrects background host_printf callers altering the global host phase; earlier sampled console-phase counts do not establish real host console CPU cost. The corrected live host markers have zero context/hook faults. A clock-sharing candidate shows no useful gain and is reverted. Temporary task-cycle hooks/storage remain absent in the retained normal build.
