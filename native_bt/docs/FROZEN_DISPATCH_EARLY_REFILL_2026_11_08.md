# Frozen task trace and early local DMA refill

Local PCM is now offered to an open DMA refill window immediately after the
actual render call returns, before scheduler profiling, sound-clock bookkeeping
and render-phase diagnostics. DSP/mixing and publication have already completed.
The existing later refill and all deadline, stop, ownership and copy-budget
checks remain. This corrects a reproducible late-handoff ordering defect; it
**does not fix LHDC continuity**. Current metronome windows end2112/11712 unfilled
PCM frames; the second has a confirmed playback-time empty-input gap. Music ends0.

## What the frozen trace establishes

An optional DispatchTrace mode records the last16 events per core and freezes
at the first finalized gap. It uses the pinned SDK's standard task-selection
trace point plus public permanent task handles. No private TCB field is read.
Other markers cover EOF/interrupted handle, render/decode phases, helper submit/
completion, post-publication progress and owner-yield intent. Each writer uses
a bounded local IRQ-mask span over atomic cells, without a cross-core lock,
allocation, kernel call or UART operation. The background logger takes one
snapshot attempt and emits separate raw records once per frozen core/window.
An already-entered other-core write can finish after freeze; timestamps retain
that fact. No retry/spin or tick sampler is introduced.

Snapshots are unavailable beyond15s from stream begin. At fixed240MHz even one
write/cycle cannot wrap the32-bit per-core head within that horizon. Begin heads
therefore replace a redundant live fill counter. Tests cover both timer and head
wrap. The trace tail can omit earlier transitions: in one capture Core1 retains
only353us before the gap. It cannot explain an entire1ms interval when that
prefix has been overwritten. Selection intervals also include IRQ/resume time;
they are not pure task CPU utilization. Runtime observer overhead is not isolated.

Initial diagnostic imagef870193daabf used the incorrect lookup btdm_task.
Its binding mask990 left role5 unbound; unknown actors in those records must
remain unknown. The bundled controller archive contains btController, and that
public task lookup now resolves on-device. New imageaf37df00e68c reports mask1022,
with all nine declared roles bound. No task-name API runs in the interrupt path.

The old diagnostic's second finalized gap is after phone pause ACK. The actual
render call has returned by the render-leave marker185us before closure, so its
PCM was already published. A controller-unidentified interval and then host
selection intervene; the post-publication progress marker runs104us before
closure. The guarded deadline is77us before closure, requiring a copy admission
more than32us earlier. Thus that later attempt is already5us past its admission
cutoff. This identifies an unsafe opportunity for bookkeeping/preemption between
publication and service; it does not prove every gap shares that cause.

The early-refill change closes that opportunity before diagnostics. It never
publishes partially mixed PCM or masks interrupts during DSP. Existing mixing
finishes before render publication, including WAV mixing. No priority, affinity,
codec, precision, buffer size, guard or copy budget changes. The64us guard and
32us copy allowance are retained. One PCM sample frame at192kHz is5.208us;
a960-sample LHDC codec frame is5ms; the current192-frame DMA period is1ms.

Trace event kind6 is a post-publication progress observation, not the exact
atomic publication time. Its samples may already have reached DMA. Submit begin/
end bracket the helper call, not an invented exact cross-core release instant.

## Validation and cost

- The actual render-step test injects preemption immediately after the early
  copy, advancing the virtual clock past the guarded deadline. The64-frame
  prefix survives. Removing only the early call fails fixture line138; the
  failed executable is independently rerun to confirm that failure separately
  from its wrapper's disk-cleanup failure. Sources/binary/receipts remain.
- Actual output/refill ASan/UBSan checks pass at100/1000 simulatedHz, with initial/
  final latches, partial copies/order, expiry/cancel, bounded bursts, startup/wrap,
  and exact split/full DSP PCM with EQ/volume changes at48/192kHz.
- Actual two-core mocked DSP/sound lifecycle passes early input release, held
  borrows, queued stop/cancel, six-block cap, completion/producer ordering,
  reserved-decode admission, restart and no timed pending polling.
- Frozen trace/actual JSON formatter ASan/UBSan checks pass, including disabled/
  stopped/unknown states, both cores, restart, wrap,15s rejection and one-attempt
  interference. No-freeze and unchecked-reader controls fail as intended.
  Actual configuration tests cover9 CMake and15 image-admission cases.
- Portable core191/191 and normal/detailed shared-trace and console admission
  checks pass. The portable core source is unchanged by the local render edit.
  All C/C++ objects use ccache with explicit -c; linking is separate.

The optional observer adds344B initialized DRAM,8B BSS and488B IRAM versus the
previous normal image. It adds no task/stack or media buffer. Its first link was
12B over IRAM. The observer already used Os; selecting Os again did not help.
Removing the redundant fill bookkeeping made it fit without moving audio code
or changing the memory split. Held maps are retained losslessly. The early-copy/
controller-label correction adds no further SRAM/IRAM; linked flash text/rodata
are4/16B smaller than the first diagnostic image. This is an aggregate size
comparison, not a CPU-speed claim. DispatchTrace is off by default; the normal
flag-off target image has not been rebuilt/measured in this checkpoint.

The expanded audit verifies all observer IRAM functions, local code/literal/data
references, direct SDK dependencies and an actual kernel task-switch callsite.
The SDK current-task getter remains checked. This is not the entire SDK callgraph
or proof of cache-off OTA continuity. The complete ccache build/recheck admits
10423 stable inputs and509.576MiB rebuildable outputs under512MiB. Disk gate
failures are retained; completed symbols, historical controls/maps and SDK logs
are gzip archived with on-disk decompression/SHA verification. Quotas are unchanged.

## Physical evidence

Current COM6 image:
**af37df00e68cf164eec9e8159dd6c047d22e4e17e95c63b8a41c687c6c7f6c1c**.
Upload921600/readback115200, all written-region comparisons match. The original
app is preserved, owned selection is VALID, automatic Bluedroid fallback is false.
Existing37 cold configurations/173callbacks pass, tracked peak56899B, minimum free
after allocation105808B and decoder stack low3484/8192B. No boot watchdog,
backtrace or idle-starvation marker is observed. COM6 currently has DispatchTrace
**enabled** for diagnosis; this is not a normal/stable release image.

Approved Redmagic/native YouTube,192kHz/S24/stereo/fixed900. Phone volume150/150;
valid DSP snapshots retain gains11/8/12dB, volume127 and flags12. Flag4 bypasses
split/crossover, not the four common filter stages. All requests are10s with an
independent14s phone pause guard; I2S spans13.045–13.266s. Counters include the
whole stopped trace and tail. Only first gaps are latched; no total is reduced
by pause classification. Sequential histories and observer effects preclude a
causal performance percentage from this small comparison.

| Image/workload | Initial shortage | Recovered | Final unfilled | First finalized gap |
| --- | ---: | ---: | ---: | --- |
| Trace-only f870/metronome1 |3648|2240|1408|5.17–5.55s before pause|
| Trace-only f870/metronome2 |1408|896|512|after pause ACK|
| Early refill af37/metronome1 |4096|1984|2112|near pause; ACK not separable|
| Early refill af37/metronome2 |23936|12224|11712|7.08–7.33s before pause|
| Early refill af37/music |0|0|0|none|

The latest active-playback gap has raw/processed PCM both0. The decoder has
waited for encoded input6871us, the host sleep intent is4624us old, and no ACL
arrival metadata is pending. Last ACL callback is13380us before the gap; latest
RTP admission11277us before it. A coherent pending-credit sample6036us earlier
has Classic0/LE0; the last completion submission is10127us earlier. This is an
SDK submission receipt, not proof that the controller processed the command.
The frozen selections show idle/blocked intervals rather than continuous decoder
computation. The missing input's source/controller/credit cause remains open;
this is separate from the earlier decoded-PCM consumer miss.

In four gap captures, elapsed host-admission time exceeds the corresponding
wire RTP media span by52.225/50.810/53.931/47.675ms. This correlates with the50ms
startup guard and motivates a controlled startup/credit-feedback investigation.
It does **not** prove that guard caused a controller or source stall. Stream
setup completes before the nonblocking guard; it is not a synchronous50ms sleep.
RTP sequence breaks are0 and the observed maximum forward timestamp step960,
matching one5ms codec unit. These observations do not identify RF throughput or
justify claiming a source packet-loss count from timing alone.

All five windows have zero queue-full/decode reject, helper/descriptor and
host/worker/UART log-loss faults. Accepted/rendered/DMA-source totals agree at
stop. Integrity is326/327/361/327/297 Verified,1/1/1/1/2 explicitly Unverified
legacy rows, zero Invalid. The final6s passive window has195 Verified and zero
Invalid/Unverified, monitor source121 at921600. YouTube is PAUSED; I2S stopped.
Stopped heap free/minimum/largest is62668/29020/59392B; decoder/render/logger
stack low waters5912/3772/1276B. These are finite-history measurements, not leak,
acoustic, sustained1Mbps, BLE/control, security, full feature or OTA qualification.

Evidence:

- native_bt/_build/experiments/frozen-dispatch-1794154641956/
- native_bt/_build/experiments/frozen-dispatch-1794154641956/early-refill/
- native_bt/_build/hardware/runs/1794156022332313100-f870193daabf/
- native_bt/_build/hardware/runs/1794157422692209900-af37df00e68c/

Development branch only. Main/stable releases remain unchanged. The master goal
is incomplete; the next focus is the input/admission timing and startup credit
feedback, preserving the consumer-ordering correction and independent counters.
