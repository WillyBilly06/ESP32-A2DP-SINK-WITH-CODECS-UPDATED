# Stage-specific Bluetooth diagnostics

Development instrumentation; no new production qualification is implied.
Do not infer radio loss, CPU-only decoder cost or a loss-free stream from an
aggregate drop count. The legacy `sbc_pcm` record applies to every codec.

| Record | Meaning |
|---|---|
| `vhci_timing` | Separate HCI packet/ACL/event/byte counts, callback context, receive rejection, callback maximum, first pending receive to host-poll maximum, host-poll maximum and ingress queue high water/rejection |
| `media_trace` | Individual exception: stage, microsecond timestamp, writer-local ordinal, actual/expected RTP sequence, RTP timestamp, current/previous SSRC, payload size, encoded queue occupancy and result |
| `media_drop_reasons` | Separate lifetime counters for queue full, decoder rejection, reset failure, PCM failure, PCM timeout, PCM BUSY attempts and upstream sequence/SSRC changes |
| `worker_timing` | Bounded worker-step wall time, total, maximum and histogram; a step has budget two and also services control work |
| `owner_wait_reasons` | Owner iteration count and a separate count for every subsystem requesting an immediate next iteration; distinguishes which wait predicate keeps the owner runnable |
| `decoder_execution` | Codec media/resume calls, elapsed wall time, maximum and histogram |
| `decoder_reset` | Codec reset calls and their own timing distribution |
| `pcm_delivery` | PCM provider calls, including output submission and checksum accounting, with their own timing distribution |
| `native_i2s` / `audio_clock` | Existing DMA EOF gaps, render time, descriptor faults, source frames, underruns, occupancy and clock state |

Exception stages are `input_sequence`, `input_ssrc`, `queue_full`,
`worker_sequence`, `worker_ssrc`, `decode_reject`, `reset_failed`, `pcm_failed`,
`pcm_timeout` and `pcm_busy`. A contiguous PCM BUSY episode emits one exception;
the attempt counter still counts every BUSY response. Input continuity is checked
**before** encoded-queue admission. Thus a locally rejected packet can cause a
later worker gap without being misreported as an upstream gap. Upstream gaps
still do not identify a particular radio/controller/peer cause.

The host producer and decoder consumer have separate 16-record SPSC diagnostic
rings. Each record includes a writer-local ordinal; `host_trace_lost` and
`worker_trace_lost` expose ring overflow. The existing low-priority logger drains
at most two entries from each ring per 100 ms iteration. No UART output, heap
allocation, locks or waiting are added to transport or media callbacks. Heavy
exception rates may lose diagnostic entries; their counters remain visible.

Timing uses 32-bit microseconds, wrapping about every 71.6 minutes; compute
unsigned differences between nearby snapshots. Totals/counters are lifetime
values for the finite trial. Histograms have seven inclusive upper bounds
125/250/500/1000/2000/4000/8000 us plus a final overflow bucket. All timing is
**elapsed wall time including preemption**, not measured CPU utilization.
In synchronous mode a decoder call can invoke PCM inline; its record explicitly
sets `includes_inline_pcm:true`. These nested times must not be added together.
Deferred mode measures codec calls and PCM delivery separately. The first
pending receive-to-poll indicator is not exact per-packet HCI queue residence.
Logging overhead is present in these measurements and must be accounted for in
any later performance comparison.

The deployment receipt now counts task/interrupt watchdog warnings separately
from boot/fatal markers, with idle starvation and sampled running tasks by core.
A watchdog warning fails runtime health even if boot readiness succeeds. New
trial archives retain the exact ELF as well as the image and source receipts so
backtraces can be decoded against the actual executable.

## Scheduling changes under validation

`build_probe.ps1 -DecoderCore 0` or `-DecoderCore 1` selects the experimental
decoder placement. The build receipt, CMake cache, compiler definition and
created task must agree. The controller/host stay on CPU 1; renderer priority 6
and logger priority 1 stay on CPU 0; the decoder retains priority 5. This is a
measurement switch, not a claim that either arrangement passes every workload.

The decoder now queries actual runnable work and codec/control/output deadlines
in both output modes. It waits on task notifications when idle and services
expired deadlines immediately, replacing the old synchronous-mode 20 ms polling
fallback. The existing bounded media budget remains two. `taskYIELD()` alone
does not schedule a lower-priority idle task while the decoder remains runnable,
as specified by the [FreeRTOS kernel documentation](https://www.freertos.org/Documentation/02-Kernel/04-API-references/04-RTOS-kernel-control/00-Kernel-control).
Changing CPU placement and removing a polling timer are separate changes.

Product-configuration load, product-IO load and startup-sound retirement now use
completion notifications with absolute deadlines instead of 10 ms flag polls.
The actual helper passes seven ASan/UBSan host cases covering pending and
unrelated notifications, deadline expiry, rounding and long uptime. General
control/provider/name waits and startup sound pumping still need further audit;
these changes do not establish that every runtime path is fully asynchronous.

## Instrumented CPU 1 reference

Run `1793825586886774600-cc6321d64499` decoded LDAC 96 kHz while maintaining the
secured BLE connection and live Deep Bass/Balanced writes. It recorded 5,625
retired packets and **1,663 queue-full drops**, with zero upstream sequence gaps,
HCI ingress rejections, decoder rejections or PCM failures. Queue occupancy
reached 8/8. The largest decoder wall span was 92,352 us; 15 watchdog events
reported CPU 1 idle starvation. This is a software queue/scheduling failure in
the observed run, not evidence attributing missing packets to RF.

The exception rings reported 784 host and 454 worker records lost; aggregate
reason counters are distinct from those diagnostic losses. One latency metric
(`first_rx_to_poll_max_us`) underflowed after a callback preempted its caller's
clock sample. That metric is excluded from this run. A post-exchange clock read
and regression correct the race. Other counters and timing spans are independent.
See [the sanitized reference](RF_CORE1_DIAGNOSTICS_2026_11_04.json).

## Existing captured scheduler failure

Reclassifying the preserved run `1793823953028623200-6bceaf1587e5` found **22
task-watchdog triggers and 22 CPU 1 idle-starvation warnings**. CPU 0 was sampled
in IDLE0 all 22 times. CPU 1 samples were decoder 11, controller 4 and main/host 7.
These snapshots prove starvation occurred; they do not allocate CPU usage or
identify the sole responsible task. This older trial had no distinct media-drop
reason or phase timing instrumentation. Its original readiness/fatal fields
missed nonfatal watchdog warnings. The corrected
[SC/bond trial record](RF_SC_BOND_2026_11_04.json) preserves this distinction.
