# Stage-specific Bluetooth diagnostics

Development instrumentation; no new production qualification is implied.
Do not infer radio loss, CPU-only decoder cost or a loss-free stream from an
aggregate drop count. The legacy `sbc_pcm` record applies to every codec.

| Record | Meaning |
|---|---|
| `vhci_timing` | Separate HCI packet/ACL/event/byte counts, callback context, receive rejection, callback maximum, first pending receive to host-poll maximum, host-poll maximum and ingress queue high water/rejection |
| `media_trace` | Individual exception: stage, microsecond timestamp, writer-local ordinal, actual/expected RTP sequence, RTP timestamp, current/previous SSRC, payload size, encoded queue occupancy and result |
| `codec_configuration` | Actual stream-start CIE bytes, codec ID, negotiated sample rate/channels and PCM container size; container size alone does not establish encoded bit depth |
| `media_input` | Source media packets/payload bytes and first/latest RTP timestamps, reset on stream epoch; counts input before local queue rejection |
| `media_drop_reasons` | Separate lifetime counters for queue full, decoder rejection, reset failure, PCM failure, PCM timeout, PCM BUSY attempts and upstream sequence/SSRC changes |
| `worker_timing` | Bounded worker-step wall time, total, maximum and histogram; a step has budget two and also services control work |
| `owner_wait_reasons` | Owner iteration count and a separate count for every subsystem requesting an immediate next iteration; distinguishes which wait predicate keeps the owner runnable |
| `decoder_execution` | Codec media/resume calls, elapsed wall time, maximum and histogram |
| `decoder_reset` | Codec reset calls and their own timing distribution |
| `pcm_delivery` | PCM provider calls, including output submission and checksum accounting, with their own timing distribution |
| `native_i2s` / `audio_clock` | Existing DMA EOF gaps, render time, descriptor faults, source frames, underruns, occupancy and clock state |
| `render_work` | Existing renderer elapsed total/call count, maximum, input occupancy, stack low water and allocation/request failure details |
| `i2s_descriptor_faults` | Separate invalid buffer shape, exhausted descriptor map and descriptor-order counts, with expected/observed index and event size; original validation remains enforced |

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

## Instrumented CPU 0 comparison

Run `1793826207937572100-db937eece724` retained the same phone, LDAC 96 kHz,
secured BLE reconnect, Deep Bass/Balanced writes and eight-slot queue. It decoded
**11,238 packets / 92,016,640 PCM bytes**, with **zero drops, resets, ingress
rejections, upstream sequence breaks or watchdog warnings**. No diagnostic
records were lost. Queue occupancy peaked at 2/8; maximum codec wall span was
4,111 us, versus 92,352 us in the CPU 1 reference. Playback was paused and the
released baseline returned cleanly. See [the result](RF_CORE0_DIAGNOSTICS_2026_11_04.json).

These are live runs at different playback offsets/durations, not identical
encoded-vector benchmarks. The CPU 0 trial also corrects the diagnostic clock
race; it retains the old decoder wait loop for the placement comparison. It
does not include A2DP I2S output. A subsequent full-output image contains the
notification/deadline changes and is a separate qualification gate.

For maximum LHDC V5 checks, preserve actual CIE/phone readback alongside timing.
AVDTP LHDC bitrate capability limits and an Android 1000 kbps quality request do
not by themselves prove constant 1000 kbps wire traffic. `media_input` enables
an elapsed-time payload throughput estimate, including codec packet framing;
snapshots are separate atomic observations and may differ by an in-flight packet.
Verify RTP clock frequency independently before using RTP deltas: the captured
LDAC stream advances timestamps at approximately 48 kHz while producing 96 kHz
PCM. Do not infer wire bitrate by substituting the PCM rate into RTP time.
Do not estimate across stream epochs, SSRC changes or timestamp discontinuities.

## Full output is still failing

The [full core-0 trial](RF_FULL_CORE0_2026_11_04.json) used deferred PCM and the
notification/deadline changes. LDAC 96 kHz reached I2S, then recorded 216,448
underrun frames, one descriptor fault and one CPU 0 idle-starvation watchdog.
Its exact ELF maps the watchdog sample to `dbt_resampler_filter` called by
`dbt_audio_render` / `render_step`. There was no CPU 1 starvation warning.
Maximum codec call time was 2,917 us, maximum renderer span 12,844 us and maximum
DMA EOF gap 11,476 us. There were 210 local queue rejections and one PCM failure,
with zero ingress rejections or incoming RTP gaps. Renderer preemption is
included in PCM provider elapsed time; these are not CPU-only cycle measurements.

This image failed before LHDC could be selected. The baseline returned cleanly.
One watchdog sample identifies where the renderer was interrupted, not its
exclusive CPU cost. Later instrumentation exposes existing renderer totals and
separates descriptor reasons; their hardware validation remains pending.

## Existing captured scheduler failure

Reclassifying the preserved run `1793823953028623200-6bceaf1587e5` found **22
task-watchdog triggers and 22 CPU 1 idle-starvation warnings**. CPU 0 was sampled
in IDLE0 all 22 times. CPU 1 samples were decoder 11, controller 4 and main/host 7.
These snapshots prove starvation occurred; they do not allocate CPU usage or
identify the sole responsible task. This older trial had no distinct media-drop
reason or phase timing instrumentation. Its original readiness/fatal fields
missed nonfatal watchdog warnings. The corrected
[SC/bond trial record](RF_SC_BOND_2026_11_04.json) preserves this distinction.
