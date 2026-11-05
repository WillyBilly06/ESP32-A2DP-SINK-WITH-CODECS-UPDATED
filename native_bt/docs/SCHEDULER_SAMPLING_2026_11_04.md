# Per-core task sampling and tick-gap diagnostics

The 1800-byte packet trial still drops packets smaller than its old limit.
It also records an I2S descriptor fault 35 us before the public task-watchdog
hook. That hook runs after SDK task diagnostics; a stack sample alone does
not identify CPU utilization or prove a blocking controller call.

`-SchedulerTrace` adds a finite, opt-in measurement to the existing audio
probe. It changes no codec, DSP, buffer target, affinity or task priority.
There is no new task, queue or dynamic allocation. The released path and
normal probe defaults stay unchanged.

Public RTOS tick hooks sample each interrupted task at the configured 1000 Hz.
Known permanent task handles identify host, decoder, renderer, logger and
controller; each core's public idle-task handle identifies idle. Everything
else remains `other`, and `known_task_mask` exposes unsuccessful name lookups.
Role numbers are 0 host, 1 decoder, 2 renderer, 3 logger, 4 controller, 5 idle,
6 other. Tick gaps record both ESP timer endpoints and the roles observed
before/after the largest gap. Those roles do not identify work inside the gap.

These are interrupted-task sample counts, **not exact CPU utilization**.
Periodic work aliases tick sampling; masked/higher-priority interrupts delay
it. The ESP32 port's `_frxt_timer_int` catch-up loop also replays overdue ticks,
which can credit several samples to the task observed after the delay. Do not
attribute the missing interval's execution from those counts. Tick gaps do not
by themselves distinguish interrupt masking, another ISR,
cache stalls or watchdog diagnostics. DMA and tick timestamps share ESP timer
units, making correlation possible without mixing SDK console timestamps.

Each core owns its counters. ISR code makes bounded atomic load/store updates;
it does not take a kernel cross-core lock or query priority. Logger snapshot
reads once and flags a concurrent update as incoherent rather than retrying.
Registration is once per boot, with the first hook removed if the second fails.
A registration failure disables sampling and is explicitly reported; audio
initialization continues and must not be mistaken for a valid sampling run.

The small public `xTaskGetCurrentTaskHandle` getter is explicitly linked into
IRAM only for this mode. Other task APIs keep the SDK flash default. Target
disassembly confirms direct sampler calls to the IRAM getter and ESP timer;
the getter's sole indirect call is a literal ROM `_xtos_set_intlevel` target.
No private controller register, task layout or protocol assumption is used.
The offline audit still treats ROM internals and unrelated I2S indirect calls
as unproven. Callback body cycles exclude hook/ISR entry and final publication,
and can include higher-level preemption.

## Local validation

ASan/UBSan pass without signed-overflow wrapping: per-core task classification,
counter/sequence/ESP-timer wrap, gap endpoint identity, nonspinning snapshots,
missing-controller accounting, invalid arguments, double initialization, and
rollback on either hook-registration failure. Six actual CMake configurations,
ten image-admission cases, three finite-deployment cases and one early CLI
rejection pass. Existing compact-mode/affinity admission and real-codec async
probe tests pass.

The full ccache ESP-IDF6.1 build and source/image admission pass. The placement
audit protects eight symbols and scans nine direct functions with no unsafe
direct call. Five indirect calls remain conservatively reported: four I2S
callbacks and the getter's manually resolved ROM call. Owned linker fragments
are now included in source fingerprints.

## First RF measurement

[Exact results](SCHEDULER_SAMPLING_RF_RESULTS_2026_11_04.json) bind source
5db1c811, image 36e01d12bdab and its archived ELF. Both hooks register and all
five task identities resolve (`known_task_mask:31`). All 37 pre-controller
codec configurations pass. Android and the ESP32 separately confirm maximum
LHDC configuration, CIE `00ff3a050000354c0102110000`. Playback is started only
after Android confirms 192 kHz/S24/stereo and the 1000 kbps nominal target;
actual encoded bitrate is not exposed.

The following interval contains stream startup, the watchdog/DMA fault and
some time afterward. It is not a steady-state throughput or CPU-utilization
measurement. Counts include the tick replay limitation above.

| Core | ESP-timer interval | Host samples | Decoder samples | Renderer samples | Idle samples |
| --- | --- | ---: | ---: | ---: | ---: |
| 0 | 75.287203–81.928202 s | 0% | 0% | 10.48% | 86.46% |
| 1 | 75.316489–81.961489 s | 62.23% | 5.97% | 0% | 11.87% |

Core1 also observes controller 9.13%, logger 10.71%, other 0.08%. The host is
the dominant observed task while the producer gets little execution and core0
often lacks input. This changes the next investigation to host phase cost and
fair scheduling, rather than assuming decoder or renderer work alone explains
the starvation. A sampled PC in this run is a product-runtime security-value
comparison, not the readiness query sampled in the prior trial. Neither PC
alone proves that its function blocks or dominates execution.

The first recorded queue rejection is sequence 9 at 75.678887 s, 628 bytes,
eight queued. Upstream sequence-break and size-rejection counters remain zero.
Those software drops precede the watchdog-related DMA event by about 4.618 s;
the later DMA fault does not explain all earlier underruns.

At the fault, independent clocks from the two tick hooks correlate as follows:

- Core0's largest tick gap: 80.285203–80.296623 s (11,420 us), idle to renderer.
- DMA descriptor fault: 80.296605 s, expected index 0, observed 1, 1024 bytes,
  valid shape/pool and not stopping; preceding EOF gap 11,866 us.
- Public task-watchdog hook: 80.296637 s, **32 us after the DMA fault**.
- Core1's largest tick gap: 80.284489–80.316627 s (32,138 us), host to host.

Both cores experience delayed tick service around watchdog diagnostics; SDK
source prints task diagnostics before the user hook. This supports the
watchdog-activity explanation for this fatal EOF gap. It does not identify
execution inside those gaps or eliminate the preceding host starvation.
Keep the watchdog and descriptor ownership checks intact.

The sampler's maximum observed body duration is 1118 cycles on core0 and 9698
on core1 (4.66/40.41 us at 240 MHz), including possible higher-level preemption.
These maxima exclude hook/ISR entry and final publication and are not averages.
Logger stack remaining is 1460 bytes. No new allocation failure occurs.

Audio qualification fails: one core1 idle-starvation watchdog, one descriptor
fault and 795968 underrun frames. Only 79680 source frames reach DMA. The finite
helper correctly fails its runtime-health gate and returns to released firmware;
phone pause is independently observed. All 2005 complete periodic records used
here pass individual CRC/time validation; three malformed/interleaved lines are
excluded. No audio continuity or feature-parity pass is claimed.
