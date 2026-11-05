# Host phase profiling

The task sampler observes host work dominating core1 while the decoder gets
little execution. This follow-up tags host work without changing priorities,
core placement, codec settings, buffers, protocol budgets or notification waits.
It uses the existing finite `-SchedulerTrace` mode and adds no task/queue/heap
allocation. Phase counters add 100 bytes of static state.

Each coherent scheduler snapshot includes non-overlapping host phase samples:

| Counter | Tagged work |
| --- | --- |
| controller_query | Public VHCI send-availability check |
| controller_submit | Public VHCI packet submission |
| runtime_poll | Portable runtime poll, excluding nested controller calls |
| wait_calculation | Owner's combined deadline/readiness calculation, excluding nested controller calls |
| gatt_poll | ATT/GATT poll and deferred setting publication retries |
| security_poll | Security preflight, Classic security and SMP poll |
| product_poll | Product peer/security/configuration polling |
| io_poll | Product I/O owner polling |
| audio_poll | A2DP profile poll |
| avrcp_poll | AVRCP profile poll |
| console_log | Main-file printf calls, forwarded with unchanged formats through vprintf |
| other | Remaining host execution and untagged startup work |

The host alone changes the current tag with atomic load/store operations.
Nested controller calls restore their caller's tag. Tick hooks count a phase
only when the sampled task is the host; decoder/controller/logger execution
cannot be charged to the host merely because a tag remains set.

`host_phase_samples` is separate from `scheduler_samples` but both are emitted
from the same captured structure and have the same `last_tick_us`/coherence
flag. Pair them on that tick timestamp, not UART publication time. The second
line is printed later; its snapshot envelope can include serialization of the
first. All prior limits remain: samples are not exact CPU utilization, and
masked interrupts/replayed ticks do not identify work inside a missed interval.

ASan/UBSan pass phase classification, all twelve tags, nested restore, invalid
tag fallback, non-host exclusion, and phase-count sum versus host-count identity,
in addition to the earlier wrap/gap/hook-registration checks. Mode admission
guards and the real-codec async probe regression pass. The all-codec ccache
ESP-IDF6.1 build, image/source admission and limited IRAM audit pass. A separate
actual SDK compile of main with tracing disabled contains no sampler references;
that is a translation-unit check, not a separate linked/default RF qualification.

## First RF phase measurement

[Exact results](HOST_PHASE_RF_RESULTS_2026_11_04.json) bind source dc418d8e,
image f25ae4ab814b and its archived ELF. This image retains the full statistics
getter in the audio poll; it was frozen before the candidate hot-status fix.
Both Android and the ESP32 confirm maximum LHDC configuration. Playback starts
after Android confirms 192 kHz/S24/stereo with the 1000 kbps nominal target;
actual encoded bitrate is unavailable.

The coherent core1 phase records at 103.361466 and 112.118466 seconds span
startup, the watchdog/DMA fault and some post-fault time. Among their 3434 host
samples:

| Phase | Samples | Fraction of host samples |
| --- | ---: | ---: |
| Wait calculation | 685 | 19.95% |
| Audio poll | 600 | 17.47% |
| Runtime poll | 580 | 16.89% |
| Other | 425 | 12.38% |
| GATT poll | 331 | 9.64% |
| Security poll | 221 | 6.44% |
| Product poll | 184 | 5.36% |
| AVRCP poll | 181 | 5.27% |
| I/O poll | 118 | 3.44% |
| Controller submit | 87 | 2.53% |
| Controller query | 22 | 0.64% |
| Main console log | 0 | 0% |

This is a biased startup/fault sample window, not exact CPU utilization or a
steady-state benchmark. Host work is distributed across polling; the samples
do not support one dominating blocking controller call. Audio polling is a
meaningful measured category. Source inspection independently identifies its
unnecessary full-statistics/stack scan; [cheap status queries](HOT_STATUS_QUERIES_2026_11_04.md)
are the next hardware comparison without changing priorities or formats.

Audio still fails: 878 queue-full drops, 851712 underrun frames, one core1 idle
watchdog and a descriptor-order fault. The DMA fault follows a 12581 us EOF gap
at 109.987727 s, 33 us before the watchdog hook. Shape/pool are valid, expected
index 2, observed 1, not stopping. Both upstream sequence and size-rejection
counters remain zero. The finite helper observes a clean released-baseline
return; the phone is paused. Three malformed/interleaved records are excluded.
No audio continuity or feature-parity pass is claimed.
