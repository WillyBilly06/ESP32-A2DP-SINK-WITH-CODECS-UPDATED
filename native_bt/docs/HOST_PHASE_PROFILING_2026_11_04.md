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

Live phase measurements and the next performance fix remain pending. No audio
continuity or feature-parity pass is claimed.
