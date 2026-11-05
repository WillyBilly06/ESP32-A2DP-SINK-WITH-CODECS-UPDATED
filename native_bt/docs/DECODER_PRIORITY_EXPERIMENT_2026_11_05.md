# Finite equal-priority host/decoder experiment

Repeated maximum-LHDC trials show little decoder service on core1 while host
work dominates and core0 often waits for input. The retained atomic-status fix
removes repeated stack scanning, but its traced RF trial still starves core1.
A tracing-disabled comparison runs first to separate profiling overhead.

`-DecoderPriority 6` prepares a separate finite comparison. The decoder remains
on the selected core (core1 in this candidate), renderer remains core0 priority6,
host remains core1 priority6, logger remains priority1 and controller settings
are unchanged. Existing RTOS time slicing can then alternate ready host/decoder
tasks at equal priority. This is not a worst-case 1 ms guarantee under controller
preemption or interrupts. No vTaskDelay, polling period, task or buffer is added.

Default decoder priority stays 5. Priority6 requires a finite live async audio
probe; codec-only, pipeline-only, checksum-only RF and kept-active builds are
rejected. The receipt, CMake cache, actual decoder compile definition and task
creation must agree. The scheduler-layout record reports the actual selected
priority rather than a hardcoded literal. The deployment helper requires a
finite observation even if all other buffer settings are default.

Ten CMake cases, eighteen actual image-admission cases, three finite-deployment
cases and one early CLI rejection pass. Real-codec async probe tests pass at
priorities 6 and 5 and verify the priority supplied to task creation. Existing
compact/affinity mode guards pass. These validate construction/admission only;
the host mock does not model ESP32 scheduling or establish radio performance.

The running priority5/tracing-disabled image was frozen before these edits.
Exact original production/tool input bytes are archived under `owning-sources`,
verified against its source manifest before copying. A separate build cannot
alter its image or ELF.

The all-codec ccache target build, source/image/layout admission and limited
IRAM callback audit pass. Physical priority6 comparison remains pending. Keep
controller ingress/credit, queue, decoder, PCM, DMA, BLE and watchdog evidence
separate. Do not promote this experimental priority or claim continuity before
its actual workload/lifecycle tests pass.
