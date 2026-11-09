# CPU1 runtime and BLE reconnect preparation — 2026-11-08

Current development image: **7d131bf9e47d0f90ec138c64159c03ed00075a2cf68b49602e28f8d5362d0352**. COM6 upload at 921600 and full readback at 115200 matched. Firmware and the passive raw-UART monitor use **921600 baud**. The original application and flash layout remain preserved; automatic Bluedroid restoration is disabled. This is unqualified owned-host development firmware.

## Changes

Bonded reconnect preparation (LTK loading / resolvable-private-address resolution) now uses the existing temporary priority-1 SMP worker on Core 0. It is preemptible by decoder priority 21 and DSP helper priority 20. Fresh-pairing work and maintenance retain Core 1. Controller priority 23, host 22, renderer 22, decoder 21, DSP helper 20 and logger 1 otherwise keep their current affinities/priorities. No task, queue, stack or audio buffer is added. The shared storage-poison latch is now a lock-free atomic bool of the same size. Existing mailbox ownership and one-byte queue tokens remain unchanged.

Completion diagnostics record the actual worker core for load/resolve operations. Stage 9 size retains success/existing in bits 0/1 and adds core-valid/core-ID in bits 2/3. These are diagnostic flags only, not codec or Bluetooth wire flags. Host tests assert actual worker execution and requested affinity: reconnect Core 0, fresh pairing/maintenance Core 1. Target footprint versus the preceding normal861ec6f23c4a image is unchanged for IRAM, DRAM data and BSS; flash text increases 208 B.

The optional build switch **-TaskRuntimeOnly** enables public FreeRTOS task-runtime observations without the tick sampler, detailed host cost markers or task-cycle hooks. It requires runtime statistics and rejects simultaneous SchedulerTrace/TaskCycleAccounting. Permanent task handles are registered once; the background logger makes at most 11 GetInfo calls without stack/state scans. Counters include interrupt time and exclude an unfinished current quantum. Sequential reads have a measured interval: they are not an atomic whole-system snapshot or pure task-instruction accounting. The current flashed image has all these optional profilers OFF.

ELF debug sections are compressed with GNU ld zlib compression. The comparison before/after compression preserved every allocated executable/data section except build metadata in .flash.appdesc. All 11 decompressed debug-section SHA256 values matched; no symbols were stripped. The compared ELF shrank from 12,170,536 to 5,222,668 B. This changes rebuildable disk usage, not firmware RAM. The 512 MiB rebuildable-artifact limit remains unchanged.

## UART ownership and overhead

Build default, generated SDK configuration and the live verified monitor all use 921600 baud. Bluetooth/decoder producers copy bounded diagnostic events and never wait for the UART. The existing priority-1 Core 1 logger owns formatting and complete-record writes; its interrupt-driven UART write may sleep in that background task. This is not a zero-cost UART path. At 8N1, 100 bytes take about 1.085 ms on the wire, in addition to formatting/interrupt work.

Expected PCM backpressure retains counters without individual normal-mode event printing. Fault/lifecycle events and first-gap snapshots remain. Nominal event draining is 100 ms, playback counters 1 s and unchanged idle detail 5 s; these are scheduler-dependent cadences. No full per-packet print is added. The passive web viewer retains individual raw records. The current console admission checks pass 4 CMake, 14 image, 9 SDK cases and 1 CLI rejection.

## Measurements and limits

All live excerpts used only COM6 and the approved Redmagic, native YouTube, LHDC V5 / 192 kHz / 24-bit stereo /phone-selected fixed 900 kbps. Each had an independent 14 s phone pause guard and actual I2S upper bound below 15 s. **900 kbps is not 1 Mbps qualification.**

The counter-only baseline image 4f7d70ffc755 charged roughly 50–52% of its partially active observation windows to the host context on Core 1, versus 20–21% renderer and 17–18% controller. These include ISR time; the windows overlap stopped time, so they are not exact playback-only CPU percentages. In the metronome window, conservative active Core 1 host bounds were 46.25–58.34%. There was no fully interior active window and no rigorous active Core 0 headroom claim. Baseline music RPA request-to-owner completion took 2,502,677 us and Android authentication failed; another baseline excerpt resolved in 1,088,871 us and completed controls.

On diagnostic reconnect image b8cc7d67eeb0, actual Core 0 completion flags were observed twice:

| Excerpt | Resolve request to owner completion | Controls observed before pause | Initial / unfilled PCM frames | Descriptor / new watchdog events |
|---|---:|---|---:|---:|
| Music |63,253us|Yes|2240 /1856|1 /1|
| Metronome |28,940us|Yes|0 /0|0 /0|

The first diagnostic music gap preceded pause and the watchdog; the descriptor fault and watchdog user-hook timestamps were 13 us apart. SDK watchdog diagnostics print in an ISR critical section before that user hook. Consequently, the hook timestamp cannot date the beginning of the alarm's UART work. Neither the watchdog nor its diagnostics have been suppressed. The earlier active gap remains an independent unresolved miss.

The normal current image removes the temporary observer. Cold checks pass 37 codec configurations / 173 callbacks, tracked allocation peak 56,899 B, minimum free heap 106,128 B and stack low-water 3596 B. Compared with the diagnostic image, the cold minimum recovers 2,956 B. There were no boot watchdog/backtrace/idle-starvation markers.

| Normal-image excerpt | Resolve completion | Controls observed before pause | Initial / unfilled PCM frames | Descriptor / new watchdog events |
|---|---:|---|---:|---:|
| Music, 13.193 s I2S upper bound |53,897us|No; completed after pause|1152 /1152|0 /0|
| Metronome, 13.159 s upper bound |55,133us|Yes: ACK, independent readback and original CCCD restored|0 /0|0 /0|

The music first-unfilled snapshot was bounded 2.861–3.126 s after the requested pause, after its acknowledgement, with raw/ready queues empty. These stop-tail frames are retained; the trace is not a during-playback BLE-control pass. The metronome capture had 148 CRC-verified and 15 legacy/unverified records, no malformed records. The preceding normal music capture had 200 verified / 15 unverified. The diagnostic music failure retained 143 verified / 22 unverified / 1 malformed, rather than discarding interrupted output.

After both normal excerpts, minimum free heap was 16,088 B, stopped free 62,088 B and largest block 59,392 B; decoder low-water 5912 B. These finite observations do not establish leak freedom, worst-case RAM or a comparison against Bluedroid under equivalent workloads.

## Reproducibility and outstanding work

The ccache target build retained 10,425 unchanged source inputs. All 1,036 compilation database entries use explicit object-only -c, with ccache Ninja rules. Offline exact-image admission and the ISR placement/reference audit pass. Existing actual scheduler/observer and host-record formatter suites pass ASan/UBSan; runtime-only configuration passes5 CMake/13 image/2 CLI cases, plus existing runtime/scheduler/cycle compatibility gates. SMP adapter lifecycle, real SDK 6.1 PSA/NVS serialization and feature-exchange host suites pass. An initial stale NVS-call-count fixture and ASan DLL startup failure are retained with their corrected outcomes. A deployment Python import failure occurred before hardware access and was retried using the IDF environment.

Evidence: native_bt/_build/experiments/cpu1-runtime-1794178979038 (runtime/compression/test receipts); experiments/reconnect-worker-core0-1794183964080 (SMP tests and footprint); hardware/runs/1794184452831027300-b8cc7d67eeb0/traces/reconnect-core0 (diagnostic trials); hardware/runs/1794185326684129200-7d131bf9e47d/traces/reconnect-core0-normal (normal trials). Raw traces and failed receipts remain. Derived trace-summary-v2/trace-summaries-v2 select captured=true first-gap records and preserve pause bounds; earlier derived summaries retained an initial uncaptured placeholder and are superseded only for that field.

CPU1 starvation/earlier active PCM gaps, occasional delayed BLE persistence/control completion, sustained maximum-rate continuity, 1 Mbps, acoustics, all-codec live regression, security/lifecycle/product/OTA parity and equivalent Bluedroid RAM/CPU comparison remain open. These finite checks do not qualify a stable release. Main later received development source at the user's explicit request; see the [reconnect stack follow-up](RECONNECT_STACK_2026_11_08.md).
