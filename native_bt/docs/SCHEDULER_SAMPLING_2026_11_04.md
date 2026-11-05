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
it. Tick gaps do not by themselves distinguish interrupt masking, another ISR,
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

Live role counts, callback overhead, interrupt gaps, maximum-LHDC behavior and
baseline return remain pending. No audio continuity or feature-parity pass is
claimed.
