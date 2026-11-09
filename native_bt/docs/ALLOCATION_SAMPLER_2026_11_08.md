# Optional allocation sampler — 2026-11-08

The transient free-heap minimum in the shared-persistence image remains unexplained. This change adds an opt-in diagnostic, not a claimed RAM/performance fix. Normal builds leave it disabled. No audio-gain change is included.

## Capture ownership

With `-HeapAllocTrace`, the first LE connection opened while I2S is running arms one capture per boot. There are 48 fixed 32-byte slots. Allocations of at least 2048 requested bytes are sampled, along with all allocation sizes while the host owner creates an SMP, persistence or general worker. The creator tag is checked against the current task and ISR state, so an interrupting allocation is not silently called an owner allocation.

Allocation/free callbacks use one strong CAS attempt to reserve a slot from an atomic admission word. There is no retry loop, allocator call, UART, flash operation, kernel lock or interrupt-masked scan in the hooks. Free callbacks perform a bounded scan of at most 48 records to filter previously sampled addresses. Collision/overflow flags are diagnostic observations, not an exact count or complete allocator coverage; the report explicitly says exhaustive=false.

The logger closes admission by clearing the open bit in that same word. It checks each reserved row's publication state once and returns if a writer has not finished. Rows are immutable after publication. There is no reset/rearm or slot reuse. Static zero initialization supplies unused fields. Capture ends two seconds after the matching LE link disappears, on a 30-second timeout, or on overflow when the logger next polls. UART reports are emitted only after I2S stops and capture freezes.

SDK hooks run after their allocator operations. A pointer can be reused on another core before a free callback arrives. Therefore `heap_alloc_observation` and `heap_free_observation` remain separate; the sampler does not invent allocation-generation/lifetime pairings. Callback timestamps are observations, not exact allocation/free instants or CPU costs.

Background CCCD jobs record 8-bit internal-heap free/minimum values around storage calls, matching the existing heap counters' capability mask. Worker retirement markers are taken before the caller invokes vTaskDelete and include stack low-water readings. Retirement is not presented as actual allocator reclamation. Live/sampled task IDs and creator tags help identify records without dereferencing a deleted task handle. Keys and storage payloads are never logged.

## Checks and scope

The actual hook state machine and exact deferred JSON formatter pass five Windows model cases under ASan/UBSan: normal/reused addresses, overflow/ISR context, a writer active during closure, concurrent writers, and delayed cross-core free callbacks. These tests do not establish MCU timing or exhaustive allocator capture.

Mode admission passes 12 CMake cases, 19 image cases, 8 SDK checks, 3 normal/diagnostic SDK transitions and 2 CLI rejections. The full diagnostic image c0489ffb46e22d6f86a221e6d23547326a8edb2c9287756f833dbbb992df6050 built with ccache and 10,429 unchanged source inputs. Image admission and the owned hook/ISR code/literal/data placement audit pass. Owned sampler IRAM object code was reduced from 849 to 465 bytes to fit the existing region; codec/DSP settings and buffers were not reduced.

Compared with normal image fb87d978320c, the diagnostic linked sizes increase 480 B IRAM, 1608 B DRAM data and 8 B BSS. These are temporary profiling costs; hardware heap results from such an image must not be reported as normal-build minimum RAM. The default repository build excludes the module and restores CONFIG_HEAP_USE_HOOKS to disabled.

Earlier annotation, native typedef, IRAM-capacity and rebuildable-disk-gate failures are retained with their receipts. Completed compiler/configuration logs and test binaries were compressed losslessly with raw SHA256 verification; the 512 MiB gate was not raised. Current SDK objects/ELF/headers were preserved.

A diagnostic deployment attempt stopped at the phone pause precheck with empty writes/write_attempts. No diagnostic image was flashed and no allocation hardware trace has yet been collected. The phone transport later recovered. The existing owned-host firmware remains the device observation scope; full 1 Mbps, sustained continuity/coexistence, acoustic and feature/security/OTA qualification remain open.

Evidence: native_bt/_build/experiments/reconnect-allocation-trace-1794188270925. This directory retains source-bound host-test receipts, target-size probes, failed/successful build logs, footprint and failed precheck evidence. Rebuildable/device evidence is not part of the Git source publication.
