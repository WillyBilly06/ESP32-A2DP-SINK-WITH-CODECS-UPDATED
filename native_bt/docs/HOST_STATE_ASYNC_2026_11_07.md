# Deferred visibility and GAP-state logging — 2026-11-07

Source/off-board change only. The full host/audio goal remains active and incomplete. COM6 still has no live UART and awaits physical recovery; this image is not flashed or runtime-qualified. The last verified flash contents remain b309817ce3d6 from the preceding full-output experiment. This change is not claimed to explain or fix the UART outage.

## Two live owner paths

The product host previously called HOST_PRINTF/vprintf directly for classic_visibility and gap_state transitions. Those calls can wait on stdio/UART serialization behind background output. Once the audio logger is initialized, the owner now copies the two typed state snapshots into its existing diagnostic trace ring. Formatting, newline/CRC handling and UART output occur on the existing background logger. No protocol reply waits for the diagnostic consumer.

Before logger initialization, the original startup output is preserved. The product call sites fall back only for DBT_UNSUPPORTED/pre-init; they never call UART on a full initialized queue. The real main-loop call sites, producer and logger spans are included in the host regression. Non-audio diagnostic builds retain their prior direct output. Owner identity and fixed-lifetime logger availability are checked before publication.

The host ring retains one producer: the host task publishes both media observations and state snapshots. The worker ring retains its separate decoder producer. Records contain copied scalar values, never pointers to mutable GAP state or borrowed packet/PCM storage. The internal diagnostic discriminator is separate from A2DP codec/wire flags. Media payload prefixes, sequence/timestamp/SSRC, codec IDs and timing-event routing are preserved.

## Bounded storage and behavior

The union reuses the existing32-byte event payload. Compile-time checks retain44-byte slots,16 slots and720 bytes per ring; two rings still occupy1440 bytes. There is no new task, queue allocation, packet/PCM buffer or stack allocation request. Read/write publication uses the existing SPSC acquire/release ownership. The consumer copies before releasing a slot and performs UART work afterward.

A full ring increments its existing diagnostic-loss counter and returns immediately. It performs no allocation, UART call, mutex acquisition, clock callback or retry loop. This can lose diagnostics under saturation; functional protocol/security state and replies are not dropped by this path. host_trace_lost now includes these state records, and host_trace_scope explicitly says media_and_host_state. Ordinals count attempted ring records, including the new state snapshots; they are not RTP sequence numbers.

Existing JSON keys/values remain. Deferred rows add event_us (publication time), host_log_ordinal and the existing logger CRC/snapshot metadata. Each accepted transition stays a separate record, with no latest-value coalescing. The logger keeps its existing bounded drain cadence. Timing reflects publication separately from later formatting/output.

## Verification

Actual queue code plus exact producer, routing hook, logger and main-call-site source spans pass AddressSanitizer/UndefinedBehaviorSanitizer without signed-overflow wrapping. Cases cover early/unavailable and wrong-owner rejection, startup fallback, mixed media/state records, full queues, no clock calls on rejection, read/write/ordinal uint32 wrap, immutable caller snapshots, preserved LHDC input-clock routing, worker/timing routes and original JSON fields. A deterministic two-thread test holds the actual logger output after slot release: the producer fills all16 slots, returns/counts the seventeenth rejection and does not fall back to UART.

An initial fixture omitted the real type aliases; that was corrected. A separate initial run loaded an incompatible sanitizer DLL; the runner now binds the compiler-matching runtime beside the test executable and records its hash. The failed runtime receipt is preserved. These are host correctness/liveness checks, not ESP32 timing, radio, acoustics or full feature qualification. No new boot selftests are added or run on hardware.

The complete ccache product build exits successfully after the compiler-output budget is restored by removing only hashed, aged PDB intermediates. Initial budget failures and final successful logs remain separate. Exact image/source/layout admission passes for unflashed image `c2ed33307c2cd47e12d5ad86970279da0f76262ff627b573eac7fe67c3565a92`, with all codec/product/security/RFCOMM flags, helper6 and TaskCycleAccounting OFF. The guarded deployment readiness substring remains unchanged.

Relative to the last b309 image, IRAM vectors/text, DRAM data/BSS are unchanged. Flash text increases532 bytes and flash rodata368 bytes. No active-heap or stack high-water observation exists for this unflashed build. Target disassembly shows both converted main call sites using the producer; that producer calls task-identity/queue helpers, and the queue object imports only bounded memcpy/memset, not UART/stdio/allocation/locking functions. Its clock hook is the existing bounded timer read.

## Remaining work

This step converts two normal live-state paths. Startup and terminal failure/parser diagnostics, other module writers and overall async coverage remain review items. The background UART driver itself may sleep, and no claim is made that every logging path is nonblocking. The observed UART/ROM silence is unresolved; no new reset/flash/phone playback is attempted while it persists.

After real COM6 recovery, verify the owned boot/source and finish the preceding b309 LDAC/codec-switch gate before deploying this candidate. Retain the corrected playback precheck/independent pause guard, COM6-only/921600 write rules, original partitions and no automatic Bluedroid fallback. Sustained LHDC192/24/1000kbps, loss-free deadlines, coexistence/control/storage/OTA/security/acoustic/lifecycle/stress and all-feature parity remain open.

Exact source backups, host results, failed/final build logs, source/image checks, target call evidence and budget-cleanup hashes: `native_bt/_build/experiments/host-state-async-1794051103761`. [Structured results](HOST_STATE_ASYNC_RESULTS_2026_11_07.json) retain scope.
