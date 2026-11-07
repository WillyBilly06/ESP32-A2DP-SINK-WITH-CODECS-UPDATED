# Deferred terminal failure diagnostics — 2026-11-07

Source/off-board change only; not flashed or runtime-qualified. The full host/audio goal remains active and incomplete. COM6 still has no live UART and awaits physical recovery. Its last verified flash contents remain b309817ce3d6; this change is not claimed to explain the UART/ROM outage.

## Preserve both failure records without an owner UART wait

The live host previously called HOST_PRINTF/vprintf for failed and parser_failure. Once the audio logger is initialized, the owner now makes one96-byte immutable snapshot containing all24 original scalar fields. It publishes that snapshot atomically through three existing trace slots. The existing background logger still emits two separate JSON rows, with every original key/value plus event_us and a shared host_log_ordinal. No mutable runtime pointer is retained.

The first five fields retain fault/controller/command/RX diagnostics; the remaining nineteen retain parser dimensions, handle/generation, link presence, packet/event/control identifiers and host fault-stage dimensions. Protocol state changes, errors, replies and teardown work are unchanged. Diagnostics cannot block those operations on UART through this path.

Before logger initialization, the original startup output is preserved. Non-audio diagnostic builds also retain their prior output. Fallback is only for unavailable/pre-init logging; initialized queue saturation never calls UART. Other startup/owner printf sites remain review items, so this is not a claim that every logging path is asynchronous.

## Atomic bounded ownership

DBT_OK confirms a private queued copy, not UART delivery. Caller storage may be released immediately after the call. DBT_NO_SPACE accepts nothing and counts a diagnostic loss.

One producer reserves three slots only if all fit. A rejected group changes no payload/cursor, performs no clock call, and counts one lost logical diagnostic snapshot. It neither retries nor waits. Normal state/media records continue to use one slot; worker/host producer separation is unchanged.

The producer copies all three payloads, gives them matching ordinal/time metadata and then release-publishes one write-cursor update. The consumer acquire-checks availability and continuation metadata, copies the complete snapshot locally, and releases all three slots before either UART write. Incomplete/mismatched private groups produce an explicit corrupt marker for the background formatting-loss count; recovery consumes a bounded slot without reading unpublished data. Reserved discriminators are internal diagnostics, not A2DP codec or wire flags.

Compile-time layout/capacity checks keep slots44 bytes and each16-slot ring720 bytes; two rings still occupy1440 bytes. No new task, queue, packet/PCM buffer or heap allocation is introduced. A failure is one logical queued event that produces the two existing output rows; host_trace_lost counts a rejected group once, not once per slot or output row.

The consumer-local record grows44 ->108 bytes (64 more bytes of local object space). Allocated task-stack sizes are unchanged. The actual app_main compiler frame remains0x3f0/1008 bytes in both the preceding and new images; the current log_loop compiler frame is0x330/816. These are prologue/object observations, not complete call-stack or live high-water measurements. Live heap/stack gates remain open.

## Verification

AddressSanitizer/UndefinedBehaviorSanitizer tests without signed-overflow wrapping compile the actual queue and exact main/producer/routing/logger source spans. Independent JSON expectations verify every one of the24 fields in both original row schemas, including high uint32 values. Caller/runtime storage is changed after enqueue to prove snapshot ownership. Existing mixed media/state, payload-prefix, timing-kind, context, startup and full-queue cases still pass.

New cases cover every occupancy0..16 at wrapped physical/uint32 cursor positions, full three-slot rejection with no clock/partial write, malformed/incomplete continuation groups, and a producer deliberately held after its first slot. The consumer sees an empty queue until final publication. That hold hook is test-only and absent from the linked firmware.

A separate blocked-output case pauses the actual logger on its first failure row, refills/reuses all16 original queue slots, and then validates all19 fields of the second parser row. The producer returns immediately/counts a new full-group rejection without direct-print fallback. This verifies the copy survives slot reuse across a UART wait, not merely that an enqueue call succeeds. These are host liveness/data tests, not MCU timing or RF qualification.

The complete product builds with ccache and all codec/product/security/RFCOMM options, helper6, current affinities/priorities and TaskCycleAccounting OFF. Exact source/image/layout admission and output-budget checks pass for unflashed image `2ade0c873b73e935a65705bbbfc06d78f648fc6990613ddfbc718c9c75575016`. No new boot selftests are added or run on the board.

Relative to prior source-only c2ed33307c2c, linked vectors, IRAM text, DRAM data and BSS are unchanged. Flash text grows716 bytes and rodata496. Target call evidence shows the failure publisher call plus the prior two state publishers; queue imports remain memcpy/memset, with no UART/stdio/allocation/locking/test-hook dependency. The actual timer hook is the existing bounded read. No live heap or stack claim follows from unchanged static sections.

## Remaining qualification

No new UART reset, flash or phone playback is attempted while COM6 is silent. After real recovery, verify the prior owned b309 boot/source and finish its LDAC/codec-switch gate before deploying newer source. Keep COM6-only access, existing guarded write921600/read115200, original partitions and no automatic Bluedroid fallback.

This change removes initialized-host UART waits from terminal failure reporting. Background driver output can still sleep, and initialization/miscellaneous writer coverage remains incomplete. Sustained loss-free LHDC192k/24-bit/1000kbps, coexistence/control/storage/OTA/security/acoustic/lifecycle/stress and full feature parity remain open.

Exact source backups, tests, independent JSON expectations, compiled call/stack evidence, successful build/admission and footprint receipts: `native_bt/_build/experiments/host-failure-async-1794053901632`. [Structured results](HOST_FAILURE_ASYNC_RESULTS_2026_11_07.json) retain scope.
