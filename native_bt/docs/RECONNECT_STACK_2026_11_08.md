# Reconnect stack budget — 2026-11-08

The optional allocation capture identified overlapping 12 KiB SMP and 4 KiB persistence-worker stack allocations during LHDC playback and BLE reconnect. The SMP stack retained 8,836 bytes at retirement. Only reconnect now requests an 8 KiB stack; fresh pairing and storage maintenance retain 12 KiB. No task, audio buffer, codec, security policy or gain setting was removed or changed.

Reconnect owns a separate procedure that dispatches bond load, RPA resolution, identity promotion and retirement. It does not dispatch fresh-pairing ECDH. A fresh pairing request while that slot is active returns busy rather than promoting the smaller worker into the crypto path. Lifecycle fixtures check this boundary and verify the stack budget on every simulated worker creation, including fresh pairing and maintenance. The real SDK 6.1 PSA/NVS-shim reconnect and feature-exchange suites pass; compilation uses ccache.

## Measured diagnostic comparison

Both readback-verified images used the same optional sampler and native YouTube music fixture on COM6/Redmagic, LHDC V5 / 192 kHz / 24-bit stereo, phone-selected fixed 900 kbps. Each excerpt had an independent 14-second pause guard and less than 15 seconds actual I2S duration.

| Measurement | 12 KiB reconnect | 8 KiB reconnect |
|---|---:|---:|
| Image prefix | e9247f2f002c | 2ec452c36f59 |
| I2S duration upper bound |12.603 s|12.621 s|
| SMP stack low-water bytes |8836|4736|
| Minimum 8-bit internal free heap |10480 B|14596 B|
| Initial refill shortage / recovered frames |0 /0|64 /64|
| Unfilled frames |0|0|
| EQ ACK, independent readback, original CCCD restoration before pause |Yes|Yes|

Both excerpts recorded zero decoder faults, media drops, descriptor errors and watchdog events. The smaller-stack legacy underrun counter is 64 because it counts the initial shortage; all 64 frames were recovered before refill closure. It is not reported as zero initial shortages. These finite digital counters do not establish acoustic continuity.

The requested allocation is 4096 bytes smaller. The observed two-run minimum improved by 4116 bytes, with scheduling and allocator-state differences; it is not an exact isolated heap-saving measurement. The sampler adds 1616 bytes of linked static data/BSS and is excluded from normal builds. Do not subtract that footprint from diagnostic minima and call the result a measured normal minimum.

The baseline persistence-stack allocation callback preceded the SMP pre-delete marker by 13726 us. The SMP task had not invoked vTaskDelete at that marker, establishing overlap in the instrumented run. Heap/HWM reads precede the marker timestamp and can delay retirement. Free hooks execute after allocator work and can observe reused addresses, so no allocation/free lifetime pairing is inferred. The bounded capture is explicitly nonexhaustive.

## Normal build and remaining scope

Normal image f3890656391e004943d5727887f8adeee03d3fd0d1bf3dc34af13567f6487811 builds with 10433 unchanged source inputs and 1036 explicit object-only compilation commands through ccache. Image admission and owned ISR-reference checks pass. Bluedroid/NimBLE hosts, allocation hooks and sampler code are absent; the Espressif BTDM controller is retained.

Normal image f3890656391e was flashed at 921600 and fully read back at 115200. The old application MD5 remained unchanged; ota1 is explicitly VALID and automatic Bluedroid fallback remains disabled. Existing cold checks pass 37 configurations /173 callbacks, tracked peak 56899 B, minimum free after allocation 106128 B and stack low-water 3468 B, with no boot watchdog/backtrace/idle-starvation marker.

Its 12.526 s guarded LHDC excerpt completed EQ ACK/readback and original CCCD restoration before pause. Minimum free internal heap was 20304 B, stopped free 62084 B and largest block 59392 B. This normal minimum is directly measured; different scheduling/overlap means it is not an isolated 8468 B improvement over the earlier shared-persistence checkpoint.

The normal excerpt retains 16064 initially short frames, 6272 recovered and **9792 unfilled**. Decoder/media drops, decoder faults, descriptor errors and watchdog events were zero; host trace loss was 1, while worker/snapshot/UART losses were zero. The first unfilled snapshot had empty input/output queues, 10.674912 s after I2S enable. Pause was requested 9.566488 s after the phone play command. These are separate MCU/PC intervals; no synchronized mapping or blanket stop-tail classification is claimed here, and the excerpt is not reported as underrun-free.

BDK Audio was restored to the foreground, native YouTube was verified paused, and the monitor resumed at 921600. The normal image has no heap-capture records.

The 4736-byte stack reading is a finite reconnect observation, not a worst-case stack proof across every NVS state. Full 1 Mbps headroom, sustained coexistence, earlier active PCM/CPU1 failures, all live codec/security/lifecycle/OTA/product parity, acoustic checks and a matched Bluedroid RAM/CPU comparison remain open. Main contains development source at the user's request; no stable custom-host release is claimed.

Machine-readable evidence: [results](RECONNECT_STACK_RESULTS_2026_11_08.json). Raw source/image-bound traces and failed receipts remain under native_bt/_build/hardware/runs/1794199943157604400-e9247f2f002c and runs/1794200654550261500-2ec452c36f59, with host/build receipts in experiments/reconnect-allocation-trace-1794188270925.
