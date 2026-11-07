# Host query and poll cost attribution — 2026-11-07

The diagnostic image adds stream-scoped CCOUNT measurements to each of the15 actual host wait queries and the existing host-call categories. It changes no codec math, DSP, queue, priority or protocol policy. This is instrumentation, not a measured performance improvement.

Exact deployed image: f8d6d5c2b60abe20d7c07588330ca1eb6801d559d85dd2e0038f75db9e29e87b. Guarded COM6 upload921600, full byte readbacks115200, own-host VALID selection, preserved original app and no automatic Bluedroid fallback are verified in the deployment receipt. Existing37 codec cases/173 callbacks pass; cold stack low3520/8192, tracked peak56899, minimum free after allocation102516.

The implementation adds480 bytes linked internal data, zero BSS/IRAM delta,1508 flash-text bytes and736 flash-rodata bytes versus exact9283. The counter cells are464 bytes plus four32-bit publication/epoch words. A small logger dedup word fits existing section alignment. No task, heap buffer, ISR instrumentation, retry loop or private kernel API is added. Trace-off compilation removes the implementation and marker dependency.

The host is the only statistics writer. A single stream lifecycle publisher changes the capture epoch; markers crossing start/stop are discarded. Per-row snapshots use one sequence check and leave output unchanged on failure. Reports are attempted after pause; incoherent used slots defer completion to a later background cycle. Query expressions evaluate once. Nested host categories are inclusive. All timings include preemption, interrupts, marker floor and possible cache delays. They are not pure CPU instruction costs and overlapping categories must not be added. Totals/counts wrap uint32.

Two guarded native YouTube120BPM traces on Redmagic9125299103B7 used verified LHDC192k/S24/stereo/fixed900, volume90/150 and independent14-second pause guards. The received configuration and566-byte media units match the prior899200 coded-bit/s workload; this is not a1Mbps qualification.

| Measurement | Trace1 | Trace2 |
|---|---:|---:|
| Host deadline collections |2337|2317|
| Deadline collection mean,us |775.516|772.23|
| Runtime poll mean,us |1346.637|1322.33|
| AVRCP wait mean,us |127.638|129.12|
| Runtime wait mean,us |131.255|126.96|
| Audio wait mean,us |92.889|90.09|
| Controller availability query mean,us |6.891|5.90|
| Controller submit mean,us |791.773|848.79|
| Empty marker mean/min,us |0.481/0.125|0.56/0.125|
| DMA source frames |2719232|2717760|
| Unfilled refill frames |17920|10112|

Submission can preempt into the higher-priority controller; these wall timings do not demonstrate a blocking loop in the SDK. The availability wrapper minimum is0.408us; no reason has been established to bypass its documented check. Captures still fail uninterrupted LHDC continuity.

Source review after measurement identifies repeated AVRCP transport deadline scans: the AVRCP bearer calls AVCTP bearer wait, then AVRCP profile wait, which calls AVCTP wait again. The pure wait functions also scan all16 label positions even with empty masks. Main deadline collection separately invokes the signed64-bit millisecond conversion for most queries; linked disassembly confirms the ROM __divdi3 call. These are candidates, not yet proven optimizations. Preserve all protocol deadlines/security semantics and measure any change against this profiling image.

The controlled-clock test executes the actual extracted C implementation/macros with UBSan, covering context/capture gating, once-only evaluation, nested inclusive calls, counter/clock/epoch wrap, extrema, used-slot masks, busy/changed sequence failures, lifecycle rejection and trace-off preprocessing. Its initial isolated off-macro redefinition failure is preserved; the harness now explicitly switches from the header stub to the actual main stub. No new boot cases were added.

Evidence: native_bt/_build/experiments/owner-query-cost-1794079703383. Physical archive: native_bt/_build/hardware/runs/1794080115221795300-f8d6d5c2b60a. Host results, source snapshots, failed receipt, exact build/admission/deploy logs, section sizes, linked symbols, guarded playback helper, raw NDJSON and full frozen cost snapshots are retained.

The full master scope remains open: maximum1Mbps, continuous loss-free playback, encrypted BLE/control coexistence, EQ acoustics, codec/product/security/OTA/lifecycle/stress parity and RAM comparison are not proved by this diagnostic stage.
