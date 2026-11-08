# Deferred CCCD writes across bond restoration

Date: 2026-11-08. Development checkpoint; the complete host-stack goal remains active.

## Change

A valid bonded client's first subscription write no longer returns ATT0x0E merely because asynchronous restoration is unfinished. The owner retains one nonzero ATT token per binding and copies the requested fields into the existing touched-value storage. It retains no batch/PDU pointer. Restoration merges those fields over the saved values, preserving other subscriptions. The existing background worker saves the merged values before a successful ATT response; only then is ATT's restored bonded state attached.

The path handles writes before the first sync poll and during NONE, LOADING, MERGE, SAVING and READY. If a write arrives after a save has been copied, the completion merges it and schedules another save before success. Existing bound writes keep their original save path. Request commands, mixed unsupported batches, unencrypted links, failed bindings and mismatched bond identities/incarnations retain error handling.

Deadline, disconnect, generation, token and security changes cannot complete a different request or fall back to ephemeral success. A persistence operation already submitted can still finish after its client disappears; its completion cannot acknowledge a reused handle. Timed-out ATT state is left to the core timeout handler. No watchdog policy, priority, affinity, codec, precision, startup hold or audio buffer is changed.

Additional linked BSS is16B: one32-bit pending token across four bindings. DRAM data and IRAM are unchanged. Flash text grows748B; rodata is unchanged. There is no new task, queue or audio buffer.

New CCCD trace stages:7 records deferred admission;8 records completion. Earlier stage and phase definitions remain in [the diagnostic report](BLE_LHDC_CONTROL_TRACE_2026_11_08.md).

## Source and target checks

The actual ATT/product/NVS-shim suite passes seven arrival/procedure cases and nine failure/retry cases, in addition to the existing suite. These cover missing binding, load/merge/save/ready ordering, unavailable allocation, prepared writes, preservation of other CCCDs, load/save errors, deadline expiry, token mismatch, reused generation, bond replacement/revocation and resource retries. Assertions check response bytes, stored values, outstanding jobs and no early ACK. These tests are offline shim evidence; the new state-machine cases were not run under a sanitizer in this checkpoint.

The first test attempt reached an older fixture that expected the readiness rejection. It now tests a genuine security-negative case (a claimed bond on an unencrypted link). A progress-output header omission failed compilation in the second attempt. Both failed receipts/logs are retained; the third run passes with ccache and explicit-c object builds.

The ccache firmware build passes source stability for10,424 inputs, offline image admission, the ISR audit and the unchanged512MiB rebuildable-output gate. Closed test binaries and the previous external build log were losslessly compressed with raw hashes and manifests; source/current SDK/firmware inputs were preserved.

COM6 image **861ec6f23c4a32ded5f074e86ca358fe081f2e8c3dd5f25cb86a7c8d31a611be** is written at921600 baud and matches full readback at115200. The bootloader, table and original application remain preserved; automatic Bluedroid return is disabled. Archive: runs/1794177520139611200-861ec6f23c4a. The viewer is reconnected at921600 with the approved USB identity.

The existing cold check passes37 configurations /173 callbacks, tracked peak56,899B, minimum free after allocations106,128B, decoder stack low-water3,600/8,192B. Cold boot has no watchdog, idle-starvation, interrupt-watchdog or backtrace markers. This is not a live all-codec result.

## Guarded live control result

The approved Redmagic9125299103B7 uses native YouTube with LHDC V5 /192kHz /24-bit stereo /fixed900kbps. The independent14-second pause guard completes; measured I2S upper bound is13,201ms. The helper uses public Android BLE APIs and generates no audio. This is an unchanged-EQ roundtrip, not a full BDK Audio UI or acoustic EQ-transition test.

The phone completes MTU517, service discovery, settings read, status subscription, unchanged-EQ write, application ACK, independent settings readback and restoration of the original CCCD. Read-only helper-file observations bound all three success callbacks before the PC pause request: ACK/readback by5,185ms and CCCD restoration by3,505ms. This does not require aligning phone and MCU clocks.

Checksummed device records show:

- CCCD load enqueued at306872840us.
- Client token1 accepted as deferred in phaseLOADING at306932864us.
- Load completion at307017890us.
- Merged save enqueued at307018617us and completed at307388040us.
- Token1 completed successfully at307389254us; ATT binds at307389628us.
- The original CCCD is restored through the normal bound-save path.

The subscription now survives the exact readiness window that previously rejected it. Its deferred-admission-to-completion interval is456,390us; that wall interval includes queueing, storage and preemption, not measured NVS CPU time. Classic remains connected and playing at the helper's active sample. This finite run does not prove repeated or sustained coexistence.

## Playback is not clean

The run records0 initial/recovered/finalized unfilled PCM frames, with accepted/rendered/DMA-source totals matching. However it also records **one DMA descriptor-order fault/recovery and one CPU1 idle-starvation watchdog event**. These invalidate a clean playback/continuity claim. The recovery accounts for at least one missed EOF and192 source frames; additional complete DMA laps remain unknown.

The descriptor fault uses esp_timer time310259498us. The watchdog user hook uses the same clock at310259511us,13us later, on Core1. UART's printed alarm timestamp is configured to use the RTOS clock. A previous commentary comparison using that printed timestamp was invalid; the hook is the correct comparison. The raw alarm identifies IDLE1 starvation, with main on Core1 and native_render0 on Core0. The near-coincident fault and hook locate the interrupt-handling window; they do not quantify which task consumed the preceding CPU budget.

The watchdog output interrupts one UART record. The capture has175 CRC-Verified rows,23 legacy Unverified rows and1 Malformed row containing the alarm. That malformed raw line is retained. The diagnostic ring's host/worker/format/UART loss counters are0; those counters do not make the interrupted record valid.

Other limits: one descriptor order fault; maximum EOF gap2,179us; first RX-to-poll maximum14,337us; host-poll maximum11,398us. Queue-full/decode/reset/PCM timeout and input-sequence counters are0. Runtime minimum free heap is7,600B; stopped free heap62,080B. No valid live DSP signal snapshot was delivered, so EQ signal quality is not qualified.

The first guarded control transaction passes; overall playback validation fails. The planned music repeat is held while CPU1 service/idle budgets and watchdog-induced diagnostic/interrupt interference are investigated. Do not disable the watchdog, suppress the fault, infer loss-free audio from0 PCM shortage alone, or attribute the fault to a specific codec calculation without measurements.

Full1Mbps, sustained coexistence/continuity, live all-codec regression, acoustics, security/lifecycle/product/OTA parity and matched-baseline RAM/CPU qualification remain open.

Artifacts: native_bt/_build/experiments/cccd-deferred-write-1794176919134, especially unit-1/2/3 logs and receipts, memory.json, phone-format-before.json, traces/ble-deferred-live-1.json/.ndjson, the independent-client observer receipt, and the source-bound deployment archive above.
