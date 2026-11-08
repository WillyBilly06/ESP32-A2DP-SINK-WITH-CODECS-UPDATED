# BLE control during LHDC: separate lifecycle diagnostics

Date: 2026-11-08. Development firmware; the full host-stack goal remains active.

## Current candidate and scope

COM6 image **c07781e066abd4be9a974ca6c148e58fa9e32facf42b998cc255e21f55dd9829** is written at 921600 baud and matches complete readback at 115200. The original application, bootloader and partition table remain preserved; automatic Bluedroid return is disabled. The raw-UART viewer is reconnected at 921600 with the approved USB identity. Archive: runs/1794175964343216000-c07781e066ab.

The existing cold check passes 37 codec configurations / 173 callbacks. Tracked peak is 56,899 B, allocation minimum free heap 106,144 B, decoder stack low-water 3,468 / 8,192 B. No boot watchdog, idle-starvation, interrupt-watchdog or backtrace markers were found. These are cold checks, not live all-codec qualification.

## Bounded logging change

CCCD jobs now have individual immutable records for enqueue, owner completion, rejection and binding. SMP snapshots use the existing host audio-trace ring in normal owned playback, keeping them separate from the connection-parameter records which can fill the legacy security diagnostic queue. An unsupported startup/foreign context retains the original logger path; a full host ring counts a host trace loss without a second enqueue or any protocol wait.

The owner copies seven values into an existing 32-byte payload / 44-byte slot. The existing background audio logger formats, checksums and transmits them. There is no new queue, task, allocation, persistent buffer, UART write or wait on the producing path. No key, nonce, random or packet payload is recorded. Linked DRAM data/BSS and IRAM are unchanged versus image57850f2d36b9; flash text grows460 B and rodata320 B. Existing codec queues, 80 ms high-rate startup hold, priorities, affinities, precision and DSP are unchanged.

CCCD stages: 1 accepted persistence job; 2 owner harvest of its result; 3 binding/readiness rejection; 4 identity/incarnation mismatch; 5 stale bonded security snapshot; 6 successful ATT binding. Job kinds: 1 load, 2 client write save, 3 pre-binding merge save. Phases: 0 none, 1 loading, 2 merge, 3 saving, 4 ready, 5 bound, 6 error; UINT32_MAX means unavailable. Late completion records retain the original job handle/generation.

New SMP records use event_us for producer time; at_us is the checksummed background collection time. Startup legacy records retain their original at_us field. trace_records_lost is the older protocol queue counter, while host_trace_lost separately covers the reused host ring. Neither is an audio packet-drop counter.

## Measurements and remaining faults

A BLE-only normal Android public-SDK helper uses only the approved Redmagic9125299103B7 and speaker6C:C8:40:42:BF:66. It does not generate audio. Playback is native YouTube, LHDC V5 /192kHz /24-bit stereo /fixed900kbps, with an independent14-second phone pause guard. The phone menu offers900kbps, not1Mbps. This helper is not a complete BDK Audio app UI test.

An earlier explicit connection-only excerpt kept Classic playing before and after BLE close and recorded0 final unfilled frames. A paused settings roundtrip on intermediate imagecb823b26b1b2 passes MTU517, four services, settings read, unchanged-EQ write, application ACK, independent readback and restoration of the original[0,0] CCCD. Its bond-load enqueue-to-owner-completion interval is7,127us; saves take14,156/13,701us. These intervals include scheduling and are not NVS CPU time.

Two intermediate-image playback attempts instead return Android GATT write133 /disconnect5 after about2seconds. Each has0 finalized unfilled frames and Classic remains connected. The legacy security queue loses records, so missing SMP events cannot establish that the events never reached the handler.

The new image's guarded metronome excerpt has a13,178ms I2S upper bound. MTU517, service discovery and public settings reads work; the subscription write returns ATT0x0E, so the EQ command is never sent. Checksummed records identify the exact race:

- LTK request accepted at193777054us.
- RPA resolve completion observed at195496049us:1,718,995us later, including queueing, work and preemption.
- CCCD load accepted at195551391us.
- Client write rejected in phaseLOADING at195609431us,58,040us after load enqueue.
- Load completes successfully at195644511us,35,080us after rejection.
- ATT binding completes at195645151us.

The owner currently rejects a valid subscription write that arrives before asynchronous bond restoration finishes. A replacement must defer it until the restored values are merged and durably saved, retain generation/token/incarnation checks, and preserve other subscriptions. Increasing a timeout or acknowledging before persistence would not address this race. That semantic fix is not included in this diagnostic checkpoint.

This excerpt also retains **33,024 finalized unfilled PCM frames**, with the first gap bounded5.480–5.863seconds before the phone pause request. At that first gap, raw/ready PCM are empty and the decoder is running a newly admitted encoded packet. There are no queue-full/decode/reset/PCM timeout, input sequence, descriptor, refill-contract, host/worker trace or snapshot/UART loss counters. Accepted/rendered/DMA-source totals match.162 UART records are CRC Verified,14 are legacy Unverified,0 Invalid. All four new SMP records are Verified. Do not infer the audio gap's cause from the slow security interval alone. The latest stopped DSP signal snapshot is invalid and does not qualify live EQ quality.

This diagnostic checkpoint narrows the control failure without fixing coexistence or LHDC continuity. Full1Mbps, sustained audio/control, acoustics, all-codec live regression, security/lifecycle/product/OTA parity and comparative Bluedroid RAM/CPU qualification remain open.

## Validation and artifacts

The actual bounded trace/logger tests pass ASan/UBSan in normal and detailed modes, including immutable CCCD fields, handle0, signed SMP results, maximum-width values, actual JSON formatting and the current accepted/rejected A2DP submission wrapper. The ATT/product/NVS-shim tests pass after updating the stale pinned-task shim and paging dependency. Their compiler steps now use ccache with explicit-c objects and a separate link. These are host tests, not RF evidence.

The final ccache firmware build passes source stability for10,424 inputs, image admission, the ISR audit and the unchanged512MiB rebuildable-output gate. The first SMP build compiled but failed the disk gate; completed test symbols and logs were losslessly compressed and hash-verified before the successful retry. Failed hardware/helper receipts are retained.

Artifacts: native_bt/_build/experiments/lhdc-ble-overlap-1794172238472, particularly traces/ble-paused-cccd-stage, traces/ble-cccd-stage-trace, traces/ble-cccd-stage-warm, traces/ble-smp-visible, smp-trace-memory.json, current-diagnosis.json, helper source/APK/readback receipts under service-v3, and the source-bound deployment archive above. CLI-before-main and Android AutoLaunch setup failures are separate from firmware results; the working helper is launched through an explicit activity. Its restoration-phase race and overlapping fallback write were corrected before the final measurements.
