# BLE/Classic coexistence investigation — 2026-11-06

Status: the reproduced prompt/re-pairing defects are patched and exercised in focused hardware traces, including durable LE re-pairing, stored-key reconnect outside consent and acknowledged EQ changes during LDAC. Full encrypted BLE/app/audio coexistence across all features/codecs remains unqualified.

## Reference and controller configuration

Inspected the actual BTstack checkout at commit `9edb5d82ea6ec4466a393415d65030929330acdf`, particularly `port/esp32/components/btstack/hci_transport_esp32_vhci.c`, `src/hci.c`, `src/ble/sm.c`, and `example/a2dp_sink_demo.c`. This is selected-section review, not a completed whole-file/whole-stack audit. No BTstack library or source file was integrated. Equivalent protocol ownership/state rules are implemented independently; upstream licensing is not silently imported into this project.

Both the reference and owned firmware enable `ESP_BT_MODE_BTDM` with the controller defaults. Current generated configuration has `CONFIG_BT_CONTROLLER_ONLY=1`, BTDM, one BR/EDR ACL and one LE connection. The LE Host Supported command uses `{1,0}`. Separate controller pools are retained: Classic1021 bytes x9, LE251 bytes x10. BTDM was already enabled; turning it on again does not repair host security/lifetime mistakes. Espressif requires send-availability checks before VHCI submission, outside critical sections or suspended scheduling: [controller/VHCI documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/bluetooth/controller_vhci.html).

## Changes

1. The ESP32 VHCI adapter now retains one in-flight submission until its send-available callback, following the asynchronous packet-ownership rule used by BTstack. An atomic flag is set before SDK submission and released by the callback. The SDK readiness query is still checked. No extra packet buffer, task, spin or wait is introduced.
2. A new Classic SSP IO request resets completed-exchange reply flags. An already encrypted, live-generation,16-byte, durable bonded Classic session can renew its scoped30s pairing lease after full current-key authorization. This does not open a general pairing window or grant BLE control.
3. Product-security BLE no longer sends proactive Security Request outside its local pairing-consent window. HCI LTK-request/reconnection handling remains available. Preflight cancellation, owner wait calculation and quiescence use the same eligibility rule; a disabled request cannot keep the owner runnable.
4. Unsupported/busy initial SMP requests use the existing bounded Pairing Failed reply/loan/retirement machinery, instead of immediate bare disconnect. Policy rejection is Pairing Not Supported; temporary busy rejection is Unspecified Reason. Consent/security checks remain in place.
5. Nonblocking metadata logs expose SMP stages, opcode/length, result, consent epoch, event time and lost-record count. At most64 rows per live generation use the existing logger queue. No keys/nonces/random/PDU payloads are logged. Pairing-feature metadata and separate Classic renewal records are retained.

## Reproduction and evidence

Image `e726205a92d8f5f68f5c43c82613fd42227f53d39f2d04bf13dcbcf82bda4b9f` was readback/boot verified after a separately documented selector readback recovery. On fresh Android pairing, the scoped Classic renewal was granted; SSP completed status0, the new durable key store readback succeeded, and16-byte encryption was verified. This validates the observed trusted-session renewal path, not every security edge case.

A12s finite exact-target96k source completed with0 new Android source underruns, no route/focus loss. LE handle0/generation3 appeared while Classic128/generation1 remained. Native playback still had1024 underrun frames; decode rejects, queue drops, RTP breaks, descriptor faults and watchdogs were0. The first starvation predates BLE. Afterward the LE pairing dialog expired and the usable Android A2DP route disappeared while native Classic stayed open. Do not treat this run as encrypted GATT/application coexistence success.

Diagnostic image `0d5f74f9b35b41ada080f47d614587956d876f53710a7a236240f541565fd0e4` was written921600 and fully readback230400, owned boot retained,37 existing cold codec cases passed. Captured LE handle0/generation2: stage6 and stage1 Security Request auth9 both had consent window0; the request was admitted (stage2). After explicit Android Pair confirmation, Pairing Request offered io4/auth13/keysize16/distribution15,15. Own begin returned4 (`DBT_UNSUPPORTED`) at event_us79321516, then own LE disconnect reason22. The Classic AVRCP channel closed afterward. This demonstrates the contradictory prompt/rejection; it does not prove every reported A2DP failure has this single cause.

Earlier zero-frame source attempts, the Android-killed app_process helper, and source apps with the stale6E address are invalid audio/coexistence evidence and are excluded. The new finite source verifies the exact connected Classic peer and routed audio address `6C:C8:40:42:BF:66`; it stops automatically within15s and records route/focus errors.

Ignored private evidence: `native_bt/_build/experiments/ble-a2dp-regression-1793967920295/`; preserve full UART and phone source receipts locally. Do not publish flash backups or private app signing material.

## Remaining gates

Extend the limited passing paths below to repeated pairing/reconnect, disconnect during active audio, sustained GATT reads/writes/notifications, all application controls, Classic failure recovery, every codec and maximum LHDC. AAC/aptX HD/LDAC startup continuity still needs work. No broad security/fault/OTA/long-duration campaign was performed in this focused diagnosis.

## Corrected prompt-policy image

Image `cd927af6aedc8f1f7a4e8d5d65166b3bf17df525042e67c58100afc0c3e3449a` was written921600. Initial230400 readback truncated; fresh115200 read-only recovery verified every application byte, unchanged prior VALID selector sequence281, bootloader/table and protected original app. No flash write/erase occurred during recovery. Owned boot and37 existing cold cases passed.

Focused `no-outside-prompt-1793973928666` and `ble-connected-route-retained-1793973980842` traces each completed a12s96k exact-target source with no Android source underrun, focus loss or route loss. LE0/generation2 remained connected beyond52s before the second run; no proactive SMP request or pairing dialog was triggered outside consent. The first native stream accepted=rendered=DMA1437696 frames,512 underrun frames,0 queue/decode/RTP/descriptor/watchdog faults. This is a specific route-retention fix, not gap-free playback or encrypted-control qualification.

The existing3s GPIO0 consent gesture was actuated via the COM6 USB DTR line for3.3s, RTS unasserted and0 UART TX bytes. Raw UART captured during monitor release observed window1 then window2 Security Requests, without a boot marker. The first attempted fresh reconnect occurred after window1 expired and correctly emitted no request. In window2, Android consent was accepted; the pairing request was admitted and SC proceeded through public key, DHKey check and encrypted identity PDUs8/9. WORK_STORE then reported failure and only LE was disconnected. Existing code discarded the actual store result (default status0); this has been corrected for diagnosis.

The storage provider prohibits downgrading an already verified candidate identity. A new pairing builds a provisional candidate record even when identity and IRKs are unchanged. The renewal implementation first validates the current RPA against the newly distributed IRK, loads the previous stable record on the worker, and preserves an existing verified flag only for exact identity/candidate/IRK equality. New identities remain provisional; authentication/SC/key-size/IRK downgrade checks remain in the provider. The exact failing storage step/result and preserved-proof flag are emitted separately as stage12 metadata. The targeted verification below now covers this path.

## Verified identity renewal and focused control coexistence

Current image `516147fc4e930190d3bf4bb13a336930606cd4b75e85572ef7a5fb6db2cd4c5f`, archive `runs/1793974457283743700-516147fc4e93`: ccache build,10349 unchanged inputs, application and selector full bytes verified at115200 after921600 writes; owned VALID boot, original app/bootloader/table preserved,37 existing codec cases passed. No Bluedroid fallback.

In the local consent window, LE0/generation3 completed SC and identity distribution. WORK_STORE stage12 reported result0, step4(commit), flags3(committed plus existing identity proof preserved). The crypto/storage worker retired successfully; no LE failure or disconnect followed. The prior record is wiped from the completion before publication. The earlier failed store did not report its actual error, so do not infer all historical failures have this one cause.

After a normal Android Bluetooth off/on cycle, Classic128/generation4 restored16-byte encryption. LE0/generation5 connected with window0, delivered an HCI LTK request (stage7 accepted), resolved the stored private identity (WORK_RESOLVE success), and retired securely without opening pairing consent. The app then accepted two further EQ changes. This verifies one normal bonded reconnect; it does not qualify malformed, stale-key, repeated-cycle or rollback cases.

Two finite12s LDAC96k/S32 source runs, `secured-eq-ldac-1793974888678` and `secured-eq-ldac-1793975071031`, used the actual BDK app to set Deep Bass(8,-1,-2) then restore Balanced(0,0,0). Fresh UI snapshots explicitly reported **Speaker accepted EQ.** after both changes in both runs. App source awaits the GATT Write Response and matching application acknowledgment before this accepted state; server control admission requires the trusted encrypted bond/cookie. Logcat did not expose raw GATT acknowledgment rows, so this evidence is application acknowledgment plus its inspected contract, not a packet capture. Balanced is restored.

Both phone sources ended normally on the exact COM6 route with0 new source underruns, no route/focus loss and no new link/security failure. First flushed native stream: accepted=rendered=DMA1438208 frames,640 underrun frames. Second: accepted=rendered=DMA1437952 frames,128 underrun frames. No encoded queue drops, decoder rejects, RTP sequence breaks, DMA descriptor faults or watchdogs were recorded. These variable short results do not establish a continuity improvement; underruns remain unresolved and acoustic click/fidelity behavior is not measured. Individual worker records were lost; aggregate snapshots and the frozen first-starvation capture were retained.

The second stream first gap(event_us462426543) had decoded input512/output0 while decoding had run167us. Last/max media intervals were24983/25174us; renderer was outside render. This identifies insufficient immediately ready output at refill, without establishing RF loss. See FIRST_STARVATION_2026_11_06.md for timestamp/coherence limits.

The Android pairing helper was also corrected to wait while **Pairing…/Connecting…** rather than repeatedly tapping the target. Interrupted helper attempts are not attributed solely to firmware. All UART/log/export readers must drain paginated400-record event batches; a single batch is not a complete long-interval trace.
