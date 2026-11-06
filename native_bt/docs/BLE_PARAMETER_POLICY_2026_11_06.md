# BLE connection policy and HD coexistence — 2026-11-06

The released Bluedroid product's GATT connect callback in `D:/bt-idf61/bt_audio_sink/main/ble/ble_unified.h:899` requests interval units 18–24 (22.5–30 ms), latency 0 and supervision timeout units 400 (4 s). This is a requested policy, not proof of the parameters selected in every baseline session.

The provided local Bluedroid `stack/l2cap/l2c_ble.c` selects controller negotiation when the role/features permit it and LE signaling otherwise. Reviewed ranges are recorded as partial coverage in [the local audit](BLUEDROID_LOCAL_AUDIT_2026_11_06.json). The complete BT component has not been reviewed.

The owned host was missing the outgoing product preference. It now has an owner-only, bounded coordinator in `probe_ble_parameters.c`. It checks LE feature support, submits tracked HCI requests where supported, and uses CID5 Connection Parameter Update signaling for compatible peripheral links. Command admission, peer signaling acceptance and actual controller update are separate states and log events. Controller-selected values are reported even when they do not match the request. Peer rejection never triggers a Classic disconnect or a codec quality reduction.

The HCI procedure follows the [Bluetooth Core HCI connection-update specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-60/out/en/host-controller-interface/host-controller-interface-functional-specification.html): controller acceptance starts a procedure; its completion event supplies the actual selected parameters. The [LE signaling specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/logical-link-control-and-adaptation-protocol-specification.html) defines the alternative peripheral request/central response exchange.

The LE signaling client reuses the existing link discovery fields because Classic Extended Features and LE connection-parameter exchanges belong to different bearer types. It adds no per-link signaling allocation or payload pool. It uses the existing identifier allocator, bounded queue, pinned TX retirement and RTX timer. One host-owner coordinator owns this link-management lane; it stops querying a cancelled/retired exchange before replacing it.

The coordinator has four fixed contexts, binds handle/generation, retains borrowed HCI callback contexts through cancellation/completion, quarantines old procedures across handle reuse, and permits one timer-driven collision retry. Queue exhaustion waits for transport progress or a deadline. Restart hold cancels optional work. Logging copies metadata into the existing logger queue; no HCI callback prints, allocates or waits.

## Evidence

- Focused fake-transport policy checks cover request bytes, feature routes, admission versus actual update, full queues, callback retirement, handle reuse, timeout, bounded retry and hold.
- Three selected real portable-C signaling checks pass: outgoing lifetime/pins, wrap/timeout/full queues and concurrent Classic discovery, plus the existing incoming LE/fixed-channel regression. Other portable tests were compiled but were not run as a broad campaign.
- The source-stable candidate `dd734e28b62edb09c6ecd7987cb0dfc8dac66e0aaee90b646ffe822d624aab8e` built with ccache, was written to COM6 at 921600 and read back at 115200, booted the owned VALID slot, preserved the original application and passed the existing 37 cold codec configurations. It does not link the Bluedroid host or restore it automatically.
- Target section changes relative to the preceding recovery image are BSS +128 B, flash text +3,572 B and rodata +512 B, with unchanged IRAM/data sizes. Native runtime structure sizes are unchanged.

Before this coordinator, one normal-app coexistence attempt on `dc129c0b0ed441942eb09c5a9196456c148c862220d5b65e14df866619bf36ce` recorded a BLE link followed by 21 input RTP sequence breaks and 76,686 initial DMA shortage frames, with zero decoder rejects, clipping or HCI ingress overflow. The client's logcat events were unavailable, so that attempt does not establish which GATT procedures completed. A shell-only helper had separately been killed before connection; that was not firmware coexistence evidence.

Two short native YouTube runs on the policy image negotiated aptX HD 48 kHz / stereo / 24-bit and measured 574.57 / 575.13 kbit/s encoded payload. Both completed BLE connection, MTU 517, service discovery, 38-byte settings read, notification setup and a successful unchanged-EQ write/ACK while Android reported Classic audio playing. Both recorded zero new queue drops, decoder rejects, input RTP breaks, initial DMA shortages, descriptor faults or DSP clipping. Gains remained 8/8/11, volume 127 and flags 12. These are separate control/transport/DSP-counter observations, not acoustic proof.

The observed BLE interval was already **30 ms** at connection in the passing runs; the update completed at **30 ms**, latency 0, with timeout changing from 5 s to 4 s. Both local/peer LL request features were present and the HCI route was used. Therefore these runs do not isolate an interval change as the cause of improvement. Connection-event scheduling, procedure timing, source state and RF variation are not isolated by this comparison. The legacy LE signaling route is locally tested but has not been exercised with a legacy physical peer.

The first client's two initial F0 status notifications were mislabeled `eq_ack:false` by that diagnostic; they were not command errors. The corrected client separates settings notifications from the later successful 0x10/0x01 ACK. The temporary app and uploaded DEX were removed; the original BDK app was not modified.

A subsequent source guard also applies old-handle quarantine to LE signaling requests; focused fake-transport coverage passes, but that guard is not part of the two recorded live binaries. It will be included in the next candidate.

## Remaining sound problem

The user reports aptX HD noise persists even after reconnecting. Resynchronization and clean transport counters are insufficient to declare that sound fixed. Next compare live target decoder PCM and DSP output against desktop replay, then follow any remaining corruption toward DMA/I2S. Maximum LHDC V5 headroom, acoustic continuity, legacy-peer and full security/lifecycle/stress qualification remain open.
