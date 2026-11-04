# Current Bluetooth radio checks

The independently authored host is still experimental. The released Bluedroid
application remains preserved in ota0. Only COM6 was accessed; the attached
peripheral is the DAC. These checks do not establish feature parity.

## Selected-peer Classic pairing

On 2026-11-04 the Redmagic discovered `NATIVE_BT_PROBE`, presented its ordinary
pairing dialog, and was paired without contact/call-history access. Android's
device-details address matched the actual HCI controller address on COM6.

- Source: `c9d080baf03b4902cafa4af6a890bf5b2b4da440`.
- Image SHA-256: `48b052be75e9118590e1174c4d818429f27a1190ef630cbe17fa165a4aa0c6db`.
- ESP-IDF 6.1, controller-only dual mode; Classic security and diagnostic GATT,
  without audio/product profiles. Built with ccache.
- Admission accepted the explicitly configured phone address. No other peer was
  authorized by this diagnostic policy.
- The native security owner reported a committed bond (event 1, status 0), then
  encryption ready (event 0, status 0, key size 16, bonded true).
- Key type 4 and `authenticated=false` describe an unauthenticated Just Works SSP
  key. This is encryption/bonding evidence, not MITM or Secure Connections proof.
- A later link-removed event retained bonded state. A tap on Connect produced no
  additional admission event; reconnection was not verified by this image.
- The unconfirmed finite trial reset and booted the preserved `1.0.4-idf6.1`
  application from ota0, reached `System ready`, and showed no fatal boot marker.

The private local archive is
`native_bt/_build/hardware/runs/1793812485016473800-48b052be75e9`.
It contains the exact image/build inputs, UART capture, final return receipt,
phone action timestamps and address-confirming device-details XML. Full flash,
NVS backups and keys are not published.

The callback's product-consent and selected-peer decisions also passed 32 host
cases with an optional/full nonblocking log queue. Those cases prove policy
preservation in stubs, not RF authentication.

## Full profile boot and saved-bond reload

The next finite trial used image
`dd5805b5468445be40a08f3d82225361a3c058bd076f1e28a48ca84c0e665fb0`.
It initialized the full product providers and all codec endpoints, reloaded one
native Classic bond, and kept new pairing closed (origin 0, Classic scan 2).
The phone still displayed the diagnostic device entry without a media-audio
control. Tapping Connect produced no new native admission event or media packets.
This does not establish a host reconnection failure: the phone's cached service
state was not refreshed in this attempt. Audio and AVRCP were not exercised.
The image returned cleanly to the preserved baseline. Its private archive is
`native_bt/_build/hardware/runs/1793813000520036800-dd5805b54684`.

## Local pairing without encoders

The development product now accepts a physical BOOT/GPIO0 hold: first release
the button after startup, then hold it for three seconds. This invokes the same
120-second local consent window as the encoder gesture. Debounce and hold
deadlines run on the existing owner; the interrupt only signals input changes.
Any edge cancels an unfinished hold; a held startup button cannot grant consent,
and a completed hold cannot repeat until release. Restart freeze and existing
pairing/bond/trust checks still apply.

Eleven ASan/UBSan host scenarios and the full ccache-enabled ESP32 build pass.
The GPIO ISR is placed in IRAM. The explicit COM6 fixture can hold DTR/BOOT after
readiness while leaving RTS/EN inactive; its modem-line record alone is not
physical gesture proof.

The subsequent COM6 trial bound source `8b9c6eb64cc91a8a622e190abfd579008f7343fe`
to image `c8c318f1470cd89fdeed9bdb3536399a56f26cad20c9f983c231d9e165ce9652`.
It started with one saved bond and the production pairing window closed. After
the BOOT fixture held/released the input, the native admission event showed the
window open. A fresh phone pairing then committed a key and enabled 16-byte
encryption. This is end-to-end window/admission evidence through the fixture;
the mechanical switch itself was not pressed by a person.

Android subsequently displayed an active media connection and an HD-audio LDAC
option; the option is not evidence of the negotiated codec. The native AVRCP
path received track title, artist and album (revision 1, generation 1), and the
profile manager reported active state with zero event errors. No media packets,
PCM frames or I2S DMA frames were observed: playback was not started. A BDK app
pairing dialog was approved late; the app remained at its audio-connection gate.
This is not a completed BLE security/control test. The exact failing stage was
not instrumented in this image.

The finite trial returned cleanly to the released baseline. Its private archive
is `native_bt/_build/hardware/runs/1793813848563190000-c8c318f1470c`.
Native Classic/LE link and failed-SMP metadata have since been added to the
existing nonblocking logger for the next ten-minute, bounded trial.

## First current-image RF streaming failure

The ten-minute trial used source `b400052c` and image
`7db0feb873069437b99f0805b20c27bdd835734ba52a0c244726b81c92b9a187`.
The phone's existing media controls started actual LDAC stereo 96 kHz playback.
The native path accepted 742 media packets, decoded 6,072,320 PCM bytes and
delivered 756,096 source frames through I2S DMA. This is transport/decode/DMA
execution evidence, not an analog-quality pass.

**The streaming test failed.** It reported 203,264 underrun frames, one DMA
descriptor error, 195 drops, 186 decoder resets and a codec-worker fault. The
maximum observed EOF gap was 13,841 us and render span 14,740 us. Output failure
latched the profile stopped; the Classic link subsequently closed. The global
oversized-PDU count reached 10, but this image did not identify their PSM/CID;
they cannot yet be assigned as the root cause. Drops likewise have not been
separated into radio loss versus host/worker backpressure.

The BDK app reached its audio gate after output shutdown. No completed BLE
control test follows from this run. Playback is paused and the preserved
baseline boot was verified after the trial. See the sanitized
[machine-readable result](RF_LDAC96_2026_11_04.json); raw evidence remains in
`native_bt/_build/hardware/runs/1793814768731988700-7db0feb87306`.

The next Bluetooth baseline uses SBC selected on the phone, with every codec
endpoint retained. LDAC 96 kHz remains a recorded failure. The next trace also
records negotiated channel MTUs and the actual HCI disconnect reason.

## Saved-bond reuse, codec selection and short BLE coexistence

The next trial bound source `81b0e955` to image
`abd119497ab135c4e22f8be88a2de07fbbfcb7a18e11c27ec8db81374f7f2c42`.
COM6 reconnected using the saved native Classic bond, reached 16-byte encryption
without a new key-store event, and opened SDP, AVDTP and AVRCP channels. Android's
codec chooser selected SBC before playback; the app and native PCM report agreed
on stereo 44.1 kHz/S16. The media channel closed/reopened during selection while
the signaling and AVRCP channels remained present.

SBC decoded 1,941,504 PCM bytes and delivered 484,096 source frames through I2S.
The BDK app briefly displayed its normal control screen and the native trace
recorded an LE connection alongside the Classic link. This does not establish
full BLE security, every command or long-duration coexistence.

**SBC output also failed:** 157,568 underrun frames, one DMA descriptor error,
242 drops and 229 resets. Its maximum observed EOF gap was 12,635 us and render
span 15,687 us. The global oversized-PDU count stayed zero, so oversized packets
do not explain this SBC failure. The shared output/scheduling path needs further
isolation; no specific interrupt or CPU bottleneck has yet been established.
Both recorded HCI disconnections had status 0/reason `0x13` (remote termination).
No LE security-failure event was logged; absence of a diagnostic event is not a
complete security pass.

The preserved baseline boot was verified and phone playback is paused. See
[the sanitized result](RF_SBC441_2026_11_04.json); raw evidence remains in
`native_bt/_build/hardware/runs/1793816091087576200-abd119497ab1`.
Bluetooth qualification should next isolate radio/profile/control exchanges
from the failing I2S path. Every production codec/output path remains retained;
neither streaming configuration is qualified for release.

## Released firmware restored

After the native trials, COM6 booted the preserved `1.0.4-idf6.1` application.
The phone was paired back to that verified address and its LDAC preference was
restored. The first app synchronization returned an invalid-handle error after
the database swap. Restarting only the BDK app resolved it: the app subsequently
showed `Connected · 1.0.4-idf6.1`, LDAC 96 kHz and the normal speaker controls.
Playback was verified paused. No app data or unrelated bonds were cleared.

The cause of the stale-handle behavior has not been established. Future trials
should close the app's old GATT connection before swapping firmware and must
qualify service-database/cache lifecycle behavior before a native release.
The private `released-phone-restored.xml` records the final app state.

## Radio/decoder-only diagnostic

The explicit finite isolation mode used source `ef5b2323` and image
`39f96d07ad25fd3f60772f9f87f8a498656bab7003bb97947a2810e0594a7663`.
All codec endpoints and product security remained enabled; decoded A2DP PCM was
checksummed without I2S forwarding. LDAC stereo 96 kHz initially progressed with
zero decoder drops/resets, and ultimately produced 14,123,008 PCM bytes from
1,724 accepted media packets with no codec-worker fault. Later it recorded 42
drops, 39 resets and 2,443 oversized dynamic-channel PDUs. This image still did
not identify those packets' length/PSM/CID, so no MTU limit has been changed.

The phone made an LE connection, then removed its phone bond during the app
attempt. No completed LE security state or controls test was observed. Classic
profile/channel state remained available after the phone paused playback. The
diagnostic returned to the preserved baseline without a fatal boot marker.
These observations establish an additional transport/interoperability problem
independent of A2DP rendering; they do not qualify uninterrupted audio or BLE.

See [the diagnostic result](RF_PROTOCOL_2026_11_04.json) and its private archive
`native_bt/_build/hardware/runs/1793818408254184200-39f96d07ad25`.
The next image records rejected PDU dimensions and bounded SMP request/receive
metadata. Receive validation, keys, trust gates and codec support are unchanged.

## Packet dimensions and SMP admission captured

Source `1d7369cd`, image
`daac108087fc71973b4a6ed830f498ee1d0e3e03433e225f9a31ea6700810b65`,
recorded the rejected packets as Classic A2DP media (PSM 25/CID 66), **673-byte
SDUs against a 672-byte receive MTU**. It also recorded an LE Security Request
submitted to the controller, a received seven-byte Pairing Request, successful
procedure-start admission, then a local security failure/disconnect (`0x16`).
The procedure emitted two failure notifications. The baseline returned cleanly.
See [the captured metadata](RF_MTU_SMP_2026_11_04.json); raw evidence remains in
`native_bt/_build/hardware/runs/1793820818840780300-daac108087fc`.

The application now shares a 1008-byte media capacity between its L2CAP profile
and decoder queue, matching the retained SDK's `BTA_AV_MAX_A2DP_MTU`. Queue depth
remains eight. The 673-byte case is accepted at that negotiated capacity and a
1009-byte case remains rejected in the channel tests. Packet validation is not
bypassed; additional queued payload capacity is bounded to 2688 bytes.

A separate host regression reproduced rejection of a peer's valid optional
LinkKey offer. Feature negotiation now retains the exact request transcript and
selects only locally supported distribution keys. Local CTKD is not offered;
SC/key-size/MITM and selected-mask restrictions remain tested. The responder's
ability to select a supported subset follows the
[SIG key-distribution rules](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/security-manager-specification.html).
This is not yet proven to be the phone's failure cause. The next trace records
public pairing-feature fields and worker/session admission results without keys.

Duplicate failure publication is also fixed and tested while accepted worker
work is still retiring. All 179 host groups, the PSA-backed probe fixtures and
the actual callback-mode tests pass. The secured target image builds/admission
checks pass. Hardware validation of these corrections remains pending.

## MTU correction and captured association mismatch

Source `61ec6219`, image
`8d968bd2be09c8896ae5910f0c6df9786caba3f55b2ca47a552222d41a2aa109`,
reported the advertised 1008-byte media capacity and recorded **zero oversized
dynamic PDUs**. The queued-payload allocation increased by the expected 2688
bytes. The failed BLE procedure now emitted one notification. Radio PCM still
recorded five drops and three resets around the failed app attempt; this is not
a loss-free/coexistence pass. See [the result](RF_MTU_FIXED_2026_11_04.json).

The actual phone request was IO=4 (KeyboardDisplay), OOB=0, AuthReq=13
(bonding/MITM preference/SC), key size=16 and distribution offers=15/15. Worker
preparation and session opening succeeded before feature exchange failed. The
old code rejected the peer MITM flag despite the speaker's NoInputNoOutput
capability and explicit allow-Just-Works policy. Further duplicate selectors in
the SC engine, session setup and crypto worker also incorrectly selected or
required Passkey for this combination.

These layers, including legacy IO selection, now share the association table.
Just Works remains unauthenticated. Local required-MITM, disallow-Just-Works,
16-byte/SC policy, existing authenticated-bond downgrade protection and
authenticated-attribute permission checks remain enforced. The IO mapping is
defined in [SIG Table 2.8](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/security-manager-specification.html).

Host tests cover all IO combinations in SC/legacy modes, the captured request,
public SC initialization, legacy and session allow/deny policy, exact transcript
preservation and authenticated-bond downgrade rejection. The real PSA two-party
regression initially failed the Just Works commitment check; it passes with the
shared selector, along with f5/f6 agreement, invalid-point, replay and key-slot
retirement checks. All 179 host groups pass. The updated target builds; actual
phone encryption/durable bonding with this latest correction is still pending.

## Secure BLE bonding, reconnect and the missed scheduler warnings

Source `0feb3ac2`, image
`6bceaf1587e5697b67b7af0fe58637db4489c6cc3713f0ad99e104b4916bfd1d`,
completed phone Secure Connections Just Works pairing, 16-byte encryption,
phase-three key distribution and durable bonding. After app restart with the
pairing window closed, stored-key reconnect verified the peer and promoted its
identity. Authentication correctly remained false for NoInputNoOutput Just Works.
The BDK app accepted Deep Bass (8/-1/-2), then read those values on fresh
reconnect. Balanced (0/0/0) was restored before playback.

LDAC 96 kHz decoding then ran with BLE connected, but accumulated 1,995 aggregate
drops and 1,579 resets across 8,466 retired media packets. There were zero
oversized dynamic PDUs and zero codec-worker faults. PCM was checksummed without
A2DP I2S output; this is not an audio continuity pass.

The earlier report missed **22 nonfatal task-watchdog triggers**, each reporting
CPU 1 idle starvation. Sampled CPU 1 tasks were decoder 11, controller 4 and
host/main 7; CPU 0 was idle in all 22 samples. This confirms scheduler starvation,
without establishing the individual cause of each dropped packet. Boot readiness
and `boot_fatal_marker:false` did not establish healthy runtime scheduling.
The released baseline returned cleanly and playback was paused. The phone/app
connection to the baseline was not rechecked after this particular return.
See [the corrected sanitized evidence](RF_SC_BOND_2026_11_04.json).

New [stage-specific diagnostics](BLUETOOTH_DIAGNOSTICS.md) separate ingress,
host wake/service timing, local queue rejection, upstream and worker RTP gaps,
codec media/reset calls, PCM delivery/backpressure, and diagnostic-record loss.
Runtime watchdog warnings now fail trial runtime health separately from boot
readiness. Target streaming measurements with this instrumentation are pending.

## Decoder placement comparison

The [CPU 1 instrumented reference](RF_CORE1_DIAGNOSTICS_2026_11_04.json) attributed
all 1,663 drops to a full decoder queue. Incoming RTP sequences were continuous,
ingress/decoder error counts were zero and 15 watchdog events reported CPU 1
idle starvation. The unrelated wake-latency diagnostic underflow is explicitly
excluded from that record.

Moving only the decoder placement to CPU 0, while retaining codec, queue depth,
secured BLE and live EQ operations, produced [11,238 decoded packets with zero
drops/resets/watchdogs](RF_CORE0_DIAGNOSTICS_2026_11_04.json). Queue high water
was 2/8 and maximum codec wall span 4,111 us. Live playback offsets/durations
differed; this is not identical-bitstream benchmarking. Both were checksum
tests. Full I2S and maximum LHDC qualification remain separate outstanding gates.

## Outstanding Bluetooth gates

The full profile build still needs sustained loss-free A2DP/BLE operation,
complete AVRCP exchanges, live codec negotiation/switching, all BDK command
flows, and lifecycle/stress tests. The named secure bonding/reconnect and EQ
readback cases above passed in checksum mode; full I2S coexistence remains open.
Diagnostic GATT readiness alone does not prove an external BLE connection.
Production pairing consent without physical controls remains a separate gap.
