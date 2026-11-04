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

## Outstanding Bluetooth gates

The full profile build still needs current-image bond reload/reconnection,
A2DP and AVRCP exchanges, live codec negotiation/switching, BLE security and all
BDK command flows, simultaneous streaming/control, and lifecycle/stress tests.
Diagnostic GATT readiness alone does not prove an external BLE connection.
Production pairing consent without physical controls remains a separate gap.
