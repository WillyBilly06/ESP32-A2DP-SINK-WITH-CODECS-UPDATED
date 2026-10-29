# Native AVDTP media manager and SBC worker

The owned sink now has a runtime channel manager, an actual asynchronous SBC
decoder worker and a restricted ESP32 RF diagnostic. This is still development
firmware: the PCM consumer counts/hashes samples and does not drive I2S. There is
no Audio Sink SDP advertisement, Firefox playback or acoustic qualification yet.

## Runtime channel ownership

`dbt_a2dp_acceptor` registers encrypted Classic PSM 0x19 with a minimum 335-byte
MTU. The channel manager has a separate incoming-admission hook before allocation
or security work, with duplicate requests handled first. This hook is distinct
from authentication/encryption policy. The acceptor reserves the speaker across
signaling peers, admits a media channel only after the Open response, and refuses
early, extra or unrelated channels. Channel serials and link generations bind
data/events to the correct lifetime. Teardown is deferred out of borrowed callbacks.

Stop closes both open and still-configuring channels. The instance cannot be
reused until the runtime returns its borrowed TX and the worker acknowledges DROP.
The portable paired-runtime test exercises admission, separate channel routing,
media traffic and delayed worker retirement through the real runtime APIs.

## SBC packet handling and decoder

`dbt_sbc_media` validates the negotiated frame format/bitpool, legal dimensions and
bitrate, and SBC's header/joint/scalefactor CRC. It validates all frames in an
unfragmented packet before dispatch and adds per-frame RTP timestamps. SBC CRC
does not cover quantized audio bits; it is not a full-payload integrity check.
Fragmented frames check remaining-fragment counts, packet size, sequence, SSRC,
timestamp, exact final size and expiry. Only actual fragmented input allocates
assembly memory, sized to the frame header and released at completion or failure.

The codec is the SDK's standalone Apache-licensed Open Interface/AOSP SBC decoder,
without its Bluedroid protocol wrapper, types, scheduler or APIs. The owned port
uses fixed-width arithmetic types on ESP32 and Linux x64. A private minimal
`common/bt_target.h` enables only the codec. The dependency audit admits this
specific codec directory and continues rejecting Bluedroid/NimBLE protocol sources
and all BTstack sources/includes. No SDK or reference tree is cloned or copied.

`dbt_sbc_decoder` owns one lazy active allocation containing codec context,
negotiated-channel synthesis data, PCM output and a bounded padded input frame.
Mono gets one channel's subband/history data. Reopen releases the old workspace
before allocation. SDK reset clears context but does not establish heap synthesis
history: the wrapper explicitly clears that history for a cold stream or reset.
Immutable codec tables remain in `.rodata`.

Actual tests first exposed negative signed left shifts in the SDK synthesis code.
`prepare_port.py` leaves the SDK unchanged and generates three small licensed
translation units in the reusable build directory, with 30 explicitly checked
shift sites/macros converted through unsigned 32-bit shifts. `-fwrapv` supplies the
fixed-point arithmetic contract. Original source hashes and generated hashes are
recorded. There are no sanitizer exclusions. PCM is compared against an original
standalone-codec build using the same explicit wrapping contract.

## Asynchronous worker

Exactly one host owner submits jobs/media and drains receipts. One decoder task
processes the copied bounded control/receipt SPSC queues, then a bounded number
of encoded media packets. Codec work, PCM callbacks and optional output start/stop
run on that worker. Wake hooks connect the portable worker to FreeRTOS.

Encoded media queue storage is lazy. The host disables ingress through a gate
epoch before publishing a control barrier. An older Start cannot reopen ingress
after Stop/Abort/Drop. Queue publication orders earlier payload copies before
worker destruction. The decoder, packet queue and partial frame are released
before a DROP receipt. Reconfiguration frees old storage first and either restores
the prior usable configuration after failure or reports Fatal. Sequence/SSRC gaps
reset codec history. Busy queues and discarded media have explicit counters.

The ESP32 diagnostic runs a 4096-byte decoder task on core 1 and a separate
2048-byte low-priority logger; logging is absent from decode and controller
callbacks. PCM tuple logging uses atomic fields and a generation check. Packet
counters can lag that tuple during a packet; final idle observations are stable.

## Verified evidence

136 owned host groups pass on Windows/Linux, including Linux ASan/UBSan and
20,000 concurrent ingress packets. Actual SBC tests use FFmpeg-generated tone
frames at 44.1/48 kHz mono/stereo. Candidate PCM matches the standalone reference,
fragmented input yields the same PCM, and allocation failure/retry and no-overlap
reopen pass. ASan/UBSan and ThreadSanitizer pass. Two real threads process 500
packets per vector through the actual decoder worker, including sequence wrap.

The restricted diagnostic image was flashed only to the verified trial slot;
complete image/selector readback, boot and original-application preservation passed.
Its linked symbols include the owned acceptor and `OI_CODEC_SBC_DecodeFrame` and
exclude `esp_bluedroid_init`. A separate optional build now verifies 16 standalone
SBC/wrapper/worker units and eight LHDC/adapter units in the actual IDF graph;
building LHDC does not constitute LHDC negotiation, linked use or playback.

In the finite 48 kHz stereo RF run:

- The Linux sender submitted 2000 media packets / 8000 SBC frames over 21.323 s.
- The ESP32 counter consumer received 8000 decoded frames / 4,096,000 PCM bytes.
- The rolling FNV-1a-32 value matched the expected value, 4109840158. This is a
  diagnostic fingerprint, not cryptographic proof of equality of every PCM byte.
- During the stream, kernel LE ATT completed 166 writes, 16 readbacks and 166
  notifications. Recorded worker drops and sequence resets were zero.
- Reported requested worker allocations peaked at 8114 bytes and returned to zero
  after close. This excludes allocator overhead, task stacks, static metadata,
  controller/SDK and the rest of the application.

`verify_sbc_rf.py` binds the complete bounded UART window and sender result to
image SHA-256 `a628b32f4c1bd219ebcc18060e6493dfea797fcfe9fc23f4253d0b989622222e`.
That run is direct AVDTP test traffic, not a BlueZ/PipeWire/Firefox product session.
It does not resolve the earlier large-payload generic Classic/BLE corruption or
qualify other codecs, long-duration playback, timing, I2S, acoustics or EQ/NVS stalls.

Target metadata measures 396 bytes for the complete acceptor (including its sink,
signaling and bearer), and 380 bytes for the worker (including decoder/framer
metadata). These nested values must not be added twice. Actual decoder working
allocations remain configuration-dependent; Linux values are not ESP32 heap totals.

Cleanup confirmed the diagnostic bond deletion after its NVS commit, no remaining
Linux diagnostic peer, restoration of the original Bluetooth daemon command,
and full-readback/boot-verified restoration of the Bluedroid audio trial. The
original application remains intact.

## Next release gates

Connect the PCM worker to the real integer DSP/I2S application and its output
start/stop contract; add honest Audio Sink SDP/discovery and measured delay reports;
integrate the other codec validators/workers and AVRCP; migrate product BLE control,
OTA and security. Run Firefox music with uninterrupted BLE traffic at every codec,
test mandatory immediate NVS commits during output, and perform equivalent-feature
RAM/CPU/latency measurements. Full stack completion remains unproven.
