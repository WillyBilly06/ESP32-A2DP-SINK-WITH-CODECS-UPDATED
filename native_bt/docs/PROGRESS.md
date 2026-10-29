# Current project progress

Updated 2026-10-29. Goal **active and incomplete**.

Current phase: source audit, owned fixes and offline verification before any new
Bluetooth connection test. [L2CAP comparison](L2CAP_COMPARISON.md) records Basic
signaling/reconfiguration corrections. [LE credit channels](CREDIT_CHANNELS.md)
now integrate incoming/outgoing setup, security qualification, credit admission,
segmented runtime transfers and channel-level failure cleanup. Current suite:
**166 groups pass** on Windows/Linux, including ASan/UBSan and TSan. The ESP32
probe builds and links the credit engine; no CoC service is enabled by default
and no post-fix RF success is asserted. ERTM and other required modes remain open.
The common codec worker and AAC CIE/LATM transport now have portable lifecycle,
rollback, gate and parser tests; the new native AAC decoder adapter still needs
actual PCM validation and target integration. The previously tested audio-app AAC
wrapper is a different path.

This report covers the full local working tree. The GitHub host checkpoint is
limited to the owned host/probe, tests, docs and portable DSP test dependency;
it does not publish or replace the locally modified Bluedroid audio application.

## Requirements retained

- Independent Classic + BLE host, concurrent operation and ESP32-supported features.
  BTstack is a pinned reference only: no copying, includes, linkage or runtime use.
- Preserve/optimize SBC, AAC, aptX, aptX-HD, aptX-LL, LDAC, Opus; add **LHDC V5**
  from `D:\ESP32 Bluetooth\LHDC-V5-Decoder-main` without copying its bundled SDK.
- Only active codec/configuration working data and necessary RAM table copies;
  release inactive resources and avoid overlapping full decoder working sets.
- Integer EQ/DSP; fix BLE EQ pops and cross-task filter state mutation.
- **Commit every EQ change to NVS immediately, even during playback.** The user
  explicitly rejected deferred, coalesced or playback-idle persistence.
- User authorized autonomous work, SSH/Linux codec tests and the opened Firefox.
  Credentials remain session-only; none are saved in source or reports.
- Organized sources, fixed reusable build paths; no SDK clones, recordings or
  repeated build snapshots. Size checks: 96 MiB reference / 512 MiB generated.

## Implemented and checked

| Work | Current evidence | Limit |
| --- | --- | --- |
| BTstack | Pinned `51e11ce3692abfe31b4ee3d66cffdeb1ae1626f2`; shallow sparse checkout ~70.6 MiB | Reference only |
| HCI | Owned FIFO, framing, eight-command identification, copied four-slot general command queue, completion/status distinction, credit gates and timeout/abort callbacks | Native/ESP32 builds; target probe result is recorded separately below |
| Runtime/ACL | HCI events drive Classic/LE links, credits, disconnect cleanup and L2CAP dispatch in one owner; live LE ATT and encrypted Classic Basic channels established | Short simultaneous diagnostic run passed; longer untraced run failed, so reliability remains unqualified |
| Basic L2CAP | Lazy RX reassembly; incoming/outgoing Classic channels; both configuration directions, MTUs, continuation, option negotiation, disconnect, RTX/ERTX, duplicate handling and information responses | Basic mode only; ERTM/LE credit channels and broader interoperability pending |
| Receive flow / SCO framing | Bounded controller-to-host ACL credits, special no-command-credit acknowledgments, generation cleanup, accepted ESP32 buffer setup; H4 SCO status/framing and explicit callback | Receive flow did not remove all corruption. SCO connection management, synchronous flow and voice profiles remain pending |
| Scheduling | One borrowed outbound SDU per connection, round-robin ACL admission, per-link signaling selection, full-PDU signaling commit and immutable transport buffer lifetime | Simulated independent Classic/LE pools and asynchronous transports; not measured radio fairness |
| GAP | Event masks, LE host support, local name/class, SSP enable, legacy advertising parameters/data/scan response; explicit Classic scan and LE advertising controls | Incoming Classic policy hook; security requests default to negative replies without a security owner. BLE bonding/privacy and initiator procedures pending |
| Classic security | Owned SSP/PIN/passkey/OOB policy, key lifecycle, encryption/key-size qualification, asynchronous durable store/delete and pending L2CAP security; live Just Works pairing and encrypted 16-byte-key channel passed | Linux diagnostic bond later disappeared; durable reconnect interoperability and other pairing methods need qualification. BLE SMP remains pending |
| ATT/GATT server | Owned LE ATT server and runtime bridge; discovery, reads/blobs/multiple, MTU 517, writes/commands, prepare/execute/cancel, per-client CCCDs, security policy, notify/indicate, deferred worker completion | Product command/OTA binding, real security/bond store, GATT client/EATT/signed writes/robust caching pending |
| SDP server | Immutable flash database, UUID aliases, all three server transactions, bounded continuation, streaming response slices and lazy runtime bridge; Linux verifier and BlueZ sdptool passed. Audio Sink record is enabled only with the experimental native I2S output build | SDP client and ordinary Linux audio discovery/playback qualification pending |
| AVDTP sink/control | Owned signaling/sink, encrypted PSM/media manager, SBC validation and actual lazy decoder worker. Raw RF/I2S diagnostic decoded 8000 frames during ATT traffic; ordinary BlueZ negotiation and two profile sessions separated by disconnect passed | Firefox media observed, but its overall run failed on host malformed-packet/shutdown handling. Other codec workers and product integration pending |
| Tests | 166 groups pass on Windows/Linux, including 10,240 generated ATT inputs, 8192 generated SDP inputs, security/key lifecycle, receive-credit/SCO, SBC CRC/fragment, session retirement/ACL loss, Basic signaling/reconfiguration, LE CoC signaling/runtime and 1,001 credit-engine round trips, codec lifecycle/AAC transport and integer audio tests plus 20,000 concurrent FIFO packets; Linux ASan/UBSan and TSan pass, including 16,384 frames through three concurrent audio owners | Host behavior and memory safety; separate finite RF evidence; CoC lacks external-peer/RF qualification |
| SBC decoder/worker | Actual 44.1/48 kHz mono/stereo frames produce PCM identical to the standalone reference; fragments, OOM/retry and no-overlap reopen pass. ASan/UBSan and TSan pass; actual two-thread worker processes 500 packets per vector | SDK codec only, owned wrapper/scheduler; full application output/timing and broad SBC conformance pending |
| Integer DSP | Flash coefficient tables for 13 rates; Q4.28 coefficients/Q5.27 headroom, fractional error carry, 20 ms ramps, render-owned histories, atomic controls | Embedded CPU and audible tests pending |
| Native DSP/I2S candidate | Active-only PCM/output rings; render-owned integer DSP, fractional clock adjustment and stop fade. Firefox played via ordinary PipeWire joint-stereo SBC during 22.119 s / 184 ATT writes/notifications; 8910 decoded frames, zero recorded decoder drops, output underruns or descriptor errors | Overall Firefox run FAILED: native BAD_PACKET fault and disconnect timeout. Exact fault time relative to overlap is not established. Acoustics, physical NVS stress and product handoff unqualified; see AUDIO_OUTPUT.md |
| DSP numerical tests | 36 cases at 44.1/48/96/192 kHz; max magnitude error ~0.00000145 dB against the coefficient response; native state 1128 bytes | Not ESP32 timing or acoustic proof |
| C++ integration | Actual DSP/storage headers: normal, ASan/UBSan and ThreadSanitizer pass with concurrent control/render/meter requests | NVS is mocked; no physical power-loss test |
| EQ persistence | Immediate single packed tuple write + commit; legacy-key load fallback; write/commit failures tested; latest app builds | Physical flash/power-loss tests pending |
| LHDC optimization | Active-size reference cosine table; release old allocation before resize; release inactive reference cache; retain explicitly forced diagnostic cache | Reference/fallback path only; no normal-fast-path CPU claim |
| LHDC tests | Baseline/candidate reference vectors and fast transforms byte-identical; resize/OOM/retry checks and ASan/UBSan pass | Full encoded-frame, ESP32 timing and playback pending |
| LHDC adapter | Raw-frame lifecycle adapter and portable sibling decoder component compile for ESP32 | Adapter lifecycle tests and A2DP negotiation integration pending |
| AAC | Active-only aligned workspace; actual AAC-LC raw-frame PCM byte-identical at 44.1/48 kHz mono/stereo; address sanitizer, failure/retry and no-overlap reinit checks pass | Target permanent workspace reduced from 20720 bytes to a 4-byte pointer; actual embedded heap/activation tests pending |
| Opus | Lazy fragment storage; scratch sized by channels/frame duration; 14 Android/PipeWire configuration cases with actual CIE parser and system libopus pass byte-identically, including fragmentation and ASan/UBSan | Native backend tests, not ESP32 timing/RF proof; target static context reduced from 6164 to 24 bytes |
| Codec switching | Old decoder cleaned up before allocating a different decoder's scratch/working set; AAC cleanup now releases its underlying workspace | Hardware switch/fragmentation tests pending |
| Audio app | Integer DSP, immediate NVS, AAC and Opus changes integrated; ESP32 app builds and trial flash passed full readback; live idle diagnostics observed | Playback/coexistence unqualified; LHDC not yet advertised by this app |

Reference cosine-table payload: N=240: 30720 -> 3840 bytes; N=480: 30720 -> 7680;
N=960: 30720 -> 15360; N=1920 unchanged at 30720. These are Linux allocation
measurements of the selected path, not total codec RAM or embedded performance.

## Still pending

GAP initiator/discovery procedures and recovery; broader receive-flow interoperability;
L2CAP modes beyond Basic; further Classic security interoperability and BLE SMP/bonding/privacy;
product ATT/GATT integration, GATT client/advanced features; SDP client;
product DSP/I2S/service handoff and measured delay, remaining codec negotiation and A2DP/AVRCP parity; remaining profiles; complete app migration; sustained Classic/BLE
hardware validation; equivalent-feature memory/CPU/latency comparisons.

AAC and Opus lifetime changes are implemented with native PCM/allocation checks;
embedded contiguous-space and switch-peak validation remain necessary. Opus retains
fragment assembly storage only after fragmentation is observed, reuses it for a
fragmented stream, and frees it on unfragmented input/reconfiguration/cleanup.
SBC now has lazy negotiated-channel state and actual PCM/allocation checks; further
SBC CPU/conformance work and LDAC/aptX optimization/benchmarks remain pending.

Immediate NVS remains mandatory. The baseline app has 3 x 128 stereo DMA frames
and its I2S ISR IRAM-safe option is disabled. The new native output candidate
requires IRAM-safe I2S and copies already rendered internal-SRAM samples in the
ISR; its owned ISR roots/direct references passed a linked-address audit. It
buffers approximately 64 ms before DMA, allocating only for the active stream.
Physical flash/cache interruption remains unverified; the address audit and
filter transition tests do not prove continuous output during an NVS commit.

## Hardware and Linux

VMware guest tools located Ubuntu; SSH works. The previously trusted `BDK SPEAKER` peer
matches the project audio/control UUIDs. Reconnect succeeded; BlueZ negotiated
LHDC V5 CIE `3a050000354c0102114000`; PipeWire reported 192 kHz/24-bit, **idle**.
No Firefox MPRIS player was found. This is the existing, unversioned firmware,
not the new host and not a proven match to this source. No music, decoder/DMA,
acoustic or coexistence result is claimed. Remembered sink volume exceeded 100%;
set a safe volume before playing music.

COM6 was positively identified by its USB serial, ESP32-D0WD-V3 revision 3.1,
factory MAC and 8 MiB flash ID, then correlated with the speaker through a
controlled codec change and UART configuration evidence. COM7/COM8 remain untouched.
The full original 8 MiB flash backup is retained with SHA-256
`c03b34c4e70b087508cb3189d34ca0b56d75cf24d39b6562f70794bdb729434f`.
The original OTA0 application, bootloader, partition table and first OTA selector
sector are preserved; trial writes are restricted to OTA1 and selector sector two.

The initial audio trial image passed complete write readback. Its readiness marker
was missed because observation used 921600 baud while the actual generated build
configuration was 115200. A subsequent read-only 115200 capture shows the trial's
`LIVE1` idle diagnostics. This establishes execution, not music continuity.
The failed raw capture remains preserved. The deployment tool now derives UART
baud from the candidate's actual build configuration and separates probe/audio receipts.

Independent probes were flashed into OTA1 with full image/selector readback and
observed `controller_identified` / `gap_state:2` markers. The controller reports
separate Classic 1021-byte/9-packet and LE 251-byte/10-packet ACL pools. The latest
combined GATT/Classic-security image has a 6676-byte host and 10616-byte runtime
(includes host, ACL, reassembly, channel manager, TX descriptors and packet buffer).
The pre-SDP identification snapshot reported internal free heap 207644 bytes and largest
block 110592 bytes. The current untraced identification snapshot reports 207260
free internal bytes. GATT metadata adds 784 bytes; security/cache metadata adds
1280 bytes, excluding worker/logger stacks and queue allocations. These exclude
unimplemented profiles/audio and do not support an equivalent-feature comparison.
The runtime includes its 104-byte receive-flow state; do not add that state twice.
Latest probe receipt: `_build/hardware/COM6-probe-trial.json`.

Subsequent `-WithGatt` probing added a diagnostic unbonded LE echo/counter service.
The server/bearer metadata adds 784 bytes on ESP32. A direct Linux kernel LE
connection negotiated MTU 517 and read the name. Linux BlueZ then completed
discovery, 200 request writes, 20 periodic readbacks, lengths 0/1/20/64/244/512,
a 512-byte reliable write and 208 received notifications in 29.6 seconds.
LE connected and BR/EDR disconnected were observed separately. Median/max write
RTT was 100.114/300.234 ms; the maximum notification interval was 707.809 ms.
These results do not prove sustained throughput, smooth audio, bonding or coexistence.
The exact image hash is bound to `_build/hardware/linux-gatt-live.json`. This was
an earlier LE-only diagnostic image, not the latest combined security image.

Earlier BlueZ generic/separate-bearer attempts timed out; traces showed no Linux
HCI connection command. Selecting LE through `Device1.PreferredBearer=le` then
`Device1.Connect` succeeded. The experimental API was enabled temporarily only
with zero active Bluetooth connections. Failure receipts/captures remain private
under the fixed `_build` paths.

The owned SDP server subsequently passed 24 live Linux transactions, including
segmented server/diagnostic attributes, and an independent BlueZ sdptool browse.
Only the SDP server and private echo service are described. Its metadata occupies
248 target bytes, with active response bytes allocated separately. The SDP-tested
image's free internal heap snapshot was 207396 bytes. See [SDP evidence](SDP.md).

The later combined diagnostic image enabled restricted Classic SSP pairing and
encrypted Basic echo alongside LE GATT. Linux Just Works pairing passed; target
events separately reported successful native NVS bond commit and encryption with
a 16-byte key. A later probe boot loaded one native bond record. These are limited
Classic security observations, not complete security/profile qualification.
The first harness incorrectly called BlueZ CancelPairing after success, which
removed the new Linux test bond; that call was removed and pairing was repeated.

Both bearers then connected simultaneously, with the Classic socket reporting a
16-byte encryption key. Concurrent traffic failed at Classic sequence 76. The
original mismatch report did not distinguish empty EOF from corrupt data, so the
harness now records actual length/hash and preserves failing bytes. A follow-up
attempt stopped before traffic because the Linux Classic bond was absent again;
this later disappearance is unresolved. With SDP present, Linux confirmed a stored
key and both bearers connected, but a 2048-byte Classic reply had one incorrect
byte at offset 2038. The expected/actual bytes and hashes were retained privately;
107 notifications were received before a later reliable-write timeout. Linux's
Bluetooth daemon did not restart. The next probe adds bounded receive/reply/TX
pattern checks to locate the corruption. A kernel-socket Classic/LE test then
completed 59 Classic echoes and 28 ATT writes before another 2048-byte payload
failed, with two changed bytes starting at offset 1017. Target pattern checks
observed the corruption already in the received/reassembled payload, then unchanged
through reply preparation and HCI submission. Optional whole-packet fingerprints
then matched the already-corrupt controller callback buffer before and after the
host copy, and in the queued owner view. Controlled single-byte fixtures check
that diagnostic distinction. The radio/controller/USB boundary before VHCI is
unresolved; this is not evidence that the owned FIFO changed the byte.

Controller-to-host receive flow is now implemented and accepted by the ESP32.
Its linked controller routine requires nonzero SCO buffer fields in Classic/dual
mode, so the host now supports bounded SCO HCI framing and reserves shared FIFO
capacity for one maximum SCO frame, while the probe still has zero synchronous
connections and no voice profiles. Default host receive capacity is three 1024-byte
ACL packets. The 4096-byte FIFO and 520-byte control reserve are retained.

A traced concurrent run passed 245 Classic echoes (sizes through 2048 bytes),
107 ATT writes, 17 readbacks, a 512-byte reliable write and 108 notifications over
15.586 seconds of overlap. The longer untraced 1000-write attempt failed at Classic
sequence 95 after 51 ATT writes. Classic-only control passed 240 echoes / 159320
bytes with LE observed disconnected at 24 checks. Neither the short pass nor the
new receive-flow setup establishes sustained/audio coexistence. See [receive flow](RX_FLOW.md).

Incoming SSP no longer automatically starts a competing encryption command on
Simple Pairing Complete. Explicit local security demand waits for its actual
authentication completion; demand received during SSP is deferred. Fresh pairing
and verified encrypted channels passed after this change. The Linux harness now
reuses an existing Classic bond: generic Pair on a BR-bonded dual-mode device can
otherwise attempt unsupported BLE pairing and time out.

One 40 MHz diagnostic image failed in SDK flash initialization before app_main;
an untraced control booted, and traced 80 MHz images booted. The probe now matches
the preserved bootloader's configured 80 MHz flash setting. The bootloader and
original application are unchanged; this configuration experiment does not prove
a universal startup fix.

Cleanup re-pairing passed, followed by an encrypted administration request to
delete only the native diagnostic bond. The target reported the bond-removed event
after the NVS commit, then disconnected. The new Linux diagnostic peer was removed;
the original saved speaker peer was preserved.

After this probe, the audio trial was restored to OTA1. Complete image/selector
readback and its readiness marker were verified at 115200 baud. The board is
running `audio_candidate_integer_dsp1` with Bluedroid, not the independent probe.
Receipt: `_build/hardware/COM6-audio-trial.json`; original application remains intact.

Local SDK headers and ROM linker maps already expose standard HCI opcodes and
controller command-table symbols. Focused linked-controller disassembly identified
the receive-buffer validation constraint. No ROM dump/Ghidra database was created;
protocol implementation remains owned code above the retained controller.

## Fixed-path receipts

`_build/host/results.json`, `_build/host/linux_results.json`, `_build/dsp/results.json`,
`_build/dsp/linux_results.json`, `_build/lhdc/results.json`, `_build/linux/latest.json`.
Codec checks: `_build/codecs/aac_results.json`, `_build/codecs/opus_results.json`.
Probe build: `_build/esp32`; audio app: `_build/audio_app`.
Small original-source snapshots: `_build/dsp/baseline` and `_build/lhdc/baseline`.
Remote test sources reuse `~/.cache/native_bt/`; no SDK/media/credentials uploaded.
SSH dependencies are confined to `_build/python_deps`; compiler cache writes are
disabled in build wrappers. Latest generated size ~365 MiB plus ~71 MiB reference.

Latest focus: native encrypted AVDTP media-channel ownership, SBC frame validation,
lazy codec state and an actual asynchronous decoder worker. The finite 48 kHz
stereo RF diagnostic decoded 8000 frames / 4,096,000 PCM bytes with the expected
rolling fingerprint during 166 ATT writes, 16 readbacks and 166 notifications.
No worker drops/resets were recorded and requested worker allocations returned
from 8114 bytes to zero. This is a counter consumer, not I2S/Firefox/acoustic
evidence; the generic large-payload dual-mode failure remains unresolved.
See [SBC pipeline evidence](SBC_PIPELINE.md) for firmware binding and exact scope.
The controller-only
probe defaults to configuring the controller with advertising and Classic scan
disabled. `build_probe.ps1 -WithGatt` opts into diagnostic LE advertising; it is
not the product control/OTA/audio profile. Its default
build no longer depends on the separate LHDC checkout; use `build_probe.ps1 -WithLhdc`
for the optional decoder object build. There is no decoder on the host loop.
`-WithClassicSecurity -TestPeer <address>` explicitly enables restricted Classic
pairing, asynchronous NVS storage and encrypted diagnostic Basic services; it is
not the product audio application.

Next: resolve the dual-mode traffic failure and Linux bond disappearance; BLE SMP
and product GATT integration; SDP client/AVDTP/A2DP/AVRCP; app handoff and live
coexistence. Flash-stall analysis preserving immediate NVS, LHDC A2DP integration,
and remaining codec work are still required. No full-stack speed/memory superiority
or production release readiness is established. See [release gates](RELEASE_READINESS.md).
