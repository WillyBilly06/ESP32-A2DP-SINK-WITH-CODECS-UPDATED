# Independent host and product feature matrix

Updated 2026-10-29. **The goal remains active and incomplete.** The user now
requires all further testing to be local and off-board, with a flashable ESP32
artifact; the ESP32 is reserved for other work. The active
[`HARDWARE_HOLD.json`](../_build/hardware/HARDWARE_HOLD.json) forbids further
flash, reset, serial reconnect or Bluetooth connections until the user releases
the hold. No new device result is implied by a local build.

The target is the existing dual-mode ESP32 controller plus an independently
authored host. BTstack is a pinned reference only. `bt_audio_sink/` is the
preserved Bluedroid baseline, not the owned-host product. A source or host-test
pass does not imply that a complete firmware linked, booted, or interoperated.

Evidence terms: **I** implemented in owned source; **H** a stated host test;
**L** target-linked in the specified image; **O** finite hardware observation;
**Gap** incomplete or unqualified. `L06` below means that symbols or codec
execution were present in the failed image with SHA-256
`06ea0f6e4a2859d64108caa235f505b70b64e152fcabbddc6ebe2ab554774085`, ELF
`7729026d18419445e8e28c329d37792070d3b5c4235158def13f603beeec66e7`.
The ELF symbol inventory does not prove the feature ran. Earlier hardware
observations are explicitly separate images. A current local candidate being
built must be identified and verified by its own receipt.

## Controller and protocol coverage

The retained original ESP32 controller provides BR/EDR and LE 4.2 operation;
the owned host must use the capabilities actually reported by that controller.
The earlier identification reported separate Classic and LE ACL pools
([architecture](architecture.md), [probe receipt](../_build/hardware/COM6-probe-trial.json)).
LE 2M/coded PHY and ISO/CIS/BIS/LE Audio are outside this controller's
capabilities; software alone cannot add those radio functions. Other host
profiles below are not automatically excluded merely because they are absent.

| Feature / role | Implemented and host-tested | Target-linked / observed | Remaining gap |
| --- | --- | --- | --- |
| Controller HCI, owner loop, bounded ingress, commands, ACL credits and RX flow | I/H: queues, command lifetime/cancellation, generations, HCI and ACL/runtime tests; [host receipt](../_build/host/results.json), [cancel receipt](../_build/host_cancel/results.json) | L06; earlier controller setup and separate ACL pools observed | Recovery, loss/credit stress and current-image RF qualification; callback/command admission is not peer receipt |
| GAP Classic and LE | I/H: scan/advertise controls, link policy and selected connection procedures | Earlier Classic and LE diagnostic links observed | Full discovery/initiator procedures, local privacy/address rotation and all supported controller features still need implementation or qualification |
| Classic security, keys, NVS | I/H: SSP/PIN policy, encrypted-channel admission and async key store | L06; earlier Just Works, local commit/reload and 16-byte key on an encrypted channel observed | Other pairing methods and reconnects; previously observed Linux bond disappearance remains unresolved |
| LE SMP, encryption, bonds and peer privacy | I/H: optional session/bearer, crypto/NVS worker, reconnect and limited peer-RPA logic; [SMP scope](SMP.md) | L06 (`probe_smp_init` symbol); failed self-test stopped before controller enable, so no SMP execution or radio observation in that image | No physical LE pairing/reconnect proof; OOB/UI and local privacy incomplete; distribution requiring final peer acknowledgment lacks a documented controller provider; zero-distribution SC does not prove IRK/CSRK or peer persistence |
| L2CAP Basic | I/H: Classic setup/configuration, security, reassembly and teardown | L06; earlier encrypted Basic exchanges observed | External interoperability and sustained stress; prior concurrent payload corruption remains unresolved |
| L2CAP LE CoC | I/H: credit channels, segmentation and security guards ([credit channels](CREDIT_CHANNELS.md)) | L06 engine symbols; no external CoC session observed | Peer interoperability, RF and load qualification; enhanced credit mode not established |
| L2CAP ERTM | I/H: negotiated required mode, FCS, lazy windows, paired simulated runtime and recovery; [feature receipt](../_build/ertm/features/results.json) | L06 ERTM engine symbols; no channel/RF observation | No Basic fallback for required ERTM; continuation/reconfiguration rejected; external peer and live loan/timer behavior unqualified. Streaming mode is not implemented |
| SCO/eSCO and voice | I/H: bounded HCI SCO framing/reservation | Earlier controller buffer setup accepted | SCO/eSCO connection management, synchronous routing/flow, codecs and voice profiles are incomplete; framing is not voice support |
| ATT/GATT server | I/H: discovery, reads/writes, prepared writes, MTU, CCCDs, notification/indication; diagnostic service | L06 diagnostic GATT symbols; earlier LE diagnostic writes/notifications observed | Product service migration and security binding; no GATT client, signed-write/robust-cache coverage or EATT qualification |
| SDP | I/H: owned server, continuation and records | L06 probe code; earlier Linux queries and `sdptool` browse observed | Client and complete product records/interoperability |
| AVDTP / A2DP sink | I/H: signaling, sink/media manager and codec worker/endpoints in optional owned probe | L06 audio init symbol; earlier separate SBC probe decoded/produced I2S during bounded ATT traffic | Full codec interoperability, externally negotiated streams, discontinuities, delay accuracy, switching and product handoff remain unqualified |
| A2DP source and live audio encoding | No owned source profile or live encoder path identified; some SDK codec sources and offline fixture encoders can encode | Test encoders are host-only evidence, not an advertised source | Source-role negotiation, audio input, encoder lifetime and peer interoperability if required by the full profile goal |
| AVCTP / AVRCP control | I/H: Basic control target/controller subset, PSM 0x17 binding and SDP records, metadata/volume/pass-through tests ([probe](AVRCP_PROBE.md)) | L06 `probe_avrcp_init` symbol; no execution after failed check and no native AVRCP RF proof | Full phone interoperability, control semantics, product UI and application providers. No claim for unadvertised player features |
| AVRCP browsing | I/H: portable protocol groundwork only | No browsing PSM 0x1b binding or advertisement | Full ERTM profile binding, player/filesystem providers, conformance and peer testing |
| RFCOMM / SPP / serial | No owned implementation identified | None | Multiplexing, credits, framing, security, server/client integration and interoperability |
| HFP / HSP / telephony | No owned HFP/HSP implementation identified | None | RFCOMM, profile control, SCO/eSCO ownership, voice routing and interoperability |
| HID, PAN/BNEP, OBEX/GOEP, PBAP and other applicable Classic roles | No owned profile implementations identified | None | Implement and qualify required roles; scope and dependencies must be decided explicitly, not assumed from controller support |
| Other LE roles/profiles and Mesh | Selected peripheral server behavior only | Earlier diagnostic peripheral observed | GATT client/central application and any required profile/Mesh semantics and security need implementation and verification |

## Audio and codec coverage

The all-codec host self-test receipt is
[`probe_check_host/results.json`](../_build/native_codecs/probe_check_host/results.json):
37 configurations, including 48 Opus packets, pass on host; its task-stack and
timer values are stubs. Its recorded `probe_gatt.h` hash currently differs from
the tree, so it is not a fully source-matched receipt; that header is tracked
by the broad harness even though this check does not exercise GATT. Rerun for
the final candidate. Real offline codec fixtures and separate port tests have
their own receipts below. Target-linked codec status is specific to L06, not
proof of decoding live peer media.

| Codec | Implementation / host evidence | Target and remaining gap |
| --- | --- | --- |
| SBC | I/H: real SBC frames, lazy worker, fragmentation and switch tests; [SBC pipeline](SBC_PIPELINE.md) | L06; one pre-controller 48 kHz stereo self-test passed exact PCM. Earlier different probe performed finite RF/I2S test. Complete configurations, interoperability, acoustic and sustained tests remain |
| AAC-LC | I/H: 44.1/48 kHz mono/stereo real encoded frames; [receipt](../_build/native_codecs/aac_real_results.json) | L06; all four initial target comparison cases failed. Local target-arithmetic oracle correction is not physically confirmed; transport and live playback remain unqualified |
| aptX / HD / LL | I/H: standard, HD and LL at 44.1/48 kHz versus SDK reference; [receipt](../_build/native_codecs/aptx_real_results.json) | L06; six initial pre-controller exact cases passed. LL uses the standard decode path for a constrained CIE; latency protocol and real peers remain unqualified |
| Android Opus / Opus 05 | I/H: guarded lazy scratch, PCM formats and exact host arithmetic-profile references; [oracle](OPUS_PCM_ORACLE.md) | L06; initial cases 0/1 passed, case 2 failed old host-profile hash and the remaining 13 did not run. Slow64 profile fix is source/host tested; target hash and PCM remain unconfirmed |
| LDAC | I/H: real AOSP-encoder frames, actual worker vs SDK decoder and finite tone correlation; [receipt](../_build/native_codecs/ldac_encoded_results.json) | L06; six initial sparse-sample cases passed. Fragmented transport is rejected; live device/peer playback and broad codec qualification remain |
| LHDC V5 | I/H: valid encoded 16/24-bit tone fixtures at four rates, lifecycle and fragment tests; [receipt](../_build/native_codecs/lhdcv5_encoded_results.json) | L06; four initial sparse-sample cases passed. 32-bit has lifecycle coverage only, is not advertised by the current probe CIE, and lacks positive encoded PCM proof. Music/peer playback and full switch/heap qualification remain |

The L06 self-test used one 8,192-byte task. Its reported cumulative minimum
stack was 4,128 bytes, tracked allocation peak 63,672 bytes, and minimum
free-heap snapshot after an allocation 149,824 bytes. These are measurements
of that finite test and allocator, not production decoder task headroom,
continuous playback, complete-call-chain stack proof, or equivalent-feature
memory/CPU/latency benchmarks.

## Baseline product functionality still requiring owned integration

| Function in `bt_audio_sink/` baseline | Owned progress | Remaining gap |
| --- | --- | --- |
| Product BLE command/status/meter service | I/H/L: current local candidate binds baseline UUIDs and protocol, bounded workers, CCCD-respecting status/meter, startup settings and EQ/control providers alongside diagnostic echo; real ATT/NVS-shim sanitizer tests pass | Remaining command providers, bonded CCCD durability, complete authorization policy, phone interoperability and real timing; no device validation of this binding |
| EQ, presets, bass boost, channel flip, bypass, volume and settings | I/H/L: BLE EQ/control now binds immediate per-command NVS commits and typed DSP publication, preserving independent volume/analysis. Existing settings load without migration | Presets/remaining product integration and physical power-cut/flash-stall/acoustic qualification. **Every EQ command must commit immediately during playback**, without debounce or coalescing; local shims do not establish durability or audible behavior |
| OTA and recovery | Baseline implements BLE OTA and recovery; owned command core delegates to an explicit provider | Implement safe product OTA provider, image validation/write/abort/status, authorization, rollback and recovery integration; diagnostic service is not OTA |
| LED matrix, brightness/effects, meter/beat visualization | Baseline implementation and settings exist | Owned product providers, control/meter synchronization, resource limits and target behavior remain unintegrated |
| Rotary encoders and buttons | Baseline binds volume, playback, track, EQ, brightness, pairing and effects | Bind physical input, debounce, profile actions and settings to the owned product; target behavior unverified |
| Startup/connect/disconnect/max-volume sounds, uploads and mute/delete | Baseline sound/overlay, storage and BLE upload paths exist | Owned providers, storage lifecycle, mixing/ducking, error handling and audio transitions remain unintegrated |
| Audio rendering, I2S, DSP and timing | Owned experimental render/I2S and DSP candidates; numerical and worker tests; [audio output](AUDIO_OUTPUT.md) | Product wiring, measured clock behavior, full sample formats, audible output and continuous playback during commits remain unqualified |
| Pairing UI, discoverability, peer/bond lifecycle and device naming | Pieces exist in probe and baseline; current local candidate loads saved name for GAP/status | Complete product policy, user consent/UI, secure identity, persistence, reset/recovery and cross-feature interaction |

## Preserved observation and next gates

[`COM6-owned-first-qualification.json`](../_build/hardware/COM6-owned-first-qualification.json)
records full readback of image L06. Its companion
[`COM6-owned-first-qualification.txt`](../_build/hardware/COM6-owned-first-qualification.txt)
records **19/37 selected configurations passed**, five comparison failures
(Opus case 2 and four AAC cases), and 13 later Opus cases not run. SBC, six aptX,
six LDAC and four LHDC cases passed, in addition to Opus cases 0/1. Controller
enable was stopped. These are finite on-device decode-check observations, not
Bluetooth playback or current-device-state evidence. The Opus profile and AAC
oracle corrections were made afterward and require separate confirmation; the
hardware hold prohibits collecting it now.

Earlier records include a finite SBC/ATT diagnostic and a 22.119-second
Firefox/ATT overlap with zero recorded decode/output counter faults, but the
overall Firefox run failed on BAD_PACKET/disconnect handling. A longer untraced
Classic/LE diagnostic also failed. No acoustic, continuous-playback or general
coexistence result follows from those partial observations. The historical
[offline evidence](OFFLINE_EVIDENCE.json) reports 166 Windows/Linux groups for
an earlier source snapshot; its source hashes must be checked before applying
that receipt to the changed tree.

To complete the goal, finish the missing protocol and product providers; run
source-matched local tests and produce a verified flashable artifact; and, only
after the user explicitly releases the hardware hold, qualify security,
interoperability, audio, power-loss persistence, continuous Classic/LE
coexistence and codec switching. Compare memory, CPU and latency against the
baseline with matching enabled features and workloads. There is currently no
evidence to claim faster, smaller, stable or equivalent production behavior.

Current local candidate: see [LOCAL_CANDIDATE_EVIDENCE.json](LOCAL_CANDIDATE_EVIDENCE.json), image `4ac970772b158c3e80e9dd9d19f076ed456b383eda793e54f6664296ef313092`. This adds linked product-control wiring after L06; it is not a new device result.
