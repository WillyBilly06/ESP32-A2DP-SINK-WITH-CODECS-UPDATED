# Independent host release gates

Updated 2026-11-04. **The custom host remains development work and cannot replace
the released Bluedroid application.** Source, local tests, isolated target codec
execution and full phone/RF/product qualification are separate evidence levels.
COM6 is authorized for testing; the released image remains pinned/preserved in
ota0. Only a DAC is attached. Do not touch other reserved boards.

| Required area | Current evidence | Still required |
|---|---|---|
| Independent host | Owned HCI/VHCI, queues, GAP, ACL/credits, L2CAP, SDP server, AVDTP sink, AVRCP control, ATT/GATT server, Classic security and SMP source/tests; full 6.1 product image links without Bluedroid/NimBLE/BTstack host integration. COM6 confirmed selected-peer Classic pairing/encryption/bond reuse, AVRCP metadata, BLE SC bonding, stored-key reconnect and EQ write/fresh readback in checksum mode | CPU 1 idle starvation and streaming drops remain; full protocol/security/lifecycle/fault and sustained RF qualification |
| Profile parity | Sink endpoints, control/metadata/volume, BLE product services and providers exist in source | SDP client/source-role scope, complete AVRCP browsing binding/providers, applicable GATT client/privacy behavior, all phone semantics; see feature inventory |
| Codecs | 37 configurations passed isolated 6.1 ESP32 decoder checks; all families remain compiled. Actual LDAC 96 kHz and SBC 44.1 kHz decoded and reached I2S, but both streaming trials failed with underruns and a descriptor fault | Repair/isolate the common output failure, then qualify every codec/variant, live switching, loss, long streaming and security/coexistence |
| Maximum LHDC | True 192 kHz/S24/stereo/1000 kbps fixture, isolated decode optimization and full-chain diagnostic instrumented | Render pipeline still misses deadline before DMA starts; no full 96-packet/I2S continuity or RF/BLE guarantee |
| Product integration | BLE command/status, LEDs/input/sounds, immediate per-EQ NVS and asynchronous worker/ticket tests | BDK phone flows on the custom host, physical LED/encoder/audio behavior, flash stalls, fault/lifecycle/resource parity |
| OTA/restart | Selector/epoch/quiescence barriers and core-0 timer dispatcher are implemented and host-tested; affinity/build binding checked | Native phone OTA, completion delivery, actual reset, power loss and rollback validation; no reset/OTA pass follows from stubs |
| Resource advantage | Static ABI sizes and image-bound isolated heap/stack/work-span measurements | Equivalent-feature/workload SRAM/CPU/latency comparison; minimum/largest heap, task usage, queue peaks and sustained stress |

Earlier finite SBC/ATT and short Classic/LE observations had explicit failures
in longer runs. They do not establish current 6.1 interoperability or reliability.
Later isolated decoder/kernel passes supersede only their named numerical gates.
Compilation and unit tests are not profile completion or production readiness.

The decoder-on-CPU0 checksum trial passed 11,238 LDAC96 packets with secured BLE
and live EQ, zero drops/resets/watchdogs and queue peak 2/8. The subsequent full
I2S/deferred-PCM run still failed, with CPU0 renderer starvation, underruns and a
descriptor fault. Core placement fixes the observed decoder/transport overload
in the diagnostic workload; it does not qualify the complete audio pipeline.

[Current COM6/Redmagic RF results](BLUETOOTH_RF_2026_11_04.md) retain successful
pairing/link observations and failed streaming measurements separately. The
released firmware returned after the latest trial; playback was paused. Its
phone/app connection was not rechecked after that particular return.

The [current released-feature inventory](IDF61_FEATURE_INVENTORY.md) records
source dependencies and gaps. [6.1 integration](IDF61_RESTART.md), dated codec/
render receipts and the machine-readable records bind claims to their actual
images. Keep the preserved Bluedroid fallback and experimental artifacts distinct.
Full flash/NVS backups and credentials remain local/ignored; never publish them.
