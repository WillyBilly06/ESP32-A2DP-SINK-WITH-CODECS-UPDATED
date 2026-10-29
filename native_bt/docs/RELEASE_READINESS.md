# Independent host release gates

Updated 2026-10-28. **Development milestone; replacement firmware is not ready to ship.**

The existing audio application uses Bluedroid. The independent probe has a diagnostic
LE GATT service when explicitly enabled, but no product audio/control/OTA profiles.
Keep these deliverables distinct in GitHub branches, binaries,
release notes and test reports. A successful lower-layer probe does not migrate
the product application.

| Required gate | Evidence now | Remaining work |
| --- | --- | --- |
| Independently authored source | Owned C11 core; explicit source lists exclude BTstack/Bluedroid/NimBLE from probe | Continue provenance/dependency review for each new layer |
| HCI ownership and command lifecycle | Copied bounded ingress; general command queue; asynchronous transport lifetime; receive credits/acknowledgments and SCO framing tests | Production recovery, synchronous connection/flow ownership and broader target qualification |
| Links and Basic L2CAP | Paired peers establish/configure/transfer fragmented data/disconnect through actual runtime APIs | External peer interoperability, traffic limits and target soak |
| Classic + BLE scheduling | Separate/shared credit accounting; accepted receive-flow setup; a short concurrent Classic/ATT diagnostic run passed | Longer untraced workload failed at Classic sequence 95; sustained traffic and audio coexistence remain unqualified |
| Controller setup/GAP | Nine-command configuration; explicit scan/advertising control; negative security replies | Outgoing procedures, discovery, privacy and procedure recovery |
| Pairing and bonds | Owned Classic SSP/PIN/passkey/OOB policy and asynchronous key store/delete; live Just Works, native commit/reload and 16-byte encrypted channel observed | Resolve later Linux test-bond disappearance; qualify other Classic methods/reconnects. LE SMP, secure connections, privacy and BLE bonds remain pending |
| BLE application | Owned ATT server/bearer: discovery, MTU, reads/writes, queued writes, permissions, per-client CCCDs and notification/indication handling; diagnostic live reads/writes/notifications passed | Product control/meter/upload/OTA migration, real security/bond integration, GATT client and optional advanced features |
| Classic audio discovery/control | Owned SDP server; Linux queries and BlueZ sdptool discovery pass for the private diagnostic service. AVDTP signaling/sink lifecycle/SBC selection and paired media-channel route host-tested with a simulated worker; ESP32 archive compiled | SDP client, actual audio service records, production media manager and decoder worker, other codec negotiation and AVCTP/AVRCP; AVDTP RF/decoded playback remains untested |
| Audio codec parity | Separate Bluedroid AAC/Opus memory changes and DSP tests | All codec configurations, LHDC V5 A2DP integration, lazy switch peaks and allocation failures on ESP32 |
| EQ persistence and playback | Immediate packed NVS commit; integer ramps; host tests | Power-loss persistence and physical continuous audio during repeated real flash commits |
| Interoperability/audio | Diagnostic Linux LE ATT/GATT run passed; no qualifying playback/coexistence run | Firefox music from Linux, verified simultaneous Classic+LE bearers, continuous reads/writes/notifications at every negotiated codec/configuration |
| Feature coverage | Explicit architecture inventory | ERTM/LE credit channels and other requested profiles/roles; reject unsupported modes accurately |
| Resource advantage | Bounded disk outputs; reported object allocations | Compare RAM/CPU/latency against equivalent enabled features and workloads, including switch peaks |

Test receipts with source hashes are generated under `_build/host/`. Keep private
UART/flash/SSH artifacts under ignored `_build/`; publish only reviewed source and
sanitized evidence summaries. Do not upload the original flash backup or credentials.

The deadline does not turn an unimplemented gate into a pass. An early GitHub
checkpoint must say development/incomplete and must not replace a stable release.
