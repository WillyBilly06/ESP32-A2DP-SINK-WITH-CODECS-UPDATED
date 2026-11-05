# Whole-stack correction pass, 2026-11-05

The custom stack remains development firmware. The user requested source fixes across every A2DP and BLE layer and stopped further functional tests during this pass. Builds and full flash-integrity verification continue. Historical test and RF observations retain their exact image scope.

## Corrections in this pass

- Restore ordinary Classic ACL admission for public SDP, independently of pairing permission. Protected A2DP/AVRCP channels still require verified encryption. The previous known-bond-or-temporary-window rule also rejected ordinary service-discovery connections.
- Restore normal no-PIN Classic pairing and discoverability while the speaker has no encrypted Classic peer. A physical pairing gesture may deliberately open pairing while a peer is present. Each accepted pairing retains an exact-link, generation-bound, 30-second key-store lease. Public audio pairing does not grant BLE control or OTA trust.
- First-time BLE control enrollment is independent of Classic audio bonds. An unprovisioned LE connection can reopen its expired bootstrap window. Automatic bootstrap ends after a durable control-trust record is committed; established control authorization remains bound to the committed SMP identity and incarnation.
- Publish Audio/Video Loudspeaker Class of Device 0x240414. The earlier product initializer left all three octets zero.
- Publish Classic Extended Inquiry Response containing the canonical name and actually installed 16-bit Audio Sink/AVRCP/SPP services. Rename completes only after HCI name, BLE scan response and Classic EIR all complete, preventing a stale Classic discovery name after a BLE rename.
- Do not start a competing local authentication request after a remote IO Capability Response, including when the cached key needs replacement.
- Remove the admission worker's fixed 20 ms wakeup. Crypto/storage completion notifications and real lease deadlines drive progress.
- Verify every Classic bond NVS commit by readback before reporting durable success. Separate HCI security events, connection request/completion, key-store results, visibility state and identity configuration have bounded asynchronous metadata logs. Keys/PINs/passkeys are excluded.
- Upload at 921600; perform full readback at 230400. A first 921600 readback failed its digest check before any write. The split-rate deployment retains full byte comparisons and preserves bootloader/table/recovery/original app.

- Make A2DP/AVRCP/SPP shutdown wait for actual L2CAP close-queue capacity. A full signaling queue previously kept these adapters immediately runnable, including already-disconnecting SPP failures. Remove the AVRCP fixed 20 ms shutdown retry and prevent repeated SPP echo-buffer allocation after an allocation failure. Stop periodic SMP wakeups after a secure session has completed and its worker has retired.

## Layer-by-layer review status

| Layer | Owning implementation | Current review result / remaining work |
|---|---|---|
| Controller / VHCI / HCI | dbt_host, dbt_hci, dbt_queue, probe_main | Controller-only dual-mode initialization and bounded event ownership retained. Controller event/credit recovery and command sequencing remain in scope. |
| GAP discovery / paging / identity | dbt_gap, probe_admission | CoD, EIR, rename, public discovery admission and idle pairing corrections above. Outgoing BR/EDR discovery/paging and speaker-initiated automatic reconnect remain incomplete. |
| Classic security / bonds | dbt_classic_security, probe_security | SSP collision deferral, no legacy PIN, durable readback and separate public/privileged admission. Earlier status 0x05 and later no-request failure are distinct observations; neither is declared resolved on hardware. |
| ACL / L2CAP Basic / ERTM / CoC | dbt_acl, dbt_l2cap, dbt_channels, dbt_ertm*, dbt_credit | Public SDP versus protected PSM boundaries checked. Fragmentation, configuration/security transitions, credits, simultaneous links and teardown remain under detailed review. |
| SDP | dbt_sdp, dbt_sdp_bearer, probe_sdp | Server uses public PSM 1. Audio Sink and AVRCP records checked against actual bindings. Client/service-discovery initiation remains incomplete. |
| AVDTP / A2DP | dbt_avdtp*, dbt_a2dp_acceptor | Sink signaling and media binding exist. All command transitions, CIEs, reconfiguration, delay reporting, transport and resource lifetime remain in scope; source role is not implemented. |
| RTP / encoded media / codecs | codec worker and SBC/AAC/aptX/HD/LL/LDAC/Opus/LHDC adapters | Existing 1800-byte x 8 encoded slots retained. Full framing and variant coverage remains under review; do not infer full codec interoperability from pre-controller decode checks. |
| AVCTP / AVRCP | dbt_avctp*, dbt_avrcp*, probe_avrcp | Automatic control connection already requires encrypted authorized links. No false browsing advertisement found; browsing profile binding/providers remain incomplete. |
| LE GAP / SMP / persistent identity | dbt_gap, dbt_smp*, probe_smp | Independent unprovisioned onboarding correction above. SC, key distribution, reconnect/privacy, mandatory replies and cancellation remain in scope. |
| ATT / GATT | dbt_att*, probe_gatt | Baseline service/command/status/meter UUIDs retained. Every opcode, permission, CCCD, MTU, asynchronous completion and reconnect path remains in scope. |
| App protocol / persistence / OTA | dbt_ble_control, product/NVS/OTA providers | Preserve each accepted EQ commit, exact command bytes, sound/file transfers and OTA transfer/restart ownership. Functional qualification deferred at the user's request. |
| PCM / DSP / clocks / I2S | dbt_audio, dbt_resampler, fixed_dsp, probe_output | Integer DSP and all controls retained. Sustained maximum LHDC still has a documented performance/continuity failure; it is not fixed by the discovery changes. Original buffer/BTA/BTC and added FIR/pacing costs remain part of the audit. |
| Peripherals / sounds / lifecycle | product IO, sound, input, LED and restart providers | Preserve all original mappings and providers. Only the DAC is connected; user waived internet recovery testing. |

Reference source is pinned in STACK_REVIEW_REFERENCE_2026_11_05.json. It is used for behavior/specification comparison, never as an integrated host implementation. The released application inventory remains IDF61_FEATURE_INVENTORY.md, pinned to 0d85a975ec079d954f1bdd3e94d2fed70848c252.
