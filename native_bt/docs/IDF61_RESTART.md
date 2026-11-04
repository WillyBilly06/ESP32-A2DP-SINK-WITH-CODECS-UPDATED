# Current owned-host integration: ESP-IDF 6.1

The [complete master objective](MASTER_GOAL_2026_11_04.md) remains active and
unachieved. This is an independently authored HCI/VHCI host project. The
released Bluedroid application remains the working fallback and is not retired.

## Provenance and current evidence

- Baseline: released `1.0.4-idf6.1`, commit `0d85a975ec079d954f1bdd3e94d2fed70848c252`.
- Owned implementation recovered from `development`, commit
  `f313fce94220a21066375e97b0d266d92418f96a`; the original dirty development
  checkout was preserved. No BTstack implementation was imported.
- Current branch: `codex/owned-bt-idf61`. ESP-IDF and its controller are the
  bundled stable 6.1 sources; Bluedroid/NimBLE hosts are disabled in the probe.
- The older `FEATURE_MATRIX.md`, `PROGRESS.md` and hardware receipts are
  historical evidence. Their old hardware reservation was explicitly released
  for COM6 in this chat. No hardware has been accessed for this checkpoint.
- Hardware available: original ESP32 revision 3.1, 8 MB flash, no PSRAM,
  I2S DAC and Redmagic NX809J. No LED matrix/encoders are attached. The earlier
  request to omit WiFi recovery hardware testing remains recorded; its working
  source/build path is retained.

Fresh local results:

| Check | Evidence and scope |
|---|---|
| Portable host | 168/168, source-hashed receipt at `_build/host/results.json`; includes 20,000 concurrent packets and 16,384 frames across three audio owners |
| ERTM | Wire/engine and extended audit pass; 67,072 enhanced-control combinations in the wire test, simulated loss/busy/window cases; no external peer proof |
| AVRCP | 24 control, metadata, continuation, browsing-wire and lifecycle groups pass; browsing application/ERTM binding is still incomplete |
| SMP core | Independent AES/CMAC/P-256 protocol vectors, simulated feature exchange/distribution/reconnect tests pass; no current target pairing proof |
| Product BLE | UUIDs, CCCDs, bounded worker admission, immediate per-EQ NVS and stale generations pass using NVS shims |
| RFCOMM | 19 wire/mux/DLC/bearer/echo groups pass locally; external conformance is unqualified |
| New PSA adapter | Actual SDK TF-PSA software provider passes AES/CMAC/SHA-256 vectors, two-party P-256 SC derivation/commitments/f5/f6, invalid points, replay rejection and repeated key cleanup on the host |
| Storage/OTA adapter | NVS reconnect and OTA flash/selector model tests pass; target source compilation passes; physical power-loss/selector behavior is unqualified |
| Full 6.1 probe | All six codec families, SMP/GATT, AVRCP, product IO/security and RFCOMM compile/link. Source stability and forbidden-host dependency checks pass. This is an off-board result |

The probe has not booted or decoded on ESP32 in this work. The historical
19/37 target codec result is not superseded by a host pass or a new link.

## Audit and RAM budget

`python native_bt/tools/audit_idf61.py` inventories current C/C++/header inputs,
records source hashes and lexical review leads, and compiles `sizeof` values
with the actual ESP32 toolchain. The [audit JSON](IDF61_AUDIT.json) inventories
375 files. A source inventory is not a completed manual correctness audit.

Measured ESP32 ABI metadata:

| Object | Bytes | Budget meaning |
|---|---:|---|
| Ingress queue including metadata | 4,112 | 4,096 payload capacity, 1,029 max H4 packet, 520-byte event reserve |
| Host including ingress | 6,696 | Not an additional allocation on top of the runtime |
| Runtime including host | 11,048 | Shared owner, ACL and signaling metadata; maximum four software link records |
| ATT server / connection | 728 / 168 | Active allocations/attribute values are separate |
| ERTM engine | 192 | Lazy window, pinned packets and reassembly storage are separate |
| Codec decoder metadata | 68 | Only the active codec's working set is allocated |
| Audio including fixed-point DSP | 2,456 | PCM rings and DMA allocations are separate |
| Secure Connections session | 1,484 | Active crypto provider and key storage are separate |

Linked section totals from the initial full 6.1 build:

| Static subtotal | Released baseline | Owned probe |
|---|---:|---:|
| DRAM data + BSS | 62,724 | 54,813 |
| IRAM vectors + text | 115,071 | 124,227 |

These builds are not feature-equivalent and are not a runtime memory comparison.
The owned image currently consumes more static IRAM. No RAM or performance
advantage is established. Runtime free/minimum/largest-block, task high-water
marks, transient codec-switch peaks and fragmentation remain mandatory gates.

## Architecture retained and changes made

1. A controller callback copies at most one bounded H4 packet, records overflow
   as a terminal fault and wakes the host owner. It does not decode or run crypto.
2. One host owner advances protocol state with explicit RX/TX budgets and sleeps
   on notifications/deadlines. Controller readiness and HCI/ACL credits remain
   separate admission conditions. The single pending SMP-failure mailbox now
   uses one bounded conditional per owner iteration.
3. A bounded media queue transfers ownership to the active decoder worker.
   Decoder/output backpressure must retain exact packet/PCM lifetime; this still
   needs review and stress qualification at maximum LHDC configuration.
4. Render-owned integer DSP and an asynchronous I2S/DMA provider are separate
   from host and storage work. The existing working peripheral/UI behavior is the
   parity reference; approximate owned LED effects are not accepted as visual parity.
5. Storage and crypto workers publish generation-bound results. Every accepted
   EQ change retains its individual immediate commit; no debounce/coalescing is
   introduced. Queue-result reservations and flash-stall bounds need further audit.
6. The 6.1 port uses public PSA AES, CMAC, SHA-256 and P-256 operations. Keys are
   volatile and worker-owned. Legacy reference tests retain a separate Mbed TLS
   branch; the target uses no removed/private crypto APIs in owned code.
7. Codec source pins normalize CRLF to LF before validation, preserving the exact
   canonical pinned source. Other edits remain rejected. Ccache is explicitly
   enabled, and the LHDC source directory is an explicit build argument. The
   1 kHz baseline tick and rollback support are retained; 2 ms event waits
   clamp to at least one tick, with output-format tests at 100/1000 Hz.

## Remaining gates and next work

Updated target evidence: [6.1 codec checkpoint](IDF61_TARGET_CODECS_2026_11_04.md).
All 37 existing codec configurations now pass on COM6. A real 192 kHz/S24/1000
kbps fixture passes sparse PCM checks but misses 35/84 isolated 5 ms deadlines
(maximum 5.190 ms, including PCM validation). This is an open performance gate,
not production or coexistence qualification. Both runs returned to the released
Bluedroid baseline. The initial checkpoint JSON above remains a historical
off-board build record; the target-codec checkpoint records these later runs.

- Finish manual callback/owner bounds and queue saturation/teardown review;
  lexical loop counts do not establish that execution is bounded.
- Replace remaining blocking legacy output-control wrappers and startup
  completion polling before production. The nonzero event-wait fix does not
  turn those control wrappers into asynchronous operations.
- Audit live product parity against the released 6.1 application: source-role
  behavior, SDP client, AVRCP browsing/application providers, BLE security/UI,
  privacy, setting/name restoration, LEDs, inputs, sounds, OTA and recovery.
- Extend target codec qualification before enabling radio. Highest priority:
  real LHDC V5 192 kHz / 24-bit / 1,000 kbps frames, cycle counts by stage,
  deadline distribution and measured spare capacity. No quality reduction is allowed.
- Establish matching baseline instrumentation for controller/ACL/RTP timing,
  queue occupancy, CPU, decoder starvation, PCM/DMA underruns and BLE/AVRCP latency.
- Qualify maximum-codec coexistence, immediate flash commits, rapid EQ controls,
  connection/pairing churn, fault injection, power loss and sustained playback.
- Keep the Bluedroid default until the complete required replacement passes
  functional, security, lifecycle, resource and stress gates.

Reference contracts: [ESP-IDF 6.1 VHCI](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32/api-reference/bluetooth/controller_vhci.html)
and [PSA migration](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32/migration-guides/release-6.x/6.0/security.html).
