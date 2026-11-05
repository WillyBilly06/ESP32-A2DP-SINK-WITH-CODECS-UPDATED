# BTstack file-to-file audit — in progress

A fresh checkout of https://github.com/bluekitchen/btstack is at D:/BluetoothReferences/btstack-20261105, pinned to upstream HEAD 9edb5d82ea6ec4466a393415d65030929330acdf. This is the same pin as the earlier nine-file reference list. That earlier list was not an exhaustive code audit. BTstack code is not integrated into the owned host or its build dependencies.

The machine-readable [coverage ledger](BTSTACK_FILE_COMPARISON_2026_11_05.json) inventories 277 owned implementation/build inputs (C/header/data/linker files, source generators, platform/product integration and fixed DSP), with source hashes and counterpart candidates. Mapping is many-to-many because the implementations divide responsibilities differently. Files, missing handlers and modules with no reachable product caller are all included. Unreviewed rows remain explicitly pending. This report does not claim that all 277 files have been compared.

## Confirmed first-layer defects

| ID | Owned location | Reference behavior | Change |
|---|---|---|---|
| ACL-001 | dbt_l2cap_receive / dbt_runtime.packet | hci.c acl_handler discards orphaned, oversized or invalid fragments locally | Explicit wire rejections no longer abort the entire host; API/resource faults stay distinct. |
| ACL-002 | dbt_runtime.packet RX-credit completion | hci.c ignores ACL on an unknown handle | Late/unknown-handle RX is discarded without manufacturing a completion for a disconnected link or aborting all profiles. |
| L2CAP-003 | dbt_channels_receive / process_command / runtime.pdu | l2cap.c validates framing and handles errors within signaling/channel logic | Framed malformed Requests enter bounded owner dispatch for Command Reject. Identified malformed Responses are discarded. Invalid framing is counted without global host teardown. |
| ACL-004 | dbt_acl_event | hci.c Role Change handler updates Classic connection state | The already-enabled event now updates the live Classic peer's role, with size/status/value checks and no LE-link changes. |
| RX-005 | dbt_l2cap_receive / release_partial / runtime poll | hci.h retains ACL reassembly inside connection storage | Product binds retained per-link regions before controller enable, uses its ACL slot directly, and expires the table once per owner turn. |
| QUEUE-006 | dbt_queue push/pop | btstack_ring_buffer.c copies contiguous spans | At most two payload memcpy spans replace per-byte masked copies; reserve, tags, sensitive wiping and atomic publication remain. |
| CODEC-007 | dbt_codec_worker_media | Reference media delivery/copy primitives | Encoded payload uses SDK memcpy; metadata and queue admission remain unchanged. |
| AVDTP-008 | dbt_a2dp_acceptor_receive / dbt_avdtp_poll | Corrupt signaling is discarded within AVDTP | Wire integrity rejection and incomplete RX reassembly no longer retire unrelated media. Local-command/mandatory-response deadlines remain fatal. Discover Accept must contain an endpoint. |
| SBC-009 | dbt_sbc_frame_decode | OI backend uses nibble CRC arithmetic | Equivalent 16-entry feedback table replaces per-bit arithmetic; all validation passes remain. |
| BUILD-010 | Five standalone codec components | Working baseline decoder units use -O2 | Restore -O2 for SBC/AAC/LDAC/aptX/Opus; retain LHDC -O3 and the guarded arithmetic profiles. |
| FIR-011 | coefficient_at | Reference resampler and owned FIR have different work | Use exact high-word interpolation under verified bounds; preserve the owned FIR response, precision and history. |

These defects can make an isolated peer/PDU problem terminate unrelated audio and BLE operation. Their presence is proven from code paths; they are not established as the cause of every reported crash or underrun. Initial Classic role knowledge still requires separate policy/review when no Role Change event has occurred.

The signaling change also checks the [Core 6.1 L2CAP specification, sections 4 and 4.1](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host/logical-link-control-and-adaptation-protocol-specification.html). Identified Responses are not answered with new Rejects, and Request rejection retains queue capacity/ownership. Internal invariant failures and exhausted mandatory-reply/resource paths are not silently discarded.

## Missing or unwired integration confirmed

- Automatic outgoing setup: product use of dbt_avdtp_command is limited to Delay Report. Generic command, paging and SDP discovery helpers do not provide a complete product peer-selection/outgoing stream procedure. The latest ELF has no reachable dbt_gap_connect, dbt_sdp_discovery_begin or probe_sdp_discover_audio_source symbol. Speaker-initiated automatic reconnect is an explicit master-goal requirement; the released app itself configured it off.
- AVRCP browsing: dbt_avrcp_browse.c is a compiled source unit but its engine is absent from linked ELF symbols. The product registers the control PSM and explicitly advertises no browsing. Its baseline product exposure is still being checked; the existence of a broader BTstack profile is not proof that a released app feature was lost.

The source and linked-image distinction matters: a file compiling does not establish a usable product feature. BLE/client scope, remaining AVDTP states, all codec-specific framing and every other pending row remain part of this audit.

## Retained architectural choices and remaining work

Controller-to-host acknowledgments are independent of ordinary command credits in both implementations. Known-link packets discarded after wire validation still release their receive buffers. Controller/transport queue loss and invalid internal state remain explicit failures. Our asynchronous/bounded callback requirement is retained instead of adopting the reference ESP32 port's indefinite mutex waits.

The reference audio example/resampler is also structurally different from the owned full FIR/integer-DSP path. Existing measured FIR/render deadline failures remain unresolved; packet-error recovery is not a maximum-rate throughput guarantee. Decoder CPU0 priority21, host CPU1 priority22 and renderer CPU1 priority20 remain selected. Codec settings and input-buffer geometry are unchanged.

Current source coverage is nineteen complete owned C bodies, twelve header contracts, five build files, three source generators and one static-data file reviewed; seven files are partially reviewed and 230 implementation/build inputs remain pending. Every compiled owned translation unit is checked against the catalog, with zero missing units. The initial directory filter wrongly omitted the real Opus scratch folder; those four files and build/generation inputs are now included. Reference section coverage and findings are recorded explicitly in the ledger. A complete HCI/transport event audit and remaining L2CAP modes/state paths are not claimed finished merely because small framing/credit files have been inspected. Next stages cover AVDTP/A2DP, SDP/GAP/security, AVRCP/AVCTP, BLE/ATT/SMP and all product/codec integration paths.

## Checks

Strict C11 Clang static analysis of ACL, L2CAP, runtime and channel manager passes without diagnostics. ESP-IDF 6.1 ccache compilation, source stability and workspace checks pass. The original recovery/counter changes add small metadata fields. The retained RX optimization additionally reserves 16384 bytes, equal to the existing four simultaneous 4096-byte PDU bounds. No task, stack, queue capacity or advertised MTU is added. This increases idle resident memory; it is not a RAM-reduction claim. Whole-image BSS changes from 33788 to 50212 bytes (+16424) relative to the pre-audit image. Startup of the retained RX candidate reports 138112 free internal heap bytes and an active 16384-byte storage binding. The separate l2cap_reassembly UART record exposes delivered/abandoned/rejected/orphaned and signaling counters.

No unit, simulated-peer, packet-injection, playback, pairing or app campaign was invoked during the user's test hold. Built-in codec startup checks, readback and ordinary startup evidence are recorded separately if deployed. This audit remains incomplete and the full goal remains active.

## Retained RX and copy ownership

The normal product must bind its retained buffers before controller enable. Bind failure parks startup; it cannot fall back to per-PDU allocation. The generic explicit provider mode remains available for other callers. Bound release clears only partial metadata; the caller-owned storage lasts for the RX lifetime. Completion, replacement, timeout, link retirement and abort use the same release path. There are no cross-link buffer leases or per-packet allocator calls in the bound path.

The ACL lookup supplies the stable slot index, removing the second link-table search. Runtime expires all slots once before its bounded RX batch, while receive retains a local deadline/generation check. Payload copies use the SDK ROM memcpy implementation. Ring publication still occurs only after payload completion; read publication still follows copy and sensitive-payload wiping. Encoded handoff input is disjoint from worker packet storage. These are bounded CPU copies, not asynchronous DMA operations or timing guarantees.

The quoted four-slot connection-setup loop contains no resource wait. It executes on the host during setup; the actual high-frequency allocation/copy paths above were the optimization target. Remaining allocation paths and every pending layer are still under review.

## AVDTP/A2DP comparison

The complete owned bodies of dbt_avdtp.c, dbt_avdtp_bearer.c, dbt_avdtp_caps.c, dbt_avdtp_sink.c, dbt_a2dp_acceptor.c and the common dbt_codec.c worker have now been read, along with their six public contracts. The ledger names the corresponding BTstack functions/line sections; it does not label entire large reference files or this protocol layer complete. Checked boundaries include signaling/media CID and generation routing, fragment sizes/counts, transaction labels, pinned TX storage, response admission, worker epochs, copied input and bounded PCM retirement.

The [AVDTP 1.3 specification](https://www.bluetooth.org/DocMan/handlers/DownloadDoc.ashx?doc_id=260860), sections 8.3–8.6, confirms fragment ordering/size checks, local corrupt-message discard and a nonempty discovery response. The new recovery change uses the existing discard count to identify wire rejection. It preserves transport/invariant/resource failures and the original local command/response deadlines. An incomplete received command can expire without terminating an already established media session. This adds no persistent storage, task, queue or resource wait. These paths still need peer qualification.

The transport diagnostic hook was also checked: HCI commands return immediately, and payload pattern scans apply only to diagnostic PSM 0x1001. Normal A2DP payloads are not scanned there. This is not an established steady-audio stall source. The common worker retains a copied encoded ring and resumes bounded PCM chunks under backpressure; codec-specific parsers, decoder internals and renderer/FIR scheduling remain open. The BTstack demo is an SBC example, not a reusable framing definition for every vendor codec.

## Codec source and build comparison

The [SBC checksum derivation](SBC_CRC_COMPARISON_2026_11_05.md) retains every packet/frame/backend validation. It changes only the checksum arithmetic and adds 16 constant DROM bytes. Linked frame-validation text grows from 221 to 290 bytes; its CRC byte loop contains no helper calls. This is not a measured throughput claim.

The [codec build-profile comparison](CODEC_BUILD_PROFILE_2026_11_05.md) identifies a missed difference from the working Bluedroid build: the owned SBC/AAC/LDAC/aptX/Opus components selected -Os where their baseline decoders selected -O2. Their standalone components now append -O2. Every unit is checked in compile_commands.json: SBC 16, AAC 19, LDAC 4, aptX 2 and Opus 131. Existing LHDC -O3 remains. Host/platform, codec settings and arithmetic guards remain selected. Relative to the CRC candidate, linked text increases by 20768 bytes and aggregate data by 64 bytes; aggregate data includes flash constants. BSS remains 50212 bytes. Stack/heap and functional consequences require appropriately scoped evidence.

AAC wrapper/transport, its channel-guard generator and build inputs were read against the actual baseline Helix selection and BTstack capability fields. No new crash source was established. Complete vendor/backend internals remain outside that completed wrapper review. LDAC count mask 0x0f matches [AOSP](https://android.googlesource.com/platform/packages/modules/Bluetooth/+/refs/heads/main/system/stack/include/a2dp_vendor_ldac_constants.h); a five-bit count change was not made. Generated LHDC explicitly reports 24-bit audio in a 32-bit container, matching the adapter check. Fragmented LDAC and broader codec variants still require explicit scope/implementation review.

## FIR arithmetic and remaining reset work

The [high-word FIR derivation](FIR_HIGH_WORD_2026_11_05.md) preserves every coefficient, rounding rule, signed 64-bit accumulation, rate and tap count. Generated-bank inspection verifies the narrow multiplication bound, DC sums and accumulation bounds. The active linked code uses SLLI 7 plus MULUH in place of the original two-word product/funnel shift. Filter text is 946 bytes versus 957; whole-image text falls by 12 bytes. Aggregate data and BSS are unchanged. No cycle or underrun improvement is established without timing evidence.

Following the FIR table exposed seven manual DSP/codec-fixture generators omitted from the initial directory filter. They are now cataloged, with unreviewed generators pending. This is a scope correction, not evidence that those files or the full audit passed. The LDAC generator tail replacement was checked against the original file: the setter is the final function, so no following function is discarded. Included SDK backend internals are still a separate incomplete review.

The LDAC sequence-gap reset path remains open: common worker discontinuity handling calls reset_decoder, which invokes SDK close/reinitialization and reallocates the working set. Normal frames with the prevalidated unchanged rate/channel mode do not take the SDK header-change reinitialization path. Same-configuration in-place reset requires complete overlap/configuration/error-state and ownership review before changing it. This recovery cost is confirmed from source; it is not proven to cause steady underruns.

## Current COM6 deployment

Image 7d4b58613694061dec7d8b9b4acfe008d943707257ff85e7977b3bb8d7fcc4be was flashed at 921600 baud to ota1 at 0x390000. Full image and selector readbacks at 230400 match; the original application is unchanged. The owned host stays selected VALID. Startup reports ready with no retained watchdog/backtrace. Decoder CPU0 priority21, host CPU1 priority22 and renderer CPU1 priority20 remain selected.

The retained RX reserve remains 16384 bytes and the encoded decoder queue remains 1800 bytes × 8. The idle snapshot is 138112 free internal bytes, 69740 minimum free and 110592 largest block. This is startup/allocation history, not a streaming RAM comparison. Live UART is restored on source 69. Built-in codec startup checks passed for all 37 configurations. They do not qualify the changed FIR fractional path, peer/error handling or sustained performance. No packet injection, playback or timing campaign ran. Archive: native_bt/_build/hardware/runs/1793915241158933700-7d4b58613694/.
