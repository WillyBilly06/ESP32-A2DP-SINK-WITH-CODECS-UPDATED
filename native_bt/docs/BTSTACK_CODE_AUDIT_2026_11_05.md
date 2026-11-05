# BTstack file-to-file audit — in progress

A fresh checkout of https://github.com/bluekitchen/btstack is at D:/BluetoothReferences/btstack-20261105, pinned to upstream HEAD 9edb5d82ea6ec4466a393415d65030929330acdf. This is the same pin as the earlier nine-file reference list. That earlier list was not an exhaustive code audit. BTstack code is not integrated into the owned host or its build dependencies.

The machine-readable [coverage ledger](BTSTACK_FILE_COMPARISON_2026_11_05.json) inventories 270 owned implementation/build inputs (C/header/data/linker files, source generators, platform/product integration and fixed DSP), with source hashes and counterpart candidates. Mapping is many-to-many because the implementations divide responsibilities differently. Files, missing handlers and modules with no reachable product caller are all included. Unreviewed rows remain explicitly pending. This report does not claim that all 270 files have been compared.

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

These defects can make an isolated peer/PDU problem terminate unrelated audio and BLE operation. Their presence is proven from code paths; they are not established as the cause of every reported crash or underrun. Initial Classic role knowledge still requires separate policy/review when no Role Change event has occurred.

The signaling change also checks the [Core 6.1 L2CAP specification, sections 4 and 4.1](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host/logical-link-control-and-adaptation-protocol-specification.html). Identified Responses are not answered with new Rejects, and Request rejection retains queue capacity/ownership. Internal invariant failures and exhausted mandatory-reply/resource paths are not silently discarded.

## Missing or unwired integration confirmed

- Automatic outgoing setup: product use of dbt_avdtp_command is limited to Delay Report. Generic command, paging and SDP discovery helpers do not provide a complete product peer-selection/outgoing stream procedure. The latest ELF has no reachable dbt_gap_connect, dbt_sdp_discovery_begin or probe_sdp_discover_audio_source symbol. Speaker-initiated automatic reconnect is an explicit master-goal requirement; the released app itself configured it off.
- AVRCP browsing: dbt_avrcp_browse.c is a compiled source unit but its engine is absent from linked ELF symbols. The product registers the control PSM and explicitly advertises no browsing. Its baseline product exposure is still being checked; the existence of a broader BTstack profile is not proof that a released app feature was lost.

The source and linked-image distinction matters: a file compiling does not establish a usable product feature. BLE/client scope, remaining AVDTP states, all codec-specific framing and every other pending row remain part of this audit.

## Retained architectural choices and remaining work

Controller-to-host acknowledgments are independent of ordinary command credits in both implementations. Known-link packets discarded after wire validation still release their receive buffers. Controller/transport queue loss and invalid internal state remain explicit failures. Our asynchronous/bounded callback requirement is retained instead of adopting the reference ESP32 port's indefinite mutex waits.

The reference audio example/resampler is also structurally different from the owned full FIR/integer-DSP path. Existing measured FIR/render deadline failures remain unresolved; packet-error recovery is not a maximum-rate throughput guarantee. Decoder CPU0 priority21, host CPU1 priority22 and renderer CPU1 priority20 remain selected. Codec settings and input-buffer geometry are unchanged.

Current source coverage is five complete owned C bodies reviewed, four files partially reviewed, and 261 implementation/build inputs pending. Every compiled owned translation unit is checked against the catalog, with zero missing units. The initial directory filter wrongly omitted the real Opus scratch folder; those four files and build/generation inputs are now included. Reference section coverage and findings are recorded explicitly in the ledger. A complete HCI/transport event audit and remaining L2CAP modes/state paths are not claimed finished merely because small framing/credit files have been inspected. Next stages cover AVDTP/A2DP, SDP/GAP/security, AVRCP/AVCTP, BLE/ATT/SMP and all product/codec integration paths.

## Checks

Strict C11 Clang static analysis of ACL, L2CAP, runtime and channel manager passes without diagnostics. ESP-IDF 6.1 ccache compilation, source stability and workspace checks pass. The original recovery/counter changes add small metadata fields. The retained RX optimization additionally reserves 16384 bytes, equal to the existing four simultaneous 4096-byte PDU bounds. No task, stack, queue capacity or advertised MTU is added. This increases idle resident memory; it is not a RAM-reduction claim. Whole-image BSS changes from 33788 to 50212 bytes (+16424) relative to the pre-audit image. Startup of the retained RX candidate reports 138112 free internal heap bytes and an active 16384-byte storage binding. The separate l2cap_reassembly UART record exposes delivered/abandoned/rejected/orphaned and signaling counters.

No unit, simulated-peer, packet-injection, playback, pairing or app campaign was invoked during the user's test hold. Built-in codec startup checks, readback and ordinary startup evidence are recorded separately if deployed. This audit remains incomplete and the full goal remains active.

## Retained RX and copy ownership

The normal product must bind its retained buffers before controller enable. Bind failure parks startup; it cannot fall back to per-PDU allocation. The generic explicit provider mode remains available for other callers. Bound release clears only partial metadata; the caller-owned storage lasts for the RX lifetime. Completion, replacement, timeout, link retirement and abort use the same release path. There are no cross-link buffer leases or per-packet allocator calls in the bound path.

The ACL lookup supplies the stable slot index, removing the second link-table search. Runtime expires all slots once before its bounded RX batch, while receive retains a local deadline/generation check. Payload copies use the SDK ROM memcpy implementation. Ring publication still occurs only after payload completion; read publication still follows copy and sensitive-payload wiping. Encoded handoff input is disjoint from worker packet storage. These are bounded CPU copies, not asynchronous DMA operations or timing guarantees.

The quoted four-slot connection-setup loop contains no resource wait. It executes on the host during setup; the actual high-frequency allocation/copy paths above were the optimization target. Remaining allocation paths and every pending layer are still under review.

## Current COM6 deployment

Image 1c6210a0610df3e61dfc357e33677341137b07db94b8456d77c9a7d220bc6f30 was flashed at 921600 baud to ota1 at 0x390000. Full image and selector readbacks at 230400 match; the original application is unchanged. The owned host stays selected VALID. Startup reports ready with no retained watchdog/backtrace. Decoder CPU0 priority21, host CPU1 priority22 and renderer CPU1 priority20 remain selected.

The l2cap_reassembly record confirms 16384 retained storage bytes. The idle snapshot is 138112 free internal bytes, 69740 minimum free and 110592 largest block. Minimum-free includes startup/allocation history; this is not a streaming RAM comparison. Live UART is restored on source 61. Built-in codec startup checks passed, but they do not qualify the changed peer/error/fragment paths. No packet injection or playback campaign ran. Archive: native_bt/_build/hardware/runs/1793911513390481700-1c6210a0610d/.
