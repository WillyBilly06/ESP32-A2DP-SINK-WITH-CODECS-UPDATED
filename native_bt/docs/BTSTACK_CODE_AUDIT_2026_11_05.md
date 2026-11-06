# BTstack file-to-file audit — in progress

A fresh checkout of https://github.com/bluekitchen/btstack is at D:/BluetoothReferences/btstack-20261105, pinned to upstream HEAD 9edb5d82ea6ec4466a393415d65030929330acdf. This is the same pin as the earlier nine-file reference list. That earlier list was not an exhaustive code audit. BTstack code is not integrated into the owned host or its build dependencies.

The machine-readable [coverage ledger](BTSTACK_FILE_COMPARISON_2026_11_05.json) inventories 287 owned implementation/build inputs (C/header/data/linker files, source generators, platform/product integration and fixed DSP), with source hashes and counterpart candidates. Mapping is many-to-many because the implementations divide responsibilities differently. Files, missing handlers and modules with no reachable product caller are all included. Unreviewed rows remain explicitly pending. This report does not claim that all 287 files have been compared.

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
| LDAC-012 | LDAC reset / SDK caller allocation | Working-buffer reset entered the general heap | Retain a configuration-sized bounded arena, preserve fresh SDK initialization, and detach ownership correctly at close. |
| L2CAP-013 | Over-limit declaration / signaling scanner | Reference keeps recombination overruns local; Core requires a signaling MTU response | Drain signaling/dynamic excess without allocation and queue the required first-request rejection. Registered fixed owners now receive explicit errors and deferred teardown; wire qualification is open. |

These defects can make an isolated peer/PDU problem terminate unrelated audio and BLE operation. Their presence is proven from code paths; they are not established as the cause of every reported crash or underrun. Initial Classic role knowledge still requires separate policy/review when no Role Change event has occurred.

The signaling change also checks the [Core 6.1 L2CAP specification, sections 4 and 4.1](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host/logical-link-control-and-adaptation-protocol-specification.html). Identified Responses are not answered with new Rejects, and Request rejection retains queue capacity/ownership. Internal invariant failures and exhausted mandatory-reply/resource paths are not silently discarded.

## Missing or unwired integration confirmed

- Automatic outgoing setup: explicit paging/SDP/signaling coordination, local stream procedures, exact matching, generation and ordered-request ranking now have source APIs. Poll/cancel coordination and SDP discovery are linked; the default dial/rank begin callers are absent. Complete durable peer-selection, reconnect/backoff/handover and queued/crossed policy remain unfinished. Speaker-initiated automatic reconnect is an explicit master-goal requirement; the released app itself configured it off.
- AVRCP browsing: dbt_avrcp_browse.c is a compiled source unit but its engine is absent from linked ELF symbols. The product registers the control PSM and explicitly advertises no browsing. Its baseline product exposure is still being checked; the existence of a broader BTstack profile is not proof that a released app feature was lost.

The source and linked-image distinction matters: a file compiling does not establish a usable product feature. BLE/client scope, remaining AVDTP states, all codec-specific framing and every other pending row remain part of this audit.

## Retained architectural choices and remaining work

Controller-to-host acknowledgments are independent of ordinary command credits in both implementations. Known-link packets discarded after wire validation still release their receive buffers. Controller/transport queue loss and invalid internal state remain explicit failures. Our asynchronous/bounded callback requirement is retained instead of adopting the reference ESP32 port's indefinite mutex waits.

The reference audio example/resampler is also structurally different from the owned full FIR/integer-DSP path. Existing measured FIR/render deadline failures remain unresolved; packet-error recovery is not a maximum-rate throughput guarantee. Decoder CPU0 priority21, host CPU1 priority22 and renderer CPU1 priority20 remain selected. Codec settings and input-buffer geometry are unchanged.

Current source coverage is twenty-six complete owned C bodies, nineteen header contracts, six build files, four source generators and two static-data files reviewed; nineteen files are partially reviewed and 205 implementation/build inputs remain pending. Every compiled owned translation unit is checked against the catalog, with zero missing units. The initial directory filter wrongly omitted the real Opus scratch folder; those four files and build/generation inputs are now included. Reference section coverage and findings are recorded explicitly in the ledger. A complete HCI/transport event audit and remaining L2CAP modes/state paths are not claimed finished merely because small framing/credit files have been inspected. Next stages cover AVDTP/A2DP, SDP/GAP/security, AVRCP/AVCTP, BLE/ATT/SMP and all product/codec integration paths.

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

The [retained LDAC workspace](LDAC_RETAINED_WORKSPACE_2026_11_05.md) replaces working-buffer heap activity during gap/error reset. It keeps the original SDK initialization, zeroes capacity-checked blocks, restricts handle allocation to setup and detaches caller-owned pointers before freeing the library handle. It adds no BSS or maximum-rate global pool. Ten generated units now carry source pins. The six existing cold LDAC cases pass, close with no remaining tracked allocations and show 33-byte mono /61-byte stereo/dual reductions in tracked peaks, including checker instrumentation. Repeated gap/fault behavior, updated regression harnesses and timing gains remain unqualified. Existing host/build contract sources were updated and syntax-compiled without executing their held campaigns.

## Over-limit signaling/dynamic stage

[Bounded over-limit handling](L2CAP_OVERSIZED_STREAM_2026_11_05.md) now drains a too-large signaling/dynamic declaration without retaining its payload. The signaling scanner stores only a partial four-byte command header and scalar state, validates complete framing and queues one MTU rejection with the first request identifier. Response-only frames do not create reject loops. Existing reply queue capacity, deadlines, generation fencing and known-link receive-credit completion remain. A policy/API error is not reclassified as a contained wire rejection.

This stage adds 80 BSS bytes and an oversized_pdus UART counter; MTUs, queues, codecs and affinities are unchanged. ERTM already validates mps+6 <= maximum_mtu. This first stage left fixed-owner policy open. The subsequent fixed-protocol stage below connects it; neither stage establishes complete wire conformance.

The follow-up list keeps its original array schema and now persists named rows correctly; earlier dictionary-style additions to that array were not serialized. Verified outstanding items from source/docs/checkpoints are explicit rows.

## Fixed ATT/SMP owner integration

The [fixed-protocol length review](FIXED_PROTOCOL_LENGTH_2026_11_05.md) compares BTstack sm.c:4427–4457 and att_server.c:898–945 with the owned L2CAP/runtime/ATT/SMP/product boundary. A completed oversized fixed PDU supplies only its declared size and at most four prefix octets through an explicit hook. ATT classifies commands/responses/requests without overwriting a pending response. SMP validates exact command lengths before pairing preparation, reuses owned two-byte slots and waits for existing transport/command/crypto/storage retirement before error admission. Malformed Pairing Failed does not create a failure loop. Per-peer disconnect intent is handled outside callbacks.

This adds 8 BSS bytes; existing SMP message slots remain 128 bytes total. Strict C11 Clang analysis of ATT, channels and runtime passes. A UUID-width guard resolves the analyzer’s unchecked mixed-width path. The full IDF 6.1 ccache build passes. No wire, pairing, playback, timing or app campaign ran, and full ATT/SMP source/lifecycle/security coverage remains incomplete. Seven newly touched pending inputs are marked partial, not fully reviewed.

## Audio copy/clock comparison

The [audio span-copy review](AUDIO_SPAN_COPY_2026_11_05.md) reads the complete owned audio body and contract against BTstack resampling/rate-estimation and baseline rendering. The reference uses 16-bit linear interpolation; the baseline disables its drop/insert logic and omits the continuous FIR now selected. This is an explicit workload difference. The selected full-precision FIR/DSP remains; its measured sustainable cost is still open. SDK clock tuning is stopped-only and takes a blocking mutex, so it is not used as a live substitute.

PCM submission now uses at most two ring-span copies. DSP block output and DMA take copy contiguous page/ring spans while preserving full-block acquire/release publication, arbitrary legal partial final pages and independent physical cursors. Single-writer submit/backpressure/render counters remain atomic but no longer use read/modify/write CAS retries. Strict C11 analysis for scalar/block paths and the full ccache build pass. Linked ISR helpers/literals remain ROM/IRAM. Text grows 100 bytes, including 24 instruction-SRAM bytes; BSS/data are unchanged. Submit and take compiler frames each grow 16 bytes, excluding callee usage. No timing, playback, wrap/concurrency or ISR-load qualification is claimed.

## Exact zero-product omission and complete output-owner source review

The [zero-tail FIR/output-owner review](FIR_ZERO_TAIL_2026_11_05.md) derives a common two-zero trailing invariant from all 387 generated rows. Regeneration preserves every coefficient word. The guarded kernel omits only zero products in steady and priming paths, keeping the complete filter span, causal delay, phase/history and rounding. Target limits are 94/46/22, with the original 64-byte FIR frame. This is an exact arithmetic work reduction, not a measured elapsed-time or underrun gain.

The complete owned output body and contract were also read against BTstack's ESP32 v5 driver refill sections and active product callers. Its 16-bit, 10 ms/six-descriptor polling adapter excludes IRAM-safe I2S, so its buffer/format/cache assumptions are not substituted here. Current linked output contains async try/begin/poll APIs and omits blocking compatibility wrappers. Failed renderer-task creation now releases its queue, duplicate initialization is rejected before state mutation, and the same-render-core shutdown invariant is accurately documented. Fault/concurrency/lifecycle qualification remains open.

The full final ccache build passes. Whole text grows 76 bytes entirely in flash; IRAM, DRAM data and BSS are unchanged. Static analysis of guarded/full FIR loops passes. No held playback, packet, timing, allocation-fault or app campaign was invoked.

## Complete integer-DSP source and exact analysis averaging

The [DSP analysis review](DSP_ANALYSIS_AVERAGE_2026_11_05.md) completes reading the owned C body/contract/generator/build file and checks all generated data against the baseline formula/control sections. The completed Q31 decimation sum permits an exact base65536 division using two native word divisions, including the negative endpoint. It replaces the average's generic __divdi3 call without changing samples, cadence, coefficients, state, controls or precision. Goertzel normalization and limiter wide divisions remain separate unresolved costs.

All thirteen-rate generated tables match; 2678 biquad endpoints are stable and the component box matches 3170260148. This is not proof of interrupted-ramp/acoustic or runtime feature parity. Baseline boost multiplier is 1.0, so no missing extra gain was established. The existing low-rate clamp difference is documented. The actual build policy remains whole-DSP -Os with a separate steady-block -O3 attribute; FIR/audio conversion stay -O3.

Three Clang configurations and full ccache compilation pass. Analysis uses two QUOU instructions, its frame remains 48 bytes, and text grows 52 flash bytes. IRAM/data/BSS are unchanged. No held timing/playback/EQ/functional campaign ran. Full DSP/audio and stack qualification remains incomplete.

## Outgoing signaling boundary and Classic request retirement

The [Classic connection review](CLASSIC_CONNECT_RETIREMENT_2026_11_05.md) follows pinned BTstack connection/disconnect sections and the Core L2CAP pending-connection/timer rules. A sent or borrowed Classic Connection Request now retains its identifier/CID through close intent and response/deadline. Obsolete unsent control pins retire before controller/credit gates, and stale-pin cancellation does not fail a changed transaction. Partial SDUs retain their loan through completion. A late Pending response cannot rearm an expired absolute limit. Static analysis and the full ccache build pass; no wire/cancellation/backpressure campaign ran.

The source now exposes acceptor/product signaling begin/status/cancel on a caller-selected existing ACL, using normal security and channel routing. There is no enabled caller or new connection at boot, and these new symbols are absent from the linked image. Remembered Classic peer selection, automatic GAP/SDP coordination and local endpoint/configuration/start procedures remain missing. The completely read product-peer publisher/header is a BLE grant/OTA snapshot, not that policy. Larger channel/runtime/product bodies remain partially reviewed.

This stage adds 392 flash text bytes; IRAM, DRAM data and BSS are unchanged. Normal timers, reply capacity, codecs, MTUs, input geometry and affinities remain. Full channel lifecycle, remaining LE/incoming pending close and outgoing/feature/runtime qualification remain unfinished.

## Composed owner transport coordinator

The [owner connection coordinator](A2DP_CONNECT_COORDINATOR_2026_11_05.md) now composes explicit peer selection with existing/new ACL, collision-safe GAP paging, Audio Source SDP discovery/retirement and exact signaling setup/observation. Product owner polling, waits, restart cancellation and exact SDP routing are wired. Context-plus-token adapter identity prevents cross-context stale calls; per-layer page/SDP/signaling outcomes remain distinct. The SDP client channel no longer also claims a server slot. No new task or default context/buffers are created.

The target ABI measures a 428-byte caller-owned context and 168-byte result, with additional caller-supplied SDP buffers. Persistent BSS grows 8 bytes; IRAM/data stay unchanged and flash text grows 3504 bytes. Strict Clang analysis and full ccache compilation pass. No explicit begin caller is enabled: no default peer or new outgoing operation is started at boot. Remembered-peer persistence/backoff/handover and local endpoint/configuration/open/start remain missing, and all wire/fault/resource/stack/timing qualification remains incomplete.

This adds two authored source/contract inputs to the ledger. The larger changed GAP/SDP/product files remain partial; compilation and source integration are not complete reconnection/audio proof.

## Isolated read-only endpoint/capability queries

The [peer query review](AVDTP_PEER_QUERY_2026_11_05.md) adds a per-command completion route and caller-owned Discover/Get[All]Capabilities/GetConfiguration operation. A query response no longer goes through the sink's Delay Report completion; default and remote handlers stay separate. Callback storage detaches before notification, while timeout-pinned TX storage keeps its independent transport lifetime. Unsent cancellation can finish locally; borrowed/submitted requests retain the query until a terminal result.

All up to 62 endpoint records and complete raw capability/configuration bodies are preserved. Reserved discovery bits are ignored, identifiers/duplicates/transport TLVs are checked, and undersized caller buffers get explicit no-space/required length. Structural success is not compatible codec selection. Product begin checks the full current signaling identity. No query caller is enabled or new query sent at boot.

Target ABI is query context 64/result 36/callback 8 bytes plus caller response storage. Audio-profile size grows 396→400 due to one signal pointer, but measured aggregate BSS 50308, IRAM/data are unchanged. Default linked flash text grows 104 bytes; active query/stack/wire/timing behavior remains unqualified. Strict Clang and final ccache compilation pass. The new query body/contract are cataloged; selection, SetConfiguration/Open/Start, worker rollback/collision and automatic reconnect policy remain unfinished.

## Initial local configuration and bounded metadata work

The [local configuration review](AVDTP_LOCAL_CONFIGURE_2026_11_05.md) compares BTstack initiator:271–409 to the owned sink/signal/bearer. A copied but blocked command now waits asynchronously for decoder READY, then peer acceptance commits it. Reject/cancel/Abort retain ownership until serialized DROP acknowledges older resources. Two concrete integration faults are corrected: app configuration notification precedes pending-metadata release, and failed profiles still run retiring worker drain. Result fields distinguish transport, peer, worker, cleanup and cancel/deadline outcomes. There is no new task or automatic caller, and local Open/Start/reconfigure and selection remain unfinished.

BTstack hci.c:416–442 also uses finite synchronous connection searches. The highlighted owned allocation loop scans at most four entries and copies six address bytes; it does not wait for credits, I/O or another task. No underrun cause or timing gain is established by replacing this lookup.

Strict analysis of three protocol files and the final ccache build pass. Compiler-only context/result sizes are 100/72 bytes. Profile grows 400→404 bytes and aggregate BSS grows 50308→50316; IRAM/data are unchanged, flash text grows 1320 bytes. Source/compilation are separate from unfinished runtime/collision/cancel/fault/feature/audio qualification.

## Local Open and outgoing media establishment

The [Open/media review](AVDTP_LOCAL_OPEN_2026_11_05.md) compares pinned initiator:322–345 and avdtp.c:983–1063 plus the primary A2DP setup/streaming/MTU sections. Local Open now reuses the same prepared-command/worker operation slot as Set Configuration. Worker READY precedes wire release; Accept commits initiator direction and media deadline. Rejection and pre-wire cancel retain the configured decoder after any accepted worker receipt. Abort promotes cleanup to DROP; uncertain post-wire cancel/fatal state retires the affected profile.

Media begin reserves exact channel identity before asynchronous event routing; only its OPEN event binds it. Remote-Open media admission remains separate. Status is readiness, with detailed reasons in ordinary channel events; no audio start is implied. Final DROP-to-profile completion and retained Open cleanup readiness now request the next owner iteration without spinning while a provider is pending. Shared sink/profile storage remains 292/404 bytes, aggregate BSS 50316, IRAM text 122255 and DRAM data 23893 are unchanged. Caller context/result grow eight bytes each to 108/80. Default flash text grows 196 bytes.

Strict Clang and the full ccache build pass. No begin caller is enabled; source APIs and idle deployment do not qualify local Open/media/cancel/collision/worker failure or streaming. Start/reconfigure, automatic codec/peer policy, remaining file audit and full feature/security/lifecycle/audio performance qualification are unfinished.

## Local Start and acknowledged renderer rollback

The [Start review](AVDTP_LOCAL_START_2026_11_05.md) compares pinned initiator:333–340/595–604 and A2DP3.2 with the owned worker/ticket contracts. Start now checks exact signaling/media lifetimes, prepares the renderer asynchronously and commits streaming on peer Accept. Accepted preparation must receive local SUSPEND READY before rejection/pre-wire cancellation can return a reusable Open stream; REJECTED alone is not treated as stop proof. Fatal/failed rollback or uncertain post-wire cancellation retires the affected profile. Abort supersedes with DROP.

Preparation, rollback and DROP have separate tokens/admission/receipts/deadlines. Superseded preparation and rollback receipts remain diagnostics without completing newer work. Start rejection SEID is distinct from a codec category. Incoming list commands validate endpoint identity before a local-operation collision. No new task, default context or automatic caller is added. Strict Clang and final ccache/source/workspace checks pass; caller context/result become 140/112 bytes (+32 each), while sink/profile 292/404, BSS 50316, IRAM 122255 and DRAM data 23893 remain unchanged. Flash text grows448 bytes. Active Start/rollback/cancel/fault/Abort and audio headroom remain unqualified. Local reconfigure/suspend/close/abort, codec/default-peer policy and full source/feature/security/lifecycle qualification remain unfinished.

## Wire-first Suspend and preserved streaming admission

The [Suspend review](AVDTP_LOCAL_SUSPEND_2026_11_05.md) compares pinned initiator:341–348/631–639 and the existing worker contracts. Local Suspend sends without a pre-wire worker job and retains media admission while awaiting peer acceptance. Accept creates a separately timed backend SUSPEND job; READY finalizes Open. Rejection/known-unsent cancellation retains streaming; failure after peer acceptance retires the affected profile through DROP. The result separates peer acceptance from backend completion and preserves existing Abort/late-receipt ownership.

Cancellation now tries the exact unsent/unborrowed command primitive before treating transmission permission as uncertain wire admission. It does not release actual submitted/borrowed work early. Strict Clang and final ccache/source/workspace checks pass. All target context/result/sink/profile sizes remain140/112/292/404; BSS 50316, IRAM 122255 and DRAM data 23893 are unchanged. Flash text grows240 bytes. No new task/default caller is added. Active procedure/cancel/queue/collision/fault/audio qualification, local Close/reconfigure/Abort, peer/default policy and full source review remain unfinished.

## Local Close and channel-reason preservation

The [Close review](AVDTP_LOCAL_CLOSE_2026_11_05.md) follows pinned initiator:349–359/607–620 and the owned worker/media contracts. Local Close keeps the prior stream until peer Accept, then runs decoder release and transport closure independently. Caller/configuration ownership is retained through both completions, including normal early media closure. Reject/cancel cannot claim the prior stream was restored when its channel is gone. Abort and failures retire through existing owner paths.

The channel finish audit found CLOSED means previously opened, not successful retirement. Production profile routing now preserves reason/status through event_ex and annotates raw media event/reason/result separately. Failed transport retirement or media deadline cannot complete Close successfully. Legacy simple wrappers remain available for explicitly normal events. Larger channel/root bodies remain partial.

Strict analysis of sink/acceptor and final ccache/source/workspace checks pass. Context/result grow 140→148 and 112→120 bytes; sink 292/profile 404, BSS 50316, IRAM 122255 and DRAM data 23893 remain unchanged. Flash text grows 344 bytes. No automatic caller/task is added, and active wire/worker/media/cancel/collision/fault/audio qualification remains unfinished. Local reconfigure/Abort, peer/default policy and the full source/feature/security/lifecycle review remain open.

## Local Reconfigure and one-workspace restoration

The [Reconfigure review](AVDTP_LOCAL_RECONFIGURE_2026_11_05.md) follows pinned acceptance/reject/request packing sections and the owned caps/copied-CIE/worker contracts. The exact current Open endpoint now accepts a supported codec selection into copied normalized configuration. Wire SEID+Codec excludes unchanged Transport/Delay. Worker READY precedes release; peer Accept alone commits and publishes new parameters. Old committed caps stay owned until then.

Reject/pre-wire cancel after changed preparation requires a separately acknowledged RECONFIGURE job from old caps; construction REJECTED already proves old restoration, while fatal or failed rollback retires the profile. Rollback operation/token now distinguishes restoration from Start SUSPEND and from primary work using the same opcode. No second large decoder workspace, task or default caller is added. Any media loss records closure and its media outcome.

Strict Clang and final ccache/source/workspace checks pass. Context/result grow 148→152 and 120→124 bytes, while sink 292/profile 404, BSS 50316, IRAM 122255 and DRAM data 23893 are unchanged. Default flash text grows 20 bytes. Active configure/restore/cancel/allocator/Abort/media/quality behavior remains unqualified; local Abort, peer/default/collision policy and full source/feature/security/lifecycle/audio qualification remain unfinished.

## Local Abort and pending-media ownership

The [Abort review](AVDTP_LOCAL_ABORT_2026_11_05.md) compares pinned initiator:361–374/623–629, public admission:1310–1334 and the current signal/worker/channel contracts. Accepted local Abort releases wire independently of decoder teardown, then joins peer acceptance, actual worker retirement or explicit no-worker-needed state, and every bound/pending media lease. Configured/no-media and already-released backend cases do not synthesize worker receipts. Cancellation/rejection/failure after acceptance retires the profile instead of claiming the prior stream was restored.

The profile captures staged media identity before acceptance, never rebinds an aborted setup and distinguishes a normal never-opened cancellation from a real failure by reason/status. Pending media participates in deadlines/readiness despite a zero established binding. Crossed remote Abort retains DROP/reply/media ownership; blocked provider states do not create immediate polling spins. No new task or persistent instance is added.

Strict Clang and final ccache/source/workspace checks pass. Context/result/sink/profile sizes remain 152/124/292/404, BSS 50316, IRAM 122255 and DRAM data 23893 are unchanged; default flash text grows 408 bytes. No automatic caller or held campaign is added. Existing procedures return BUSY without accepting another intent, unlike the reference queued Abort flag; arbitrary queued urgent/crossed policy, automatic selection/reconnect and full source/feature/security/lifecycle/audio qualification remain unfinished.

## Exact requested-format capability matching

The [matching review](A2DP_EXACT_SELECTION_2026_11_05.md) follows pinned manual SBC/vendor configuration entry points, actual codec validators and the app preference bridge. A new pure module checks available audio Source role, mandatory Transport+Codec, full codec/vendor identity and an exact caller-specified CIE against both current validators. Delay needs explicit policy, known version and mutual support. It does not invent source-only bitrate/quality fields, lower the requested format or transmit a proposal.

The owner adapter consumes current terminal accepted complete Discover and Get[All]Capabilities query objects/tokens and finds the actual peer record. Stale, pending, partial, cancelled or rejected receipts cannot become a successful match. No heap/decoder/task/default caller is added. Strict Clang and full ccache/source/workspace checks pass. Request/result ABI is 12/24 bytes, matcher own frame 416 bytes excluding callees. Default image omits matcher symbols; text/BSS/IRAM/data are unchanged. This is not complete automatic ranking, caller policy, codec-extension or active-format qualification.

Two authored source/contract inputs enter the ledger: 283 inputs, 27 reviewed C bodies, 20 reviewed headers; 205 pending and 19 partial remain. Larger root/codec/backend bodies retain scoped review. Full feature/security/lifecycle/audio/headroom and crossed/reconnect policy remain unfinished.

## Requested-family codec candidate generation

The [generation review](A2DP_CANDIDATE_GENERATION_2026_11_05.md) adds explicit family/rate/channel generation for current SBC/AAC/aptX variants/LDAC/Opus variants/LHDC layouts, followed by exact matching. Nonzero codec-specific requests stay strict. SBC defaults fit current frame bounds; AAC object/VBR/bitrate, Opus duration/location/raw field and LHDC numeric ranges/depth/features retain distinct semantics. LDAC source quality and non-negotiated PCM depth are not invented as CIE fields. No automatic cross-family ranking or caller is supplied.

The owner adapters share current discovery/capability receipt fences. Known profile/runtime/config/signal storage is protected against output aliasing. Strict Clang, target compilation and full ccache/source/workspace checks pass. Request/result ABI is 28/36 bytes; generator own frame 208 excludes matcher 416 and the remaining chain. It is not stack-headroom measurement. Default image omits generator symbols; text/BSS/IRAM/data remain unchanged. Active/malformed/codec/quality/latency proof is missing.

The ledger now has 285 inputs, 28 reviewed C bodies and 21 header contracts; 205 pending and 19 partial remain. Larger root/backend bodies retain partial scope. Ranking, durable peer/default/reconnect, crossed policy and full feature/security/lifecycle/audio qualification remain unfinished.

## Asynchronous ordered-request capability ranking

The [ranking review](A2DP_ORDERED_RANKING_2026_11_05.md) compares pinned A2DP discovery scheduling, available opposite-role SEP filtering, sequential capability completion and incoming configuration handoff. Caller buffers and one reused query drive complete assessment; explicit ordered requests retain strict format constraints and first-discovery tie order. A failed, rejected, malformed or truncated capability result, or unsupported requested vendor layout, cannot produce a successful partial lower proposal. Ranking does not transmit Set Configuration or override app preferences.

Each owner step performs one query transition, at most four SEP skips or one generator evaluation. No heap/task/default instance is added. Cancellation/overall timeout waits for the accepted query callback fence; stale tokens retain storage and request matching signal retirement without an immediate polling spin. Incoming configuration returns conflict after that fence. Root terminal result acknowledgment detaches the caller route. These paths remain unqualified on wire.

Strict Clang analysis, target C11 and ccache/source/workspace checks pass. Context/config/result is 384/24/224 bytes plus caller-supplied buffers. Begin/poll own frame is 96 bytes and result 64, excluding callees/full chain. Owner routing links poll/generator code despite no automatic begin: text grows 7372 bytes and aggregate data 112; BSS, DRAM data and IRAM remain unchanged. No speed/headroom improvement is claimed. The ledger now has 287 inputs, 29 reviewed C bodies and 22 header contracts; 19 partial and 205 pending remain. Complete peer/reconnect/default policy, queued/crossed controls and full feature/security/audio/lifecycle qualification are unfinished.

## Direct output rendering and strict C11 cycle profiling

The [direct-render review](AUDIO_DIRECT_RENDER_2026_11_05.md) compares pinned ESP32 audio callback staging with the owned SPSC page/ring and DSP contracts. Conversion and the existing integer block DSP now operate in the same unpublished free output span, split at 128-frame/page/ring limits. Final output publication and sound mixing order remain unchanged. This removes the 1024-byte temporary array and one 8-byte-per-stereo-frame copy without changing signal arithmetic, buffers, rates, precision or codec settings.

Matched target compilation reduces render own frame 1120→112 bytes and removes its memcpy relocations; default linked text is 448 bytes smaller with unchanged BSS/DRAM/IRAM. Full-chain task headroom, boundary/concurrency/output/state equivalence and speed are unmeasured. The optional profiler now uses SDK XTHAL_GET_CCOUNT on Xtensa, preserving elapsed core cycles while avoiding esp_cpu.h bare asm failure under C11. Normal/profiled block/scalar configurations compile; no held campaign is invoked.

## aptX wrapper source review

Complete authored dbt_aptx.c/backend/header/codec CMake template bodies were read against pinned full vendor/codec identity boundaries (avdtp.c169–183, avdtp_util.c881–900/1723–1753), SDK context initialization/reset and decode32 output contracts, and current product compile entries. Existing 4/6-byte raw quanta, 1024-byte bounded S32 PCM output and deferred continuation preserve their contracts. SDK reset preserves HD mode; the adapter uses an aligned owner workspace and does not free it through SDK aptx_finish. Product entries retain -O2/-fwrapv. Strict wrapper Clang analysis reports no diagnostics.

No new cause of the reported aptX crash is established in these boundaries. SDK internals, unsupported LL extensions and active parity-loss/lifecycle/quality/crash behavior remain unqualified. The standalone codec template was source-reviewed; the current product builds its aptx_decoder component. The ledger now contains 31 reviewed authored C bodies, 23 headers, 7 build inputs, 4 generators and 2 static-data inputs; 19 partial and 201 pending remain among 287 total. This does not imply all codec algorithms or Bluetooth layers are reviewed.

## LHDC recovered-error and concealment accounting

The [LHDC quality review](LHDC_QUALITY_VISIBILITY_2026_11_05.md) follows pinned sibling channel returns and peak concealment alongside selected BTstack codec-health counters. The C99 decoder can return success after right-channel silence or complete frame concealment, so generic worker rejection was incomplete audio-health telemetry. Source-pinned notices now preserve those decisions separately without changing PCM/recovery behavior. The existing decoder owner publishes fixed lock-free counts and the latest notable RTP/frame tuple; the lower-priority logger emits separate decode, channel and frame-quality records. Snapshot reads are capped at 3 attempts; normal frames do not erase anomaly context. No task, queue, decoder UART or allocation is added.

Cold startup checks are excluded. Counts are cumulative modulo uint32 and metadata is only the latest notable event, not complete event replay or proof of RF cause. Strict adapter Clang, target compilation, pinned generation and final ccache/source/workspace guards pass. Snapshot 80 bytes, state 32→56, static+88, resume own frame 112 unchanged; text+1816. Live event/error/concurrency/acoustic/headroom proof remains missing. The raw alternate session adapter remains unlinked and partial; its old-workspace/config-input borrow edge was subsequently fixed in the PCM boundary checkpoint. It is not established as the product crash source.

The ledger now has 287 inputs: 32 reviewed authored C bodies, 26 header contracts, 9 build files, 5 generators, 2 static data, 18 partial and 195 pending. Complete upstream codec algorithms, feature/security/lifecycle and all-layer parity remain unfinished.

## Defined LHDC S32 conversion and raw configuration lifetime

The [PCM boundary review](LHDC_PCM_BOUNDARIES_2026_11_05.md) fixes an unsafe positive S32 endpoint: INT32_MAX expressed in binary32 rounds to 2^31 before conversion. The pinned port now saturates that endpoint explicitly, with unchanged finite in-range truncation. Unordered scaled PCM becomes a sticky infinity peak, selecting existing frame/overlap concealment and explicit nonfinite flag/count. Finite peak decisions retain their values; unsafe fast/finite-only compiler modes are rejected. No negotiated rate/depth/bitrate, FIR, integer DSP or other codec is lowered. Live boundary/output/state/timing proof remains missing.

The alternate raw-session adapter now copies its validated 24-byte requested config before releasing old workspace, following the canonical worker ownership boundary. It remains unlinked in the product. Strict Clang/target/generated compilation and final ccache/source/workspace guards pass. Raw open own frame grows 48→80; snapshot ABI 80→84, static counter+4 (aggregate BSS+8), text+56. The ledger has 33 reviewed authored C bodies, 26 contracts, 9 build files, 5 generators, 2 static-data files, 17 partial and 195 pending among 287 inputs. Full codec algorithms, all-layer feature/security/lifecycle and runtime parity remain unfinished.

## Codec packet storage and complete authored Opus path review

The [packet-storage review](CODEC_PACKET_STORAGE_2026_11_05.md) compares the byte-identical working-baseline Opus fixed buffer with pinned BTstack retained frame storage and full authored Opus/scratch/LDAC parser bodies. Opus now reserves the unchanged 6144-byte limit separately during codec setup, clearing only logical cursors on reset/expiry/media and releasing at close. Length-based active/deadline predicates avoid treating an always-allocated buffer as a pending packet. Ordering, expiry, configuration and PCM retirement remain unchanged. LDAC replaces its validated staging byte loop with memcpy while retaining padding/bit bounds.

Strict Clang/target and final ccache/source/workspace checks pass. Media/expiry/reset/clear object sections have no allocator/release relocations. State 136→132 still rounds to 136 in the main allocation; separate pool costs 6152 accounted bytes while configured, cold tracker 6160. Open own frame remains 96; media grows 48→64. Text is 84 bytes smaller; BSS/data/IRAM unchanged. No speed/headroom improvement is claimed, and configured Opus RAM is higher than the prior unfragmented case. Low-memory setup can reject earlier; active failure/fragment/expiry/codec-switch/audio proof is missing.

All 129 compiled Opus backend units use the scratch override; the guarded decoder symbol is linked, the unguarded one absent. This does not fully audit upstream algorithms. Coverage now has 36 reviewed authored C bodies, 31 header contracts, 9 build files, 5 generators, 2 static data, 16 partial and 188 pending among 287 inputs. Full source/feature/security/lifecycle/concurrency/audio parity remains unfinished.

## Transport discard wakeup and TX spans

The [transport review](TRANSPORT_DISCARD_DEADLINE_2026_11_05.md) rereads full authored runtime/RX-flow/ACL/HCI/L2CAP bodies alongside pinned HCI fragmentation, recombination and completed-packet sections. Bufferless oversized RX discard now contributes its existing wrap-safe timeout; previously the wait calculation recognized only non-null payload storage. Local expiry does not depend on transmit credits/readiness. No polling, task, pool or timeout value is added.

TX assembly copies at most three logical source spans instead of dispatching each byte, with prefix/body presence validation, integer offsets before pointer arithmetic and a disjoint scratch/count caller contract. Cursor/credit consumption and controller buffer lifetime are unchanged. Strict Clang/target and final ccache/source/workspace checks pass. Peek own frame 48→64, wait 48 unchanged; text+80, no BSS/data/IRAM change. No timing or live boundary/interoperability proof is claimed. Final source-index correction was rebuilt and separately read back before commit.

Coverage now contains 37 reviewed authored C bodies, 35 header contracts, 9 build files, 5 generators, 2 static-data inputs, 15 partial and 184 pending among 287 inputs. Channel/provider/ERTM and callback error/retirement policy still require inter-layer review; full feature/security/lifecycle/audio/headroom qualification remains unfinished.

## LHDC configuration-time window and readiness

The [window setup review](LHDC_WINDOW_SETUP_2026_11_05.md) follows selected sibling window/transform/init/cleanup and current band/constant data. FFT tables/FAC models were already configuration work, but the ramp could allocate on first decode. Target initialization now prepares/checks it before initialized/READY. Status propagates for required transform/reference cache, window allocation and entropy storage; failed tails are released before existing adapter cache cleanup. A valid generic shape is retained/cached; host window-file timing and diagnostic reference selection remain supported. No window value, arithmetic, negotiated parameter or working-storage size is lowered.

Generated target compilation and final ccache/source/workspace checks pass. Portable Clang decoder diagnostics are empty; IMDCT retains one original unused pointer-cursor increment warning. The target-only cold call is source-reviewed/target-compiled, not covered by the host macro. Constructor relocations confirm window/readiness calls. Text+72, no BSS/data/IRAM change; four cold LHDC peaks unchanged. No live first-frame latency, setup failure, reference/generic, low-memory, switch/acoustic/headroom proof. Coverage remains 37 authored C bodies, 35 contracts, 9 build inputs, 5 generators, 2 static data, 15 partial and 184 pending among 287.

## Current COM6 deployment

Image 4c24457a6aba94837aee242eae3c43b7513abbb6aeef4d2bd521b673d6f60413 was flashed at 921600 baud to ota1 at 0x390000. Full image and selector readbacks at 230400 match; original application full-region MD5 is unchanged. Owned host remains selected VALID with automatic Bluedroid restoration disabled. Ordinary idle startup is ready with no watchdog/backtrace; decoder CPU0 priority21, host CPU1 priority22 and renderer CPU1 priority20 remain selected.

Idle heap is 137912 free/67356 reported minimum/110592 largest bytes. The minimum is an aggregate of regional low watermarks, not proof of saved live memory. Text is 882811 (72 larger), BSS 50412, IRAM 122255 and DRAM data 23893 unchanged. All 37 existing cold codec checks pass (RF false); four LHDC cases have zero tracked-peak allocation delta, and overall cold peak remains 56899. UART source 127 is connected. No separate live setup/failure/reference/generic/low-memory/codec-switch/playback/RF/app/acoustic/headroom campaign ran. Archive: native_bt/_build/hardware/runs/1793947670582980800-4c24457a6aba/.
