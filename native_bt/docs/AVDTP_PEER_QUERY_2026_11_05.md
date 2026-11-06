# Isolated AVDTP endpoint queries — 2026-11-05

The transport coordinator can open signaling, but the sink previously routed every local completion through its Delay Report handler. The signaling engine now supports one caller-owned completion for the exact local transaction. Default Delay Report completion and incoming command/response handlers remain independent. An unsuccessful command admission cannot replace an existing listener. No reference implementation is integrated.

## Borrow and cancellation fences

The caller keeps the immutable completion object/context live through its notification return. The engine detaches that pointer before calling it and never touches it afterward. Normal response/cancel retires local TX first. Timeout returns the callback borrow while preserving separately pinned transport TX bytes; later signal clearing releases those bytes silently. This distinction prevents freeing a transport buffer merely because a query result is ready.

Unsent/unborrowed local work can cancel immediately, without affecting a pinned mandatory remote response. A borrowed, fragmented or submitted request remains pending through its response/timeout/clear. The query layer retains a non-repeating per-object caller token, binding and wire label. Stale/error cancellation is not a release fence; only a matching terminal query result permits context/buffer release. Completed transactions may win cancellation.

## Read-only query support

New dbt_avdtp_query.c/h supports Discover, Get Capabilities, Get All Capabilities and Get Configuration. Requests encode the remote SEID with reserved request bits zero. Product entry points require the exact current signaling channel identity and normal owner polling; there is no default query/context, extra task, query-owned heap allocation, decoder construction or configuration side effect.

Discover decoding preserves up to 62 records with SEID, in-use, media-type and Source/Sink fields. It checks identifiers/shape/duplicates and ignores reserved field bits. No codec is inferred from endpoint identity, and records are not filtered to SBC. Capability responses preserve raw TLVs, including alternative content-protection entries and unknown capability categories. Transport shape/duplicates are checked through the existing parser; Get Configuration also uses configuration-category rules. Structural query success does not establish a compatible codec or complete mandatory-service/selection policy. The future selector must still check required services, media direction, supported codec fields and an exact valid configuration.

Complete response bodies are copied into caller storage before transport release. The copied view is used for validation, including overlapping input-copy safety. An insufficient buffer reports required length and exposes no successful partial response. Separate fields retain transport outcome, reply type, peer error, payload status, validation detail, cancellation and response presence. Rejected or structurally bad capability data does not become a false supported-codec result or reset live decoder state.

## Reference comparison

Pinned BTstack avdtp_initiator.c:167–260 was read for label/SEID discovery and capability response handling. The owned engine retains its existing complete-message reassembly instead of integrating that parser. The [Core 6.1 field interpretation rules](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/architecture,-change-history,-and-conventions/general-terminology-and-interpretation.html), section 2.4.1, explain ignoring reserved fields. This change applies that treatment to the new endpoint iterator; it is not a completed reserved-field audit of every layer.

## Evidence and remaining work

Strict C11 Clang analysis passes for the changed signaling engine and new query body. The first target build rejected misleading statement layout; it was corrected, and the final full ESP-IDF 6.1 ccache build/source-stability checks pass. A compiler-only target object, not linked/executed, measures query context 64 bytes, result 36 and completion 8; response bytes are additional caller storage. The signal gains one four-byte pointer and the measured audio-profile size changes 396→400. Whole-image BSS nevertheless remains 50308; IRAM 122255, DRAM data 23893 and aggregate data 396577 are unchanged. Whole text changes 870867→870971, entirely in flash. No heap improvement is attributed to section layout. Activation stack/cycle consequences remain unqualified.

There is no automatic query caller. Query API symbols are compiled but unlinked in this image; common completion dispatch remains active. Codec selection, local Set Configuration/Open/Start, worker rollback/collision handling, durable reconnect policy and full feature/runtime qualification remain unfinished. No new outgoing RF operation is enabled at boot.

Flash/readback and ordinary startup evidence are separate. Built-in codec checks do not exercise endpoint queries, cancellation/listener lifetime, fragmented responses or malformed/capacity paths. No unit, simulated-peer, packet-injection, paging/pairing, playback, app, timing or fault campaign was invoked under the user hold. No sustained audio headroom or underrun improvement is claimed.
