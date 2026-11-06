# Classic connection retirement and signaling entry — 2026-11-05

## Compared boundaries

Pinned BTstack 9edb5d82ea6ec4466a393415d65030929330acdf remains reference-only. Compared avdtp.c:331–389 (SDP-backed connection entry) and l2cap.c:2936–2979 (disconnect/open-failure lifetime), with owned GAP/SDP/acceptor contracts, Connection Request/Response, close/readiness, pin/commit/cancel, owner polling and Pending deadlines. These are section comparisons, not a completed L2CAP/GAP/profile audit.

The [Core 6.1 L2CAP specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host/logical-link-control-and-adaptation-protocol-specification.html), sections 6.1.2 and 6.2, distinguishes the pending connection state, where both CIDs are not yet available, from a connected channel and bounds extended response timing. The implementation keeps explicit ownership through those boundaries rather than inventing a destination CID or copying reference state transitions.

## Active protocol corrections

Closing a Classic Connection Request that was sent, or is pinned by the transport, now returns BUSY with retained close intent. Its identifier, serial and local CID remain associated with the response or existing bounded deadline. A final success supplies the real peer CID before normal disconnect. A refusal or timeout follows normal retirement. Close readiness stays false while waiting, so the owner sleeps on protocol/transport events and deadlines rather than spinning. Unsent, unlocked requests can still retire immediately. LE setup and incoming-pending cancellation policies remain separate incomplete reviews.

The runtime can retire an obsolete or explicitly cancelled unsent signaling pin before checking controller availability or ACL credits. This is local progress. Once any ACL fragment was admitted, the complete SDU finishes before its pin is returned. Cancellation of an obsolete pin no longer fails a newer/current transaction that occupies the channel slot. Still-valid transport failure retains its existing terminal path. Standalone mandatory replies are not discarded merely because an application wants to close.

A closing Classic outgoing setup does not start normal configuration/feature discovery or emit a new initial OPEN notification. Existing already-open operation, security qualification and mandatory protocol reply capacity remain. No channel/queue/payload capacity or task is added.

The Pending handler also had a wrap-safe deadline gap: a response processed after pending_limit could turn the negative remaining interval into a large unsigned number and rearm the deadline. The absolute limit now stays expired in that case. This matters because the bounded RX batch runs before the timeout sweep.

## Outgoing signaling source API

The acceptor now supplies connect_begin/status/cancel entry points, exposed by the product audio wrapper. Begin runs on the host owner outside channel callbacks and uses an exact existing Classic ACL plus normal asynchronous encryption/authorization. The caller selects/verifies its peer and SDP service. PENDING copies a 16-byte identity value; no caller storage is borrowed. Current readiness checks the full serial/generation/handle/CID identity. Cancellation stores exact-channel close intent or retires the active signaling/media/worker binding through existing owner polling. Detailed failures remain in the existing channel event hook.

This only establishes signaling transport. It does not implement remembered-peer persistence, automatic paging/SDP coordination, local Discover/GetCapabilities/SetConfiguration/Open/Start procedures or complete reconnection. Current source has no automatic caller: the new entry points compile into their objects but are absent from linked firmware symbols. No new outgoing RF operation is started at boot. Existing incoming setup remains selected. A signaling L2CAP initiator remains an audio Sink; audio direction and codec flags are unchanged.

The fully read dbt_product_peer.c/h is a bounded, generation-scoped BLE permission/OTA snapshot with three-attempt readers and a fail-closed sequence ceiling. It is not a Classic remembered-peer database. Reusing it as that policy would be incorrect. Caller grant/lifecycle security qualification remains open.

## Evidence and limits

Strict C11 Clang analysis passes for channels, runtime, acceptor and product-peer publisher. Full ESP-IDF 6.1 ccache compilation and source stability pass. Final text changes 866971→867363 bytes, entirely in flash; IRAM 122255, DRAM data 23893, aggregate data 396577 and BSS 50300 are unchanged. No payload buffer, task, queue, MTU, codec configuration or affinity changes.

The readback/startup receipt is separate from wire evidence. No unit, simulated peer, packet injection, pairing, playback, app, timing, Pending-flood, cancellation-race or backpressure campaign is invoked under the user hold. Ordinary startup runs the existing automatic codec checks; they do not exercise these paths. Partial-SDU, late-response, cancellation/CID reuse, low-resource, remaining LE/incoming setup and complete outgoing/feature/runtime qualification remain unfinished. No speed or underrun gain is claimed.
