# Owned SDP client and channel adapter

## Implemented layer

The owned client now implements Service Search, Service Attribute and Service Search Attribute transactions. It shares the existing server's bounded data-element grammar through two pure validation helpers; the server response path and public records remain unchanged. A separate L2CAP adapter binds a fresh client to a caller-selected OPEN SDP channel. It neither registers a duplicate PSM nor opens a connection or starts a query.

The adapter matches handle, local CID, link generation and channel serial. Product dispatch must route this exact channel to the client before the SDP server, including while closing, so response PDUs are not mistaken for server requests. Other SDP channels keep their server role. MTU/configuration changes close this discovery channel for fresh negotiation. Runtime callbacks prove borrowed SDUs retired before request storage can change or clear succeeds.

## Transaction rules and bounds

Each continuation uses a new transaction ID. Opaque state is copied exactly with lengths 0..16; it is never interpreted as an offset or required to change between positive-progress responses. IDs never wrap on a channel, and query identities include the channel binding. Stale response IDs are counted without extending deadlines. Header/body lengths, response type, requested maximums, stable search totals and complete handle counts are checked. Attribute fragments may cut data elements anywhere; only the assembled final lists are validated. Attribute IDs remain ordered and paired with complete values.

The [Core SDP specification, sections 4.2-4.7](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host/service-discovery-protocol--sdp--specification.html) defines these wire rules. BTstack sdp_client.c at pinned commit 9edb5d82ea6ec4466a393415d65030929330acdf was compared for transaction/continuation framing and response-length checks. No BTstack implementation is integrated.

Request/result storage is supplied by the caller, bounded to 4096 bytes each, with no large default buffer. Request validation reserves room for the maximum continuation field up front. Overflow is an explicit NO_SPACE outcome, never a truncated success. Large databases can be queried by record or smaller attribute ranges. Optional remote ErrorInfo is counted but not retained; the peer error code remains separate from local protocol/transport errors.

## Asynchronous ownership

One immutable request view is pinned per transport loan. If the response arrives first, bytes and continuation state go into disjoint caller/private storage; the request is not rewritten until tx_done proves retirement. An UNSENT return is valid only after no SDU was admitted and no transport still borrows it. A response contradicting an UNSENT report is a protocol failure. Link loss/abort retires through the actual runtime callback.

Cancel/timeout cannot force-release a pinned view. The pending result exposes must_close, and the adapter initiates channel teardown while retaining storage. A stalled retirement is separately latched; it does not create a zero-deadline retry spin. The caller must observe closure requirements even while the result is PENDING, then await the real TX callback. Data is exposed only for complete validated SUCCESS and remains borrowed until the next successful begin.

Phase and whole-query deadlines are bounded. Each continued response makes byte/handle progress. Send readiness is distinct from transport capacity: a BUSY runtime send returns the unsent view once and waits on capacity/protocol events, not an immediate retry loop. Reconfiguration/close handling also waits for actual signaling capacity. Receive/event callbacks do not mutate the channel manager; owner poll performs close requests. Malformed SDP affects this channel rather than aborting the host.

## RAM and evidence

The ESP32 compiler reports 108 bytes for the client, 28 for its channel adapter and 32 for its result snapshot. The request/result buffers are additional caller choices. There is no heap allocation, task, queue, application callback or product instance in this change, so default runtime RAM does not increase. Compiler-only size evidence is retained under native_bt/_build/experiments/sdp-client/.

Portable Clang strict C11/static analysis and full ESP-IDF 6.1 ccache compilation pass. Source stability/workspace checks cover the new target units. The first target attempt found a GCC indentation warning treated as an error; it was corrected before the successful build. Flash/startup evidence is recorded separately. No unit, simulated peer, SDP, paging, codec, playback or app campaign ran, following the user's instruction.

The product still needs persisted-peer policy, client channel dispatch/creation, A2DP service extraction and outgoing AVDTP setup. These modules are not an enabled automatic reconnect feature. Continuations, early responses, storage exhaustion, ID exhaustion, stale/malformed PDUs, cancellation, MTU changes and closure races remain functionally unqualified. Maximum-rate audio headroom and reported codec crashes also remain unresolved.
