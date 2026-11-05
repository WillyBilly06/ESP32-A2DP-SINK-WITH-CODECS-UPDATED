# GAP-owned Classic paging and handover

## Implemented interface

GAP now provides dbt_gap_connect/cancel/result around the owned dbt_page controller procedure. It checks incoming identity ownership, current links and pending accepted ACL reservations before starting. The caller authorizes peer selection and lends an initialized 48-byte page context plus its matching runtime ACL view. Both remain valid until GAP's terminal result consumes the operation. A standalone page result is not the storage-release fence after GAP registration.

The result retains the original outgoing HCI result separately from an incoming handover's command/final statuses and the overall handle/generation. User cancellation is separate from internal collision cancellation. A connected link can win cancellation; callers must explicitly disconnect its exact generation if cancellation is still desired. Terminal completion is consumed once; stale tokens return EMPTY. No pairing window, bond privilege or protected-profile access is granted here.

## Simultaneous connection ownership

A matching incoming request is copied into the existing incoming registry and held in its bounded reply queue. GAP requests cancellation of the outgoing page and waits for its command/final-event fences before submitting the incoming decision. Only one capacity reservation covers both directions. Unrelated mandatory replies bypass the deferred descriptor; completed descriptors can retire even when HCI command capacity is full. No already-published HCI bytes are moved.

If the outgoing link establishes first, it remains the reported winner and the pending incoming request is rejected. If the page retires first, the existing incoming policy decision can proceed. The incoming reply and its final event retain the overall operation until retirement. Incoming statuses and outgoing statuses are not overwritten into one ambiguous result. Runtime publishes ACL links before GAP handles final events; the selected link generation is captured at that boundary.

All operations use owner events/deadlines. A deferred decision does not make the owner immediately runnable merely because it is queued. Rename/setup waits for actionable mandatory reply work; no polling spin, task or delay loop is added.

## Controller timing and error corrections

GAP explicitly programs a 5000 ms Connection Accept Timeout through HCI 0x0c16 before READY. Deferred incoming decisions have a 4500 ms host response bound, while page cancellation retirement is capped at 4000 ms. An admitted incoming decision then receives the existing 60000 ms final-event guard. These are explicit host policies, not claims of measured latency or full RF qualification. The [Core HCI specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host-controller-interface/host-controller-interface-functional-specification.html), sections 6.7, 7.1.5/7 and 7.3.14, supplies the timing units and ownership boundaries.

Expected cancellation of an unsent incoming command after its final event no longer marks GAP failed. Its tracked callback still retires and is logged as ABORTED with no fabricated controller status. Other transport/controller failures retain closed failure handling. ACK stage flags are now published after applying their admission outcome.

SCO/eSCO rejection uses the synchronous command, and only ACL Connection Complete events reach the outgoing page. This prevents an unsupported synchronous procedure's failure from being treated as the ACL page result.

## RAM and evidence boundary

The ESP32 compiler reports GAP state 392 -> 412 bytes, a 20-byte increase. A caller-owned page context remains 48 bytes and the copied GAP completion snapshot is 32 bytes. No product paging instance, new task, queue or heap allocation is added. Existing security log records remain unchanged; a resolved stage records a deferred decision's final choice.

Full ESP-IDF 6.1 ccache compilation/source stability/workspace checks and portable Clang C11 static analysis pass. Compiler-only ABI evidence is retained under native_bt/_build/experiments/gap-paging/. Flash readback and ordinary startup are recorded separately. No paging, collision, pairing, playback or app test campaign ran, following the user's instruction.

The product still does not select a peer or call this API automatically. Persisted-peer policy, outgoing SDP and A2DP initiation remain incomplete. End-to-end reconnection, all controller/error interleavings, maximum-rate audio headroom and the reported codec crashes remain unqualified. This is controller/GAP integration, not completed automatic A2DP reconnection.
