# Owned Classic HCI paging procedure

## Implemented layer

The independently authored dbt_page module adds the outgoing BR/EDR Create Connection/Cancel HCI lifecycle. It is compiled into the native library in both host and ESP32 component manifests. The [GAP integration](GAP_PAGING_INTEGRATION_2026_11_05.md) now supplies owner routing, reservations and deferred incoming handover. Product peer selection/bond policy, outgoing SDP and A2DP initiation remain incomplete. No automatic paging policy is enabled.

## Ownership and completion

All calls run on the existing host owner. One caller context is reserved per procedure, with no allocation, task, lock, busy wait or application callback. The ESP32 compiler reports 48 bytes for dbt_page_t and 16 for its result snapshot. The current product instantiates none, so this pass adds no runtime RAM.

The tracked Create command token is also the procedure identity. The host supplies process-unique tokens that do not wrap across host reinitialization. Rejected begin leaves the old result and caller token unchanged. Address/parameters are copied, and parameter storage may be released when begin returns. Local-controller addresses and existing Classic links are rejected before queuing. A free ACL-slot snapshot is required; the connection manager must keep that slot reserved until completion.

Create Command Status records admission only. Final Connection Complete records the actual outcome/handle. A queued, unsent command can be canceled through its tracked token. Submitted commands retain the context until their callbacks retire. A queued cancel is removed if establishment wins, preventing it from canceling a later procedure for the same peer. A submitted cancel keeps its callback ownership.

Successful Cancel Command Complete still waits for final Connection Complete. If establishment wins, the result is CONNECTED with a live handle, not CANCELLED. The caller must reconcile the handle with the runtime's exact link generation and explicitly disconnect it if cancellation is still required. The module grants no profile access or security trust.

The snapshot exposes Create, Connection and Cancel status independently, with validity bits, plus cancellation/timeout flags. No HCI payload pointer is retained and no UART work runs in this layer.

## Scheduling and error handling

The original page deadline includes initial command queuing. Expiry requests cancellation rather than silently forgetting an admitted controller procedure. Cancellation is queued once; a full host queue waits for its existing completion/capacity notification. Unique-token exhaustion and nonrecoverable submission errors fail closed instead of spinning on an impossible retry. The retirement deadline bounds waiting for the controller's final procedure event. Missing retirement fails the host without resetting hardware; normal page rejection/cancellation does not fail it. Submitted command callback fences additionally retain the host's bounded queue/response deadlines.

The owner combines dbt_page_wait_ms with runtime/host deadlines and forwards final events only after connection-manager arbitration. A final event that wins a queued request still removes unsent commands before allowing context reuse. Host failure requires normal full controller/transport recovery before another procedure, not merely reinitializing the page object.

Packet types are derived from the identified controller's feature page: Basic Rate three/five-slot support and EDR two/three-Mbit and multi-slot support. Unsupported EDR types use the HCI exclusion bits. No supported bitrate or packet type is deliberately disabled. Cached page repetition/clock offset and role-switch permission remain explicit caller parameters.

## Protocol references

The [Core HCI specification, sections 7.1.5 and 7.1.7](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host-controller-interface/host-controller-interface-functional-specification.html) defines command admission, final completion and cancel retirement. Packet-type bit meanings are in section 7.1.5. Local feature positions were checked against the installed SDK's hcidefs.h; no SDK host implementation is linked by this module. BTstack remains a pinned behavior reference only.

Direct dbt_page callers still require exclusive establishment ownership. The GAP integration now arbitrates matching incoming setup by retiring the page before submitting its decision; product policy must use that integration rather than routing competing lifetimes by address alone. Automatic A2DP reconnection remains incomplete.

## Evidence and unqualified work

- Full ESP-IDF 6.1 ccache build, source stability and workspace checks passed. The new target translation unit is present in compile_commands.json and was actually compiled.
- Portable Clang strict C11 syntax/warnings and static analysis passed. These do not simulate controller behavior.
- A target-compiler-only ABI probe reports the context/result sizes above; it executes no code.
- At the standalone 278bd103 checkpoint, the product firmware binary remained exactly SHA256 1814f791e74a2a2461f6b53956d151c13b7f8afd3fa7cf7eb37c259299679ada because unused procedures are removed by the linker. No redundant flash/reset was performed at that checkpoint. Later GAP integration has separate image/deployment evidence.
- No unit, HCI simulation, pairing, paging, playback or app test campaign ran, following the user's instruction. The module is not functionally qualified.

Qualification still needs success/rejection, unsent cancellation, cancellation retirement, establishment-versus-cancel races, full queues, token exhaustion, timeout, malformed/stale events, host abort, context reuse and product collision/admission integration. Maximum-rate audio performance, reported codec crashes and full feature parity remain unresolved.
