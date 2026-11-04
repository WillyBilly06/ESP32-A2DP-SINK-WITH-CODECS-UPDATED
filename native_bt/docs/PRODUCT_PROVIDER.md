# Product provider integration

`probe_product_startup_worker()` runs off the host loop, loads the existing NVS
snapshot, verifies that the running and selected partition is an owned
`native_bt_probe` OTA slot, and chooses the opposite `ota_0` or `ota_1` label.
It does not erase, select or restart. The object, label and hook context must
outlive worker and owner use. Startup failure leaves the provider unavailable;
the caller can still choose how to expose other GATT functions.

The owner publishes `dbt_product_peer_value_t` with the exact current handle and
generation, independently verified encryption and durable bond state, and
explicit name/OTA grants. A normal baseline OTA grant sets `allow_ota`, project
`native_bt_probe`, `trusted_digest=false`, and length zero to accept the size in
the begin command. A pinned grant can instead provide length, CRC and trusted
SHA-256. Revocation publishes a zero-generation snapshot, or invokes
`probe_product_peer_lost()` for the matching peer. The snapshot uses sequentially
consistent atomic fields with bounded reads; the worker does not inspect mutable
host or SMP state. The owner must derive bond/encryption from actual security
completion, not HCI command admission. The grant and its source are the owner's
policy responsibility; the module does not provide a UI or authenticate images.

Route the GATT generic command hook to `probe_product_command()` with the
product object as context, or provide a mux. It handles name; epoch-bound OTA
uses the separate GATT OTA hooks described in `OTA.md`. LED and
sound require an explicit `other_command` provider whose success means its
operation completed. Unavailable providers fail. EQ and flags keep their existing
separate persistence/publication hooks and EQ worker. Route the GATT owner-side
peer-lost hook to `probe_product_peer_lost` and the general worker's periodic
and retirement hook to `probe_product_worker_poll`. Retirement must be refused
when it returns false; it retries backend cleanup on the next retirement poll.
The worker must poll at least every 20 ms or on owner wake while active so
disconnect, timeout and explicit actions do not depend on another command.
`wake_owner` and `wake_worker` are bounded notifications, not work callbacks.
Configure GATT's trusted-control cookie capture and worker-safe validation
before initialization. Every privileged remote job captures the current cookie;
queued non-EQ providers revalidate immediately before provider entry and the
owner revalidates before success and status publication. Accepted EQ/control
persistence obligations still commit after revocation, but stale success and
status are suppressed. A provider already past its admission fence may finish
an in-flight operation; the stale result cannot become a new grant's ACK. A
success response still waiting for an ATT TX loan is fenced on revocation; an
already borrowed runtime TX cannot be retracted by this layer. Write Commands
arriving while such an ACK is pending are ignored so they cannot replace its
ATT token; clients should wait for the request response before sending them.

Name writes commit `audio.devname` on the worker, then publish a tokenized
owner request. The owner takes it with `probe_product_take_name()`, applies the
name to GAP/advertising, and calls `probe_product_complete_name()` only when
that operation actually finishes. The worker sends success only after both
commit and apply succeed. A timeout or failed apply can occur after the NVS
commit: it returns an error, but the committed request remains available for
the owner to apply. The owner must reconcile GATT's status snapshot with the
durable name after a late apply; an old completion token cannot complete a new
request. Generic worker waiting is bounded by the configured name timeout.

After an authorized OTA end is verified, the owner-driven GATT handoff first
requires a successful quiescence preparation, then calls
`probe_product_request_activation()` and the worker performs selection.
`PROBE_PRODUCT_SELECTION_AMBIGUOUS` must be reconciled before another update or
restart decision. No worker resets the device. The selector/rollback and
integrity limits in `OTA.md` apply, especially the distinction between the app's
compile options and the preserved bootloader's actual capabilities.

`python native_bt/tools/test_product.py` runs the actual control parser, product
mux and OTA state/ESP adapter with in-memory NVS, SDK and SHA shims, plus a
concurrent snapshot publication/read stress test; it compiles the new product
units with the ESP32 toolchain. Tests model stale generations, cancellation,
timeout, name commit/apply outcomes and ambiguous selector failure. They do not
prove target link, actual NVS/flash, image integrity, timing, phone behavior or
RF. `python native_bt/tools/test_ota.py` covers the independent OTA state and
selector model.

The main glue initializes this provider on its startup worker. The runtime
publishes name/OTA access only for a live encrypted committed 16-byte bond that
also matches the local trusted-control policy. OTA may use a null manifest;
this is peer authorization, not image authenticity. The main glue refreshes
security before fixed-CID ATT dispatch, reconciles durable names through the
correlated asynchronous GAP operation, and updates the GATT name only after
success. `probe_application_set_other_provider` is a pre-startup registration
point for a future LED/sound provider and its nonblocking audio-busy snapshot;
without one those commands fail. Its mux rejects name/OTA during a busy sound
loan and defers activation while busy, without deferring EQ commits. The owner
must wire the pre-selector preparation and restart coordination in `OTA.md`;
object compilation alone does not prove that integration.
