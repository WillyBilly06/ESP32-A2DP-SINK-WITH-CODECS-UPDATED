# Bonded CCCD persistence: offline implementation

The product GATT adapter binds a live connection to the exact stable identity
and an opaque SHA-256 digest of its committed bond material. The SMP worker
computes that digest when it stores or loads the bond; the host receives only
the digest, never the keys. RPA reconnect uses the resolved storage identity.
When an identity candidate requires promotion, binding waits for its durable
promotion and verified encryption.

The non-EQ control worker loads and saves the four CCCDs in one NVS blob per
identity. The blob stores the complete digest; a new bond with different key
material loads zero subscriptions until it writes its own. The FIFO worker
orders accepted saves before later reconnect loads. Owner completions check
handle, generation, identity and digest. A stale save may have reached flash,
but cannot acknowledge or attach state to a later connection. A bonded ATT
request is acknowledged only after `nvs_commit` succeeds; commands and mixed
attribute/CCCD batches are rejected on this path. Unbonded subscriptions are
ephemeral. Storage failure rejects the request; an ambiguous save poisons the
provider for the remainder of the boot. A failed worker allocation or load
job allocation is retried with bounded scheduling delay; accepted EQ jobs
remain on their existing independent immediate-commit lane.

The product entry point must configure both SMP security and committed-bond
query callbacks before GATT initialization. Both features must link
`probe_cccd_store.c` exactly once. The adapter does not have an unbond API:
after bond removal the old blob can remain, but it cannot be restored under a
different bond digest. Repeated re-pairing of one identity reuses its key;
historical distinct identities can leave orphaned blobs until explicit future
maintenance. No automatic erasure is performed.

Offline evidence: `test_ble_control.py` exercises the actual GATT attachment
against a deterministic worker/storage shim, including commit-before-ACK,
reconnect, changed incarnation, stale completion, failure and allocation
retry. `test_probe_smp.py` exercises the RPA and generation gates and the real
provider with SDK SHA-256 and an in-memory NVS shim. These do not establish
flash behavior, radio operation, or hardware playback.
