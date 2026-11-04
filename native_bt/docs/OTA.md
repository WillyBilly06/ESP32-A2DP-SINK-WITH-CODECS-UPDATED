# Independent BLE OTA provider

With the required product/GATT hooks configured, the OTA path uses baseline wire opcodes:
begin `20 <length-le32>`, data
`21 <sequence8> <bytes>`, end `22`, and abort `23`. Responses use baseline
`OTA_READY` (`21`), progress (`20 <percent>`), complete (`22`), failed
(`23 <reason>`) and abort ACK (`10 23`). Ready follows successful backend setup;
progress follows completed writes. On the epoch-bound GATT path, complete is
held until verification and the owner-requested selector action both succeed.
The completion notification must reach runtime TX admission before a restart
handoff is eligible; TX admission is not proof of peer receipt.

The baseline command format has no independently trusted image digest or CRC.
The owner queries the locally admitted trusted control bond at begin and
rechecks its exact resolved identity, bond-incarnation digest, handle and
generation. Each transfer gets a fresh nonzero command epoch, copied into every
queued worker job. Re-pairing, revocation, disconnect, stale completion or an
epoch mismatch cancels authorization; encryption or a bond alone is not a
grant. The grant can optionally carry an independently trusted CRC-32 and
SHA-256. With no trusted digest, SDK verification of the image's embedded hash
plus stream-to-flash SHA-256 readback check detects corruption, but does not
authenticate the firmware. A digest sent by the same untrusted BLE peer is not
independent authorization. No signed image manifest loader, image-signature
verifier or signing infrastructure is included. The first sequence value is
accepted as the starting value, then increments modulo 256; duplicates and
out-of-order chunks fail. An 8-bit replay after exactly 256 chunks cannot be
identified by sequence alone, and requires a trusted digest to detect at finish.
The product app's actual sequence and burst behavior have not been verified.

The ESP backend needs an explicit approved OTA partition label and rejects the
`recovery` label, the running or selected boot partition and oversized images.
The running and selected boot partitions must match and have a valid or
undefined SDK OTA state. An inactive slot may be reused even if an older
selector references it. The owner must ensure the chosen label does not refer to
another image that must be preserved. Existing baseline and recovery images are
never selected automatically. It starts an SDK app-update session via
`esp_ota_resume(..., OTA_WITH_SEQUENTIAL_WRITES, 0, ...)`; in the inspected SDK,
`esp_ota_begin` can erase the inactive otadata record, so it is deliberately
avoided. Only the approved inactive application slot receives sequential erase
and writes. On completion, the SDK image verifier checks image integrity and
chip/revision compatibility; the provider requires an embedded image hash,
checks project identity against the grant and running application, and compares
the received stream's SHA-256 against a full flash readback. Exact transfer length
is always checked; CRC and SHA-256 are also checked against an independently
trusted manifest when one is supplied. These checks are not a substitute for a
trusted grant or configured secure boot.

`dbt_ota_activate()` is a distinct worker-side owner action after verification;
it rechecks the grant and image, requires a valid or undefined running OTA slot,
and only then asks the SDK to select the staged image. It never resets.
Selection mutates otadata and an error can leave its outcome ambiguous; the
provider enters `DBT_OTA_FAULT` and the owner must inspect and reconcile state
before further actions. On power loss before selection, a new RAM session cannot
activate the partial or completed slot. The SDK selection operation rewrites the
inactive selector sector while retaining the active selector; a local model
covers erase, invalid partial record and complete record. App compile flags do
not establish what rollback behavior the preserved bootloader supports; manual
recovery and an explicit activation decision remain the owner's responsibility.
This is a software model, not real flash power-cut or bootloader evidence.

The GATT adapter requires the status CCCD and the configured OTA admission,
worker, selector and TX-admission hooks. It rejects new transfer jobs after end
is admitted and retains the end request while the owner schedules the selector
on the worker. A failed or ambiguous selector cannot produce a successful end
ACK or restart handoff. Selector admission can race revocation or timeout;
the selector may already have changed in that case, but the result is treated
as ambiguous and cannot authorize restart.

After verification, before selector admission, the adapter fences new GATT
mutations and normal local EQ/control admission. The owner-side `prepare` hook
must pause physical sampling and drain already-observed events individually
through the narrow `probe_gatt_control_eq_delta_drain` and
`probe_gatt_control_toggle_drain` operations. It must wait for each immediate
commit and for `probe_gatt_restart_prepare_quiescent`, which excludes the held
end request but checks other GATT/CCCD jobs, pending status and ATT TX loans.
The hook returns BUSY while progressing and INVALID on failure; neither blocks
the general worker. Selector and complete ACK wait for successful prepare.

After selection `probe_gatt_restart_quiescent` checks the remaining GATT work.
The owner must also fence and drain all other mutation producers, admitted
persistence and trust/security work, A2DP/output and sound loans, and refuse
restart on any latched fault or retained input event. Only after those checks
may the owner pass true to
`probe_product_runtime_restart_ready`. The handoff is single-use, requires
fresh trust, starts no earlier than 500 ms after TX admission, and expires after
10 seconds. On fault or timeout, cancel the GATT fence and do not reset. The
current product build wires the coordinator and terminal action described in
`OTA_RESTART_BARRIER.md`; this is source/target-link evidence, not an executed
ESP32 update. Merely having
historical `VERIFIED` state never restarts. No implementation may infer peer
receipt from runtime TX admission.

The standalone `dbt_ota_control_command` remains useful in isolated state tests.
`dbt_ota_esp_*` functions accept `dbt_ota_esp_t *` as their backend context; a
composite application context needs thin wrappers to combine them with the grant
and clock hooks. All commands, backend functions, `dbt_ota_tick`, and activation
are serialized on the generic control worker, never on the host loop. On link
removal, the owner calls atomic `dbt_ota_request_cancel` and must wake/schedule
the worker to call `dbt_ota_tick`, including on timeout when no command arrives.
Cancellation cannot interrupt an already executing flash operation. The session
object and approved label must outlive every worker reference.

No frame-sized buffer is allocated by this provider: SDK session allocation is
lazy and readback uses a 512-byte worker-stack buffer. The flash driver may
serialize with immediate EQ NVS commits; target latency, stack and audible
effects remain unmeasured. Queue admission, an ATT write response, and flash
operation completion are separate events. The existing application wiring,
partition layout and rollback configuration must be reviewed before enabling.

`python native_bt/tools/test_ota.py` runs portable state/wire tests and the ESP
adapter against in-memory SDK and hash shims, then compiles both ESP32 source
units, including a forced rollback-enabled compile branch. These mocks do not
test real SHA implementation, flash erase, image verification, boot selection,
hardware, or power-loss recovery.
