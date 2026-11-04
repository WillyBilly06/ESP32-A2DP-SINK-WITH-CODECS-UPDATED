# Independent BLE OTA provider

This provider is unwired and disabled until the application explicitly creates
an OTA object, supplies all hooks, authorizes the peer, selects a slot and routes
commands to it. Its baseline wire opcodes are begin `20 <length-le32>`, data
`21 <sequence8> <bytes>`, end `22`, and abort `23`. Responses use baseline
`OTA_READY` (`21`), progress (`20 <percent>`), complete (`22`), failed
(`23 <reason>`) and abort ACK (`10 23`). Ready follows successful backend setup;
progress follows completed writes, and complete means the image was verified
in the inactive slot. Complete does not select it or reboot.

The baseline command format has no independently trusted image digest or CRC.
An explicit local grant must authorize the current encrypted, bonded peer and
project; the hook rechecks it and the peer generation on every command and
activation. The grant can optionally carry an independently trusted CRC-32 and
SHA-256. With no trusted digest, SDK verification of the image's embedded hash
plus stream-to-flash SHA-256 readback check detects corruption, but does not
authenticate the firmware. A digest sent by the same untrusted BLE peer is not
independent authorization. No manifest loader, signature verifier, owner UI,
grant issuer or signing infrastructure is included. The first sequence value is
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

Use `dbt_ota_control_command` directly when the control-hook context is the OTA
object, or route opcodes `20` through `23` via an application-owned provider mux.
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
