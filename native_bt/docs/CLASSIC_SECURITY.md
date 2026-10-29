# Classic security ownership

`dbt_classic_security` owns host policy and procedure state. The retained controller
performs link authentication, SSP cryptography and encryption. BLE SMP is separate
and remains unimplemented.

The module handles IO capabilities, explicit Just Works policy, confirmation,
passkey/PIN interaction, OOB replies, known-key lookup and key updates. It rejects
debug/unit keys, tracks effective key type after a changed key, and uses explicit
encryption/key-size completion before authorizing protected channels. Controller
command admission is not treated as authentication success. A colliding incoming
procedure can complete through actual events within a bounded deadline.
Incoming Simple Pairing Complete no longer starts a competing encryption request.
Local service demand made during SSP is deferred until pairing/key notification
completes, and local encryption waits for actual authentication completion.

Key and cipher epochs prevent an old size-query result or key rotation from
authorizing a new cipher. Link generations and storage tokens reject stale callbacks.
The optional L2CAP security provider permits a pending connection response while
security runs; final connection success still requires the authorization check.

## Persistent-store boundary

The core's load hook must be nonblocking. The store hook admits a bounded job;
the owner calls completion only after durable success or failure. Admitting a job
does not mark the bond durable. Delete follows the same rule and invalidates
authorization. Queued sensitive HCI records and consumed command parameters are
cleared after their ownership ends.

The diagnostic ESP32 adapter preloads its separate `native_bonds` namespace, then
uses a bounded queue and low-priority NVS worker. Each accepted key update/delete
performs its own NVS commit. It preserves the original application's namespace.
Only the explicitly selected test peer can pair or request encrypted administration.
Security logs contain metadata; keys, PINs, passkeys and OOB values are not logged.

## Evidence and remaining qualification

Native policy/lifecycle tests and Linux sanitizers pass. Live restricted Just Works
pairing, a committed native bond, later record reload, and encrypted channels with
a 16-byte key were observed. Encrypted administration produced a bond-removed event
after commit; a subsequent probe boot loaded zero bonds. These observations are
not physical power-loss qualification or verification of every pairing method.

The Linux experimental GATT path later lost its diagnostic Classic bond. A kernel
ATT/Classic run retained enough bond state for encrypted cleanup, but traffic failed
with changed bytes already visible in the target's reassembled receive payload.
The source is unresolved. PIN/passkey/OOB methods, Secure Connections policy,
production pairing controls, recovery and broader interoperability need live tests.
