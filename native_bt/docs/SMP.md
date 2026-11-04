# Independent SMP work in progress

## Probe attachment candidate

`esp32_probe/main/probe_smp.c` and `.h` provide a separately owned, opt-in
attachment for one active LE SMP procedure. The peer-RPA option is opt-in and
has not been validated on the target or over RF.
Call `probe_smp_init(runtime, owner_task, &config)` after runtime identification
and before enabling advertising or starting an LE connection. It registers LE
CID 6 before any peer can send SMP. The configuration's local address must
equal the controller public address used by this probe's public advertising;
random local and enhanced privacy addresses are currently rejected. The main
fixed-data hook must route CID 6 to `probe_smp_receive` and continue routing LE
ATT CID 4 to GATT. CID 5 is LE signaling and is handled by the channel manager.
An unaccepted SMP PDU must be handled explicitly by the owner.
Forward link changes to `probe_smp_link`. Feed HCI events to
`probe_smp_event` before GAP: a true return claims the LE LTK Request and must
suppress GAP's negative reply. Encryption Change must also reach this function;
it returns false for that event so other observers may process it. Call
`probe_smp_poll(now)` outside runtime callbacks and include
`probe_smp_wait_ms(now)` in the owner's sleep calculation. Central pairing
initiation requires `probe_smp_start(link)`. Central bonded reconnect is a
separate explicit `probe_smp_reconnect_start(link)` on an already established
central-role ACL; neither API creates a connection. `probe_smp_take_failure` reports a failed
procedure for application policy; the adapter does not itself disconnect a
peer or change diagnostic GATT permissions.

Add `dbt_smp_session.c`, `src/esp32/dbt_smp_worker_mbedtls.c`,
`src/esp32/dbt_smp_nvs.c`, and existing SMP dependencies to the target build,
plus SDK mbedTLS, NVS and random-provider dependencies. The adapter starts a
single priority-1 worker lazily on pairing with a **12,288-byte task stack**,
and retires it after secure completion, failure or disconnect. FreeRTOS frees
the self-deleted task's resources when its idle task reaps it; a rapid next
pairing can transiently overlap the departing and new task allocations. The
stack size is not a measured requirement. Worker allocation failure fails the
procedure; a stuck in-flight worker cannot be force-deleted while it might
hold crypto or NVS state. Crypto,
bond lookup, key generation and NVS commit/readback occur on that worker.
Two one-byte FreeRTOS queues signal ownership of single sensitive mailboxes;
each mailbox and worker copy is wiped after consumption. The host never waits
on either queue. Pairing preparation has a 30-second owner deadline; reconnect
uses the host command timeout. Generation
checks discard late results. The worker uses `esp_fill_random`: the target
must ensure the SDK's hardware entropy requirements are met before pairing.
The NVS namespace is `native_le_bonds`; it is never erased or initialized by
the adapter, and the worker closes its handle on retirement. This attachment
does not itself configure NVS encryption or establish its protection at rest.
An ambiguous mutation latches a provider failure in RAM; the persistent intent
below independently blocks access after reset. Stale commits can still have
storage effects when their owner receipt is discarded.

The eight-entry per-identity retry guard table has no eviction. Unknown RPAs
share one bounded retry bucket; a uniquely resolved identity uses its own
guard. To opt in to peer privacy, leave both configured distribution masks at
zero and set `config.peer_identity_only=true` with SC bonding and `require_sc`.
The adapter requests only the peer's identity key: initiator/responder masks
are `(2,0)` when the speaker is peripheral and `(0,2)` when it is central.
The local controller address remains public; no local IRK is distributed.
An RPA first pairing must negotiate the peer IdKey, and its identity address
and IRK are accepted only from encrypted phase-3 distribution. Before storing
an RPA pairing, the worker also verifies that the distributed IRK resolves the
actual connection RPA. They are stored as a provisional candidate and do not
imply MITM authentication. Resolution
uses `ah` on the crypto/storage worker and inspects at most 32 namespace
entries, rejecting malformed records, overflow, ambiguous matches and unknown
IRKs. RPA resolution alone does not establish peer possession of the LTK.
Non-resolvable random peers and enhanced controller privacy address types are
unsupported. With bonding enabled, a peripheral LTK Request for an
otherwise idle public/static or supported RPA LE link is claimed, then the worker loads its NVS
record. The owner checks the generation, stored key size and configured SC and
authentication requirements, matches EDIV/Rand against the role-correct key,
and sends the positive or negative HCI reply. Explicit central reconnect uses
the peer LTK and its stored EDIV/Rand in a tracked LE Start Encryption command;
missing or disallowed records fail without issuing that command. The positive command completion
is not encryption: only its later correlated successful Encryption Change
qualifies the link and sets `peer_key_verified`. Missing records, rejected
policy, allocation failure or expired storage work issue a negative reply;
late work cannot supply a key. The owner must ensure that no other component
replies to the claimed request or concurrently initiates encryption on the
same link. Pairing and reconnect key commands use the shared host's tracked
command API: on disconnect an unsent command is cancelled and wiped, while a
submitted command remains owned until its completion and cannot qualify the
disconnected generation. HCI Encryption Change does not carry a generation:
if a submitted key command or accepted command is abandoned before its
encryption event is resolved, the probe quarantines that connection handle for
SMP until reboot to prevent a late event qualifying a reused handle. This is a
bounded table of 16 handles; overflow disables further SMP procedures until
reboot. Diagnostic GATT remains separately accessible. An SC bond with zero distributed keys can reconnect using its SC
LTK, but this provides no IRK or CSRK coverage. After an RPA resolves to a
provisional candidate, only the correlated LTK encryption event verifies the
peer key; a separate journaled NVS commit is required for the status field
`identity_promotion_committed`. A late receipt cannot qualify the expired
connection even if its storage mutation completed. Direct reconnection to a
different candidate public/static identity is not yet indexed by this adapter.
OOB is refused because the probe has no pre-exchange mechanism for the
worker-owned SC private key. `probe_smp_set_passkey` supplies one fresh value
for one procedure; actual UI, identity binding and truthful I/O capability
are the application's responsibility. Numeric Comparison exposes explicit
value and user-confirmation APIs and is never automatically accepted. The
existing final-key link-ACK provider gap remains: no submission, buffer
completion, NVS commit or successful encryption is converted to an ACK. An
SC configuration negotiating zero distributed keys can progress without an
ACK; that does not establish IRK/CSRK coverage or peer bond persistence.

`test_probe_smp_compile.py` compiled this adapter with the existing Xtensa
probe compiler flags. `test_probe_smp.py` uses the real session/runtime code
with synchronous queue and crypto shims. Its reconnect fixture also runs the
real NVS serialization adapter over an in-memory NVS shim and simulates
controller command callbacks. It covers public-address SC zero-distribution
records, legacy local LTK and EDIV/Rand, failed authentication/SC/key-size
policy, missing records, OOM, late storage, stale generations, explicit central
and peripheral reconnect, role-correct legacy keys, command-versus-
encryption separation, encryption loss, and queued-versus-submitted command
cancellation. Those shims are not physical
flash, FreeRTOS scheduling, actual controller completion, or RF evidence. The
paired-runtime test also exercises an RPA first pairing with real mbedTLS,
encrypted identity distribution, a simulated final-key ACK, journaled NVS
storage, rotated-RPA resolution, LTK reconnect and promotion. It injects
Identity Information and Identity Address as separate ACL packets in one
eight-packet runtime poll, including a case where Encryption Change is in the
same poll; overflow, allocation failure and disconnect wiping are checked.
See the separate paired runtime and mbedTLS worker tests below for their
narrower existing evidence.

### Persistent storage intent

The NVS adapter serializes mutations in the existing dedicated namespace with
one bounded, versioned `smp_txn_v1` intent. It contains the operation, claimed
identity, presence and SHA-256 digest of the prior serialized record, digest
of the intended new record, and a checksum of the intent. It does not contain
key bytes. The adapter commits and reads back the intent before changing the
bond; it then commits and verifies the bond state, removes the intent, commits
and verifies the removal, and only then returns success. Invalid, unsupported
or unreadable markers fail closed; a valid pending marker blocks normal loads
and mutations. SHA-256 detects accidental inconsistency, not malicious NVS
tampering; namespace confidentiality and authentication remain platform duties.

Recovery is explicit and worker-only. `dbt_smp_nvs_pending` reports the
non-secret identity and operation. `dbt_smp_nvs_reconcile` requires that exact
identity and operation, validates and hashes the current record, and removes
only the intent if it matches its recorded before or after state. Its `applied`
result identifies the matching target state; false means the before state is
still present. Neither result proves peer storage or that a flash write was
performed. A third state, invalid record or marker, or cleanup failure remains
blocked. Reconciliation never rewrites a bond or erases the namespace.

The probe offers asynchronous `probe_smp_storage_inspect_start`,
`probe_smp_storage_reconcile_start`, and `probe_smp_storage_result` for an
application-controlled recovery flow. They run only when no SMP procedure is
active, use the lazy worker, and retire it before returning a result. They do
not run automatically or supply user approval. Old version-2 records remain
readable without migration, but historical failures from before journaling
cannot be detected from an absent intent. A host in-memory model exercises
power-cut stages and failures; NVS commit/readback does not itself establish
physical power-loss behavior.

This is experimental offline code, not a target-integrated security provider. It has not been linked into the ESP32 firmware or tested over RF and must not be used to grant production access or claim complete SMP support.

The feature-exchange engine negotiates an owned legacy or LE Secure Connections phase-2 engine using the exact request and response transcript. Its retry guard must be bound to a claimed peer identity and retained across links; the owner must report the final verified pairing result. Keypress notification, the Peripheral Security Request, cross-transport key derivation and automatic I/O interaction are not implemented. Legacy bonding currently requires both peers to distribute an LTK; LE Secure Connections rejects unilateral OOB and unsupported distribution masks rather than downgrading.

The legacy engine supports Just Works, Passkey Entry and bilateral OOB, and can distribute LTK/EDIV/Rand, identity and signing keys after a correlated encryption event. Passkey UI must truthfully report I/O capabilities and create a fresh six-digit value for every pairing; legacy passkeys have limited entropy and limited resistance to attacks. OOB authentication is an application assertion about a peer-bound secure OOB channel, not a property the engine can establish. Security fields are valid only in the secure state; Just Works is unauthenticated.

The LE Secure Connections phase-2 engine supports Just Works, Numeric Comparison, twenty-round Passkey Entry and bilateral OOB. It checks peer commitments and DHKey checks before releasing a candidate LTK, requires user acceptance for Numeric Comparison, and waits for a correlated encryption event. Its phase 3 supports negotiated identity and signing keys. For OOB the application must have exchanged identity-bound data securely before pairing, and the worker must retain the private key corresponding to the exchanged local public key. The application asserts whether that channel supplies authentication.

Both engines start bond storage only after encryption and phase-3 completion and mark a bond committed only after an atomic, durable storage acknowledgment. The owner supplies trusted previous bond properties to the downgrade gate. The storage implementation must serialize this lookup and commit, including other transport writers, or recheck the policy atomically. A reconnect engine loads a trusted identity, checks security properties, key size and EDIV/Rand, releases the appropriate local or peer LTK once, and waits for a correlated encryption event. An identity address received during distribution is provisional: promotion requires a reconnection using that address and LTK and a further durable storage acknowledgment. The caller must bind the candidate address to the actual controller connection; privacy address resolution and signing counter management remain outside this code.

The isolated SDK NVS adapter uses a dedicated namespace handle supplied by the application. It serializes a versioned bond record, checks downgrade policy, calls nvs_commit, and verifies readback. It does not initialize or erase a partition or namespace. All namespace access and competing writers must be serialized on a storage worker. A failed or ambiguous mutation poisons the adapter; reopen and reconcile storage before retrying. A failed delete does not establish that a record disappeared. Record format version 2 has no migration implementation. The in-memory NVS shim tests adapter behavior only and do not demonstrate real flash durability or power-loss recovery.

The owner-only fixed-channel adapters check LE CID, handle and generation and retain one blocked inbound PDU per link. A further PDU receives backpressure; the caller must bound any additional queue or disconnect. They do not reassemble ACL or attach to the runtime. Intermediate phase-3 PDUs advance after ordered non-automatically-flushable transport admission, without asserting delivery. The receiver of the globally final key completes phase 3 on receipt of that valid PDU; its sender instead waits for a genuine acknowledgment of every link fragment containing that final key. The submission token and link generation correlate the admission and final-ACK callbacks, including an ACK arriving before the admission callback. An absent or negative final ACK leaves that sender incomplete or failed; timeout and disconnect must never promote the bond.

`dbt_smp_session` is an optional attachment manager for one existing LE runtime link. It registers CID 6, stages feature exchange and allocates exactly one selected legacy or SC bearer after negotiation. Its owner must forward the runtime's fixed-data, link and HCI packet hooks, call session poll outside callbacks, and dispatch copied crypto and storage jobs to workers. The Peripheral LTK Request must be routed through exactly one session/GAP path so that GAP does not also send a negative reply. The Central submits LE Enable Encryption and the Peripheral answers only a matching zero Rand/EDIV pairing LTK request; a successful correlated command result and later Encryption Change are distinct gates. A premature encryption event is rejected. The caller must use actual connection addresses, guard their generations against stale events, and disconnect or otherwise revoke link access on session failure; the shared runtime can update its raw encrypted-link flag before the session rejects an event. Do not authorize services from that flag alone.

Session storage waits are bounded by the existing host command timeout even if no worker claims the job. A timed-out or canceled worker write can still reach storage and requires provider reconciliation. Closing waits for borrowed runtime transmit data and outstanding HCI commands. Newly paired sessions report peer reconnect qualification as false; the separately owned reconnect engine remains the path for validating a later bond use. Zero SC distribution masks mean no identity or signing keys were exchanged, regardless of successful encryption or local bond commit. OOB session attachment, real storage and actual controller/RF behavior have not been demonstrated.

The session status reports negotiated key masks separately from phase-3 completion and the admitted-local/received-peer masks. An admitted local key is not a claim of peer consumption. The manager currently handles HCI Encryption Change v1 for pairing on an initially unencrypted link; v2 events, encryption refresh, bonded reconnect attachment and handle-reuse quarantine remain integration work.

`dbt_smp_dist_hci_buffers_completed` records only correlated HCI buffer accounting. It does not advance phase 3: the HCI specification permits an implementation to release a buffer early after moving data to other storage. `local_pdus_admitted`, `hci_buffers_completed`, `peer_final_received`, `final_ll_acked`, `encryption_verified`, `bond_committed`, `peer_key_verified`, and `identity_promotion_committed` have distinct meanings. Encryption and reconnect flags require the owner to bind the supplied key and successful event to the correct connection generation; they are not direct RF measurements. Neither local commit nor HCI submission proves peer storage. See Core 5.4 Vol. 3 Part H section 3.6.1 and Vol. 4 Part E section 4.1:
https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/security-manager-specification.html
https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host-controller-interface/host-controller-interface-functional-specification.html

ESP32 provider gap: the existing SDK's `components/bt/include/esp32/include/esp_bt.h` documents VHCI readiness and HCI receive callbacks, and `components/bt/controller/esp32/bt.c` delegates submission to opaque `API_vhci_*` functions. Read-only symbol inspection of the existing `controller/lib_esp32/esp32/libbtdm_app.a` found VHCI entry points, but no documented per-SDU final-key acknowledgment contract was located. Symbol names alone do not establish semantics. No supported controller ACK provider is implemented or assumed; the final sender therefore cannot report successful phase-3 completion on this controller until a suitable provider is documented and integrated. The worker, storage, guard reporting, bearer, controller encryption events and user I/O still need runtime integration.

The separately owned mbedTLS worker uses the existing SDK interfaces for AES, CMAC, P-256 validation, key generation and ECDH, rejects debug keys and reflected public coordinates, and clears its private key after derivation. It must run on a crypto worker, with a correctly seeded platform CSPRNG supplied by the application. The worker must be zero-initialized, is bound to one generation, and must be closed and replaced after failure or completion. OOB preparation reuses its pre-exchanged key only within that generation. Generations must not be reused while work is outstanding. Crypto, storage and NVS commits must not run in the host loop or controller callbacks.

SDK-only worker and NVS implementation files are under `native_bt/src/esp32/`; the portable `native_bt/src/*.c` units and public SMP headers do not include SDK or mbedTLS headers. The exact Linux upload list and sanitizer prerequisites are in `native_bt/tests/smp/linux_inputs.txt`. `test_smp_linux.py` drives ASan/UBSan for the portable code, SDK mbedTLS worker and NVS shim; this Linux driver has not yet been executed in the current workspace. LeakSanitizer is disabled for the CPython-hosted run.

Offline verification: test_smp.py checks Bluetooth Core v5.4 crypto vectors, feature exchange, paired legacy and SC exchanges, failure paths, bounded backpressure, key distribution, identity promotion and simulated bond lifecycle. test_smp_worker.py builds the existing SDK mbedTLS sources for Windows and tests real P-256/AES-CMAC and AES peer exchanges with a host CSPRNG. The current test_smp_session.py fixture passes paired portable runtimes with an in-process HCI controller and the SDK mbedTLS worker, including legacy Passkey, SC Just Works, Numeric Comparison, Passkey, stale storage receipt, missing final ACK and simulated commits. test_smp_nvs.py exercises the NVS adapter against an in-memory failure-injectable shim. test_smp_mbedtls_compile.py compiles the isolated SMP units with the existing ESP32 SDK configuration; it does not link or run firmware. Deterministic test keys and passkeys are fixtures. These checks do not prove real flash persistence, target linking, radio operation, phone interoperability, resource use or performance.

Compile-only ESP32 sizes are 480 bytes for a session, 936 bytes for legacy state, 1,484 bytes for SC state, 956 and 1,552 bytes for their bearers, 64 bytes for feature state, and 532 bytes for reconnect state. Each active pairing session allocates an eight-PDU receive queue (528 bytes) and then only the selected bearer, plus alignment padding; the queue is wiped on failure and released on close. This accepts an eight-ACL runtime burst without performing crypto or storage in callbacks; overflow fails the procedure explicitly. The probe also allocates a reconnect object only for an active attempt. These are not peak heap measurements: the mbedTLS worker allocates an internal context and MPI/ECC state, and copied jobs, queues, storage buffers and simultaneous links add memory. No target resource or latency criterion has been established.

Bluetooth Core v5.4, Volume 3 Part H, sections 2.2, 2.3, 2.4 and appendix D:
https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/security-manager-specification.html
