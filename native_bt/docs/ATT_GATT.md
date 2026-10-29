# ATT server and LE bearer

`dbt_att.c` owns an unenhanced LE ATT server. Its ordered immutable attribute
database can remain in flash. `dbt_att_bearer.c` connects it to fixed CID 4 in the
owned runtime; it does not replace the dispatcher for SMP or other profiles.

Implemented server procedures include MTU exchange through 517, 16/128-bit UUID
discovery, service groups, reads/blobs/multiple reads, ordinary writes/commands,
prepare/execute/cancel, notifications and confirmed indications. Unsupported
commands are ignored; unsupported requests receive an ATT error. EATT, signed
writes, robust caching/database hash and GATT client procedures remain pending.

The protocol definitions are reviewed against the Bluetooth SIG
[ATT specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/attribute-protocol--att-.html)
and [GATT specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/generic-attribute-profile--gatt-.html).
No implementation source from BTstack is included or linked.

## Ownership and memory

All server/bearer calls run on the host owner. Short response and notification
slots are 23 bytes each. Long outgoing data and prepared write fragments allocate
only when needed, and are released on admission, cancellation or disconnect.
Ordinary read allocation uses the attribute's size bound and negotiated MTU,
rather than retaining a full MTU buffer per idle connection. There are at most
eight prepared fragments per client, with an explicit total payload budget.

The diagnostic server plus runtime bridge occupy 784 bytes on ESP32, excluding
the flash database, stack, long replies/prepares, and the diagnostic echo value.
This is an additional feature cost, not an equivalent-feature stack benchmark.

A pinned ATT view must remain valid until the runtime returns its borrowed SDU.
`DBT_TX_ADMITTED` means every fragment was accepted by HCI transport; it does not
mean the peer received it. Indication success requires the client's confirmation.
Thirty-second transaction expiry makes the bearer terminal and informs its owner.
The owner must close/reestablish the connection rather than reuse that bearer.

## Security, subscriptions and writes

Read/write policy checks encryption, authentication, minimum key size and optional
authorization. Encryption alone never establishes authentication. A separate
security owner must provide verified key/security metadata; SMP and the bond store
are still pending. Notification-only values retain their security checks.

CCCD values are per client. Unbonded connections start from zero; a security owner
can restore bonded settings through the load callback. Bonded CCCD updates require
an application write callback that persists the batch. These hooks have native
tests, not completed production bonding/power-loss qualification.

Prepared writes reach one application batch callback in FIFO order. The callback
must validate the entire batch and commit atomically, or fail without mutations.
The server does not claim to make an arbitrary application callback atomic.

For work such as NVS, the callback can copy a bounded job and return
`DBT_ATT_DEFERRED`. The ATT write response waits for `dbt_att_complete` on the host
owner, using both connection generation and transaction token to reject stale
worker completions. The application migration must still commit **every EQ update
immediately**, with no debouncing/coalescing/idle deferral; deferred ATT response
does not authorize deferred persistence. This worker/NVS mapping is not wired into
the product yet. Write commands have no ATT response and require application-level
acknowledgement if durable completion must be reported.

## Verified scope

The current native suite has 136 groups, including Classic security, SDP, receive flow and AVDTP. ATT
checks include 10,240 generated opcode/length
cases with allocation reclamation, security, per-client CCCDs, prepare limits/OOM,
deferred completion, timeouts, and paired runtime fragmented reads/disconnects.
Linux normal and ASan/UBSan runs pass.

`build_probe.ps1 -WithGatt` enables an unbonded diagnostic echo/counter service
under UUID `12345678-1234-1234-1234-12345678f000`. It preserves Classic capability
flags but does not offer Classic profiles. The default probe keeps advertising off.

Live Linux BlueZ 5.85 testing selected LE with `Device1.PreferredBearer=le` and
`Device1.Connect` (the experimental preference API was temporarily enabled in the
test VM). Discovery, 200 request writes, 20 periodic readbacks, lengths 0/1/20/64/
244/512, a 512-byte reliable write and 208 notifications passed on the ESP32.
Measured median/max write RTT was 100.114/300.234 ms; maximum notification interval
was 707.809 ms in the 29.6-second run. This is not a throughput, audio continuity,
bonding, or simultaneous Classic/BLE qualification. Earlier connection attempts
using generic/separate-bearer paths timed out; traces and failures are preserved
privately. The direct kernel LE test also negotiated MTU 517 and read the name.
