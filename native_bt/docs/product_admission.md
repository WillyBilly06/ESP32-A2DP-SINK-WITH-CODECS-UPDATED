# Product pairing admission and durable LE trust

`DBT_PROBE_PRODUCT_SECURITY` is a separate product policy. Without it the
existing Classic diagnostic adapter still admits only the configured test
address. Product Classic accepts known cached durable bonds outside a pairing
window; an unknown peer and any new Classic key require a current local grant
bound to its live ACL generation. LE bonded reconnect remains available outside
the window, while every fresh SMP pairing procedure requires a live grant.

There are two grant origins. A physical pairing gesture opens or extends a
120-second window. A power-on window opens for 120 seconds only after the
worker successfully enumerates both owned bond namespaces as empty, the trust
index is empty, and the Classic cache successfully loads zero bonds. A missing
namespace is empty; an entry, pending intent, over-limit scan or enumeration
error is never silently treated as empty. This allows first setup when the
optional input device cleanly NACKs, without treating failed cleanup or a
general input error as absence. Expiry removes pairing grants and pairing LED
feedback; it does not disconnect an established audio or LE link.

For product mode, an unencrypted LE peripheral connection receives a bounded
SMP Security Request containing the configured bonding, MITM and Secure
Connections requirements. The peer chooses whether to start encryption or
pairing; transport submission proves neither. The request may be sent once
outside the pairing window to prompt reconnect encryption for existing bonds.
Because the peer's bond identity cannot always be resolved before encryption,
an unknown peer might also show an OS pairing prompt outside the window. Its
actual Pairing Request is still refused unless a local window is open; the
prompt itself creates neither a bond nor control trust. Each distinct physical
gesture may prompt again after the cooldown. Unsent requests have a bounded
lifetime and are cancelled on expiry, restart freeze, link replacement, or
peer-initiated pairing/encryption. An exact generation/CID/callback/context
fence prevents cancellation of unrelated fixed-channel traffic; once a first
fragment is sent, the loan must finish. The two-byte request is sent only when
the LE ACL payload fits it and the L2CAP header in one fragment. This follows
the [Bluetooth Core Security Manager specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/security-manager-specification.html),
which defines Security Request and bars sending it during an active pairing or
encryption procedure.

The physical gesture can also authorize exactly one already-live, encrypted,
committed LE bond. The owner binds its handle, generation, resolved identity
and bond-incarnation tag before queuing storage; multiple eligible peers are
ambiguous and receive no elevation. This lets a historical bond gain control
trust through a fresh physical gesture without deleting its keys. A fresh LE
pairing similarly produces a trust candidate only while its lease, window and
exact live generation are valid and SMP reports an encrypted, committed bond.
Once the owner queues that exact candidate, the grant is irrevocable even if
the link later disconnects or the window expires, but it is not usable until
the worker reports successful durable commit and readback. A changed bond tag
is a different incarnation and fails the trusted query until admitted again.

The separate `native_trust` NVS index holds at most eight identity/tag records.
It uses a committed and verified intent, record write/readback, and intent
retirement; storage failures poison the runtime policy and an unresolved
intent blocks startup. Queue pressure never authorizes a peer. The owner-only
`probe_admission_trusted_le` query checks the current live link, encryption,
committed SMP identity and tag, and durable trust index on every call. Storage
and NVS work remain on the startup/storage workers. No bond or unrelated NVS
namespace is erased.

The owner captures a non-repeating atomic control cookie bound to that trusted
live generation and bond incarnation. Link removal, re-pair, changed trust, or
failed security revalidation invalidates it; a regrant gets a different
cookie. Workers validate only the atomic cookie immediately before entering a
non-EQ privileged provider, and the owner rechecks before sending its result.
Once a provider has passed that fence and begun an operation, a simultaneous
revocation cannot undo it; the stale response is suppressed. Already-admitted
EQ and control persistence remains irrevocable even on disconnect or trust
revocation, while stale responses and peer notifications are suppressed.

The restart coordinator can freeze new admissions without cancelling already
admitted security work, then wait for admission, Classic and SMP quiescence.
The reversible input pause prevents new sensor samples after acknowledgement;
the producer first drains its already-copied partial sample, and the owner
drains every queued EQ/control action individually into the persistence lane.
Quiescence of that input queue means admission only: the coordinator must also
wait for the persistence workers and all output loans. Cancellation resumes
sampling. A pairing disconnect already submitted is fenced until its original
generation disconnects or the preparation fails; an unsent command is
cancelled before its generation fence is cleared.

The host tests exercise the adapter and trust policy with deterministic fake
queues/NVS, including stale generations, replacement tags, expiry, queue and
allocation failure, a failed storage transaction, and concurrent cookie reads.
Target-object compile is not evidence of successful pairing, secure persistence
on real flash, product authorization on hardware, or acoustic performance.
