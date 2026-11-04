# Physical pairing gesture: offline implementation

The middle encoder gesture snapshots the Classic ACL identity of the current
AVDTP signaling peer (including a paused session). It submits one disconnect
for that handle and generation, waits for controller status and link removal,
then requests discoverable and connectable Classic scanning. With no active
AVDTP peer it requests scanning directly. Pending queue pressure is retried;
rejected commands, a missing disconnect event, and failed scan completion are
counted as failures. Command admission alone is not disconnect completion.

The product IO owner and audio peer query run only on the host owner. The
runtime link hook must forward every link change to
`probe_product_io_owner_link`: on removal it cancels a queued, unsent command
before the host sends a potentially reused handle. A submitted command is never
replayed. Other Classic and LE links and all bonds are left alone.

This action does not relax the peer-restricted Classic security test policy.
Discoverability does not authorize a new peer, demonstrate successful pairing,
or change that policy. The baseline also plays a pairing sound and shows a
pairing animation; equivalent feedback and broader product pairing admission
remain separate, incomplete integration work. No live device observation is
implied by the host-only tests or target compilation.
