# AVRCP resource waits must not report immediate work

The actual probe can retain controller work while all sixteen local AVCTP
transaction labels await peer replies. Its wait function previously returned
zero even though another controller command returned `DBT_BUSY`. A high-priority
owner could therefore remain runnable without protocol progress. This is a
reproduced saturation defect, not proof of the earlier maximum-LHDC RF cause.

The wait predicate now distinguishes controller-label capacity from target
replies, which use peer labels. It also respects the fifteenth-notification
limit that reserves one local label for ordinary commands. An inadmissible new
notification no longer prevents an ordinary status/metadata command using that
reserved slot. The pending notification obligation is retained.

Target vendor replies blocked by an active continuation wait for its existing
deadline or a peer event. Pass-through remains runnable during continuation.
Pinned transmit storage is not reported as available. Existing response and
continuation deadlines are unchanged; a matching response makes retained work
immediately runnable. No delay, allocation, queue or priority change is added.

The same regression harness compiled against the unmodified `84590659` source
fails at line153 after sixteen real commands have completed their runtime
borrows and are still awaiting responses. Ordinary exchange and SDP pass in
that reference. The frozen reproduction is in
`_build/avrcp_wait_before/reproduction.json` with its exact source hash.

The changed actual probe passes exchange, SDP and wait/backpressure groups with
audio-output disabled and enabled. Tests cover sixteen outstanding commands,
sixty-four repeated nonproductive poll attempts, immediate wake after a real
matching response, fifteen real indefinite interim notifications, reserved-slot
status progress, continuation deadline wake, and target replies/pass-through
while local labels are full. The paired-runtime exchange and pinned-disconnect
test also passes. The runner uses ccache and records transitive input hashes in
`_build/avrcp/probe-wait-results.json`.

These are source/host state-machine tests. Target compilation, RF interoperability,
saturation timing and any measured CPU/audio benefit remain separate gates.
