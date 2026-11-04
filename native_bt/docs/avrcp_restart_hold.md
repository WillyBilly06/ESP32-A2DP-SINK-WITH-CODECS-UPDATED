# AVRCP restart hold

The host owner asserts `probe_avrcp_restart_hold(true, now)` before checking
the restart barrier. It blocks new control-channel admission, automatic
outbound connection attempts, local key requests, incoming AVRCP messages,
and new volume-notification production. The owner must keep forwarding channel
events and running both the runtime and `probe_avrcp_poll` while held. The hold
closes each extant PSM 0x17 channel without disconnecting its ACL or unrelated
profiles, and retains the exact channel generation and serial in close state.

`probe_avrcp_restart_quiescent()` is true only while the hold is active after
all control channels, borrowed transport data, attached bearer and profile
storage have retired. Queue pressure or stalled controller credits do not turn
into success after a timeout. A release request during teardown defers resumed
admission and automatic connection until the old teardown actually completes;
the caller continues polling. The existing diagnostic error count is retained.
The wider restart coordinator must also fence new producers and drain runtime
ingress, persistence, other profiles, and output loans.

Offline simulated runtimes exercise held PSM close, a pinned runtime loan and
stalled credits, cancellation while teardown is pending, eventual reconnect,
and preservation of the Classic ACL. An adapter harness exercises simultaneous
staged channels, close backpressure, unrelated PSM preservation and a stale
generation callback. Target compilation is not device or radio evidence.
