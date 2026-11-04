# ERTM channel integration

The Classic channel manager can query and cache the peer's Extended Features
Mask for the lifetime of a link generation. The query uses ordinary runtime
signaling, reserves its identifier, and distinguishes a queued request from one
fully admitted to HCI. Rejection, cancellation, malformed response and timeout
leave the feature unavailable. A response from another generation, identifier,
or before admission is not accepted.

Services can explicitly require ERTM with `dbt_channels_require_ertm()` and
outgoing channels with `dbt_channels_open_ertm()`. These operations enable the
ERTM and FCS-option feature bits; they do not register an AVRCP browsing
service. Normal Classic services remain Basic. A required ERTM channel checks
the peer feature mask before sending the RFC option and fails closed if the
peer does not support ERTM, proposes another mode, rejects the request, or
supplies invalid window, MPS, timer, FCS, or MTU values. RFC parameters are
directional. Requests carry zero timers; successful responses carry the local
non-flushable sender timers. The implementation uses the default 16-bit FCS;
one peer requesting No FCS does not disable it. Required mode has no Basic
fallback. Initial configuration accepts complete options across request fragments,
responds to every fragment, and retrieves continued responses with empty requests.
Recognized option values and duplicate markers are staged per channel without
retaining unbounded option bytes; malformed or conflicting options fail closed.
Results become active only after the final response is admitted, and link
generation, signaling identifiers, and setup deadlines still apply.

Live Standard-process ERTM reconfiguration supports an unchanged or increased
MTU via `dbt_channels_ertm_reconfigure()`, and peer requests for unchanged or
increased MTU. It waits for an outgoing SDU to be acknowledged before starting
or responding, blocks new application SDUs until both configuration directions
finish, and continues the existing engine for incoming data and supervisory
traffic. The engine, sequence numbers, RX assembly, timers, retransmission window
and any prepared frame remain in place. The new receive MTU takes effect after
a positive response; the peer MTU takes effect after the local response is
admitted. Both directions must finish before the reconfiguration callback.
A changed RFC mode, window, MaxTransmit or MPS and an MTU decrease are rejected;
unknown mandatory options that fit the bounded response receive result 3 with
the option, while oversized unknown-option responses fail closed. The API caps
local MTU increases at the manager maximum. Caller-selected timer changes, FCS
mode changes, Extended Flow Specification and arbitrary option negotiation are
not implemented. Timeouts or cancellation can close the channel. This is
offline behavior, not external-peer or hardware qualification.

Per-channel engine state is allocated on channel creation. Retransmission
window metadata, segmented receive SDUs, and encoded outgoing frames allocate
only as needed. The runtime schedules I/S frames with ordinary ACL credit and
transport gates. An application SDU remains borrowed until peer acknowledgment
and release of any prepared frame, or an abort/link-loss callback. Timers start
when a whole frame has been admitted to HCI, rather than at preparation. The
callback is not a claim of application consumption or RF qualification.

The offline `test_ertm_features.py` exercises discovery and paired simulated
controllers through the owned runtime, including segmentation, acknowledgment
loan, recovery after a deliberately dropped frame, simultaneous directions,
Basic and LE coexistence, split request/response negotiation with subsequent
data transfer, malformed and duplicate continuation options, simultaneous MTU
reconfiguration, active-SDU draining, post-change data, incompatible option
rejection, bounded timeout and stale generation, required-mode rejection,
default FCS, OOM, security
revocation and link loss with a pinned ACL frame. It also runs 23 existing
Classic/LE channel tests. These host tests do not establish interoperability with
an external peer or audio hardware. ESP32 object compilation is not a firmware
link; profile registration and on-device validation remain separate steps.

Protocol basis: [Bluetooth Core L2CAP](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/logical-link-control-and-adaptation-protocol-specification.html),
sections 4.12, 5.4, 5.5, and 8.6.
