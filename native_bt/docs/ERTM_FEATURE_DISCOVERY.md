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
fallback. ERTM configuration continuation and reconfiguration are not supported
and are refused or cause channel disconnect.

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
Basic and LE coexistence, required-mode rejection, default FCS, OOM, security
revocation and link loss with a pinned ACL frame. It also runs 23 existing
Classic/LE channel tests. These host tests do not establish interoperability with
an external peer or audio hardware. ESP32 object compilation is not a firmware
link; profile registration and on-device validation remain separate steps.

Protocol basis: [Bluetooth Core L2CAP](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/logical-link-control-and-adaptation-protocol-specification.html),
sections 4.12, 5.4, 5.5, and 8.6.
