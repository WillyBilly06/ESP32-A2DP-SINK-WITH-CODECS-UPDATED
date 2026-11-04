# Owned SDP server

The server implements ServiceSearch, ServiceAttribute and ServiceSearchAttribute
requests from [Bluetooth Core Vol 3 Part B](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/service-discovery-protocol--sdp--specification.html).
It matches 16/32/128-bit UUIDs across attribute values and emits ordered attribute
lists with continuation responses bounded by the peer MTU and requested limit.
Malformed syntax, unknown handles and invalid continuation state receive protocol
errors. This is a server; an SDP client remains to be implemented.

Records and attribute values are immutable and may reside in flash. The server
generates the handle attribute and streams a requested slice without constructing
the complete result in RAM. Registration validates element types, lengths, nesting,
UUID classes, ordered attributes and unique handles. Limits are 32 records, 64
attributes per record, 4096 encoded bytes per record and eight nested containers.

Continuation state binds the query, database version, channel lifetime and offset.
It contains no pointers. Its checksum checks consistency; it is not an authentication
mechanism. A database change requires a new nonzero database version.

The runtime bridge registers Basic PSM 1. Idle channels allocate no payload buffer;
each pending response gets an exact-size allocation. An inline seven-byte resource
error remains available on allocation failure. Responses remain borrowed until
runtime admission or cancellation; disconnect cleanup returns their storage.

## Evidence and limits

Eleven added host groups cover the core and actual paired-runtime bridge, including
8192 generated requests, UUID aliases, golden wire responses, continuation, OOM,
credit stalls and disconnect. The expanded 136-group suite passes on Windows/Linux and Linux
ASan/UBSan, with 20,000 concurrent ingress packets.

On ESP32, 24 Linux request/response transactions passed, including five transactions
to collect server attributes and fourteen to collect the diagnostic record with a
seven-byte attribute limit. BlueZ `sdptool browse` independently found the diagnostic
name and PSM. The boot reports 248 bytes of SDP metadata, excluding active response
allocations and shared runtime storage. Image identity is retained in the live
receipt; these figures do not compare equivalent complete stacks.

The probe publishes only its SDP server and a private encrypted echo service.
It does not advertise AudioSink or AVRCP. Classic/LE simultaneous traffic remains
failing and no audio/profile qualification follows from this discovery result.
