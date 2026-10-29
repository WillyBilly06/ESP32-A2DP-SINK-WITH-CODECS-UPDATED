# Owned AVDTP audio sink acceptor

`dbt_avdtp_sink` implements an audio acceptor's endpoint/configuration/control
lifecycle above the owned signaling engine. Endpoint descriptors and capability
bytes are immutable and may reside in flash. One sink instance reserves one active
audio stream/decoder, selected from up to 16 endpoints. An application must reserve
that instance across signaling connections; creating independent instances for the
same speaker would defeat endpoint ownership and the single-decoder memory policy.

The current services are basic media transport, media codec and delay reporting.
Initialization refuses descriptors exposing unimplemented optional transport
services. Codec-specific validators are callbacks; SBC has an owned validator for
selection cardinality, supported fields and bitpool bounds. Other codec validators
and their application descriptors still need integration. No endpoint is advertised
by the current controller probe.

## State and operation ownership

Discover reports each SEP and its actual In Use state. Get Capabilities omits delay
reporting; Get All Capabilities includes it. Set Configuration validates the target
SEIDs, transport services and codec parameters, then copies only the selected
configuration. Get Configuration returns that committed selection. Reconfigure
preserves the existing transport/delay settings and replaces the codec selection
while the stream is Open. Start and Suspend process SEID lists in order: the valid
prefix takes effect even when a later item fails, and the rejection identifies the
first failing item. Close and Abort stop the worker and wait for media release.
Delay reports use the remote SEID and the caller's measured 100-us units.

Configuration, open, start, suspend, reconfigure, close and active Abort operations
enqueue worker jobs. The host owner never constructs a decoder or performs audio
processing. A job must copy its bounded configuration before its enqueue callback
returns. Readiness receipts are applied only on the host owner, with matching
signaling binding and operation token. A successful enqueue does not acknowledge
the peer. Worker readiness allows a response to be queued; the corresponding
protocol state changes only after complete response admission to L2CAP/HCI.
Admission does not prove RF delivery.

The worker must serialize jobs, release old large decoder state before replacement,
and acknowledge DROP only after all earlier work and resources are drained. A
rejected reconfiguration guarantees the old configuration is still usable. If a
worker cannot restore it after an allocation/initialization failure, it must report
Fatal, causing explicit channel shutdown rather than a false usable state.
These are integration contracts, currently exercised with test providers; actual
codec construction, peak heap and audio timing remain unqualified.

An Abort for the same SEP may supersede a pending worker operation before a response
has been queued. Its old receipt becomes stale, the worker's copied input remains
valid, and serialized Abort cleanup must finish before its response is admitted.
Invalid Abort SEIDs are silently retired, with no invented rejection or lingering
unanswered-command timer. Incoming Abort after a response is already queued/pinned,
fragmented Abort preemption and outgoing Abort preemption still need broader
signaling recovery work. They are not represented as qualified interoperability.

## Media, timeouts and disconnects

The application must distinguish signaling and media L2CAP channels on PSM 0x19,
verify the media channel's handle/generation and bind its unique lifetime after
the Open response is admitted. A production A2DP registration must support at least
the profile's 335-byte L2CAP MTU. The component also permits smaller MTUs in transport
stress tests; those tests are not an A2DP conformance claim.

Media is admitted to a borrowed, nonblocking worker callback only while Streaming,
after Start readiness and response admission. It is gated immediately during stop,
reconfiguration and shutdown operations. RTP payloads retain their codec-specific
headers. Queue refusal and malformed media are explicit return values/counters,
not silently reported as decoded audio. SBC frame boundaries, fragment assembly,
PCM output and I2S handoff are still separate work.

Worker completion, media establishment and media release have bounded deadlines,
including wraparound to a zero deadline. The owner must wake on worker queue
availability/receipts; a busy work queue does not induce a polling spin. A fatal
operation or timeout requires closing/quiescing the runtime signaling channel and
calling shutdown. Shutdown preserves pinned bytes and waits for an asynchronous
DROP receipt before the sink instance can be reused. Stale completions cannot
reconfigure a retiring instance. Even after a timeout, a real DROP acknowledgment
may complete cleanup; a timeout alone never asserts that decoder memory is freed.

## Current evidence

130 total groups pass on Windows/Linux, including Linux ASan/UBSan and 20,000
concurrent ingress packets. Fourteen groups added for this layer exercise lifecycle
ordering, lazy allocation/OOM, rollback/fatal results, stale receipts, aborted work,
ordered command lists, delay reporting, media gates, cleanup and generated requests.
SBC tests cover every one-bit combination of its advertised fields and malformed
selections. They validate negotiation metadata, not actual encoded-frame decoding.

Two actual owned runtime instances establish separate signaling and media channels
with a 335-byte L2CAP MTU. They complete configuration/open/start, route a 193-byte
RTP packet through a 27-byte HCI MTU, suspend and close. The test worker is explicitly
simulated. A separate BLE bearer progresses while Classic response credits stall;
the sink does not enter Streaming until those response bytes are admitted.

The five AVDTP/SBC-validation units compile into the ESP32 archive. They remain
unreferenced by the controller probe and are excluded from its final image. Target
metadata measures 164 bytes for signaling, 284 for the complete sink including that
signaling, and 24 for a runtime adapter. These are not additive twice and exclude
all active buffers, decoder/worker/controller memory and the rest of the host.
There is no new RF, decoded audio, I2S or acoustic evidence for this layer.

Next work is the production PSM/media manager, native decoder worker and actual SBC
frame path, audio SDP records, then other codec validators/adapters and AVRCP. The
existing long-run Classic/BLE corruption remains a release blocker.

Wire/state requirements were checked against the Bluetooth SIG
[AVDTP 1.3 specification](https://www.bluetooth.org/DocMan/handlers/DownloadDoc.ashx?doc_id=260860)
and [A2DP 1.3.2 specification](https://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=457083).
No BTstack implementation, headers or runtime dependencies were introduced.
