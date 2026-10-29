# Owned AVDTP signaling transport

This is the transport/control foundation for the independent audio profile. It
does not yet implement stream endpoint state, codec negotiation or playback.
The Bluedroid application remains the running audio firmware.

## Interfaces and ownership

`dbt_avdtp` has one signaling-channel owner, one local command transaction and one
remote command/response slot. Local and remote transaction labels are independent.
An incoming command callback may copy/queue its response immediately or arrange
worker completion and respond later on the owner. Other callbacks must defer new
commands until the next owner iteration. Borrowed incoming data cannot outlive a
callback; codec construction and decode/DSP work stay outside this owner.

`dbt_avdtp_bearer` attaches an already-open Classic PSM 0x19 channel with its exact
serial, link generation and negotiated MTUs. The future profile manager must
distinguish signaling from media channels on this shared PSM. The adapter neither
registers the PSM nor advertises an Audio Sink SDP record by itself.

All transmit bodies are copied before the API returns. Bodies through 8 bytes and
signaling packets through 16 bytes use inline storage. Larger bodies use exact
allocations. A packet allocation exists only until admission/cancellation cleanup;
fragment assembly is allocated only after an actual valid Start packet. No codec
workspace, per-codec tables or permanent media buffers are added here.

`tx_peek` pins an immutable complete signaling L2CAP SDU. Its matching commit runs
only when every underlying ACL fragment has been admitted. A cancelled transport
attempt returns the pin without advancing the message. The runtime copies each
ACL fragment into its own retained HCI transport buffer; freeing the signaling
SDU after final admission does not release bytes still borrowed by VHCI.

Transmit and response deadlines are separate: a local response timeout starts
after final command admission. A command waiting on credits is still subject to
its transmit deadline. Partial receive, deferred remote response and outgoing
response deadlines are bounded as well. Wraparound and late commits are tested.
Timeout fails the signaling instance closed. The profile must close its channel,
quiesce runtime TX and clear it; clear refuses while a borrow remains pinned.
An MTU change requires quiescence and rebinding; stale limits are not reused.

## Wire behavior

Single/Start/Continue/End framing, transaction labels and response message types
are implemented. Start and Continue L2CAP payload sizes must agree; End cannot
exceed that length. Packet count, type, transaction, size and assembly capacity
are checked before dispatch. Partial messages never execute a command. An
identifiable Abort can interrupt partial reception for the same SEP. Unsolicited
or stale-channel responses do not complete a transaction. Fixed response body
lengths are checked; unknown signals receive a General Reject.

`dbt_avdtp_caps` iterates immutable TLVs without allocation. It checks framing,
transport-category formats, duplicates, mandatory configuration categories and
Reconfigure's application-only categories. Capability responses preserve repeated
content-protection alternatives. Unknown capability categories can be skipped but
cannot enter a configuration. Codec and content-protection specific semantics
must still be validated by their implementations. Parsing a service does not
enable recovery, reporting, multiplexing or compression.

The media helper parses the uncompressed/unmultiplexed RTP header, including
CSRCs, extension word lengths and padding, into a borrowed view. Codec-specific
payload headers and frame reassembly remain the profile/codec adapter's job.

## Evidence and remaining work

116 native groups pass on Windows and Linux, with Linux ASan/UBSan and 20,000
concurrent FIFO packets. Fifteen new AVDTP groups cover copied controls, label
reuse, fragmented duplex traffic, allocation failures/retry, malformed sequences,
partial Abort, response lengths, timeout/wraparound and pinned cleanup, capability
formats/alternatives, RTP bounds and generated inputs. The two real runtime peers
exchange a 193-byte signaling body through a 48-byte L2CAP MTU and 27-byte HCI
MTU, while a separate BLE bearer progresses with Classic credits stalled.
This is a host simulation; it does not resolve the recorded RF corruption.

All three source units compile into the ESP32 component archive. They are
currently unreferenced by the controller probe and linker garbage collection
excludes them from its final image. `measure_avdtp_target.py` reports target ABI
sizes and that distinction in `_build/host/avdtp_target.json`. There is no new
AVDTP hardware/playback evidence and no additional board flash for this work.
The target ABI measurement is 156 bytes per signaling instance plus 24 bytes per
runtime adapter. This excludes active dynamic buffers and all future profile,
codec, worker and controller state; it is not total stack RAM or a comparison
against an equivalent feature set.

Next layers must implement endpoint ownership and capability selection, state
transitions, simultaneous-command conflict rules, outgoing Abort preemption,
transport channel establishment/release, delay reports, codec worker readiness,
profile discovery and external peer interoperability. SBC/AAC/aptX variants,
LDAC/Opus/LHDC V5 negotiation and playback all remain in scope.

## Reference review

The wire requirements were checked against Bluetooth SIG's
[AVDTP 1.3 specification](https://www.bluetooth.org/DocMan/handlers/DownloadDoc.ashx?doc_id=260860),
particularly sections 6.2, 8.3-8.4, 8.6-8.21 and 9. The existing pinned BTstack
header was inspected for its layer/API organization only. No BTstack source,
headers, libraries or runtime calls are included in the owned implementation.
