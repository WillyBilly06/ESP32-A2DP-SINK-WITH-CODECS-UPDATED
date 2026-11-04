# RFCOMM and optional serial endpoint

The independently authored RFCOMM path is implemented and host-tested for one
user DLC per mux. The existing Bluedroid application remains the baseline. No
RFCOMM service or SDP record is installed automatically by the library. The
current combined local candidate explicitly enables `DBT_PROBE_RFCOMM`, routes
its hooks and installs the borrowed echo SDP record; its target-link evidence
is in `LOCAL_CANDIDATE_EVIDENCE.json`. The individual suites provide no
external-peer, Bluetooth, audio or hardware evidence.

`dbt_rfcomm.c` implements SABM, UA, DM, DISC and UIH frame encoding/decoding,
role C/R, independent bit-serial FCS, one/two-octet length, credit octets, and
one bounded MCC item. Decoders borrow input. UIH FCS covers the address and
control fields, not the payload. `dbt_rfcomm_mux.c` implements control-DLC
startup/shutdown, crossed disconnect and admission-based T1 with generation
and CID binding. `dbt_rfcomm_credit.c` supplies initial PN and per-DLC credit
transactions. No reference implementation is linked or called.

`dbt_rfcomm_dlc.c` implements one negotiated user DLC: direction/server-channel
mapping, initial PN including credit and older non-credit responses, SABM/UA,
mandatory two-way MSC before data, UIH transmit/receive, credit grants,
non-credit MSC/FCon/FCoff gating, DISC, T1/T2, duplicate initial and post-open PN,
TEST, RPN default-value queries and refusal of unsupported port-setting changes,
RLS echo/status and NSC. Local MTU and initial credits are bounded. Completion
means the complete frame was admitted to HCI, not acknowledged by the peer or
received over RF. This is a serial-byte endpoint, not a full V.24 implementation:
port setting changes are not applied, and line/break status has no application
event interface. Multi-DLC sessions, reopening a DLC without a fresh PN, and complete
concurrent command handling remain unsupported. A legacy flow-off may abort one copied but unsubmitted app
frame to free the single transmit slot for the control response; the completion
reports ABORTED. Frames already submitted to the runtime retain their buffer until
its completion callback.

`dbt_rfcomm_bearer.c` binds explicitly to an already-open Basic PSM 3 channel.
It allocates eight ingress buffers and one transmit buffer per active bearer,
with capacity at most 256 bytes each; partial allocation failure is rolled back.
It copies channel ingress, checks channel serial and link generation, and keeps a
pinned TX view until runtime completion. Initial or subsequently granted receive
credits cannot exceed four outstanding data frames, preserving bounded ingress
capacity. Queue overflow, malformed/unsupported procedures and timeouts fail the
channel closed. Owners zero-initialize objects, forward channel events/data,
call poll outside runtime callbacks and retain the object until all TX callbacks
return. The bearer copies one app frame into its active buffer; it does not
borrow caller bytes after successful send. Callbacks are non-reentrant.

`dbt_rfcomm_spp.c` is an opt-in one-session endpoint. It registers an encrypted
Basic PSM 3 service, exposes accept/event/data/poll forwarding functions, and
constructs a borrowed SerialPort SDP record with the configured RFCOMM channel.
The application must install and maintain those hooks, keep the endpoint alive,
and only add the returned record to its SDP database after endpoint initialization
succeeds. The endpoint does not install the record or enable the probe itself.
Its one small early control slot bridges channel-open callback to owner poll;
allocation and channel closure occur later in owner context. This contract must be
wired and tested before publishing the record. No SPP qualification is claimed.

`esp32_probe/main/probe_rfcomm.c` is an optional encrypted echo application.
Explicit initialization registers its endpoint and supplies a borrowed SDP
record labeled `Native RFCOMM Echo`; the probe owner must forward the composite
accept, channel-event and data callbacks and call its poll, then explicitly add
the record to the shared SDP database. It never logs payload bytes. Echo storage
is allocated for an active bearer and released on closure. One initial receive
credit prevents a second credited data frame until the previous echo is admitted;
legacy non-credit overflow fails the affected channel closed.

The restart hold rejects new sessions and echo submissions, drains an already
submitted TX, closes the matching DLC and PSM 3 channels, and reports quiescence
only after channel slots, callbacks and active echo/bearer storage are gone.
Cancelling the hold reopens admission, though a disconnect already sent is not
undone. Publishing/removing the SDP record remains the owner's database decision;
the hold gates incoming RFCOMM requests. The standalone wrapper has not been
wired into the probe main or the existing SDP database.

`tools/test_rfcomm_target.py` compiles all six owned RFCOMM sources and the probe
wrapper as standalone ESP32 objects using the toolchain recorded in the fixed
ESP32 build tree; it checks the workspace before and after and records hashes in
`_build/rfcomm/target-objects.json`. These objects are not a firmware link or
hardware test.

`tools/test_rfcomm.py` builds in `_build/rfcomm`, checks workspace budgets before
and after, and hashes sources. The Windows host suite covers published SABM/UA
and PN/UIH vectors, C/R, extended lengths, credits, corruption and 100,000
malformed inputs; mux timeout/cancel/duplicate/collision cases; standalone legacy
flow, RPN/RLS/NSC; allocation rollback; paired simulated runtime credit and legacy
DLC byte transfers; credit exhaustion/replenishment, reconnect, and link loss
with a pinned data frame; plus endpoint registration and SDP record response, probe echo, hold/cancel,
stalled-TX drainage and allocation-failure cleanup.
The encrypted-policy tests set simulated link state; they do not establish real
pairing or encryption. Paired owned implementations are not an interoperability
test, and target resource, sustained workload, external-peer and hardware
validation remain required.

Protocol sources: [Bluetooth RFCOMM 1.2](https://www.bluetooth.com/specifications/specs/rfcomm-1-2/),
[Bluetooth RFCOMM test suite](https://files.bluetooth.com/wp-content/uploads/2024/10/RFCOMM.TS_.p7.pdf),
[Bluetooth Core 1.1 RFCOMM](https://www.tscm.com/BluetoothSpec.pdf),
[ETSI TS 07.10](https://www.freecalypso.org/pub/GSM/ETSI/by_chapter/07.10-7.2.0.pdf), and
[Serial Port Profile 1.2](https://www.bluetooth.com/specifications/specs/serial-port-profile-1-2/).
