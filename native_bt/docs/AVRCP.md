# Independent AVCTP and AVRCP work

This is offline groundwork, not an advertised or linked profile, a complete
speaker implementation, an interoperability result, or hardware evidence.
The Bluedroid application remains the active baseline.

Primary references: [AVRCP 1.6.3](https://files.bluetooth.com/wp-content/uploads/dlm_uploads/2024/10/AVRCP_v1.6.3.pdf),
[AVCTP 1.4](https://www.bluetooth.com/specifications/specs/a-v-control-transport-protocol-1-4/),
and [Bluetooth Assigned Numbers](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Assigned_Numbers/out/en/index-en.html).
The independent implementation uses the specifications and does not depend on
BTstack code or headers.

## Control channel

`dbt_avctp` uses caller-provided storage, one outgoing message at a time, 16
independently correlated controller labels, bounded fragment reassembly, packet
pin/commit/cancel, and fail-closed transaction, TX and RX deadlines. At most 15
notification registrations leave a label available for other traffic. Labels
are scoped to a unique nonzero channel lifetime `binding`. A timeout closes
the channel instead of recycling an ambiguous label. A nonconforming duplicate
identical to a later transaction after the four-bit label wraps cannot be
distinguished by the wire protocol.

`dbt_avrcp` implements UNIT INFO and SUBUNIT INFO target responses, pass-through
without vendor-unique data, capabilities, absolute volume, play status,
metadata, addressed-player selection, PlayItem and AddToNowPlaying framing.
It handles all defined notification payload shapes, registration, INTERIM,
CHANGED and REJECTED responses. Application callbacks must actually implement
and advertise only features the product supports. The owner must validate
player and UID state for PlayItem and AddToNowPlaying.

Recognizable malformed AVRCP control PDUs receive command-specific REJECTED
responses; unsupported generic AV/C commands receive a NOT IMPLEMENTED echo.
Configure a disjoint echo buffer to retain one generic reply while TX is busy.
A second pending generic echo returns BUSY; absent or insufficient echo storage
returns NO_SPACE while TX is busy. Those bounded failures must be handled as
channel errors, not silently dropped. An unparseable AV/C frame without enough
header information is reported INVALID. Incoming NOT IMPLEMENTED echoes for
fixed-size controls are checked against their submitted operands; a metadata
request echo is checked structurally and by PDU, not byte-for-byte against the
full requested attribute list.

Controller and target metadata support AVRCP START/CONTINUE/END, separate from
AVCTP fragmentation. Target snapshots are bounded and expire if the peer never
continues. A controller can abort explicitly; a valid interleaved vendor response
aborts its incomplete metadata, and insufficient reassembly storage schedules an
abort and returns NO_SPACE. In each case the callback receives an `aborted`
message. A sent continuation's label stays quarantined until its matching late
response arrives; its ordinary response deadline fails the channel closed if it
never arrives. Do not treat NO_SPACE with `abort_needed` as an already failed
transport; continue polling to send the abort. Controller restart policy and
application-level metadata refresh remain the owner's responsibility.

Callbacks borrow views only. Copy work for asynchronous processing and return
the full 64-bit lifetime token on the owner thread. A successful enqueue copies
bytes and is not evidence of peer receipt. On a browsed-player change, the owner
must reject a pending UID-changed notification using
`dbt_avrcp_cancel_notification`; the browse state-change callback identifies
that point. After the AddressedPlayerChanged notification has been queued, the
owner calls `dbt_avrcp_addressed_player_changed` to queue REJECTED for dependent
registrations. The volume registration is preserved. The application owns those
cross-channel events, truthful capabilities, playback controls and persistence.
A remote SetAbsoluteVolume must not itself complete an outstanding volume-change
notification. No NVS, decoder, DSP or I2S work runs here.

## Browsing profile groundwork

Browsing PDU framing covers SetBrowsedPlayer, GetFolderItems, ChangePath,
GetItemAttributes, GetTotalNumberOfItems and Search. Commands and responses are
length checked; player, folder, media-item and attribute boundaries are checked
against their scopes. Unknown browsing PDUs produce GeneralReject and malformed
recognized commands receive command-specific errors, with bounded deferred
responses under TX backpressure. The parser also accepts a GeneralReject for
an outstanding controller transaction.

Each role tracks its selected browsed player, UID counter, folder depth and
search validity. Browsed-player and addressed-player UID counters are separate;
Now Playing scope 3 uses the addressed-player counter. State-mutating browsing
requests are serialized. Non-mutating requests carry a snapshot epoch: a late
controller reply is marked `stale` and must not be used, while a target worker
completion after invalidation returns EMPTY and queues UID Changed on the wire.
On receiving UID Changed, the controller invalidates its cached counter and
search. The owner forwards control-channel UID or player changes through
`dbt_avrcp_browse_uids_changed` or
`dbt_avrcp_browse_addressed_uids_changed`; it also handles player availability,
actual filesystem operations and cross-channel notifications. After the successful target SetBrowsedPlayer response is fully admitted by
the bearer, a `state_change` callback requests cross-channel notification cleanup;
admission is not proof that the peer received that response. Nested item shape validation
is not proof that supplied players, paths, names, UIDs, attributes or feature
bitmaps describe actual product data.

Browsing frames are unfragmented at AVCTP. The maximum accepted payload is bounded
by the supplied TX buffer and negotiated channel MTU, rather than a fixed 512-byte
AV/C control-frame limit. The browse parser and state machine are not connected
to the runtime. Do not advertise browsing until an ERTM-capable channel and
bearer, the application operations, notifications and interoperability have all
been integrated and verified. The present control AVCTP adapter deliberately
rejects browsing PSM admission.

| Feature | Current scope and remaining work |
| --- | --- |
| Pass-through, capabilities, volume, play status, notifications | Framing and target handoff; product policy and full interoperability remain |
| Element metadata | Controller and target continuation, abort and expiry; product metadata source and refresh policy remain |
| Unit/subunit discovery | Target responses; controller discovery APIs remain |
| Browsing and player/UID lifecycle | Standalone parser, validation, state and deferred handoff; ERTM bearer and application integration remain |
| PlayItem, AddToNowPlaying, SetAddressedPlayer | Control-channel framing; application must apply and synchronize player/UID semantics |
| Player settings and cover art | Not implemented |

No AVRCP profile version or supported-feature bitmask should be advertised from
these primitives alone.

## Resource and integration contract

Control uses Classic PSM `0x0017` and profile identifier `0x110e`; controller
service class UUIDs are `0x110e` and `0x110f`, target service class UUID is
`0x110c`. Browsing uses PSM `0x001b` and its required L2CAP channel mode. These
identifiers are not authorization to advertise an incomplete service.

After negotiating an open control channel, initialize the profile, initialize
AVCTP with its hooks, exact MTUs and a unique serial, then attach the AVCTP
bearer. Route control SDUs through `dbt_avrcp_bearer_receive`, and poll and
schedule with the AVRCP bearer functions. A future ERTM browsing adapter should
route complete browsing SDUs through `dbt_avrcp_browse_receive`, poll with
`dbt_avrcp_browse_poll`, and use `dbt_avrcp_browse_wait_ms` for deferred replies
and deadlines. The runtime must own the transport pin until admission or abort.
On shutdown, quiesce callbacks and worker publications, return runtime borrows,
clear the bearer, and invalidate token-bearing work before reinitialization.
Channel serials must not be recycled while old workers may complete.

Each AVCTP packet buffer must hold the negotiated TX MTU. Reassembly and TX
capacity must each hold a 512-byte AV/C frame when fragmented control metadata is
used. Controller reassembly, target metadata snapshot and generic echo storage
are independently supplied on demand; keep all buffers disjoint from transport
storage. Control serialization uses bounded 512-byte stack frames. Browsing
serializes directly into its caller-owned AVCTP TX buffer; inputs must be
disjoint from that buffer. Account for these allocations in application memory
budgets. Product EQ changes still require immediate NVS commits, and this code
does not change that requirement.

## Offline evidence

`python native_bt/tools/test_avrcp.py` builds a fixed test library under
`_build/avrcp`. Its cases include independent control and browse wire vectors,
paired operations, fragmentation, simultaneous notification/control labels,
generic and command-specific rejects under backpressure, continuation abort and
interleaving, late-response quarantine, player/UID/search transitions, stale
worker completion, scoped UID separation, nested item bounds, a browsing SDU
larger than 512 bytes, and 30,000 deterministic generated control and browsing inputs. A mocked
runtime exercises borrowed-packet admission and close. This is host-only
validation, not real-runtime integration, target-link, RF or acoustic evidence.
Full conformance, real peers, ERTM, resource and latency measurements remain.

## Paired real-runtime simulation

`python native_bt/tools/test_avrcp_runtime.py` passed three offline cases on the
Windows host. The harness instantiates two actual owned HCI, ACL, L2CAP channel,
runtime, AVCTP and AVRCP stacks, with a real ATT server/bearer on the LE fixed
channel. A simulated controller supplies bootstrap events, bounded ACL buffers
and completed-packet events; this is not an ESP32 controller or a radio test.
The source list is explicit and excludes the incomplete ERTM engine. The same
source list also passed an ESP32 GCC 14.2 syntax-only check with warnings treated
as errors; this was not an ESP32 link, flash or runtime test.

- A volume registration remains active while a 600-byte metadata reply uses both
  AVRCP continuation and AVCTP/ACL fragmentation. A pass-through completes
  before the metadata completes, and real ATT MTU/read exchanges run on the LE
  fixed channel during the same workload. Classic credits are set to zero in the
  simulation, while LE ATT makes progress; the test verifies that the AVCTP
  packet remains pinned and unchanged until credits are restored.
- A Classic disconnect while the first HCI ACL fragment is borrowed returns the
  profile TX loan and detaches the bearer. Reusing the handle creates a new
  generation and channel serial; the old channel snapshot and deferred target
  token are rejected. The outstanding simulated HCI transport borrow is checked
  for byte stability before release.
- A separate runtime TX loan on the same Classic link causes AVRCP admission
  backpressure without retaining a false pin. Disconnect returns that competing
  loan with LINK_LOST and clears the pending bearer.

These are finite paired simulations and do not prove timing, coexistence, RF,
codec playback or audible behavior. The generated fixed-path report
`_build/avrcp/runtime-windows.json` contains test names and source hashes. The
runner checks workspace budgets before and after builds.

The same credential-free runner supports Linux ASan/UBSan:
`python3 native_bt/tools/test_avrcp_runtime.py --sanitize --cc gcc`. To copy
only its required project files into an isolated Linux workspace, use
`python native_bt/tools/test_avrcp_runtime.py --list-inputs` and preserve their
paths beneath `native_bt/`. Its Linux output is
`_build/avrcp/runtime-linux-asan.json`. The Linux sanitizer run was not executed
in this Windows session and must not be reported as passed until independently
run and checked.
