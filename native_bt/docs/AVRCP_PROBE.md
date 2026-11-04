# Owned AVRCP probe control integration

The `probe_avrcp` module binds the independently authored AVCTP/AVRCP control
implementation to one encrypted Basic L2CAP channel (PSM 0x0017). The parent
firmware must supply an authorized-peer policy, route callbacks on the host
owner, and enable the records only after this module initializes. Browsing is
not bound or advertised. This is implementation and offline test evidence,
not Bluetooth qualification, RF, audible-output, or live-peer evidence.

Initialize `probe_avrcp_init(runtime, volume_adapter, context, initial_volume)`
when the runtime is ready. The adapter must be bounded, nonblocking and update
only the audio volume; it returns `DBT_BUSY` when a bounded publication queue is
full. It must preserve all other audio controls, including EQ. It must not
perform NVS or other blocking work from the host owner. Any independent EQ
setter still commits NVS immediately as required by the application. On
successful AVRCP initialization call `probe_sdp_enable_avrcp(true, true)` before
`probe_sdp_init`; no AVRCP records appear without this explicit opt-in. Combine
`probe_avrcp_accept` with other incoming channel admission, route channel events
and PSM 0x0017 data to the module, and call `probe_avrcp_poll` and combine
`probe_avrcp_wait_ms` with the owner's other deadlines. The central encrypted
peer/security authorization policy still applies. The controller initiates a
bounded number of connection attempts only on an authorized encrypted Classic
link, and handles simultaneous channel establishment by closing both channels
and applying bounded retry jitter.

The controller requests capabilities, playback/track change registrations,
current metadata and play status. A changed registration is rearmed and track
changes schedule a metadata refresh; notification labels coexist with ordinary
commands. `probe_avrcp_key` queues a single press or release for the supported
player operations; the caller supplies the corresponding release and timing.
The target implements volume-up/down press/release, capabilities, absolute
volume and volume-change registrations. Unsupported target player operations
are answered as not implemented. Remote-origin absolute volume does not trigger
an echo notification. `probe_avrcp_local_volume` must be invoked by the owner
after an independently originated volume change is published to audio. It
sends changed notifications to outstanding registrations.

The target SDP record advertises Category 2; the controller record advertises
Category 1. Both identify AVCTP 1.4 and AVRCP 1.4. Browsing, cover art, player
application settings and other unsupported features are not advertised. These
records and the protocol implementations have not undergone a conformance test.

A single 3,584 + negotiated-peer-MTU byte allocation exists while the channel
is active (peer MTU is capped at 4,096); it contains 512-byte RX, TX and echo
areas, a 2,048-byte metadata continuation area, and a transport packet. The
allocation remains alive until the runtime returns any pinned outgoing loan.
The owner retains at most 16 target work items and 16 remote volume tokens.
Metadata is copied into bounded UTF-8 title/artist/album snapshots, each at
most 95 bytes, with a truncation flag. `probe_avrcp_snapshot` is a bounded
atomic reader for a separate logger; formatters must escape metadata bytes and
must not log directly from callbacks. Over-limit continuation is aborted rather
than allowing unbounded allocation.

Offline checks:

* `python native_bt/tools/test_avrcp_probe.py` checks control response vectors,
  category records, concurrent notification labels, metadata snapshot, remote
  absolute volume, a busy volume adapter, local change notification, supported
  and unsupported pass-through, collision retry and pinned-loan shutdown using
  a mocked transport.
* `python native_bt/tools/test_avrcp_probe_runtime.py` uses the actual probe
  module and two simulated owned HCI/ACL/L2CAP runtimes, a scripted remote
  target and a 600-byte continued metadata response. It checks an outstanding
  outgoing loan during a credit stall and link teardown. Its ERTM entry points
  are test stubs that fail closed; this test covers Basic mode only.
* On Linux the second driver supports `--sanitize` for ASan/UBSan and
  `--list-inputs` for a credential-free minimal source copy. Only the ordinary
  Windows host runs were executed here; sanitizer execution remains separate
  evidence to be recorded by whoever runs it.

These tests do not verify application wiring, audio gain behavior, phone
interoperability, target-link success or on-device operation. The firmware
owner must integrate and target-build the module and its matching core sources
before hardware validation.
