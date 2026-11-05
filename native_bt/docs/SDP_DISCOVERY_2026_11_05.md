# Audio Source discovery and channel-local SDP recovery

## Implemented path

The product can now register a caller-owned asynchronous Audio Source discovery on an existing Classic ACL link. It opens SDP PSM 1, routes that exact channel to the client before the server, queries Audio Source service records, selects a supported AVDTP transport, and retires the discovery channel. Other SDP channels remain server channels. This supplies discovery routing; persisted-peer selection and outgoing AVDTP stream setup are still pending, so automatic reconnect is not enabled by this change.

`dbt_sdp_discovery_begin` validates the live ACL generation and disjoint caller context/request/response storage before accepting an operation. The product wrapper retains its routing registration until `probe_sdp_discover_result` returns terminal `DBT_OK` and detaches it. Cancellation requests teardown; it does not release caller storage. No new task, heap allocation, default packet buffer, or discovery instance is created.

The request searches UUID 0x110A and retrieves record handle, service classes, protocol descriptors, profile descriptors, and supported features. The numeric service iterator visits all records and protocol alternatives within the validated response. Bluetooth-base UUID equivalence is preserved for 16-, 32-, and 128-bit representations. A transport stack must describe L2CAP followed by the requested versioned AV protocol. Class, protocol, profile, and features stay correlated to the same record; capabilities from separate records are never merged.

Audio Source selection requires PSM 0x0019, AVDTP major version 1 and an Audio Distribution profile descriptor with major version 1. Supported candidates are ordered by protocol version and then profile version. Optional record handle/features have explicit presence bits. The returned service contains numeric values, not pointers into receive buffers.

The [Core SDP specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host/service-discovery-protocol--sdp--specification.html), particularly the transaction rules and ProtocolDescriptorList definition, remains the wire reference. BTstack at pinned commit `9edb5d82ea6ec4466a393415d65030929330acdf` is a comparison reference only. No BTstack implementation is integrated.

## Completion and scheduler ownership

Opening, querying, closing, and done are separate states. Phase and whole-operation deadlines are finite. Continuation, request pinning, and early-response handling use the existing owned SDP client/adapter. A completed response followed by ordinary remote channel closure can still be selected; actual ACL link loss remains a channel error. Failure outcomes retain the channel reason/result and nested query result separately.

Only owner poll mutates channel topology or submits teardown. Receive/event callbacks record state. Once closing, discovery waits for actual channel and transport retirement; a timeout cannot release an outstanding TX loan. The result is terminal only after the adapter can clear safely. Wait calculations request immediate service only for actionable state changes or real signaling/TX capacity. Expired whole-operation deadlines latch once instead of creating a close-wait retry spin.

## SDP error isolation

Previously, the product called `dbt_channels_close` inside a data callback. The channel manager rejects that mutation context, after which the product issued an ACL disconnect. A malformed or pipelined SDP request could therefore drop other audio/control channels on the same link.

`dbt_channels_request_close` now records an exact serial/generation/handle/CID close intent, safely from a channel callback. Owner poll performs the close when signaling capacity permits. The product uses this path instead of disconnecting the ACL. Unsent server replies on a channel already disconnecting are released rather than submitted as invalid runtime sends. Existing Basic-channel reconfiguration handling is preserved.

The existing low-priority logger receives a separate `sdp_channel_error` record with client/server role, exact channel identity, PDU number, byte count, local result and trace-loss count. Queue submission is non-blocking; packet contents and keys are not logged. This reuses the existing queue and its loss accounting.

## ESP32 RAM and build evidence

Target compiler constants establish:

| Object | Bytes |
|---|---:|
| Caller discovery context | 220 |
| Result snapshot | 72 |
| Numeric service | 16 |
| Iterator cursor | 4 |
| Product routing pointer | 4 |
| Channel, before and after close intent | 172 |
| Channel manager, before and after close intent | 2592 |
| Existing diagnostic queue item, including SDP error | 28 |

The request/result buffers remain additional bounded caller choices, up to 4096 bytes each. The close intent fits existing channel padding. Default runtime static storage increases by the four-byte routing pointer; no operation is automatically instantiated. Compiler artifacts and the build receipt are retained under `native_bt/_build/experiments/sdp-discovery/`.

ESP-IDF 6.1 compilation with ccache and source/workspace integrity checks completed successfully. Clang static analysis covers the SDP parser, discovery state machine, server bearer and channel manager. These checks do not establish peer interoperability or runtime correctness.

No separately invoked unit, simulated-peer, discovery, paging, codec, playback or app campaign was run, following the user's instruction. The firmware's built-in codec startup self-checks did execute on boot; their passing results are not peer/playback qualification. Continuations, cancellation races, MTU changes, queue saturation and all peer interactions remain functionally unqualified. Maximum-rate audio headroom, reported codec crashes and full feature parity remain unresolved.

## COM6 deployment

Image `4c4043e3e17446aaabd7f34ef7d6a18237b568d1ecf3412447f95e1bb6bc7ebc` was written to ota1 at 0x390000 using 921600 baud. Full candidate and selector readbacks at 230400 matched. Whole-region MD5 before and after writes confirms the preserved original application is unchanged. The owned host booted ready with ota state VALID and automatic Bluedroid restoration disabled. No boot fatal marker, watchdog or backtrace appeared in the retained startup window. This is an ordinary startup observation, not a sustained runtime result.

The startup snapshot reports 154596 bytes of free internal heap, 130580 bytes minimum free and a 110592-byte largest block. Those figures include the startup self-check history and no audio stream; they do not establish an equal-workload RAM advantage. UART collection resumed with an acknowledged, identity-verified live COM6 source. The raw capture and receipt are archived under `native_bt/_build/hardware/runs/1793905259013397400-4c4043e3e174/`.
