# L2CAP source comparison and owned fixes

This is a source/state-machine review, not a full Bluetooth conformance result.
No BTstack implementation, header or runtime dependency is integrated. The
current requested phase is offline source audit/fixes before further connections.

## Reference identities and scope

The original sparse reference remains pinned at
51e11ce3692abfe31b4ee3d66cffdeb1ae1626f2. The user linked current master;
`git ls-remote` identified 0f4dd2ac1240a503edc4ec7750a32704f85448fc. Its
[l2cap.c](https://github.com/bluekitchen/btstack/blob/0f4dd2ac1240a503edc4ec7750a32704f85448fc/src/l2cap.c)
has 6,293 physical lines and SHA-256
c3c986853d59b33af2f97042b18d38e63dff12c548e748df219f1c4925651787.
That comparison file and metadata stay under reference/, outside all build paths.

Our corresponding work is split among dbt_channels.c, dbt_l2cap.c, dbt_acl.c,
dbt_runtime.c, dbt_rx_flow.c and the security owner. BTstack also performs ACL
recombination in hci.c; comparing only its l2cap.c against our reassembly file
would omit a boundary. File length is not a feature-parity or memory benchmark.

## Concrete findings

| Area | Reference behavior / protocol check | Owned issue and current change |
| --- | --- | --- |
| Data during Close | l2cap_acl_classic_handler_for_channel drops data outside OPEN; Core Vol 3 A 6.1.6 ignores data while waiting for disconnect | The old dispatcher returned INVALID, causing runtime to abort the entire host. The captured 608-byte PDU on media CID 65 reached this path. Closing/closed/initially-unready data now increments ignored_dynamic_pdus without upper-layer delivery or host failure |
| Disconnect request after channel removal | Reference signaling fallback acknowledges the CID pair in CLOSED; Core 6.1.1 specifies a response | Ours sent Command Reject after the local channel was removed. It now acknowledges the repeated request without recreating state |
| Connection-response identity | Core 4.3 requires ignoring BOTH CIDs on non-success results, including Pending. Reference correlates these responses by identifier | The previous audit incorrectly strengthened the SCID check before reading Result. Corrected: Pending/Refused are correlated by live link generation and outstanding identifier; successful responses still validate the CID pair. Tests now deliberately use invalid CIDs for Pending/Refused |
| Reconfiguration data | Reference Basic receive helper gates on its OPEN representation; Core 6.1.4 requires processing Basic data during reconfiguration | We retain delivery for an already-open channel undergoing reconfiguration, with its current MTU/security checks. This intentional difference must not be changed solely to match a helper's state guard |
| Oversized active-channel data | Reference drops Basic payloads beyond local_mtu | Owned receive now counts oversized_dynamic_pdus and drops without upper delivery or host abort |
| Oversized signaling | Core 4.1 requires rejecting the first Request, and discarding an oversized frame containing only recognized Responses | Ours copied the first command's ID regardless of its type. It now identifies the first Request after validating the entire C-frame; response-only oversized frames are counted and discarded |
| Hinted options | Core 4.5 forbids making a hint the sole reason for rejecting configuration | Ours counter-proposed Basic mode even for a hinted unsupported mode and rejected a hinted undersized MTU. Unsupported hints no longer trigger rejection; an undersized hinted MTU preserves the previous value |
| Basic transmission during reconfiguration | Core 6.1.4 permits Basic-mode data transfer in CONFIG after a channel was open | RX already allowed this, but prepare_data blocked TX. Both directions now use their current MTUs during reconfiguration; initial CONFIG and disconnect still block data |

The closing-PDU source diagnosis is bound to image
d85bf6be79440780ac52564c8048f03c72e33a4448e2a4f278f20dc9caa529f9.
The new corrections are host-tested; no post-fix RF success is asserted here.

## Reviewed boundaries and remaining coverage

| Boundary | Owned coverage | Remaining limitation |
| --- | --- | --- |
| Incoming/outgoing Basic setup | Both configuration directions, asymmetric MTUs, incoming security wait, duplicate and pending handling | Broader external-peer interoperability |
| Options | MTU, flush, QoS shape, Basic-mode preference, unknown mandatory versus hinted options and continuation | No ERTM/Streaming implementation; unsupported mode proposals remain refused |
| Request identity | Per-link IDs, CID/generation/serial checks, copied bounded signaling | More delayed/reordered-peer scenarios and request-ID wrap qualification |
| Timers | Sent-request RTX, pending ERTX, local unsent bounds, wrap and bounded pending lifetime | Production recovery and wider peer timing tests |
| Data ownership | Lazy fragmented RX, immutable borrowed TX until admission/return, disconnect cleanup | Generic earlier single-byte RF corruption remains unresolved; these source fixes do not explain that failure |
| Scheduling | Per-link signaling and separate/shared HCI-credit accounting | Measured radio fairness and sustained mixed traffic |
| Security | Encrypted/authenticated policy hooks and a distinct Classic security owner | BLE SMP/bonding/privacy and broader Classic method qualification |
| LE credit channels | Reference includes K-frame data plus setup, security, credits, teardown and scheduling | Owned incoming/outgoing LE CoC signaling and runtime now integrate the data engine. Initial/granted credits wait for signaling admission, TX uses a borrowed scatter view, stalls sleep, invalid credit/data closes only the affected channel, and lazy allocations/loans clear on failure. Full paired runtime, asymmetric-CID and fault-isolation tests pass; external-peer/RF qualification and BLE SMP remain pending |
| Reference advanced modes | Reference includes ERTM/FCS/SAR and enhanced credit channels | ERTM, Streaming mode and enhanced channel establishment/reconfiguration remain missing; Basic tests and the credit data engine do not supply that coverage |

As of 2026-10-29, 166 portable groups pass on Windows and Linux, with ASan/UBSan
and TSan. The ESP32 probe builds; its Basic and credit-channel paths link. The probe registers no CoC service by
default, so target linkage is not evidence of a live credit-channel connection.
The test runner now checks its explicit inventory against all exported C tests
so a compiled test cannot silently disappear from the executed list.

There is no reference-code
substitution, reference-library execution or full-layer differential claim.
The tests do not establish acoustic continuity, physical NVS continuity, complete
profile coverage or a resource advantage over BTstack.

Protocol source: Bluetooth SIG
[L2CAP specification, sections 4.3 and 6.1](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/logical-link-control-and-adaptation-protocol-specification.html).
