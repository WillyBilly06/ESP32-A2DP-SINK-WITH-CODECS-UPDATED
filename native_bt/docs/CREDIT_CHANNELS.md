# LE credit-based L2CAP channels

Updated 2026-10-29. The owned LE credit-based channel path is implemented in the
channel manager and runtime, offline-tested and linked into the ESP32 probe.
The probe does not register or advertise a CoC service by default. This is not
external-peer/RF qualification or full L2CAP feature parity.

`dbt_channels_register_le()` borrows an immutable SPSM descriptor list, suitable
for flash. `open_le()` and incoming request handling validate LE CID range,
MTU/MPS, duplicate endpoints, security policy and bounded allocator availability.
Connection requests/responses/credit indications use LE signaling CID 5. Channel
state and initial/returned credits advance only after the complete corresponding
signaling PDU is admitted. Pending setup has bounded timers and generation checks.

Authenticated policy and minimum key size require an explicit security provider.
Encryption alone does not satisfy them. Incoming requests are refused with the
appropriate qualification result while security is insufficient. Outgoing opens
require prior qualification. Security revocation during response admission or an
open channel triggers disconnect without delivering unqualified data. BLE SMP,
key generation/storage and privacy are still separate incomplete work.

`dbt_credit.c` implements K-frame segmentation, reassembly, per-channel credits,
zero-credit enforcement and overflow checks. Only fragmented SDUs allocate their
declared length. Completion, close, link loss, host abort and failure release the
partial and lazy channel state. Complete received SDUs borrow ingress bytes.

The runtime borrows one outbound SDU per connection until admission or abort.
A scatter view and copied two-byte first-frame length prefix feed the existing HCI
fragmenter directly, without an extra MPS-sized TX buffer. Each complete K-frame
consumes one channel credit regardless of its ACL fragment count. Credit stalls
sleep while other links/signaling continue. Explicit local credit grants depend
on available receive capacity; the core does not automatically grant on arrival.

Credit overflow, zero-credit RX and invalid K-frame dimensions disconnect only
the affected CoC channel. Other Classic/LE channels continue. Dynamic TX loans
return on disconnect/abort; actual link-loss outcomes retain LINK_LOST identity.

The 166-group Windows/Linux suite includes:

- 1,001 data-engine round trips for SDUs 0 through 1,000 bytes and MPS 23 through 96.
- Full HCI/ACL/runtime simulation with fragmented bidirectional CoC transfers,
  zero-length SDUs, credit exhaustion/replenishment, and no spin during stalls.
- Simultaneous opens producing asymmetric local/remote CIDs, exercising grant routing.
- Classic traffic continuing while CoC is stalled, then a malformed-credit fault,
  followed by continued Classic and fixed-channel BLE transfers.
- Security refusal/key-size qualification, revocation before and after admission,
  duplicate requests, no duplicate initial credit grant, outbox backpressure,
  setup deadline, OOM, host abort, disconnect and handle-generation reuse cleanup.

Linux ASan/UBSan and TSan pass. The ESP32 probe builds and links the engine;
compiled object checks find no mutable static payload/state in dbt_credit.c.
No Bluetooth connections or flash writes were performed in this offline phase.

Linked ESP32 ABI: one active LE channel requests 84 bytes of metadata; inactive
channels allocate none of this extension. The runtime object is 10,760 bytes,
including its host and 2,416-byte channel manager. These nested sizes must not be
added. Fragmented RX can simultaneously own a shared per-link PDU assembly and
a channel SDU assembly; allocator overhead, controller buffers and task stacks
are additional. These ABI/allocation dimensions are not physical free-heap or
equivalent-feature performance measurements. Reproduce with
`tools/measure_credit_target.py` in the ESP-IDF Python environment.

Remaining: external-peer interoperability, sustained Classic audio/product BLE
workloads and physical heap/CPU/latency measurements; BLE SMP/security provider;
ERTM, Streaming mode, enhanced credit-based multi-channel setup and reconfiguration;
full product service/profile integration. Paired tests use the owned code at both
ends and do not prove interoperability with every external implementation.

Protocol: [Bluetooth SIG L2CAP specification, sections 3.4, 4.22-4.24 and 10.1](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/logical-link-control-and-adaptation-protocol-specification.html).
