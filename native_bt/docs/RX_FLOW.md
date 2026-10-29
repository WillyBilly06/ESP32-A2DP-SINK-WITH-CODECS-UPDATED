# Controller-to-host receive flow

The owned ACL receive-flow module configures Host Buffer Size and then enables
ACL flow control. It waits for both actual command completions before reporting
active. It returns completed-buffer counts after the owner has processed the
borrowed HCI packet and its profile callbacks. These acknowledgments use the
special HCI command path, independent of ordinary command and outbound ACL credits.
Disconnect retires pending counts for that connection generation.
Protocol references: [Core Vol 4 Part E, sections 4.2/4.3 and 7.3.38-40](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-60/out/en/host-controller-interface/host-controller-interface-functional-specification.html).

The probe advertises three 1024-byte ACL buffers. Its existing 4096-byte H4 FIFO
also has capacity for one maximum-size SCO frame and the 520-byte control reserve:
`3 * (1024 + 7) + (255 + 6) + 520 = 3874` bytes. No additional SCO payload pool is
allocated. H4 SCO length/status framing and an explicit borrowed-data route are
implemented; synchronous connection ownership, SCO transmit flow and voice profiles
remain pending. The current controller probe configures zero synchronous connections.
Only ACL receive flow is enabled.

The linked ESP32 controller routine rejects zero SCO size/count fields in
Classic/dual mode. The initial zero-SCO setup returned status 0x12. With the bounded
SCO framing/storage capacity declared, the controller accepted the setup. An absent
SCO consumer fails explicitly; it is not silently advertised as a working voice
profile.

Acknowledgment storage remains unchanged until transport release. Counts are
removed only on transport admission, without consuming a normal command credit or
waiting for a success response. Controller error replies fail the host. Buffer
capacity is validated against the FIFO; connection generations prevent old counts
from being credited to a reused handle. Receive acknowledgments get transport access
before ordinary command/TX work to avoid starvation.

## Evidence and limits

The current 130-group Windows/Linux suite and Linux ASan/UBSan pass. Added checks
cover setup limits/support masks, rejection/timeout, zero command/ACL credits,
retained packet bytes, disconnect/reuse, error replies, paired Classic/LE payloads,
and SCO framing/status/route behavior. Target `rx_flow_ready` confirms accepted
buffer setup and enable replies. This does not qualify SCO/voice interoperability.

The initial 1021-byte receive-buffer configuration did **not** remove the observed simultaneous-traffic corruption.
The full-window test completed 112 Classic echoes and 61 ATT writes before one
byte changed at offset 1017 of a 1021-byte payload. The error was already present
in the controller callback buffer: before-copy, after-copy and owner fingerprints
were identical. A separate encrypted Classic-only control completed 240 echoes
(159320 bytes), with LE observed disconnected at 24 checks. These finite results
do not prove continuous audio or universal coexistence.

Using 1024-byte host receive packets then passed a complete concurrent diagnostic
run: 245 encrypted Classic echoes (sizes 1/48/193/672/1021/2048), 107 ATT writes,
17 readbacks, a 512-byte reliable write and 108 notifications. Traffic overlapped
for 15.586 seconds. The 1029-byte H4 bound accommodates the 1024-byte payload plus
header; controller-reported outbound ACL limits remain separate. This is a
finite positive result. The subsequent untraced 1000-write attempt failed at
Classic sequence 95 after 51 ATT writes, before the requested workload completed.
The same single-byte corruption appeared in the reassembled receive data. Thus
the 1024-byte setting alone has not resolved the intermittent failure; the short
pass does not establish the root cause or qualify an A2DP/codec workload.

The radio/controller/USB boundary before VHCI remains unresolved. The tests retain
their image identities and private failed bytes. Runtime/protocol sizes exclude
the controller, tasks, dynamic payloads, codecs and future profiles; no complete
stack memory or speed advantage is established.
