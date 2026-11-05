# Classic paging/admission review

## Source correction

GAP accepted an incoming Classic Connection Request solely through the product admission hook. The product hook allows ordinary public SDP access when admission is not frozen and did not reject the local controller identity. The GAP layer now compares all six HCI-order address octets with the identified controller address before invoking that hook. A matching local address receives the existing asynchronous Reject Connection Request with reason 0x0f. No pairing/admission lease is granted, and no task, allocation, persistent state or PIN requirement is added. Normal distinct-address peers keep the existing public/protected service policy.

The Bluetooth SIG recommends refusing connection initiation/admission for peers claiming the local address: [SIG notice](https://www.bluetooth.com/learn-about-bluetooth/key-attributes/bluetooth-security/impersonation-pin-pairing/). Legacy PIN pairing is already disabled in this firmware. This missing check is not evidence that the firmware was exploitable through legacy pairing, or that it caused the user's reported playback/pairing failures.

BTstack hci.c at pinned commit 9edb5d82ea6ec4466a393415d65030929330acdf explicitly rejects this identity before its application acceptance callback (lines 4692-4710). It was reviewed as a behavioral reference; no BTstack implementation is integrated.

## Reconnection work still required

The current dbt_gap API has no outgoing Create Connection procedure. Speaker-initiated reconnect, outgoing SDP and A2DP initiation remain incomplete. This pass does not claim to add them. Review of the Core HCI procedure and current host/runtime contracts established these implementation constraints:

- Create Connection Command Status is admission, not completion; the final event supplies the ACL handle.
- Cancel Command Complete is not final retirement. A corresponding Connection Complete still follows a successful cancellation. Keep the request identity reserved until that boundary.
- If connection establishment wins cancellation, return/retire the actual live link explicitly; never report it as already gone.
- An unsent tracked host command can be removed synchronously; an already submitted command retains its normal completion ownership.
- Runtime allocates and publishes link changes before the GAP observer sees the final HCI event. Admission/security integration must therefore recognize an authorized outgoing request before that observer, with exact peer and generation ownership.
- Simultaneous incoming/outgoing connections need explicit collision handling. BTstack's incoming-accept state ignores old outgoing Page Timeout/Connection Exists outcomes; ordinary address matching alone is insufficient.
- Reserve link capacity, retain bond/security policy, avoid packet-type quality reduction, and use completion/deadline wakeups rather than retries in a loop.

[Core HCI specification, sections 7.1.5-7.1.8](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host-controller-interface/host-controller-interface-functional-specification.html) is the primary protocol reference. These are design constraints for the remaining implementation, not passing interoperability results.

## Evidence boundary

Compilation/source-integrity and flash/startup checks are recorded separately. No connection, spoofing, pairing, playback or app test campaign was run, following the user's instruction. Maximum-rate audio performance and the reported codec crashes remain unresolved.
