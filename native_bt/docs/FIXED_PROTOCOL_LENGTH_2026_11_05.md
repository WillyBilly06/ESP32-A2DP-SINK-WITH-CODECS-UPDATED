# Fixed-protocol length rejection — 2026-11-05

This is a source/ownership stage of the [BTstack comparison](BTSTACK_CODE_AUDIT_2026_11_05.md), not full BLE or performance qualification. Reference checkout: BTstack 9edb5d82ea6ec4466a393415d65030929330acdf, outside the build. No reference implementation is integrated.

## Compared boundaries

| Owned code | Reference sections | Result |
|---|---|---|
| dbt_channels.c / dbt_runtime.c | src/hci.c:1268–1375 and fixed-channel dispatch | Over-limit fixed PDUs retain only four prefix bytes, drain the declared payload and invoke an explicit owner hook after complete framing. They never enter a normal receive handler as a fabricated short PDU. |
| dbt_att.c / probe_gatt.c | src/ble/att_server.c:898–945 | Commands and response methods do not create error loops. A known request receives Invalid PDU; an unknown even request receives Request Not Supported. A busy response retains its original storage and defers peer termination. |
| probe_smp.c | src/ble/sm.c:4427–4457 | Exact SMP command lengths are checked before starting pairing preparation. Unsupported/invalid-length commands queue an owned Pairing Failed message through existing per-link slots. Existing TX/command/worker/storage cleanup finishes before the replacement reply is admitted. |
| probe_main.c | owner integration, no one-file counterpart | Fixed-owner intent is wired into runtime; disconnect intent remains per peer and is serviced outside callbacks, including GATT-only builds. |

The [Core 6.1 SMP specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host/security-manager-specification.html) defines Invalid Parameters for invalid command lengths. The [ATT specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host/attribute-protocol--att-.html) supplies request/command/error semantics. BTstack drops malformed SMP in its preliminary validation; this implementation uses the protocol error and existing safe teardown rather than copying that policy.

## Ownership and scheduling

The fixed hook receives borrowed prefix bytes only during the callback. ATT uses its existing five-byte short error buffer; an in-flight response is never overwritten. SMP reuses existing two-byte per-link Security Request storage. It changes those bytes only after the prior loan retires and the matching failed pairing/reconnect slot finishes cleanup. A malformed Pairing Failed terminates without sending another failure loop. New pairing/reconnect/trust queries are refused for a rejecting generation. Other peers retain independent slots.

Controller/TX backpressure waits on existing owner events and a finite deadline. It does not loop waiting for credits or synchronously run crypto/storage from the receive callback. If a required error cannot be admitted before the deadline, the affected peer is explicitly retired. Resource/invariant failures remain explicit; they are not reported as successful wire rejection.

The added UUID-width guard prevents the mixed-width comparison from treating a short UUID as a 128-bit UUID when an invalid width reaches that helper. Clang previously reported that unchecked-width path; it no longer reports it. Valid UUID representations remain unchanged.

## Evidence limits

Strict C11 Clang analysis covers dbt_att.c, dbt_channels.c and dbt_runtime.c. The full ESP-IDF 6.1 ccache build passes with stable source fingerprints. Target text changes 865051→866743 bytes, aggregate data 396561→396577 (includes flash constants), and BSS 50292→50300. The actual linked SMP slot array remains 128 bytes. No task, payload pool, queue, MTU, codec setting or affinity changes. Deployment evidence is recorded separately. No unit, simulated-peer, packet-injection, pairing, playback or app campaign is invoked under the user's test hold. Ordinary boot runs the existing automatic codec checks; those do not qualify these new wire paths. Full ATT/SMP lifecycle/security and malformed/backpressure qualification remain open. No throughput or underrun improvement is claimed.
