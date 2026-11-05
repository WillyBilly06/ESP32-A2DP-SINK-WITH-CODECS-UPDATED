# Incoming Classic setup ownership and stage logging

## Implemented behavior

GAP now retains four bounded incoming ACL setup identities, matching the runtime's maximum link count. An identity remains live while its decision is in the GAP reply queue, its tracked HCI command callback is outstanding, or an admitted controller procedure awaits Connection Complete. A rejected/unsubmitted HCI decision creates no accepted controller procedure.

A final event observed before submission removes the queued decision instead of allowing a late Accept/Reject to act on a later peer request. A command already copied into the host queue is canceled only if unsent; an already submitted command keeps its callback fence. Repeated requests for an already owned, unfinished identity reuse that decision without a second policy callback or deadline extension. A new request after a final event can use another entry while the old callback retires.

The owner can query pending identities and count accepted ACL reservations. Reservation counting excludes peers already published into the matching ACL runtime, because runtime link-added callbacks run before the GAP event observer. This provides the missing incoming-side state for future outgoing paging/collision integration. It is not the completed collision/reconnect policy.

Reply bodies keep their existing copied lifetime in the HCI queue. The reply entry's incoming index uses prior padding; no payload bytes or wire format tags are added. Mandatory reply queue entries and their outstanding HCI acknowledgments now precede nonurgent rename/setup work.

Setup has a conservative, explicitly owned 60000 ms host retirement bound, not a claimed Bluetooth-defined deadline. Missing final retirement fails closed. Its deadline participates in owner waiting even when command capacity is full. No polling task, busy wait, repeated allocation or periodic retry is added. The production event dispatcher supplies current owner time through dbt_gap_event_at; the legacy event API remains available with last-poll time semantics.

Unsupported eSCO setup now uses Reject Synchronous Connection Request (0x042a), while the existing unsupported SCO rejection remains valid. The [Core HCI specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/host-controller-interface/host-controller-interface-functional-specification.html), sections 7.1.8, 7.1.9 and 7.1.28, supplies the command/final-event boundaries. No synchronous audio profile is added or advertised.

## Nonblocking diagnostics and RAM

An optional owner observer copies public metadata into the existing security log queue with zero wait. Formatting/UART output stays in the existing low-priority logger. Queued, submitted, acknowledged, completed, retired and expired each produce a separate classic_connection_stage record with peer, command token, opcode, flags, capture time and explicit status/outcome validity. Acknowledgment is not labeled connection success. Keys and PINs are absent. Optional trace loss remains counted.

The ESP32 compiler reports GAP state 304 -> 392 bytes, a total increase of 88 bytes: four 20-byte setup entries and observer storage. The prior reply padding accommodates its owner index/counter layout. Reply entries remain 16 bytes and security log records remain 28 bytes, so the eight-entry log queue does not grow. No task, stack or heap allocation is added. Compiler-only size artifacts are retained under native_bt/_build/experiments/incoming-registry/.

## Released-policy clarification

The released baseline 0d85a975ec079d954f1bdd3e94d2fed70848c252 explicitly calls set_auto_reconnect(false) in bt_audio_sink/main/main.cpp at line 2296. Its native sink still saves last_bda. These owning files were clean when inspected. Speaker-initiated automatic reconnection remains an explicit new master-goal requirement, rather than an enabled baseline behavior proven lost in the port. Full paging/admission/collision policy, outgoing SDP and A2DP initiation still need integration.

## Evidence boundary

ESP-IDF 6.1 ccache compilation, source stability and workspace checks passed. Portable Clang C11 syntax/static analysis passed for GAP. Target-only compilation establishes the sizes above without executing a test. Flash readback and ordinary startup are recorded separately. No simulated controller, connection, paging, pairing, codec, playback or app test campaign ran, following the user's instruction. These checks do not qualify this lifecycle under RF load or establish full feature parity. Maximum-rate audio headroom and the reported codec crashes remain unresolved.
