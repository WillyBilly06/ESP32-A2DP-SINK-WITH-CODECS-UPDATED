# Owned OTA preparation and restart

The product build wires the existing BLE OTA commands to trusted admission,
transfer epochs and an owner/worker handoff. Unencrypted privileged writes
request real security; a bond alone is not a locally admitted control role.
First-boot and gesture policy, incarnation-bound NVS trust and security prompts
are described in `product_admission.md`. The wire format is retained; phone
interoperability is not established by local tests.

Verified END fences new mutations before selector activation. The production
`probe_restart` coordinator pauses fresh physical input sampling, drains copied
partial samples and observed EQ/control changes individually into their normal
immediate persistence lane, and waits for admitted control/CCCD/security/trust
work. It permits an already-issued AVRCP key release before holding AVRCP,
stops new audio/output/RFCOMM producers, closes dynamic channels, and waits for
worker/drop receipts, pinned TX, sound/output tickets and HCI transport work.
No guessed delay substitutes for resource retirement. A retained or failed
output ticket cannot report quiescence.

Only after preparation succeeds can the worker select the validated inactive
owned slot. END acknowledgment/status waits for selector success. Terminal
restart requires a fresh trusted epoch, selected state, status runtime-TX
admission, the runtime's 500 ms grace/10 s deadline, and a second quiescence
check. Main explicitly requires BOTH quiescence and runtime approval before
calling the platform restart. Runtime TX admission and controller completion
are not proof the phone consumed a notification.

Fault, expiry, stale epoch or preparation failure suppresses reset. Cancel
reverses producer holds; already-sent closes still retire normally, and borrowed
memory is never freed speculatively. Startup failures use the separate persistent
owner park, not automatic reboot. The coordinator itself never selects a slot
or resets hardware. Existing image/owned-slot/recovery and hardware-hold gates
remain applicable; no current ESP32 was accessed.

Local tests cover the real coordinator with subsystem/transport shims, an
adversarial main-handoff stub, actual GATT/runtime state machines with callback
and selector shims, and profile-specific simulated/owned-output tests. They are
not a single complete phone/SMP/NVS/audio/controller pipeline or hardware test.
