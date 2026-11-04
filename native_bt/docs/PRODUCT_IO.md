# Product IO bridge

This bridge is source-implemented and locally tested with fake board/storage
providers. It has not been enabled in the firmware, linked on target, or run on
hardware. LED rendering remains an approximation of the baseline visuals; see
`LED_INPUT.md`. No hardware, filesystem mount, NVS, or Bluetooth operation was
performed by the tests.

After board identity and GPIO wiring are verified, mount the `spiffs` partition
at `/spiffs` in the authorized startup path with formatting disabled. Initialize
output, then call `probe_product_io_startup_worker(initial, hooks)` on the boot
worker before GATT state configuration. It checks that the `spiffs` partition
is already mounted; it never mounts or formats it. Flags allow LED, input and
sound independently, with SPI2/GPIO4/16x16 and I2C1/SDA23/SCL22/address 0x07,
buttons GPIO18/21/19. These are baseline defaults, not board verification. The
startup call fills the LED and sound portions of `initial` for enabled
providers. Before constructing the sound state, startup inspects all four
sound slots and refuses any blocked, malformed or unreadable recovery record
without changing any slot. If all slots are safe, it reconciles each valid
pending transaction and requires a CLEAN result. A failure during reconciliation
fails startup; a previously completed reconciliation can already have changed
its slot, while the backend retains its transaction recovery evidence. No
filesystem format or blanket cleanup occurs. A startup failure after an IO
worker was created is not retryable in the same process; restart cleanly after
resolving the error. The SPI/LED and
input workers are separate from the host and the GATT general worker.

Route the general GATT command callback through `probe_product_io_command` for
commands 5..7 and 0x10..0x14, with the existing product mux handling name and
OTA. The callback's context is ignored; `hooks.context` belongs to the immutable
startup hooks. The `peer_valid` hook must perform a bounded read of an atomic,
generation-bound, encrypted/bonded peer snapshot, rather than access live host
state. It is checked before and after operations. Both product and IO poll hooks
must be called on every general-worker poll, including retirement, without
short-circuiting either. Combine their boolean results for retirement. Likewise
call both peer-lost hooks on the owner. IO peer loss only publishes a
nonblocking sound cancellation; later status uses the peer generation.

From the owner loop, drain input actions with `probe_product_io_take_input` and
retain a popped action until its owner operation accepts it. Route each EQ
+1/-1 action separately to `probe_gatt_control_eq_delta(band, delta)`; retry
BUSY and never merge accepted detents. Map volume, transport, pairing and flag
actions to the corresponding existing host-owner actions. Translate effect
preview/confirm and brightness actions to `probe_product_io_local_led_relative`,
using a unique nonzero application token; the LED worker resolves deltas after
any preceding BLE write. `ADJUST_BRIGHTNESS` takes raw ±5, `ADJUST_EFFECT` and
`PREVIEW_DELTA` take ±1, and `CONFIRM_PREVIEW` takes zero. Absolute operations
remain available through `probe_product_io_local_led`;
queue admission is not completion. Drain `probe_product_io_take_local_led`,
then reconcile the LED view from `probe_product_io_led_snapshot` with
`probe_gatt_control_led_applied`. The snapshot also exposes a durable write
when a subsequent SPI transfer fails. For BLE LED commands, the worker returns
success only after matching durable and transfer completion. A unique atomic grant is
cancelled by peer loss before the LED task's commit admission, preventing both
commit and show; after admission, the operation may complete and a lost peer's
ACK is suppressed. Peer loss must first revoke the atomic peer snapshot, then
call the IO peer-lost hook. It waits at most three seconds for a result; late completion is correlated and never becomes a
later command's ACK. Failure may still have committed, so reconcile snapshots.

Call `probe_gatt_control_worker_required(probe_product_io_needs_worker())` on
each owner wake and keep driving the GATT owner loop. Local LED/play admission
and completion wake the owner. The demand includes queued work, in-flight LED
and sound loans. GATT-worker retirement quiesces the current sound/upload and
LED jobs but preserves the persistent sound manager, LED provider and input
worker for a subsequent lazy worker; it does not reinitialize their storage or
callbacks. The owner must continue draining completion queues. Provider-wide
shutdown is not implemented; these singleton providers live until reset.

`probe_product_io_play` copies an owner request into a four-entry queue.
`probe_product_io_take_play_result` returns a FIFO start result for every
accepted request; `DBT_OK` means the playback object was prefilled/attached,
not that the sound finished or was heard. A busy or failed start can retain an
output token until real retirement. The manager and its render wake callback
remain persistent across GATT-worker changes. While a sound loan exists, BLE
LED and sound-storage commands are rejected, local LED jobs wait, and the
product mux must reject/defer blocking name/OTA/activation work using the atomic
`probe_product_io_sound_busy` snapshot. This avoids adding those long storage
operations to the sound-pump worker during playback, but is not a target timing
guarantee: the sound ring holds only 256 frames, about 1.33 ms at 192 kHz.

Drain `probe_product_io_take_notice` and hand each copied timeout notice to
`probe_gatt_control_notify` on the owner, preserving and retrying on BUSY and
letting GATT reject stale peer generations. Publish changed
`probe_product_io_sound_status()` through
`probe_gatt_control_sound_applied`. Feed existing meter values and owner audio
activity to `probe_product_io_meter`; it only performs atomic publication.

`python native_bt/tools/test_product_io.py` exercises the real BLE parser,
LED core and sound upload state machine with fake storage, LED transport, task
and playback manager. It covers all-slot recovery preflight, blocked/error cases and successful
reconciliation before sound initialization, completion-gated ACK, LED failures
and late receipts, generation cancellation, real WAV upload, timeout notices,
input queue handoff, loss before/after LED admission and stale generation,
relative LED updates, playback gate/receipt, and worker recreation. Recovery in
this integration test uses a fake backend; the separate in-memory SPIFFS shim
executes the actual backend across creation and replacement crash boundaries. It does not model
FreeRTOS scheduling, SPIFFS latency, actual sound pumping or hardware.
`test_product_io_target.py` syntax-checks the bridge with the current Xtensa
SDK flags; a combined target link is still required. Integrator build sources
are the bridge, LED/input sources listed in `LED_INPUT.md`, `dbt_sound.c`,
`dbt_sound_mix.c`, `dbt_sound_spiffs.c`, and `probe_sound.c`; SDK dependencies
include SPI, legacy I2C/GPIO, SPIFFS, NVS and esp_timer. No live qualification
is implied by any of these checks.
