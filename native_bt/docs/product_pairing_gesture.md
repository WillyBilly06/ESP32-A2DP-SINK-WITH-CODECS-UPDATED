# Physical pairing gesture: offline implementation

The middle encoder gesture snapshots the Classic ACL identity of the current
AVDTP signaling peer (including a paused session). It submits one disconnect
for that handle and generation, waits for controller status and link removal,
then requests discoverable and connectable Classic scanning. With no active
AVDTP peer it requests scanning directly. Pending queue pressure is retried;
rejected commands, a missing disconnect event, and failed scan completion are
counted as failures. Command admission alone is not disconnect completion.

The product IO owner and audio peer query run only on the host owner. The
runtime link hook must forward every link change to
`probe_product_io_owner_link`: on removal it cancels a queued, unsent command
before the host sends a potentially reused handle. A submitted command is never
replayed. Other Classic and LE links and all bonds are left alone.

After scanning is confirmed the action requests the pairing sound and LED
animation. The animation changes to its brief connection feedback only when
an AVDTP signaling session opens after the gesture. This is connection
feedback, not proof of a new bond. The action does not relax the peer-restricted
Classic security test policy; broader product pairing admission remains
incomplete. No live device observation is implied by host-only tests or target
compilation.

Before enabling the controller, the output and persistent IO provider start;
an unmuted, present startup sound is handled by a dedicated boot worker and its
output loan is retired before enabling Bluetooth. A missing file, muted sounds,
or unavailable sound provider skips optional feedback. A retained loan or
failed output prevents controller startup. Startup attachment is reported
separately from loan retirement and neither proves an acoustic result.

An AVDTP signaling connection carries its ACL generation into the owner.
Configuration schedules a connected sound after 400 ms unless a reconnect
followed a disconnect in under five seconds; a started stream, low internal
heap, or stale connection suppresses it. Volume feedback is gated by that
profile identity, two-second connection grace and cooldown, and uses overlay
mode. Disconnect cancels pending connected feedback; the baseline defines no
disconnect WAV. A sound epoch cancels queued stale jobs, and active playback
uses the normal worker stop and output-loan retirement path. Cancellation
that races a worker already opening a sound requests stop as soon as the worker
regains control; it cannot retroactively undo samples already submitted.

Startup fault lifetime: ESP-IDF deletes its main task when `app_main` returns.
Startup failures therefore suspend the owner indefinitely instead of returning
or automatically rebooting. Providers, workers and any unretired output loan
remain allocated; the bootstrap worker may continue its normal retirement if
it can. Its owner notifications still refer to a live task. An unexpected task
resume returns to the suspension loop. Faults before controller enable do not
enable it; a failure at or after that call retains the actual controller state
without claiming it was disabled. The owner does not automatically retry the
failed startup or erase settings. The host harness covers the park loop and
startup exit structure, not FreeRTOS scheduling, controller behavior, output
retirement, or acoustic behavior on the target.
