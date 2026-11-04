# Sound output integration candidate

The optional `DBT_PROBE_SOUND_BUILD` path connects the owned sound worker to
the actual native render/output code. It has not yet been wired into
`probe_main` or a product dispatcher. The build owner must add
`probe_sound.c`, `dbt_sound.c`, `dbt_sound_mix.c` and its chosen storage backend,
define `DBT_PROBE_SOUND_BUILD` consistently for the probe sources, and retain
the output, DSP, resampler, heap, FreeRTOS, timer and I2S dependencies. For the
SPIFFS backend also add `spiffs` and `nvs_flash` dependencies. The existing
unconfigured output path remains available without this flag.

The render owner applies the borrowed sound mix hook after input conversion,
fractional FIR and integer DSP, before publishing frames to the DMA ring.
The ISR remains a consumer of already-rendered samples. The sound worker is
the only producer of its separate sound ring; it never writes the codec input
ring. With no codec output active, the renderer creates zero-valued base frames
and mixes the sound into them. With a codec active, sound is an overlay so the
music producer remains intact. If a music stream is open but has no input,
the renderer may add bounded silence only as its output ring empties, allowing
the sound to progress; clock tracking is reset and suspended across that
silence and reset again when music samples resume. A real clock/latency or
audibility result requires separate measurement.

The sound worker wakes render after publishing PCM or requesting stop; render
wakes the persistent sound dispatcher after consuming frames or retiring a
loan. The worker-to-render wake also matters during standalone priming, before
there are I2S EOF interrupts. These notifications do not establish that either
task meets its real-time deadline.

Output requests from the codec and sound workers share one durable ticket and a
bounded admission gate. Workers that cannot acquire the gate or enqueue receive
`DBT_BUSY`. An admitted request waits at most 1,500 ms for its own completion;
on expiry it marks the ticket abandoned and returns `DBT_BUSY`. The single
retirement slot remains occupied and `probe_output_stats().request_fault` stays
set until render acknowledges cancellation. New requests fail busy while it is
occupied. The late renderer skips a queued cancellation, or reverses a
completed late start/reservation/attachment before returning ownership; a
wrong-ID completion cannot release the ticket. If render never progresses,
the fault and at most one ticket and any associated active output/sound
allocation remain pinned; no automatic reboot occurs. `probe_output_stats`
also exposes the count of stale completions. Host and ISR paths must never call
the request APIs.

The caller first initializes output, the sound provider, and one long-lived
zero-initialized `probe_sound_t`. Prefer `probe_sound_init_with_wake` with a nonblocking,
render-task-safe callback that signals the dispatcher's persistent semaphore;
its callback and context must outlive all render references. This avoids
retaining a deletable worker task handle when the logical worker retires and
is recreated. The original `probe_sound_init` task-notification API remains
available for a worker whose handle has the required lifetime. Initialize the
manager once; do not reinitialize, rebind, or destroy it merely because the
logical worker changes. Worker transfer is allowed only after its calls are
quiescent and while the persistent manager and callback remain valid. The
application routes
sound control/upload commands to the same worker that owns the provider, using
the existing `dbt_sound_command` API. `probe_sound_disconnect` is the only
host-safe nonblocking entry and only requests cancellation. The worker calls
`probe_sound_poll` regularly and on wake; `DBT_EMPTY` from that call reports an
aborted upload, with the old peer identity in the supplied output. The owner
must suppress notifications for stale peers and inspect `last_result` for a
playback read failure.

`probe_sound_play` returns success after output reservation, sound prefill and
attachment. An attach timeout can return `DBT_BUSY` with `play` and `token`
still retained: do not destroy or reinitialize `probe_sound_t`; continue worker
polling until render acknowledges retirement and those fields clear. If render
is permanently dead that bounded allocation remains pinned. If a sound is
already active, success means it replaced the single pending slot, not that the
queued sound has begun. The caller supplies a
preferred standalone rate; while music is active the negotiated output rate is
used and the sound is resampled to it. A conflicting start or resource failure
returns an error without a fake playback acknowledgement. `probe_sound_stop`
requests the fade and clears the pending slot; keep polling until the pointer
and token are retired. The worker must not free its provider or callback context
while output may still refer to it.

When a codec stop or a new codec rate arrives during a sound, the render owner
requests sound stop, waits for its worker and the render fade to drain, retires
the borrowed pointer, and only then closes/reopens output. A standalone sound
also waits for its rendered ring to drain before output teardown. Sound worker
cleanup occurs after the render owner publishes token retirement; it never
frees a pointer still borrowed by render. This is an explicit lifecycle, not a
claim that a codec change or sound event is gapless or acoustically clean.

`python native_bt/tools/test_probe_sound_output.py` runs the production render,
request, sound and DSP code with thread-safe simulated tasks and a mock I2S
driver. It checks standalone playback and stop, sound over running 16/24/32-bit
music, an open music stream without PCM, return to music, codec stop during
sound, standalone-to-codec rate change, OOM, wrong-token rejection and 32 pairs
of competing codec requests. The same test passed Windows UBSan via
`--sanitize` (the linker emitted runtime-library warnings). The existing output
format, typed-control and clock tests also passed locally. The integration test
compiles the three changed output/sound/audio units with the existing ESP32
compiler and `-Werror`; it does not link a complete firmware.

The mock also exercises the persistent external wake callback without a task
handle. It pauses render past the deadline, injects an unrelated
completion ID, checks retained tickets and rejected new work, then resumes render
and verifies cancellation/recovery. It tests an admitted codec stop abandoned
before execution and sound attachment timeouts both before and after render borrows the
pointer, including caller stop/poll while render is paused. These are controlled
host schedules, not an exhaustive proof of every scheduler interleaving.

No physical I2S, GPIO, flash, serial, Bluetooth, acoustic, real-time or
long-duration test was performed. The user's hardware hold remains active.
