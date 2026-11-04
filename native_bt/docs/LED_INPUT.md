# LED and input provider: offline candidate

The providers are opt-in linked and integrated through `DBT_PROBE_PRODUCT_IO`.
The implementation covers all 22 user-selectable IDs and retains the baseline
wire schema and NVS keys. Integer renderers remain **approximations**, not a
pixel-for-pixel port of the baseline's floating-point algorithms. Spectrum
now tracks per-column smoothed heights and held peaks; ripples, rain drops,
stars, meteors, balls and lava blobs have active per-object state. Fireworks
and bursts use up to 100 and 80 moving particles respectively, with rocket
gravity and finite lifetimes; scope joins successive history samples and the
spiral renders three audio-driven arms. The baseline and candidate differ in
rounding, color composition, random call ordering, audio input, and demo timing.
The current tests do not establish visual parity or target performance.

`dbt_led_esp32_start` must run during boot on a non-host worker after board and
pin identity are established. It recovers any interrupted LED write, reads NVS,
opens its own SPI bus (normally SPI2, GPIO4, 3.2 MHz), allocates only the active
matrix framebuffer and DMA buffer, and starts a low-priority task. For a 16x16
matrix those buffers are 768 and 3094 bytes; shared active-effect scratch is
256 bytes. Fireworks lazily allocates 1024 bytes and burst 800 bytes; switching
releases the old allocation before obtaining the new one. An allocation
failure reports `DBT_LED_NO_MEMORY` rather than an activated effect; after a
durable settings commit the receipt retains `committed=true` and rendering
can retry. Rows use the baseline serpentine wiring, GRB order and 4-bit WS2812
symbols. SPI is exclusively owned by this provider. The task renders about 30
frames/s; synchronous transfers borrow the DMA buffer only until return. The
wire time alone is about 7.7 ms for 256 LEDs at 3.2 MHz, without CPU overhead.

`dbt_led_meter` accepts three already-computed 0..100 levels from
`probe_output_meter`. They are mapped -60..0 dB values, so a flash lookup
converts them to approximate linear amplitude for the effect renderers; bars
use the dB scale directly. `dbt_led_audio_active` supplies the owner's playback
state. More than five seconds without active audio selects effect-specific
synthetic demo inputs and beat periods; baseline demo frame accounting and all
of its waveforms are not fully reproduced. By default the provider estimates a
bass onset from snapshots. `dbt_led_beat` optionally publishes a detector edge
and intensity from the control owner, replacing that estimate after the first
publication. No detector is wired by this module; the output snapshot updates
approximately every 40 ms and can miss short beats. No DSP executes in the
host or LED task.

BLE full settings use `[effect, brightness(0..100), speed, r1,g1,b1,r2,g2,b2,
gradient]`. Use `DBT_LED_SET_ALL`, `DBT_LED_SET_EFFECT`, or
`DBT_LED_SET_BRIGHTNESS` with the corresponding value in that structure.
`DBT_LED_PREVIEW_EFFECT` changes the live view without persisting. Speed,
colors and gradient control Ambient, while other effects use their own colors
and audio response, consistent with the baseline division of responsibilities.
Effect IDs are Spectrum 0, Beat Pulse 1, Ripple 2, Fire 3, Plasma 4, Matrix
Rain 5, VU 6, Starfield 7, Wave 8, Fireworks 9, Rainbow Wave 10, Particle Burst
11, Kaleidoscope 12, Frequency Spiral 13, Bass Reactor 14, Meteor 15, Breathing
16, DNA 17, Audio Scope 18, Bouncing Balls 19, Lava Lamp 20, and Ambient 21.
ID 22 is the baseline's internal volume overlay and is not a user effect.
`dbt_led_volume` publishes a 0..127 absolute volume for a 2500 ms overlay,
with 1500 ms hold, 1000 ms fade, smoothed white-to-red bars and the baseline
minimum brightness 10. `dbt_led_pairing_mode` starts/stops the two-second blue
pulse; `dbt_led_pairing_success` exits mode and starts two 150 ms cyan pulses
separated by 100 ms dark intervals. Success takes priority over mode, then
volume; explicit mode false cancels pairing feedback. These publications are
atomic and ephemeral, are consumed on the next worker render, and do not alter
NVS. The worker uses monotonic elapsed milliseconds; host tests can use
`dbt_led_render_at` with a deterministic clock. The 10-byte internal blob keeps
unaffected fields byte-for-byte, including the full 0..255 brightness.

One control owner submits via `dbt_led_esp32_submit`, with a nonzero token never
reused during that provider lifetime. Admission is nonblocking and is not
completion. Drain `dbt_led_esp32_receipt`; BLE success is legal only for the
matching receipt with `DBT_LED_OK` and `committed=true`. Preview is never a
durable write. The bounded SPSC queues require input and BLE jobs to be
serialized through the same producer. The BLE worker must bridge its synchronous
command API to this receipt, without blocking the host loop. A failed transfer
following durable persistence returns `DBT_LED_TRANSFER_FAILED` with
`committed=true`; the worker retains the committed state and retries rendering.

Persistence uses a checksummed, versioned `led/dbt_intent` blob containing the
new settings plus the prior presence and values of `audio/led_effect` and
`led/{effect,bright,settings}`. It commits PENDING first, writes the baseline
keys, then commits DONE. On restart, only a valid pending intent is replayed, and
only if each observed key still matches its recorded old or desired value.
Conflicting external edits, invalid journal versions/checksums or read errors
fail startup without erasing or blindly replacing data. DONE does not overwrite
later baseline changes. On any write error, the instance rejects further writes
until stopped and restarted; no failed receipt is a success ACK. Recovery may
finish an intent after its caller saw failure, because the pending intent may
have reached flash before the error. Atomicity ultimately depends on NVS's
individual-blob integrity; cross-namespace writes are recovered, not a single
atomic transaction. There must be one LED provider instance and no concurrent
external writer to these keys during an active update or recovery.

`dbt_input_esp32_start` runs from an input task, never the host. It configures the
specified I2C port and GPIO inputs, validates Seesaw product 5752, and does not
reset or scan it. Baseline defaults are I2C1, SDA23, SCL22, 400 kHz, address
0x07, and physical buttons 18/21/19. Pin/board configuration must be confirmed
by the integrator; 0x36 is an alternate address. Initial Seesaw NeoPixels are
green/red/blue/yellow; brightness mode switches bass to white, effect mode
switches treble to purple, and pairing changes mid to cyan. I2C feedback errors
are counted and retried. Poll on the input task approximately every 25 ms and
drain `dbt_input_next` on the single control owner. Button debounce, multiclick,
four-count detents, brightness/effect modes, pairing and GPIO controls map to
the actions in `dbt_input.h`.

Each EQ detent is a separate +1/-1 action. The owner must retain an action on
BUSY and immediately persist every event through the same EQ path as BLE; it
must not merge EQ actions. Queue-full retains the exact sample for retry. Large sampled movements up to
4096 counts per encoder are drained in bounded 20-count batches without
repeating button debounce or coalescing EQ events. Jumps above that limit or an
ambiguous half-wrap explicitly increment `invalid_channels` and rebase only the
affected encoder, losing that channel's unobserved movement but keeping the
input task live. New physical motion, especially reversals, while sampling is
paused cannot be individually reconstructed. Map LED
preview/confirm, volume, transport, pairing and flags to the existing owner
operations. Owner control, GATT completion and CMake links are opt-in wired
through `DBT_PROBE_PRODUCT_IO`; see the current local candidate evidence.
Physical behavior and visual parity remain unqualified.

Offline tests check the baseline's pairing envelope at quarter-period points
to within one output level, exact success and overlay timing/precedence, the
spectrum's immediate rise, approximately 0.85 fall and 30-update peak hold,
independent beat-triggered rings, active-only allocation, OOM retry, and
bounded matrix access. The tests do not compare every baseline pixel or
establish real-time performance.

For shutdown, stop producing jobs, drain LED receipts, then call
`dbt_led_esp32_stop` on a non-host worker; it refuses outstanding jobs.
`dbt_input_esp32_stop` removes the owned I2C driver. No task may concurrently
enter either provider during release. CPU load, stack margin, flash stalls,
electrical timing, visible output and audio coexistence are unmeasured.

Offline tests: `python native_bt/tools/test_led.py`, `test_led_platform.py`,
`test_input.py`, `test_input_platform.py`, `test_led_target.py`. The LED host
test exercises every ID and a Bass Reactor ring-history vector; the NVS fake
injects faults through commit and recovery, including an intervening edit; the
SPI fake checks literal symbols; and the I2C fake checks NeoPixel commands and
mode feedback. The target script uses the existing Xtensa SDK compile flags for
syntax only. It does not link or touch a device. If the integrator elects to
bind this candidate, add `src/dbt_led.c`, `src/esp32/dbt_led_esp32.c`,
`src/dbt_input.c`, `src/esp32/dbt_input_esp32.c`, and dependencies
`esp_driver_spi`, `driver`, `esp_driver_gpio`, `nvs_flash`, and `esp_timer`.
