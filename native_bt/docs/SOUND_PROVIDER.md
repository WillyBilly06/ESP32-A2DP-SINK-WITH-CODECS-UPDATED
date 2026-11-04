# Owned sound provider and overlay candidate

This is a standalone source candidate, not yet wired into the probe or product.
`dbt_sound.c` owns one worker-serialized upload and its status;
`dbt_sound_mix.c` owns one active, worker-fed PCM ring and a nonblocking
render-owner mix hook; `src/esp32/dbt_sound_spiffs.c` supplies an explicit
ESP32 SPIFFS/NVS backend. The application must supply the dispatcher, bounded
copied command queue, sound event routing, renderer and lifecycle barriers.
None of these files mounts, formats, initializes, or erases a filesystem.

## Protocol and storage

The baseline unified protocol accepts mute `10 value`, delete `11 type`,
upload start `12 type sizeLE24`, data `13 seq8 bytes...`, and end `14`.
Types 0–3 correspond to `/spiffs/startup.wav`, `pairing.wav`, `connected.wav`
and `volumemax.wav`. The status byte uses bits 0–3 for file presence and bit 7
for mute. Mute/delete return `[10, command]` only after the respective backend
operation succeeds; failures return `[11, command, error]`. Start and each
written data chunk return `[31]`; a fully sealed, read-back, validated and
committed upload returns `[32]`; upload errors return `[33, code]`. The owner
must publish `[06, status]` when the client needs a sound status update. A
play command is not part of this BLE sound protocol: startup, pairing,
connected and max-volume events come from the application.

Chunks must be nonempty, at most 512 bytes, sequential (including 8-bit wrap)
and fit exactly within the declared size. A 30-second inactivity deadline,
peer handle plus globally unique nonzero link generation, worker-side peer
validity checks and a nonblocking atomic disconnect request guard a session.
`dbt_sound_poll` runs on the worker and reports a timed-out/cancelled owner; send
a failure notification only if that same generation is still connected. The
host must update its synchronized peer-validity view before or alongside
`dbt_sound_disconnect`; it must never call storage or `poll` in a host callback.
If disconnect races a commit already admitted, disconnect returns `DBT_BUSY`:
the worker may finish the storage mutation, but the host must suppress its
response for the stale generation. Backend errors can have ambiguous durable
effects; do not turn them into successful acknowledgments.

The baseline wire format has **no offset or CRC field**. This implementation
enforces the cumulative byte offset internally and computes CRC-32 over received
bytes, then compares it against a complete read-back of the temporary file.
That detects observed read-back differences; it is not an independent sender
checksum, power-loss guarantee or proof of flash persistence. The upload is
accepted only for a complete RIFF/WAVE file with bounded chunks, PCM format,
mono/stereo, 8/16/24-bit samples and 8–192 kHz rate. Sizes, byte rate, block
alignment and data bounds must agree. At most 256 RIFF chunks are scanned.

The backend uses exclusive `.dbt_sndN.tmp` names and leaves pre-existing temp
or backup names untouched. SPIFFS rename rejects an existing destination, so
after verifying both old and new WAVs it persists an exclusive 28-byte v1
`.dbt_sndN.txn` intent containing the slot, old-presence flag, old/new lengths,
old/new CRC-32 and intent CRC. It renames the old file to `.dbt_sndN.bak`,
renames the verified new file, verifies it again, then removes the backup and
intent. CRCs identify versions for accidental corruption; they are not a
cryptographic authentication mechanism.

After storage is mounted and before `dbt_sound_init`, the worker must inspect
all four slots using `dbt_sound_esp32_inspect`; a valid interrupted transaction
can be finished or rolled back with `dbt_sound_esp32_reconcile`. Both APIs
require the storage worker, playback and external writers to be quiescent.
Reconciliation checks fixed slot paths and every present candidate's exact
length, CRC and WAV validity before changing files. It preserves an old version
until a verified new version occupies the final path. An operation failure
returns an error and leaves the intent for another inspection; no success may
be acknowledged until reconciliation actually succeeds.

This **two-rename replacement is not crash-atomic**. Cuts before a valid intent,
a partially written/corrupt intent, an unexpected file or mismatched CRC leave
the slot blocked with ambiguous evidence preserved. A temp file created before
the intent exists cannot safely be distinguished from an unrelated pre-existing
file and is not automatically removed. Recovery of those states requires
explicit inspection and a separately authorized policy. The model tests cannot
prove actual SPIFFS ordering or power-loss behavior; physical qualification
remains required. New uploads also need enough free space for the temporary
file while preserving the old file.
Mute is stored in the baseline NVS namespace `audio`, key `sound_muted`, with a
commit checked before success; the caller supplies already initialized NVS.

## Playback ownership

The worker calls `dbt_sound_play_open` with a real output adapter and an
allocator. Both output start and stop hooks are mandatory; start returning
false must leave no output side effects. A muted or absent sound fails rather
than producing a fake success. Max-volume sound is forced to overlay. Other
types may request exclusive output; the product owns its A2DP/I2S handoff and
must preserve the intended startup/pairing/connected/max-volume semantics.

The worker pumps source reads and fixed-point linear resampling into a lazy
256-stereo-frame SPSC ring; a 256-frame input cache and state are allocated
only while this sound is active. WAV 24-bit input is reduced to 16-bit before
resampling, matching the baseline's conversion. Playback is capped at ten
seconds, with integer 10 ms fades, duck/restore ramps, bounded overlay gain and
saturating 32-bit mixing. The render owner may call `dbt_sound_mix` only while
its borrowed playback pointer is valid; it performs no allocation, filesystem,
logging or blocking work. Wait for `dbt_sound_play_ready` before first
publishing the pointer to render. Underflow produces silence for the effect and
holds ducking while the producer is active; it is not an audible-quality proof.

At finish or rate change, request stop with `dbt_sound_play_request_stop`, let
the worker pump to acknowledge it, and keep the render hook running until
`dbt_sound_play_drained` reports the ring drained and duck restored. Stop is a
bounded fade and discards the remaining queued effect after the fade. Then
explicitly quiesce **both** worker and render, ensuring neither has an in-flight
call or retained pointer, and call `dbt_sound_play_close` on the worker. Only
after close may a new rate/configuration be opened. The drained flag is not a
thread-join barrier. A playback read failure is reported by `play_pump`; the
application must retain that failure even if the remaining ring drains.
`dbt_sound_play_queue` provides the baseline's single pending slot: later
requests replace the prior queued sound. After close the worker can take it,
which drops the pending sound if it is now muted or absent; explicit stop must
also clear that slot. The product dispatcher still owns event policy and must
call these functions on the worker.

## Evidence and limits

`python native_bt/tools/test_sound.py` passed Windows host protocol, WAV,
sequence-wrap, CRC/read-back fault, upload failure, timeout/cancel, hook/OOM,
8/16/24-bit conversion, rate switch, fade, saturation and cleanup checks. It
also exercises the actual ESP32 provider against an in-memory SPIFFS/NVS shim,
including creation and replacement cuts after each modeled open/write/flush/
sync/close/rename/unlink, partial intent, corrupt temp, and failed recovery
operations followed by retry. It runs 64 worker/render thread iterations.
The same tests passed with `--sanitize undefined` (Windows UBSan; linker emitted
runtime-library warnings). A Windows AddressSanitizer attempt failed before
the test entered `main` with process status `0xc0000139`; it is not an ASan
pass. `test_sound.py` separately compiles all three source files to ESP32
objects using the existing compiler with warnings treated as errors.
Receipts are in `_build/sound/`; workspace guards run before and after builds.
`python native_bt/tools/test_sound_linux.py --list-inputs` prints the minimal
credential-free source list for a Linux ASan/UBSan run; the Linux driver has not
been run in this workspace.

No complete firmware link, target execution, real SPIFFS/NVS write, power-loss
test, I2S/acoustic output, Bluetooth interaction, codec coexistence or target
performance measurement has been performed for this provider. The user's
hardware hold remains active.
