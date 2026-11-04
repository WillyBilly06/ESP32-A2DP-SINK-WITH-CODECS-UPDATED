# Ticketed renderer integration — 2026-11-04

This checkpoint extends the resumable codec worker with nonblocking stream
completion and adds nonblocking request/PCM entry points to the actual renderer.
It does not qualify the full custom host or replace released Bluedroid firmware.

## Ticket ownership

One fixed result slot combines ticket identity and state in a single lock-free
32-bit atomic. Reserve is one CAS attempt; queue publication never waits. A
stale poll/cancel cannot change a different current ticket. The 29-bit sequence
skips zero and current/last-retired sound token identities on wrap.

Begin returns PENDING only after accepting a copied descriptor and a nonzero
ticket. BUSY accepts nothing. Poll consumes one terminal result. Cancellation
in WAITING or COMPLETING preserves ownership until the render owner retires the
side effect; cancellation after DONE first schedules owner-side reclamation.
CANCELLED is explicit and is not confused with BUSY, INVALID or successful
application. Cancelled asynchronous results retain the slot until consumed.

An attached sound play object remains borrowed beyond its successful attach
completion. A release cancellation does not cancel that earlier loan. Clients
must observe sound-token retirement before destroying the object. The existing
legacy wrapper retains its previous late-cancel reclamation policy while its
remaining callers are converted.

The slot uses the same three 32-bit words as the old gate/result/id fields.
No new PCM pool, mutex or task stack is introduced by the ticket primitive.

## Worker and provider path

The codec worker holds its control job and receipt capacity through an accepted
stream ticket. Media stays gated until real completion. A newer epoch requests
cancellation before the next job begins; an accepted pending operation is never
treated as a rejected BUSY that can resume the previous producer.

Begin/poll/cancel advance once per scheduler invocation. A 1,500 ms deadline
latches a fault and requests retirement; a permanently unresponsive renderer
keeps its bounded ownership rather than freeing live resources or spinning.
Later notification can complete reclamation. Constructor/configure work still
has a separate bounded-allocation qualification requirement.

`probe_output_try_pcm_format()` makes one whole-block submission attempt and
returns BUSY without copying or waiting when full. The opt-in `-AsyncAudio`
product-probe build uses that entry point, deferred PCM, renderer stream tickets,
and codec-worker notification/deadline waiting instead of its 20 ms poll.
Codec telemetry is published only after a successful stream completion.

The default build keeps the prior path. Sound's high-level legacy client and
the render task's existing poll remain open work; opting in does not establish
that every runtime-critical operation is asynchronous.

## Verification scope

- 177 portable checks pass, including ticket phases, stale completions, overflow
  ids, reservation abort, sequence wrap, control/media fencing, cancellation
  before DROP, deadlines and receipt-capacity reservation.
- Actual render/output/sound code passes concurrent mocked-I2S tests with UBSan.
  Tests pause the renderer and prove begin/poll/cancel use no task wait or
  blocking queue send, check immediate full-ring BUSY without partial publish,
  reclaim an already-applied start, and retain attached play objects through
  cancellation during completion. Existing mixed/standalone/fade/restart cases
  continue to run.
- The opt-in probe audio path passes the AVDTP/real SBC, Opus, AAC and aptX
  host harness with the actual audio ring and a simulated output handshake.
- The 80 real-codec deferred PCM/byte/timestamp and lifecycle cases still pass
  AddressSanitizer, with UBSan on owned adapters/worker/harness.
- Full-feature ESP-IDF compilation is separate evidence from device execution.
  This checkpoint has not flashed or accessed COM6.

```powershell
python native_bt/tools/test_host.py
python native_bt/tools/test_probe_sound_output.py --sanitize
python native_bt/tools/test_probe_audio_worker.py --async-output
python native_bt/tools/test_probe_codec_check.py --deferred --sanitize --lhdc-source <decoder-repository>
./native_bt/tools/build_probe.ps1 -AsyncAudio -WithProductIo -WithRfcomm -WithLhdc -WithOpus -WithAac -WithAptx -WithLdac -KeepOwned:$false -LhdcSourceDir <decoder-repository>
```

## Still required

Convert the high-level sound client; replace renderer polling with explicit
readiness/fault notifications and stop/meter/clock deadlines; qualify quiescence
on every fault/format/sound transition; run the real DSP/I2S pipeline on COM6;
measure maximum-rate RAM, cold/steady deadlines and underruns; then test actual
Classic/BLE/AVRCP/storage coexistence and the complete feature/security/stress
matrix. Immediate per-EQ NVS persistence and every codec/feature remain binding.

The sound-client conversion must also update `probe_product_io.c`: `sound_tick`
and restart/quiescence currently count only `player.play`/`player.token`, and
`play_step` publishes its result immediately after `probe_sound_play`. Preparing
tickets must count as busy, retain the epoch/result reservation, and not report
successful playback or permit destruction before the asynchronous open/attach/
retirement outcome is known. Preserve its existing queued sounds and overlays.
