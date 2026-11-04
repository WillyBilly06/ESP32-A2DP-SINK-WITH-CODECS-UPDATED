# Asynchronous sound manager — 2026-11-04

The opt-in `-AsyncAudio` path now prepares and plays notification WAVs through
bounded manager steps. It uses renderer begin/poll/cancel tickets rather than
the compatibility wrappers that wait for output progress.

## Ownership and replies

Open, prefill, reserve, attach, play and release are explicit states. Each poll
does one state transition, one bounded pump, or one nonblocking ticket attempt.
WAV reads remain on the provider/storage worker; they do not run in transport,
decoder, render or ISR callbacks. There are no repeated allocation retries.

A pending start counts as busy even before `play` or `token` exists. Product I/O
pins the original copied job and reserves its reply slot until an actual start
outcome is known. An epoch change/worker retirement cancels it and suppresses
success for the retired request. Queued retirement results are drained with a
bounded one-job step, not silently discarded.

Timeout/stop of accepted reserve or attach retains the ticket and play object
until the renderer acknowledges retirement. Release completion must also
retire the sound token before freeing a successful attachment. A release
cancellation alone is not permission to free that earlier loan. Faults retain
bounded ownership rather than freeing live buffers; explicit stop can retry a
release that was never accepted. Standalone, overlay, latest-one queued sounds,
max-volume overlay policy and codec takeover remain represented.

## Renderer readiness

The render task now waits on notifications and explicit stop/meter/clock
deadlines rather than an unconditional 10 ms poll. Malformed-DMA ISR exits wake
the render task as well as publishing failure. Stop/fade and short-sound
prefill make bounded progress without requiring a poll.

The readiness change exposed a continuity risk: eagerly refilling a virtual
silence base could outrun an unfinished WAV producer. `dbt_sound_play_mix_frames`
now bounds normal file rendering by actual available samples. A live empty ring
waits for producer data; EOF/stop permits fade/drain silence. While DMA has a
safe reserve (or has not started), an overlay producer also gets a turn instead
of being padded with avoidable gaps. A genuine low-water music emergency still
prioritizes DMA/music progress; full load/stress qualification remains required.

DMA failure can retire a sound loan that cannot drain, on the render owner,
before the manager destroys it. This is separately tested from a normal end.

## Verification

- Actual concurrent render/output/async-sound code passes UBSan with simulated
  FreeRTOS tasks and I2S. Tests cover pending ownership, no task waits/blocking
  queue sends, missing files, OOM, stop before open/during reserve, accepted
  timeout, attach cancellation at three handoff phases, overlays, latest queued
  sound, codec takeover and failed-DMA retirement.
- Product-I/O tests cover preparation busy status, reply-slot retention,
  deferred completion, epoch cancellation and worker retirement.
- Sound-provider/mixer tests cover available-sample bounds, EOF/stop behavior,
  WAV/protocol/lifecycle and producer/render thread stress.
- Existing legacy sound/output and 16/24/32-bit output-format tests remain
  separate regression checks. The previous path remains the default.
- The full ESP-IDF opt-in build is a compilation gate. No new COM6/device
  execution is claimed by this sound-manager checkpoint.

```powershell
python native_bt/tools/test_probe_sound_output.py --async-client --sanitize
python native_bt/tools/test_product_io.py
python native_bt/tools/test_sound.py --sanitize undefined
python native_bt/tools/test_probe_sound_output.py --sanitize
python native_bt/tools/test_probe_output_format.py
```

## Next hardware gate

Run an isolated, finite decoder→integer DSP→I2S diagnostic on COM6 with no
controller/NVS/product-provider initialization, retaining the pinned released
baseline and core-0 return path. Highest priority remains true
192 kHz/stereo/S24/1000 kbps input, actual DMA continuity and cycle/heap/stack
measurements, followed by the complete coexistence workload.

The current renderer's 192 kHz/S32 stereo configuration derives 65,536 bytes
of input-ring storage, 131,072 bytes of output-ring storage and 3,072 bytes of
DMA payload, before driver/metadata/stacks. These are source-derived sizes, not
measured runtime memory. Qualify actual RAM and constructor failures before
claiming a full-product fit or a RAM advantage. No sample-rate/depth/bitrate
reduction is allowed to make the test fit.

RF, analog sound quality, click-free EQ, storage coexistence, all feature/security
lifecycles and sustained stress remain open. The full master goal is incomplete.
