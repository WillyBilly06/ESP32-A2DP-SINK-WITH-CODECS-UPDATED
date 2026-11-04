# Resumable codec output — 2026-11-04

The master goal requires nonblocking audio output, retained ownership under
backpressure, bounded decoder work and full codec/lifecycle preservation.
The former worker treated every non-OK media result as a drop/reset, while
`probe_output_pcm_format()` waited up to 100 ms. Returning BUSY immediately from
that provider without changing codec ownership would have lost decoded audio.

## Implemented ownership

The codec API now distinguishes accepted `DBT_PENDING` work from BUSY/error.
All six codec families have resume operations, including aptX/HD/LL variants.
After one decoded block is accepted as PENDING:

- The decoder owns its existing PCM workspace. It does not overwrite the view
  until the worker consumes it and invokes resume.
- The worker pins the existing encoded queue slot by retaining `media_read`.
  Each codec retains its validated frame cursor; AAC and LHDC also retain
  completed fragment storage when further frames still need it.
- The worker submits at most 256 sample frames per provider call. OK consumes
  that complete chunk; BUSY consumes none and returns control immediately.
- Only after the PCM view is consumed does resume decode the next saved frame
  or complete the packet. No accepted frame is decoded a second time.
- Control jobs take priority. Suspend/reset/reconfigure/close/abort/drop cancel
  the retained view before decoder/input storage can be released.

There is no second PCM allocation or unbounded packet list. The metadata is
worker-owned; no new locks or shared mutable codec state are introduced. AAC's
bounded 64-unit LATM metadata is retained in its active working set rather than
on an expired stack frame; resource measurements must account for that change.

Each deferred scheduler iteration performs one codec emission/resume or one
PCM chunk, subject to its supplied budget. The original four-job control bound
remains. Codec constructors and stream providers still need their own bounded,
nonblocking completion contracts; this module does not certify a blocking client.

## Wake and failure semantics

After a BUSY result, the scheduling query reports not runnable. A renderer-space
notification retries the retained view. One absolute deadline detects 200 ms
without progress; successful chunks refresh it, so a valid multi-frame packet
is not rejected merely because its total playback duration exceeds 200 ms.
Fragment-expiry queries replace the need for unconditional periodic polling.
Expiry cannot invalidate a held completed packet/PCM view.

Fatal provider failure or stall closes admission and reports a worker fault;
the owner's DROP reclaims the working set. It does not allocate a replacement
codec just to report a fatal failure. BUSY never increments packet loss or
resets decoder history. Encoded queue saturation still returns an explicit
NO_SPACE result; complete receive-credit/backpressure integration remains a
separate requirement.

## Current evidence

- 171 portable host checks pass, including exact sample/timestamp ordering,
  pinned queue slots, saturation, deadline wrap, suspend/reconfigure/cancellation,
  no callbacks after retirement, and allocation/release symmetry.
- 80 cases execute the actual SBC, AAC, aptX/HD/LL, LDAC, LHDC and Opus algorithms.
  Every PCM byte and per-sample timestamp matches synchronous decoding despite
  simulated full-renderer responses and a 150 ms pause. Cases include all 37
  fixture configurations, completed fragmentation, multi-frame batches and the
  192 kHz/stereo/S24/1000 kbps fixture.
- The real-codec suite also cancels live borrows, suspends/restarts and
  reconfigures before dropping the retained state. All cases release their
  allocations exactly once.
- AddressSanitizer covers all source-built codec units, including the separately
  instrumented guarded Opus library. UndefinedBehaviorSanitizer additionally
  covers the owned adapters, worker and test harness. These 80 cases pass.
- The isolated ESP32 diagnostic now has a separate deferred-worker check that
  compares complete PCM and sample-timestamp hashes with its direct decoder
  phase, injects BUSY responses, checks heap retirement and records stack/work
  spans. Its target execution is a separate gate, not implied by host success.

Reproduce host checks:

```powershell
python native_bt/tools/test_host.py
python native_bt/tools/test_probe_audio_worker.py
python native_bt/tools/test_probe_codec_check.py --deferred --lhdc-source <sibling-decoder-repository>
python native_bt/tools/test_probe_codec_check.py --deferred --sanitize --lhdc-source <sibling-decoder-repository>
```

## Product integration remains open

The product probe still uses its legacy output/control provider. The deferred
worker flag is not enabled there merely because the codec tests pass. The next
step is a nonblocking producer admission API, safe producer/render quiescence
on format/fault/stop, asynchronous stream tickets/completion, and notification/
deadline-driven scheduling. Preserve sounds, mixes, volume/EQ, controls, OTA
retirement and all generation fences while replacing the provider.

Real DSP/I2S and simultaneous Classic/BLE/storage stress remain unqualified.
The released Bluedroid path remains intact and the full master goal is incomplete.
