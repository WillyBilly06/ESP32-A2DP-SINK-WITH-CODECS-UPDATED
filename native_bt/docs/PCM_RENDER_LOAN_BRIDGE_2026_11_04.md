# Ticketed PCM loans in the experimental renderer

`-PcmLoans` connects the decoder worker's immutable PCM loan to the actual
render owner. The copied-input path remains the default. This option requires
asynchronous I2S and a finite, unconfirmed boot; it excludes checksum-only RF
mode and cannot keep the candidate selected permanently.

The existing single ticket gate copies one small request descriptor, including
rate, channels, container width, frame count and PCM pointer. It never copies
the PCM block. The renderer accepts at most 256 frames, processes bounded
128-frame units, then acknowledges retirement only after consuming the view or
retaining its final raw boundary frame in owned storage. Pointer fields are
cleared before completion. The renderer and generic worker both retain their
earlier integer math, phase/history, clock policy and ownership fences.

Cancel before dequeue, during rendering, during completion and after DONE all
require owner-side retirement. A stale id never acknowledges a current view.
DMA failure cancels the active view before reporting completion. Reset, close,
codec reconfiguration and source-buffer reuse remain behind the worker's PCM
retirement fence. Stop can then drain/fade processed output without accessing
the codec buffer. All runtime begin/poll/cancel attempts are nonblocking.

The renderer omits its previous input ring. At 192 kHz/stereo/S32 its remaining
output pages require 102400 bytes; nominal DMA payload adds 3072 bytes. At
96 kHz the corresponding payload is 56320 bytes. Driver/allocator overhead,
decoder state, encoded queues and other product resources are additional.
Omitting an allocation does not establish full-product maximum-rate admission.
Output target remains 64 ms; no codec setting, DSP stage or precision is lowered.

Separate `i2s_pcm_loans` records expose renderer accepts, normal retirements,
active cancellations, active view, maximum renderer hold time and the input
allocation avoided by a successful stream open. These counters describe view
ownership, not successful DAC playback. Rejected/queued-before-accept requests
are outside the renderer-borrow counters. The isolated pipeline has its own
`pipeline_pcm_loans` record. Existing CRC/snapshot windows still apply to the
periodic logger; event and isolated-diagnostic output have their stated scope.

Local validation:

- Actual provider/render code passes ASan/UBSan with 16/packed24/S32 sources,
  cancellation at four phases, stale identities, DMA faults, fractional carry,
  allocation unwind and restart. Retired source storage is poisoned to detect
  reads after acknowledgement. The I2S device/queues are deterministic stubs.
- Concurrent async sound/output code passes UBSan with loans enabled: pending
  ownership, stop/timeout/cancel, overlays/latest queued sound, missing files,
  OOM, codec takeover and DMA-fault retirement. The external test producer may
  wait while pumping sound; production callbacks use begin/poll and never wait.
- Real profile/worker tests select the actual loan hooks for SBC, Opus variants,
  AAC and aptX variants, using a delayed provider mock.
- All 80 real-codec cases pass both the existing copied path and asynchronous
  loans with a delayed provider mock. Every PCM byte and sample timestamp
  matches synchronous decoding across 37 configurations, fragmentation,
  batching, a 150 ms stall and the maximum LHDC fixture. Codec sources use ASan;
  owned adapters/worker/harness also use UBSan. This does not test real I2S timing.
- The 184 portable groups and copied-input sound/output regression pass.
- Seven actual CMake configurations, eleven actual image-admission cases and
  two early CLI rejection cases verify the optional mode. The image validator
  binds receipt, CMake cache and actual provider/client compile flags.

```powershell
python native_bt/tools/test_probe_output_loan.py
python native_bt/tools/test_probe_sound_output.py --async-client --pcm-loans --sanitize
python native_bt/tools/test_probe_audio_worker.py --async-output --pcm-loans --decoder-core 0
python native_bt/tools/test_probe_codec_check.py --deferred --pcm-loans --sanitize --lhdc-source '<LHDC checkout>'
powershell -File native_bt/tools/build_probe.ps1 -PcmLoans -AsyncAudio -DecoderCore 0 -WithProductIo -WithRfcomm -WithLhdc -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir '<LHDC checkout>'
```

The isolated pipeline also accepts `-PcmLoans -PipelineDiagnostic`. Compilation
and local tests do not qualify maximum LHDC RAM, CPU headroom, sound continuity,
physical output or complete feature parity. Hardware validation is pending.
Released 1.0.4 remains the fallback.
