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

The [source-bound local receipts](PCM_RENDER_LOAN_LOCAL_RESULTS_2026_11_04.json)
also record a separate `-PcmLoans -PipelineDiagnostic -AudioStageProfile` image
that builds and passes offline admission. That isolated image has not run on
COM6; its controller-free layout is distinct from the full RF image.

The [first full-product COM6 trial](RF_PCM_LOANS_LDAC96_2026_11_04.json) confirms
56320 bytes of live LDAC96 PCM/DMA payload versus the previous 89088-byte
policy, a 32768-byte input-ring removal. All 4882 renderer loans retire; stale
receipts, request faults, PCM failures/timeouts and descriptor errors remain
zero in this finite trial. Maximum renderer hold is 15359 us. Rendered data
includes the existing finite stop/drain tail; no DAC/acoustic quality pass is
inferred from a retired input pointer.

Audio continuity still fails: mean work is 1950 us per bounded render unit
against a 1333 us/128-frame budget, with 1290 encoded-queue drops, 1317376
underrun frames and five core0 watchdog warnings. Duration/control differences
prevent an isolated CPU-speed comparison with earlier images.

The [subsequent maximum LHDC attempt](RF_PCM_LOANS_LHDC_MAX_2026_11_04.json)
confirms 192 kHz/S24/stereo/1000 kbps target in Android but still fails an output
page allocation: 4096 bytes requested, 11440 bytes free after partial allocation,
largest block 3200. No LHDC I2S start follows. The input ring is omitted, but the
102400-byte output-page requirement and remaining product/codec allocations
still need SRAM work. Full headroom, storage/OTA/controls, lifecycle stress and
feature parity remain unqualified. The bounded capture hit its 1 MiB limit;
three malformed/interleaved/truncated records are excluded. COM6 returned to
released 1.0.4 with System ready and phone playback was verified paused.
