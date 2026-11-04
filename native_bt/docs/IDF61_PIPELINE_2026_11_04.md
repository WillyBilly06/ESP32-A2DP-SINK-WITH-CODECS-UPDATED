# Maximum-rate LHDC to DSP/I2S diagnostic â€” 2026-11-04

This checkpoint uses actual ESP32 decoder, asynchronous worker, integer DSP and
I2S code. Its 96 loss-free source packets are the previously pinned true
192 kHz, stereo, S24, requested 1000 kbps fixture (1001600 payload bits/s).
A periodic 5 ms timer wakes the source; enqueue lateness and packet retirement
latency are measured separately. Each worker iteration performs at most one
codec emission/resume or <=256-frame PCM delivery. External callbacks never wait.
A complete pre-DSP PCM FNV hash is checked against 4008775619.

The diagnostic has an explicit build/deployment mode. It forbids controller,
security, product, filesystem and NVS initialization, does not mark its OTA
trial valid, and returns from core 0 to the pinned released ota0 image. Admission
verifies board identity, flash/layout, baseline contents and current selector.
The helper writes/readbacks only the trial slot and older selector sector.
Exact tested images, build receipts/source manifests and serial output are
archived locally. Full flash/NVS backups remain private and ignored.

## Measured allocation failure and correction

The first hardware image (source 5cd4a816, image SHA-256
95d79362cf7f332dad245231ae933b680d6e25662779d0545b9f76c0edcbf3cd)
configured the decoder but could not start output: DBT_NO_SPACE. Before setup,
free internal 8-bit heap was 280256 bytes, largest block 114688. After decoder
configuration and the failed output attempt, free was 216656, largest 110592;
the failed attempt reached a minimum of 151116. The renderer requested a single
131072-byte output block, which exceeds the available regions. No PCM or DMA
ran; zero underruns in that failed run are not continuity evidence. Decoder
and audio buffers were reclaimed, the baseline was selected and its clean boot
was observed. The retained renderer task/queue account for fixed initialization
resources; reclamation does not mean every task was destroyed before reset.

The first page correction used four 32768-byte output pages. That still failed
before output start: minimum internal free heap reached 52768 bytes, but the
remaining blocks could not satisfy another 32768-byte request. Cleanup and the
released baseline boot were again verified. This was not a playback result.

The current correction uses 4096-byte pages, retaining the exact logical ring
capacities and 64 ms output prefill target. At 192 kHz there are 32 output pages
(total 131072 bytes); the input remains 65536 bytes. The maximum supported ring
uses a 64-entry pointer table, adding 256 bytes on the target. No rate, depth,
bitrate or PCM precision changes. No extra PCM copy or allocation retry is
introduced. Constructor failure unwinds every allocated page. Normal <=128-frame
render units cross at most one page boundary; a full maximum-capacity mixer call
is bounded to at most 65 spans, including one ring wrap.

Host checks: 179/179 portable tests; ASan/UBSan poisons each allocation's unused
arena tail to catch intra-slot overruns, and covers all 65 constructor failure
points, page/ring/uint32-counter wrap, full-capacity sound mixing and all-sample
16/24/32-bit mono/stereo comparisons with a small ring through multiple wraps.
A three-thread producer/render/consumer test crosses 16 pages and uint32
counter wrap while comparing every sample to the DSP reference. Clock/resampler
tests and concurrent actual async sound/render with mocked I2S pass. The full
opt-in all-codec/product-I/O/security/RFCOMM image also compiles and passes offline
image/source/layout admission; it has not been deployed by this checkpoint. The actual linked image passes the owned ISR placement/direct-reference
audit. These are separate from device results below.

## Device result

A 460800-baud readback attempt failed with a transport digest mismatch before
the boot selector write. The helper reset without admitting that image. Partial
write attempts and verified completions are now archived separately on errors.

The same 4096-byte-page image was verified and executed at 230400 baud:

- Source checkpoint 16feae58; image SHA-256
  8bb9a2e6a9d3b63246f28c29bba9f17f071d17140fffcffb45fd96b62f183d03.
- Decoder and renderer construction both succeeded with unchanged rate/depth/
  bitrate and ring capacities. Internal free heap after open was 15848 bytes,
  largest block 10240; minimum during the test was 14384. Tracked decoder peak
  was 63688; renderer rings plus DMA payload requested 199680 bytes.
- The test **failed its functional/continuity gate** before DMA playback began.
  It supplied four packets, retired two, delivered 2880 PCM frames and rendered
  640 frames in 26010 us. The finite depth-two encoded queue filled. No complete
  96-packet PCM hash or DMA continuity result was obtained.
- Five render work spans totaled 14812 us, maximum 3062 us for a <=128-frame
  unit. At 192 kHz the 128-frame budget is approximately 667 us. This exposes a
  render-throughput problem. The spans combine conversion, resampling and DSP;
  they do not yet identify the cost of each stage or exclusive CPU utilization.
  Initial control ramps and diagnostic overhead are included.
- Worker spans totaled 16477 us, maximum 4364 us; PCM hashing accounted for
  1747 us. Source enqueue lateness reached 3630 us and packet retirement latency
  9169 us. These are distinct from decoder-only timing measured earlier.
- Core-1 stack remaining was 5428 bytes; renderer 4224. Decoder/audio buffers
  were reclaimed, no fatal boot marker occurred and the released 1.0.4-idf6.1
  ota0 boot reached System ready. The full product candidate was not flashed.

The next gate is measured conversion/resampler/DSP stage profiling, then exact
integer optimization and a rerun of the same full-quality workload. Enlarging
the queue would not fix the measured render throughput. Controller/BLE and
storage coexistence are not yet admitted by this 14 KB controller-off margin.

[Machine-readable image-bound evidence](IDF61_PIPELINE_RESULTS.json) preserves
both allocation failures, the transport failure and this failed-throughput run.

## Reproduce

```powershell
powershell -File native_bt/tools/build_probe.ps1 -PipelineDiagnostic -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir '<LHDC checkout>'
python native_bt/tools/validate_local_image.py
python native_bt/tools/audit_audio_isr.py
# COM6 authorization and preserved baseline are prerequisites:
python native_bt/tools/deploy_audio_trial.py --pipeline-only --baud 230400
python native_bt/tools/test_host.py
python native_bt/tools/test_audio_pages.py
python native_bt/tools/test_probe_sound_output.py --async-client --sanitize
python native_bt/tools/test_audio_clock.py
```

The short instrumented fixture is not a long-duration or RF/coexistence test.
Worker/render timings are elapsed work spans, including interruptions, rather
than exclusive CPU accounting. Hashing and stack inspection add diagnostic work.
Continuity counters are captured before deliberate stop/fade, so the stop tail
is excluded. Analog quality, full-product SRAM advantage, all profile/security
lifecycles, immediate per-EQ NVS coexistence, OTA and long stress remain open.
The released Bluedroid firmware remains the fallback. The master goal is incomplete.
