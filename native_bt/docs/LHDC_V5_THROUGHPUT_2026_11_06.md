# LHDC V5 throughput work - 2026-11-06

**192 kHz LHDC playback remains unqualified and still underruns.** None of the whole-core or placement trials below is a completed fix. The user reports aptX HD and LDAC sound good on image92fc7e5705b7; that feedback does not qualify all rates/features or these later experimental images.

## Implemented, exact changes

The optional -UnityPcmCopy path replaces general per-sample conversion/phase bookkeeping with bounded copies into existing unpublished DSP/output spans for copied S32 stereo at unity clock. A span is at most128 frames and can read at most two input-ring segments. Other formats, borrowed views and fractional/adaptive modes keep their existing renderer. Final controls, mixing, input/output release publications and frame counts are preserved. No PCM buffer or persistent field is added. Linked cost relative to92fc is348 bytes of flash text with unchanged IRAM, DRAM data and BSS.

The actual general block renderer and candidate pass192 stream pairs under ASan/UBSan:787776 output frames and every DSP/resampler state match;262144 bytes take the copy path explicitly. Cases cover four rates,16/24/32-bit mono/stereo, unaligned caller input, near-wrap counters, non-power-of-two output pages, variable budgets, mixer callbacks, rapidly changing integer EQ/routing, and nonzero-phase/adaptive fallbacks. This is host correctness evidence, not target throughput or acoustic qualification.

## Measured bottleneck

At192k, the original Core1 run records IDLE1 watchdog warnings while native_render is active and CPU0 is idle. At a later copy-path first shortage, input has1024 ready frames, output has0, six encoded packets are queued, decoding waits for PCM space, and rendering has been active for several milliseconds. This is output processing/scheduling pressure; adding encoded buffers does not address that captured event.

The existing closed-channel handoff was made optionally selectable for LHDC with -LhdcCore0Render. Bluetooth/controller stay on Core1, decoder priority21 on Core0, renderer priority20. It uses the existing lazy6144-byte auxiliary render stack plus TCB; it does not create a new task class or migrate active I2S. Both whole-Core0 runs still fail.

The same-core admission threshold formerly protected only two128-frame DMA blocks:1.33ms at192k, shorter than an observed3.27ms decode. The high-rate branch now reserves one5ms frame interval plus those blocks (1216 output-ready frames), capped by the existing3072-frame target. Empty-input and control/failure/owner progress remain possible. The extracted actual function passes bounded ring/core regression checks under UBSan, including counter wrap and lower-rate compatibility. Windows runtime-link import warnings are retained in the receipt. This changes admission, not buffer allocation. Physical testing still fails overall throughput after this change; it must not be described as the maximum-rate solution.

An optional -DspBlockIram trial places the existing integer block function and hot analysis/filter/limit/ramp helpers in instruction RAM. ELF addresses verify placement. It uses2532 additional IRAM bytes, with unchanged DRAM data/BSS. Arithmetic is unchanged. The fixed900 physical trial still fails; no target speed benefit is established. The option remains off by default.

## Keep UI selection and actual bitrate separate

The Redmagic resets both sample rate and quality after reconnect. Each accepted high-rate trial must explicitly restore192k/24-bit and its requested quality, then audit received payloads. Auto selected400000 compressed bits/s in one run:506 media bytes with two frames. Fixed900 produced566 media bytes with one frame, or899200 compressed bits/s after excluding the two-byte media header and two-byte per-frame headers. The frame duration is5ms. The phone UI currently offers fixed900 as its highest fixed option; none of these live runs proves1000kbps.

The existing96-packet192k/S24/1000 encoded fixture is reused in the host arithmetic comparison. Three complete output hashes match baseline/candidate (2238681348 on this host), with all56 independent PCM points passing per trial. Host timing/stack values are stubs; they are not ESP32 deadline measurements. The historical Xtensa fixture hash is different, so the host gate requires same-host consistency rather than silently treating a target hash as a host oracle.

## Physical trial receipts

Each play/pause command interval is about12.1-12.3s with an independent14s pause guard, native YouTube on the approved Redmagic, COM6 only. Post-pause UART collection retains setup/drain/tail; counters therefore do not describe exactly the command interval. Images and selectors were written at921600, read back at115200, and matched. Owned VALID boot and37 existing cold codec cases pass; original app region is preserved and no automatic Bluedroid restore occurs. No new boot cases were introduced.

| Candidate | Received compressed bps | Initial shortage frames | Late-refill recovered | Unfilled frames | Descriptor errors |
| --- | ---: | ---: | ---: | ---: | ---: |
| Unity copy, Core1, Auto | 400000 | 4224 | 1664 | 2560 | 0 |
| Unity copy, Core1, fixed900 | 899200 | 173312 | 47552 | 125760 | 2 |
| Unity copy, Core0, fixed900 | 899200 | 1117824 | 469504 | 648320 | 0 |
| Unity copy + reserve, Core0, fixed900 | 899200 | 1171840 | 532608 | 639232 | 0 |
| Unity copy + IRAM DSP, Core1, fixed900 | 899200 | 466368 | 201856 | 264512 | 1 |

All listed high-rate trials have zero recorded decoder rejects, peak concealments, nonfinite frames and decoder-clipped frames. This does not establish acoustic correctness. Raw watchdog output interleaves/damages some UART JSON and worker detail rings lose records; retain raw receipts and do not claim complete call-by-call timing from these captures. Latched post-pause aggregates are kept separately. Initial shortage counters include late-recovered frames; use the separate refill fields rather than labeling every initial shortage as irrevocable loss.

The first Auto400 run is a lower-bitrate observation and is excluded from fixed900/full-target qualification. The fixed900 runs remain failures even where individual counters improve. Different YouTube segments and startup/tail durations prevent an isolated speedup percentage claim across these runs.

Exact images and complete build/readback/boot records are under native_bt/_build/hardware/runs. Per-trial UART, settings snapshots, comparisons and failed receipts are retained privately under native_bt/_build/experiments/lhdc-mantissa-1793995602565. Current IRAM image:12a5d586064e7b6271552449ddc42672aa5285bba6aee146ce4b3341852be7fb.

## Unadopted arithmetic experiments

The -LhdcFastMantissa comparison reuses a sliding byte window, an already loaded sign bit, and32-bit peak bounds. Two million planes /32012351 coefficients match the original under address/undefined checks, and the maximum host fixture hashes match. One target cold comparison changes total time by less than1%, with unchanged reference-point error and tracked peak allocation; this does not establish a speedup. It remains disabled in subsequent builds. The generated port preserves the original and its host tracing.

A private limiter prototype replaces a wide rational division using an integer reciprocal with bounded Newton steps and one exact quotient correction. It matches the original over all2033398579 legal divisor/magnitude values. It has not been integrated or timed on ESP32. The existing DSP/EQ remains fixed-point and the codec transform remains its original floating-point implementation; there is no unchecked full fixed-point conversion or lowered quality.

## Reproduce focused checks

Run tools/test_audio_unity_copy.py for complete PCM/state comparisons, tools/test_decode_reserve.py for the actual scheduling guard, and tools/test_probe_codec_check.py --lhdc-max-fixture --lhdc-esp32-paths --lhdc-source followed by the existing decoder checkout path. Add --lhdc-fast-mantissa only for the unadopted arithmetic comparison. Builds use ccache. The host fixture include order now selects the same generated2048-byte LHDC headers as the target, correcting the previous accidental1536-byte upstream header selection. Failed pre-fix receipts remain archived.

## Next engineering step

Whole-DSP placement on either core is insufficient under these observed workloads. Investigate bounded partial DSP assistance on the existing Core0 render task while the decoder is waiting for PCM space, with one active DSP owner and explicit completion/lifetime fences. Preserve exact block math, use unpublished existing output spans, return immediately while work is pending, and defer stream teardown/control application until borrowed work retires. This is a proposed next step, not an implemented or qualified feature. Full maximum1000kbps, BLE/control/storage coexistence, lifecycle, acoustic and sustained headroom gates remain open.
