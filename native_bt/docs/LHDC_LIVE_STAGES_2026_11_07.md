# Live LHDC stage-cycle snapshots — 2026-11-07

Two small decoder candidates preserved PCM but did not improve whole-decoder timing on the current native YouTube workload. Their original entropy path is restored before collecting a live baseline.

The existing optional stage clock now supports the complete stay-native audio product as well as the existing isolated maximum diagnostic. It does not enable new boot cases, maximum fixtures, fallback firmware or additional tasks. Existing stage marks record six channel stages: preamble, entropy, mantissa, dequantization/SNS, IMDCT and overlap. CCOUNT measurements include preemption and instrumentation overhead; they are not exclusive arithmetic cycles.

The decoder alone updates its original64-bit finite counters. Every32 channel calls it publishes low-word sums, maxima, count, configuration generation, rate and container width through a fixed lock-free atomic mirror. A reader takes at most3 attempts and leaves its output unchanged if unavailable. There is no allocation, UART output, lock acquisition or resource retry in the producer. Main startup enables live visibility only after cold checks; it touches only an atomic flag. Cold measurements remain hidden. The background logger formats lhdc_live_stages records with named arrays, CRC/snapshot metadata, modulo2^32 sums and explicit instrumentation/cadence labels. A configuration close publishes the final partial group; snapshots can lag up to31 channel calls during a stream.

Host ASan/UBSan covers the actual collector and extracted logger: cold/live separation,32-channel cadence, forced odd publication, unchanged reader canary,32-bit revision/sum/generation wrap, final partial publication, inactive behavior and preserved64-bit finite sums. Six actual CMake mode-gate cases pass. The existing diagnostic suite passes after correcting its stale Core1 mock expectation to the shared configured decoder core. The stale failure is preserved; no production task affinity was changed.

Complete ccache build/source/admission pass. Debug candidate70b00966ad9067fbeba01225efc0a1fd9e2486a913ec097b8e13da05a5def623 is1339952 bytes. Relative to59b2, IRAM-56, BSS+176, data0, flash text+812 and rodata+432 bytes. Relative differences also include removal of the unproven batch loop. The mirror/counters are only linked with -LhdcStageProfile; this is profiling overhead, not a production memory saving. Candidate not yet flashed at document creation.

Experiment: native_bt/_build/experiments/lhdc-live-stages-1794062555568.

    python native_bt/tools/test_lhdc_live_stages.py
    python native_bt/tools/test_codec_diagnostic.py

The live build uses the normal full-product flags with -LhdcStageProfile. It does not use -LhdcMaximumCheck or -CodecDiagnosticOnly. New validation checks the real decoder/main compilation definitions and linked source recipe before admitting that mode.

## Verified COM6 live profile

70b00966ad90 is flashed at921600, full written-image readback at115200 matches, original app preserved, owned VALID and no automatic Bluedroid return. All37 cold configurations pass; tracked peak56899, minimum free103116, cold stack low3464 bytes. No live-stage row appears before playback.

Native YouTube metronome at actual192k/S24/fixed900, phone max150/150, produces5632 published channel calls in configuration generation7. Per-stereo-unit means at nominal240MHz are: preamble41.88us, entropy483.35us, mantissa140.00us, dequant/SNS211.00us, IMDCT853.96us, overlap99.49us. These sum to1829.69us. The whole codec execution counter records2469.94us per actual decoded unit across2824 units, leaving about640.26us outside the channel marks. That remainder includes frame output and wrapper work and is not attributed solely to a limiter or conversion. Maxima are per-channel, may include preemption, and must not be summed as if simultaneous.

This instrumented live workload differs from the historical isolated1000kbps fixture: IMDCT is now the largest measured channel stage. The next measurement should split the frame output/safety scan/interleave and wrapper remainder before changing their math. Linked-object inspection places lhdc_dec_decode_frame (0x8ed bytes) in flash text; its channel/FFT kernels use IRAM. Software-float division and float-to-double diagnostic helpers are referenced, but there are no undefined soft-float multiply/comparison/conversion helpers in this decoder unit. This is compiled dependency evidence, not an instruction-latency claim.

The run has zero decoder rejects/nonfinite/concealed frames; finite sample clipping and refill total7680 (including startup/drain) remain. Playback is paused and90/150 volume restored. Stage instrumentation is not a throughput optimization and has no production-readiness claim. Raw records, analysis, build and readback receipts are retained in the experiment.
