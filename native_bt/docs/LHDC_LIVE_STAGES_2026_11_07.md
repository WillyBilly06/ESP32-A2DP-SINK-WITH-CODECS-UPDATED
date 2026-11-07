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
