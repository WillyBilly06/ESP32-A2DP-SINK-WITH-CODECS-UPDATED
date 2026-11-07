# Bounded FAC zero-symbol runs — 2026-11-07

The earlier per-symbol write-cancellation shortcut had no demonstrated target speed benefit and cost224 instruction-RAM bytes. It is removed. This candidate instead batches pass-1 zero symbols while a full adaptive model is provably unchanged: outgoing history zero, freq0>=2 and below its rescale threshold, and the original range comparison selects symbol0. Range/code/input cursor/history cursor remain local during the run; state is committed at its bounded exit. Every boundary uses the original single-symbol decoder.

The count remains the loop bound. A positive accepted range needs at most3 renormalization byte steps. Input padding and cursor advancement are identical. No table, probability, bitrate, gain, format, buffer, task, allocation or affinity changes. Active host tracing uses the original per-symbol loop, preserving exact debug output/counter behavior. Production test counters are absent.

Host ASan/UBSan compares605475 call boundaries and2201355 symbols, including complete range/model/history/output/rescale state,13 windows, truncated/empty input, random/deterministic streams and cursor wrap.1582860 symbols use batching and41198 zero-padding reads exercise that path; these are artificial test counts, not music occurrence rates. Trace output matches exactly.48 independent full-PCM encoder cases and80 sanitized real-codec deferred cases pass, with no new boot cases.

The complete ccache build/source/admission gates pass. An initial final disk-quota failure was retained; only rebuildable host PDBs were removed with hashes and the rerun passes. Candidate59b2b6e2967f25b8006b3d9d24d3645048553d6db5ea7e78a69b7ddec463b05d is1338768 bytes. At document creation it is not flashed and no speed improvement is claimed.

Current931 baseline at actual192k/S24/fixed900 records2421.04us per decoded unit (2823 units), including codec wrapper/preemption. Prior931 observation was2431.98us/unit. Refill totals include startup/drain and remain unqualified. Each native YouTube command interval is bounded, independently paused after14s, and phone volume90/150 is restored after max-volume testing.

Experiment: native_bt/_build/experiments/lhdc-fac-run-1794061546925.

    python native_bt/tools/test_lhdc_fac_run.py --decoder "D:/ESP32 Bluetooth/LHDC-V5-Decoder-main/decoder"

The guarded playback harness now saves partial rows incrementally and failure receipts even if the final logger HTTP request fails. It does not treat lost logging as a successful trace.
