# Exact zero-tail FIR work and output-owner review — 2026-11-05

## Source and generated-data proof

Every coefficient row in each 96/48/24-tap bank ends with two exact zeros: 129 rows per bank, including the phase-wrap row. The generator now derives the common trailing-zero count from every row and emits DBT_RESAMPLER_TRAILING_ZERO_TAPS. Regeneration changed only this metadata line; all 21,672 Q30 coefficient words (86,688 flash bytes), delta/accumulation bounds and symmetry metadata remain identical.

For either omitted index, both adjacent endpoint coefficients are zero. Therefore their difference, interpolation at every Q32 phase remainder, signed product with every input and accumulated contribution are zero. Removing those additions leaves the original sum and saturation unchanged, including extreme input values and priming history. Retained operations stay in the same order.

The kernel uses 94/46/22 accumulation iterations for those generated banks. The declared 96/48/24-tap span, causal delay, 128-frame history, count/priming boundary, phase advancement, interpolation precision, all nonzero coefficients and final rounding stay unchanged. Unknown metadata, unsupported tap counts and the forced-full-tap compile option retain the original loop. No coefficient bank or signal quality is reduced. Folded-zero and paired-product experiments remain disabled.

## Output-owner comparison

The complete probe_output.c body and header were read: allocation, task initialization, ticket admission/completion/cancellation, PCM loans/copied input, sound transitions, stop/drain/fade, DMA descriptor ordering, diagnostics and restart fencing. This source reading is not lifecycle or concurrency qualification.

The pinned BTstack ESP32 v5 audio reference, port/esp32/components/btstack/btstack_audio_esp32_v5.c:41–386, uses 16-bit samples, six DMA descriptors and a 10 ms nominal DMA interval. Its ISR publishes byte counts and wakes run-loop polling; the run-loop supplies data through zero-timeout I2S writes. That file explicitly excludes CONFIG_I2S_ISR_IRAM_SAFE. These are materially different format, buffering and cache-safety assumptions from this product's retained 32-bit/IRAM-safe direct refill path. No reference implementation is integrated or used to lower precision.

Owned source contains blocking compatibility wrappers, but their callers are excluded by DBT_PROBE_ASYNC_AUDIO. Both before/final linked symbol inventories contain the selected try-PCM and begin/poll APIs and omit those compatibility entry points and request_output. They are not an established steady-audio stall in this configured image; alternate configurations remain separate.

The review found a real startup cleanup defect: successful queue allocation followed by renderer task-creation failure returned NO_SPACE without deleting the queue. A retry could overwrite its handle. Failure now deletes that new queue and clears the handles. Duplicate initialization returns BUSY before modifying an existing instance. The startup-owner-only contract is explicit. A stale core-0 comment was corrected to the actual invariant: channel operations and their ISR belong to the pinned render core. Current affinity remains CPU1; no scheduling policy changes.

## Compiler and resource evidence

Strict C11 Clang analysis passes with guarded and forced-full-tap loops. Full ESP-IDF 6.1 ccache compilation passes with stable source fingerprints. Xtensa output selects literal loop limits 94/46/22. FIR function text changes 946→997 bytes and its own compiler frame remains 64 bytes. Final whole-image text changes 866843→866919 bytes, entirely in flash; IRAM text remains 122255, DRAM data 23893, aggregate data 396577 and BSS 50300. No task, stack allocation, buffer, queue capacity, codec setting, MTU or affinity changes.

This establishes fewer zero-contribution operations, not a measured elapsed-time or underrun improvement. The startup failure path has not been fault-injected. Flash/readback and ordinary startup evidence are recorded separately; automatic codec checks do not exercise FIR phase/history or output lifecycle under load. No unit, playback, pairing, app, timing or packet-injection campaign is invoked under the user's hold.
