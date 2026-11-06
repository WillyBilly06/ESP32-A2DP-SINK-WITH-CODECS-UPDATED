# First-starvation stage capture — 2026-11-06

The prior short AAC runs produce PCM without decoder rejects/queue drops but still underrun, and aptX HD has a residual2-frame underrun. The ordinary16-entry worker trace ring loses events; its missing rows cannot establish radio loss or explain a precise first gap.

The passive -SchedulerTrace mode now freezes the first non-stopping DMA refill shortage of each output stream. It retains separate first_starvation, starvation_media, starvation_decoder and starvation_renderer UART records, joined by stream_serial and event_us. event_us is the refill-shortage time, not the later analog playback time. Logger transmission is outside the ISR; a capture persists across UART backpressure and audio stop until the next output start.

The capture includes taken/input/output frames, accepted/rendered/DMA-source counts; most recent any-link ACL callback and host poll; media callback begin/admission result/epoch/sequence/timestamp/gaps; decoder operation/wait phase and queue depth sampled at that phase change; and whether rendering was active. An in-progress media admission publishes DBT_PENDING and admission time0. Different producer stamps are adjacent observations, not a simultaneous global lock.

Each producer has its own atomic version and fields. Readers make one attempt and report coherent=false when they overlap publication. The first capture cells are atomic too, so a new stream/reset cannot cause a C data race with the logger. No retry loop, task, packet queue, allocation, callback logging or wait is added. The callback uses only IRAM code and internal DRAM cells. Stage times wrap modulo2^32 microseconds; analyze only bounded nearby intervals. Any-link ACL timing alone does not identify an audio packet or prove RF loss. The decoder queue value belongs to its phase-change instant and can become stale before the ISR snapshot.

Exact target compilation with this mode enabled and disabled passes. There is no undefined atomic helper reference. New timeline data is196 bytes; sent retains its64-byte own stack frame (the capture is inlined). The logger own frame is576 bytes; full call-chain/high-water and callback duration still require hardware observation. Full ccache build passes with10349 unchanged source inputs and no added boot test. Hardware readbacks and focused traces now exist, as below.

## Targeted hardware evidence

All runs used the verified COM6/Redmagic pair; ordinary worker trace loss is not treated as packet loss. Audio-stage records remain separate.

- Image `3121bc09045d3ce409f7507c93688895ae10f2f192d34a5b6ce2b3995431b2a6`, AAC/Core0: first short run0 underruns, resume384. At event_us182707948, decoded input512/output0, decoder waiting for PCM space, renderer active for222us; latest media gap27353us, maximum35002us. This proves ready decoded PCM missed an immediate refill; it does not prove RTP/radio loss.
- Image `fb1568e58d38cf32137a2bcc8fa750d7b55f6899b865643b48c84811cbb9783f`, AAC/Core1 trial:512 underruns, mean render approximately638us versus Core0 approximately434us. First gap had empty input/output with decoder executing590us after recent media admission. Core1 was not an improvement; the later SMP diagnostic build restores AAC/Core0.
- Image `e726205a92d8f5f68f5c43c82613fd42227f53d39f2d04bf13dcbcf82bda4b9f`, LDAC96k: finite source completed without Android route/focus loss, native total1024 underrun frames. First capture event_us1655429951 had decoded input512/output0, decoder executing212us, recent media gap24855us. The first capture predates the recorded BLE connection. No decoder rejection, encoded queue drop, RTP sequence break, DMA descriptor fault or watchdog was recorded. This does not establish all subsequent gaps occurred before BLE.

Private ignored evidence: `native_bt/_build/experiments/aac-first-starvation-*`, `aac-core1-render-first-gap-1793967006270`, and `ble-a2dp-regression-1793967920295/bdk-real-audio-coex-1793972991903`. UART source158 was identity-verified. Logger low-water observed1408 bytes; this does not replace a worst-case stack proof.

An asynchronous AAC priming candidate is parked unbuilt/unflashed under the ignored `aac-priming-unbuilt` experiment. It is not part of the accepted implementation and has no performance claim.

The subsequent verified image516147fc, after successful durable LE re-pairing and stored-key reconnect, retained the exact A2DP route while the app acknowledged EQ changes. Second flushed LDAC96k stream had128 native underrun frames despite0 transport/decode faults. First event_us462426543: input512/output0, decoder active167us, recent/max media gap24983/25174us, renderer outside render. This remains an output-readiness/lead-time investigation; variable short-run counts do not prove a performance gain.
