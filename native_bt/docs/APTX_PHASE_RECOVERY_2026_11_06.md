# Bounded aptX parity-phase recovery — 2026-11-06

The user's aptX HD report remains: initially smooth, then static/stutter after 2–3 seconds. A prior YouTube trace had approximately 575 kbit/s payload with clean queue/rejection/DMA counters, so the audible onset is not yet explained by those counters alone.

## Confirmed integration defect

The generic worker resets a decoder after a missing RTP sequence or a rejected packet. The old aptX reset cleared freeaptx's parity phase to zero while a continuing encoder retained its phase. Retrying ordinary decode is not resynchronization. Separately, a partial parity failure can already have advanced predictors before returning its unconsumed codeword; that replay was rejected in the previous change.

A controlled **reset-only variant of the current adapter**, not a historical full-firmware or Bluedroid measurement, reproduces the loss problem. The identical input stream with one packet missing yields zero subsequent PCM and five decode drops for Classic aptX; HD yields 23,600 bytes versus the synchronization reference's 24,112 bytes, with a mismatch. The recovery-enabled adapter matches the reference in the focused cases below.

## Runtime ownership and bounded work

Clean-stream S32 decoding stays on the existing path. A reset of a decoder that has accepted audio arms resynchronization. Suspend/start preserves that need; a newly opened codec starts normally. Recovery calls the existing library sync API with at most **one new codeword** (4 bytes Classic/LL, 6 bytes HD), alongside its bounded internal cache. The existing 1,024-byte scratch is reused; up to eight packed S24 samples are expanded backwards into S32 Q31 without precision loss or another buffer.

This deliberately avoids passing a full A2DP packet to the local sync API: its input-size expression has an operator-precedence problem, and its short-return handling can treat an output-capacity exit as a parity error. With one codeword and a 24-byte packed-output allowance, the output capacity cannot cause that false error. The cached-input and main loops are bounded by the one-word input and at most five cached bytes; no owner, controller credit or external event is polled.

After validated synchronization and retirement of the library's byte cache, the adapter returns to normal S32 decoding. Recovery state uses existing struct padding: target `aptx_state_t` remains 24 bytes and the freeaptx context remains 4,404 bytes. Six fixed atomic diagnostic counters add 24 bytes of BSS, with no new heap allocation, packet queue, PCM pool or task. Counters have one writer and use load/store publication without CAS retry loops.

`DBT_PROGRESS` is an explicit internal codec result for bounded input/internal progress that produces no PCM. Both worker modes retain the input slot and resume it on a later work quantum. It does not acknowledge a wire transaction and does not own a PCM loan. `DBT_PENDING` without an accepted PCM view remains invalid. Control jobs can retire a partly processed packet; codec/queue teardown does not wait for resynchronization.

The separate `aptx_resynchronization` UART row reports probe calls, consumed bytes, bytes discarded by the sync algorithm, completed recovery, parity rejects and active state. These are decoder-lifetime counters, **not RF-loss counts**. Sync discarded bytes include reacquisition/warm-up and must not be relabeled as missing radio data. The existing input RTP discontinuity counters remain separate. DSP clipping, PCM queue and DMA rows are unchanged.

## Verification scope

The real freeaptx/owned-worker focused check passes:

- Six existing clean PCM comparisons: Classic, HD and LL at 44.1/48 kHz.
- Twelve continuing-source recovery comparisons: Classic/HD × legacy/deferred worker × missing packet/parity damage/pause-resume. S32 PCM matches standalone public synchronization calls driven one codeword at a time.
- Partial parity error is reported rather than replayed.
- A single recovery work quantum retains the input, advances only one probe, and CLOSE reclaims all codec/queue memory.

This is production C with the real codec and fake PCM I/O, not RF, DAC or acoustic proof. The reset-only negative variants fail as expected. No additional startup test cases were introduced. The ESP32 image builds with ccache and stable sources. Relative to the preceding diagnostic image, target sections change by BSS +24 bytes, flash text +1,860 bytes and rodata +272 bytes; IRAM/data sizes are unchanged. Runtime probe duration and audible recovery still need target measurements.

Candidate SHA-256: `dc129c0b0ed441942eb09c5a9196456c148c862220d5b65e14df866619bf36ce`. Deployment and bounded native YouTube pause/resume results are recorded separately when observed. Full protocol-layer review, all-feature qualification, maximum LHDC V5 headroom and the reported audible HD fault remain open.
