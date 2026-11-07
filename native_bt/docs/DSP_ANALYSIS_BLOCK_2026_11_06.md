# Exact block analysis accumulation — 2026-11-06

The complete Bluetooth/audio goal remains active and incomplete. This is a measured DSP candidate, not a maximum-LHDC qualification. It changes no codec, feature, sample precision, EQ filter, analysis cadence, packet or PCM capacity, affinity, priority, task, queue or DSP state layout.

## Preserved contract

The block renderer previously calls analysis for every stereo frame before shifting that frame from Q31 into the DSP's Q27 working scale. At 192 kHz, decimation is 17: most calls only add the signed mono average and increment a counter. The meter/Goertzel step occurs at the original decimation boundary.

The helper accumulates the intervening mono samples and performs the same Q27 scaling in a bounded pass. For a valid partial window, count k is less than nonzero decimation d. A run n is min(frames,d-k). An incomplete run updates the same sum/count with n samples. A complete run accumulates the first n-1 samples and passes the last unscaled mono sample to the original analyzer, which owns averaging, count/sum reset, meter/Goertzel updates and the window rollover. The count and sum at every analyzer invocation are therefore identical to the per-frame path.

Signed mono averaging still uses the original int64 sum/division by two, including truncation toward zero for odd negative sums. No reciprocal, reduced precision, changed analysis rate or skipped analyzer output is introduced. Analysis-disabled blocks retain identical PCM scaling and no analysis-state changes. The scalar coefficient-ramp prefix is unchanged; arbitrary partial-block and ramp-to-steady boundaries retain their existing state. The owner-only contract prohibits controls or DSP observers from modifying these fields mid-call. Only already-unpublished PCM is processed.

A valid run consumes at least one of at most 128 frames. The bounded loop does not wait for input, a task or hardware, and has no allocation or lock. An out-of-contract partial-window state retains the old scalar loop rather than claiming a new repair of invalid DSP state.

## Correctness and static cost

The actual block/scalar comparison passes ASan/UBSan for 75941 blocks and 4403129 frames in each of two arithmetic modes (proven sum and conservative split fallback): 8806258 frame comparisons. Every PCM sample and complete DSP state match across all 13 supported rates, all routing/analysis flags, volume-only ramps and staggered rapid controls. The 8901 added boundary blocks per mode cover every valid decimation offset, lengths 0..128, signed odd/rail samples and a Goertzel window rollover. The word-quantization oracle also passes 2001042 cases.

Actual two-core output/borrow/stop/cancel/early-release tests and the 192 unity/general stream-pair comparison pass, including 787776 further exact pipeline frames. Host results do not establish ESP32 timing or acoustics. No boot test is added.

The full ccache product build passes exact source/image admission. Candidate b31d1013ab8bf8f961f88adbe292ae02afecc488dc06fc77a4d8096ed972fb7e adds 412 flash-text bytes relative to image 23b03035b61a, with unchanged IRAM, DRAM data/BSS and rodata. No persistent memory or stack array is added.

## Fresh physical baseline

Before flashing the candidate, the unchanged 23b03035b61a image records the Redmagic native YouTube 120 BPM - Metronome at LHDC 192 kHz/24-bit/stereo/fixed900. Actual phone volume is 80/150. A 12.209-second command interval has an independent 14-second pause guard and post-pause collection.

Accepted/rendered/DMA source counts all equal 2532480. Initial shortages 532352, late recoveries 152832 and unfilled frames 379520 are separate counters; descriptor errors are zero. Helper deltas are 1489792 frames and 1304944459 elapsed core cycles (875.924 cycles/frame); the uint32 subtraction handles the cumulative counter wrap. Local DSP has 1042688 frames and 1536527636 elapsed cycles (1473.622 cycles/frame), excluding helper frames. These elapsed cycles include preemption; they are not exclusive DSP CPU counts.

Exact images, UI/actions, UART events, snapshots, source-state tests, section deltas and failure receipts are retained under native_bt/_build/experiments/analysis-block-1794025321920 and the protected hardware run archives. The scoped hardware comparison follows; continuity and full master-goal gates remain open.

## COM6 result

The guarded b31d1013ab8b deployment passes full application/selector readbacks at 921600/115200, owned VALID boot, original-app preservation and all 37 existing cold codec configurations. No automatic Bluedroid restoration occurs. Archive: runs/1794025545910106100-b31d1013ab8b. The Redmagic reconnects on the first explicit attempt. Actual developer UI restores 192 kHz/24-bit/stereo/fixed900 and volume 80/150. Wire samples remain 566 media bytes with one 5 ms frame, or 899200 compressed bits/s after transport headers. Live 1000 kbps remains unqualified.

| Native YouTube metronome | Baseline | Candidate | Candidate repeat |
|---|---:|---:|---:|
| Play/pause command interval (s) |12.209|12.253|12.208|
| Helper PCM frames |1489792|1548928|1542784|
| Helper elapsed cycles/frame |875.924|847.593|847.392|
| Local DSP PCM frames |1042688|1011392|1010816|
| Local elapsed DSP cycles/frame |1473.622|1493.840|1488.527|
| Source accepted/rendered/DMA frames |2532480|2560320|2553600|
| Initial shortage frames |532352|514176|529024|
| Late recovered frames |152832|154112|169472|
| Unfilled frames |379520|360064|359552|
| Descriptor errors |0|0|0|

Each playback has an independent 14-second pause guard and post-pause collection. Separate intervals/boots, preemption, source position, startup and drain prevent interpreting these totals as an isolated overall speedup. Core 0's repeated elapsed DSP cost is about 3.2% lower; Core 1's elapsed per-local-frame figure does not improve. These measurements do not establish exclusive CPU cost, sustained maximum-rate headroom or a continuity pass.

The candidate's first gap has 1856 decoded frames waiting and zero processed output; the repeat has 1216 waiting and zero output. Accepted PCM eventually reaches DMA in both captures. All observed queue/decode rejects, PCM failures/timeouts, input sequence breaks, ingress rejects, watchdogs, descriptor errors, formatted-record losses and UART-write failures remain zero. First-run PCM backpressure/detail-ring loss are 8354/3662. Repeat deltas are 8288/3673, distinct from its since-boot totals. The captures have 287 CRC-verified snapshots each, plus 85/76 raw unverified trace/state records. Detailed ring losses preclude a complete per-call trace.

Active internal heap free and largest block are unchanged at 19604/18432 bytes. Candidate lifetime minimum is 19604; the older baseline lifetime minimum is 17808 and includes additional earlier workloads, so this is not a demonstrated heap saving. Both decoder stack low-water readings are 5960. Main renderer low-water is 3760 in the candidate versus an older 3616 baseline observation; auxiliary stack high-water is not independently measured. Static sections and requested PCM payload remain unchanged. Sampled DSP stays at volume 68, EQ 2/8/12, flags 12 in both candidate runs.

The helper processes 60.50% and 60.42% of source frames in the two candidate runs. This is a measured work split, not a CPU-utilization ratio. Current helper leases may process up to four blocks while the decoder is already waiting for PCM space. The next scheduling investigation should bound that lease cost against decoder progress and Core 1 local service, with the existing borrow/lifecycle fences intact. Do not repeat the previously rejected forced-half dispatch, which submitted regardless of the decoder's wait hint and queued behind active decode. A shorter lease under the retained wait condition is a distinct, unimplemented candidate; no improvement is claimed for it.

The phone is left paused on 120 BPM - Metronome in native YouTube. Broad codec/security/app/OTA/stress and acoustic qualification remain open. Lower-rate timing is not measured here; full scalar/block correctness covers every supported rate.
