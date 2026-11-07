# Skip idle DSP ramp calls — 2026-11-07

The full Bluetooth/audio goal remains active and incomplete. The retained block DSP computes once per stable segment whether volume, bypass or flip has an active ramp. If none is active, three per-sample calls that immediately return are skipped. If any is active, canonical per-sample advancement remains, including a ramp completing inside a segment. A later bypass segment checks again. Controls are applied between block calls; this adds no shared/persistent state, allocation, queue, wait or task. PCM arithmetic, coefficient updates, precision, buffer geometry, six-block cap, priorities and core affinity are unchanged.

## Correctness and compiled result

The existing independent scalar path is unchanged. ASan/UBSan with signed-overflow wrapping disabled compares every PCM sample and the entire DSP state over 75941 blocks / 4403129 frames in each of two arithmetic modes (8806258 comparisons total), across 13 rates, all routing flags, rapid controls, numeric limits and decimation/Goertzel boundaries. The existing volume-comparison diagnostic regression also passes. The independent signed64 quantizer oracle passes 2001042 cases. Actual two-core output/lifecycle tests with the selected six-block cap pass held BUSY ownership, publication, stop/cancel, ring reuse and sound signaling. No new boot cases are added.

The full ccache product image is `950428ba68dd7846609cecf42a4bbfea712c246527e664f7190ef3a6d8b756fb`, archive `runs/1794046921031039500-950428ba68dd`. Exact source/image admission, full app/selector readbacks (write 921600/read 115200), owned VALID boot and the existing 37 cold codec cases pass. Original app/table/bootloader are preserved; no Bluedroid fallback occurs. All codec/product/security/RFCOMM flags remain selected. Temporary task-cycle accounting stays OFF.

Relative to six-block image `f2637213bb695ca5123f591a6575d35ac75be0c7065a1c456cac127d92003dc8`, IRAM text grows 8 bytes; vectors, DRAM data/BSS, flash text and rodata sizes are unchanged. Disassembly confirms the idle branch skips all three calls. The compiled function frame changes from 0x460 to 0x450 (1120 to 1104 bytes); allocated task-stack sizes are unchanged. This is not an overall stack or RAM saving.

Active LHDC records retain 60704 codec-resident bytes and decoder stack low-water 5952. Prior-image free/minimum/largest internal heap is 19588/19588/18432 bytes. Candidate free is 19580–19588, minimum19580 and largest17408. Codec-switch histories differ; these observations do not establish a leak or isolated fragmentation change. Cold-check peak56899, minimum free after allocation103608 and stack low-water3456 are separate from active RF records.

## Guarded native YouTube measurements

All sample counts below are stereo PCM sample frames; one5-ms LHDC coded frame contains960 of them.

Redmagic 9125299103B7, native YouTube 120 BPM metronome, UI LHDC V5 192 kHz/24-bit/stereo/fixed900, actual phone volume80/150, sampled DSP68/EQ2,8,12/flags12. Media payload maximum566 bytes and PCM geometry are unchanged. Each command interval has an independent14-second pause guard. Final stream counters include startup/drain. Baseline runs are the preceding exact-image six-block measurements; separate segments/boots are not paired identical-input cycle measurements.

| Mode | Command s | Accepted/rendered/DMA frames | Unfilled | Helper elapsed cycles/frame | Local elapsed DSP cycles/frame |
|---|---:|---:|---:|---:|---:|
| Six-block baseline 1 | 12.086 | 2779200 | 107648 | 833.62 | 1354.82 |
| Six-block baseline 2 | 12.251 | 2825280 | 74368 | 834.41 | 1320.72 |
| Six-block baseline 3 | 12.211 | 2844288 | 72320 | 833.57 | 1336.97 |
| Idle guard 1 | 12.219 | 2801792 | 47744 | 779.34 | 1275.56 |
| Idle guard 2 | 12.229 | 2886720 | 30592 | 778.28 | 1231.65 |
| Idle guard 3 | 12.178 | 2866816 | 34048 | 778.97 | 1239.88 |

Mean helper elapsed cost changes from833.87 to778.86 cycles/frame (about6.6% lower); all three candidate observations are lower. Local elapsed cost is also lower in each comparison range. Counts include interruption/preemption and do not prove pure arithmetic CPU time or a universal throughput gain. Unfilled totals improve but remain nonzero; this is a scoped optimization, not a sustained real-time or acoustic pass.

No new queue/decode/sequence rejection or descriptor/contract error is recorded. Initial shortages82944/55872/63936, late recoveries35200/25280/29888 and unfilled47744/30592/34048 remain separate. PCM-busy observations6409/7377/6795 are backpressure/retry counts, not packet drops. Worker-detail losses2778/3201/2925 are retained separately from zero host-detail, aggregate-record and UART losses; complete per-call coverage is not claimed.

Every run still has a pre-decoder maximum RTP step27840 rather than nominal960 ticks, with aggregate clock excess26880 ticks (one extra140-ms interval). In run2, the CRC-verified refill snapshot at162672641us already has25472 unfilled frames, before the sparse clock-jump event at163138723us. The prior clock snapshot at162048923us still has maximum960. The first shortage also has1984 decoded-input frames and empty processed output. The observed input discontinuity does not explain or exonerate those preceding consumer shortages; sender/RF/credit cause remains unproven. No reset/concealment/radio-policy workaround is added.

## LDAC and live volume regression

LDAC96 kHz/stereo with32-bit phone input and optimized-quality selection:12.234-second native YouTube command interval,1443584 accepted/rendered/DMA frames and zero initial shortages, unfilled frames or descriptor/contract errors. Eight actual volume-down keys then eight up produce independently read-back80 ->0 ->80; CRC-verified DSP snapshots include both68 and0. This exercises active ramps as well as the idle branch. It does not establish audible pop elimination or exact volume-response latency. The helper is idle for LDAC; its RTP maximum resets to512 ticks. Worker-detail loss5535 is separate from zero recorded queue/decode/sequence rejection.

The phone is paused on the metronome, volume80/150 restored, Bluetooth PID8879 stable and raw COM6 UART monitoring listening. Exact source/test/build/readback/traces and analysis are under `native_bt/_build/experiments/idle-ramp-1794046756659`; [structured results](DSP_IDLE_RAMP_RESULTS_2026_11_07.json) bind the image and measurements. Retain the full six-block build recipe in [the prior scheduling record](ASSIST_SIX_BLOCKS_2026_11_07.md). Sustained loss-free deadlines, live1000 kbps LHDC, encrypted coexistence/control/storage/OTA stress, all-codec acoustics and complete feature parity remain open.
