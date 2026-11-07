# Steady full-range output candidate — 2026-11-07

Experimental development candidate. The full Bluetooth/audio goal remains active and incomplete. Three scoped LHDC measurements show lower DSP cost, but the LDAC/volume regression was aborted before playback because UART became silent during the codec-selection period. An independent read-only capture and one controlled ROM connection attempt received no bytes. Physical COM6 board/USB power-cycling is pending; no claim of current runtime health or completed LDAC regression is made. Cause remains unproven.

## Exact finalizer

A small bounded O3 helper handles only steady full-range segments with idle volume/bypass/flip ramps. It caches the unchanged gain, performs the same signed Q31 multiplication/clamp, and preserves limiter/counter updates. Active ramps and crossover routing retain their existing path. No codec, buffer, task, priority or core-affinity change is made.

The final Q27-to-S32 multiplication can use int32 arithmetic exactly: below the limiter knee, absolute output is already less than AUDIO_ONE; above it, range*excess/(range+excess) is strictly less than range. Thus the limiter result always has absolute value below2^27, including INT32_MIN input, and multiplication by16 fits int32 without changing the prior wide-clamp result. No negative signed left shift or reduced precision is introduced.

The independent scalar path stays unchanged. Existing ASan/UBSan full PCM/state tests pass 4403129 frames in each of two arithmetic modes (8806258 total),13 rates, all routing flags, rapid controls, numeric limits and analyzer boundaries. A new independent wide-division oracle checks1848830 channel samples across16384 blocks: signed arbitrary gains, explicit limiter/gain boundaries, floor correction, counter wraps and complete state. The old signed64 quantizer oracle and volume-comparison regression also pass. Existing two-core lifecycle/loan/sound tests pass with the six-block cap. No new boot test cases are added.

## Bound image and memory

Image `b309817ce3d69f8d7140f0d7cf79684a8ddc5734064f6516ddf3d74c73f39df1`, archive `runs/1794048391333768300-b309817ce3d6`, uses the full ccache native product recipe with all codecs/product/security/RFCOMM, helper6, TaskCycleAccounting OFF. Source/image admission, full app/selector readbacks (write921600/read115200), owned VALID boot and the existing37 cold codec cases passed before the later UART outage. Original app/table/bootloader are preserved; no Bluedroid restoration occurs. The last verified flash content remains this owned image; execution after the failed ROM attempt is unobserved.

ELF IRAM text grows288 bytes relative to idle-guard image950428ba68dd. DRAM data/BSS, flash text and rodata sizes are unchanged. Startup logs report unused IRAM0 region584 bytes (the section arithmetic leaves585 before final alignment), versus872 before. Allocated task-stack sizes and39936-byte PCM/DMA payload are unchanged. Observed Core1 render stack low-water changes3704 ->3624; decoder low-water remains5952. These are sampled high-water records, not all-thread worst-case guarantees.

8-bit internal heap total stays217248 bytes. Before-controller free heap changes162576 ->162252, a324-byte decrease that persists through controller/service phases. Active LHDC free/minimum is19256–19264/19256, largest16384, codec resident60704; the prior image has19580–19588/min19580 and largest17408. No allocator call or audio buffer is added by the helper. Allocation placement is a hypothesis, not an identified allocation or isolated fragmentation/leak finding. Cold checks report tracked peak56899, minimum free after allocation103284 and stack low-water3600. The scarce instruction space and actual heap reduction remain material costs.

## Guarded LHDC observations

Redmagic9125299103B7, native YouTube120 BPM metronome, UI LHDC V5 192k/24-bit/stereo/fixed900. Actual phone volume80/150; sampled DSP68/EQ2,8,12/flags12. The maximum media payload remains566 bytes with no oversize rejects. Each command interval has an independent14-second pause guard and post-pause drain. Counters below count stereo PCM sample frames; one5-ms LHDC coded frame contains960. Separate segments/boots are not paired identical-input timing.

| Mode | Command s | Accepted/rendered/DMA frames | Unfilled | Helper elapsed cycles/frame | Local elapsed DSP cycles/frame |
|---|---:|---:|---:|---:|---:|
| Idle-guard baseline1 | 12.219 | 2801792 | 47744 | 779.34 | 1275.56 |
| Idle-guard baseline2 | 12.229 | 2886720 | 30592 | 778.28 | 1231.65 |
| Idle-guard baseline3 | 12.178 | 2866816 | 34048 | 778.97 | 1239.88 |
| Full-output candidate1 | 12.241 | 2871360 | 32832 | 717.08 | 1139.69 |
| Full-output candidate2 | 12.197 | 2896320 | 17280 | 716.64 | 1112.20 |
| Full-output candidate3 | 12.141 | 2880512 | 22528 | 717.17 | 1132.83 |

Helper elapsed cost is716.6–717.2 versus778.3–779.3 cycles per PCM sample frame, about7.9% lower. Elapsed counters include interruptions/preemption; this does not prove pure arithmetic time, sustained headroom or universal speedup. Local elapsed cost is lower in all three observations. Unfilled totals32832/17280/22528 remain failures; initial shortages53568/32896/44096 and recovered20736/15616/21568 are separate.

Runs2/3 have maximum eligible RTP step960 and zero aggregate RTP-span excess, yet still miss DMA deadlines. Their first shortages contain2048/1152 decoded-input frames and empty processed output; run1 also has2048 raw frames. Run1 retains one extra140-ms RTP interval. The remaining consumer shortages cannot all be explained by that timestamp-jump observation. Source/RF scheduling is not otherwise exonerated.

No descriptor/contract/helper fault or new queue/decode/sequence rejection is recorded in the three completed traces. PCM-busy counts7966/8819/8726 are retry/backpressure observations, not packet losses. Detailed worker-ring losses3397/3714/3716 are retained separately from zero host-detail, aggregate-record and UART losses during those traces. Full per-call coverage and acoustic continuity are not claimed.

## Incomplete regression and UART recovery

After these runs, Android codec controls were set to LDAC96k/32-bit/stereo/optimized quality. The playback harness rejected stale UART before sending play. Its concurrent control task confirmed YouTube was paused and sent no volume keys; actual volume remained80. The failed attempt, final raw records, media state and source identity are preserved. Android still displayed media-active Bluetooth, which is not proof of working A2DP.

The last live UART record is a complete decoder_input_queue line at436521303us. There is no captured panic explanation. Reader close/reopen with DTR/RTS deasserted did not recover data; an independent3.047-second UART capture received0 bytes. One esptool ROM read-MAC attempt on the USB-identity-verified COM6 also received no serial data. No flash/erase was attempted during recovery. Windows reports the matching CP210x node present; this unelevated session cannot restart its driver. A physical COM6 replug was requested; COM7 is untouched. After the ROM attempt, Android shows BDK disconnected and current system media volume0/150; the earlier connected readback80/150 is separate. No volume commands were sent during the failed attempt or recovery. The phone is paused. The monitor is open/listening, but live reception is absent.

Old Trial capture records briefly appear when the monitor reader lease is released; they are marked with a separate source. They are not current runtime evidence and are not used in these measurements. The aborted LDAC attempt is not counted as a successful test. Firmware/USB/UART cause and codec-switch regression remain unresolved.

The local playback harness now checks live UART identity/source/freshness before opening YouTube, arms its independent pause guard before media navigation, and pauses in setup/error cleanup. A negative precheck run against the silent UART verifies zero ADB commands or child-process launches. This is a harness check, not a completed hardware regression.

## Off-board compiler alternatives

Isolated compiler objects use the exact selected Xtensa flags except helper optimization policy; they are not linked, flashed or timed on hardware. Current O3 helper295 bytes / total object IRAM sections5146. O3 without loop unswitching and O2 each use helper188 / total5035; Os uses helper71 / total4890. This identifies a possible256-byte object-IRAM reduction, not a proven linked-memory or speed result. Worktree/last-flashed source remains the measured O3 candidate.

Exact source/test/build/readback/traces, failed recovery and compiler receipts are under `native_bt/_build/experiments/finish-block-1794048105456`; [structured results](DSP_FULL_OUTPUT_RESULTS_2026_11_07.json) preserve scope and image binding. LDAC/control regression, UART/codec-switch recovery, sustained loss-free deadlines, live1000 kbps LHDC, encrypted coexistence/storage/OTA/acoustic/lifecycle/stress and full feature parity stay open.
