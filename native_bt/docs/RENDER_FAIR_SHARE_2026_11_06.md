# Core 1 renderer fair-sharing experiment — 2026-11-06

This development experiment addresses measured renderer starvation during high-rate LHDC. It does not establish all-codec, acoustic, coexistence or maximum-bitrate qualification.

The preceding owned image (`24f9ec682af8`) ran native YouTube with LHDC V5 at 192 kHz, stereo, 24-bit, phone fixed-quality 900 kbps. The commanded play interval was 12.073 seconds with an independent 14-second pause guard and post-pause capture. The specific user metronome video was not identified. DSP controls remained volume 93, EQ 2/8/12, flags 12.

Separate baseline counters recorded:

- Encoded queue drops, decode rejects and input sequence breaks: zero.
- Accepted, rendered and DMA source frames: 2,372,160 each.
- Initial DMA shortage: 562,048 frames; late recovery: 55,936; unfilled: 506,112; descriptor errors: zero. These are full stream/capture totals, including the source pause tail.
- Core 1 local render delay: 12,334 us for 128 frames, with 9 host and 3 controller interrupted-task samples and no renderer sample. Sampling is a timing clue, not an exact CPU attribution.
- Published DSP-job retirement delay: 5,649 us for a 512-frame job; Core 0 helper completed without failures.

`-RenderFairShare` explicitly raises only the primary Core 1 renderer from priority 20 to 22, equal to the host. Controller priority 23 remains highest; decoder priority 21/Core 0 and helper priority 20/Core 0 are unchanged. Both runnable host and renderer yield after bounded batches. Empty/pending work still sleeps on the existing notifications. No timer, new task, extra audio buffer, reduced codec setting or altered DSP arithmetic is introduced. The option defaults off and excludes the rejected host-pacer experiment.

The actual two-core mocked output lifecycle passed AddressSanitizer/UndefinedBehaviorSanitizer with the option off and on: held BUSY borrowed buffers, publication-before-DONE, stop/cancel retirement, no early free, no timed pending polling. The fair mode test checks main/helper priorities 22/20 and Core 1 bounded yields. Four compiled header cases, seven actual CMake cases and sixteen actual image-admission cases passed. The existing finite priority-mode tests passed 10 CMake, 8 compiled-header and 16 image-admission cases. These are host tests, not FreeRTOS latency measurements.

Candidate `7a1c4cbfc4411d28b87d6b1ba7e83c711d327949c1b4dfd47c421c42670c6122` built with ccache and all codecs/product security/RFCOMM enabled. It passed exact source/image validation. Relative to the selected baseline, DRAM data/BSS and IRAM are unchanged; flash text increases 76 bytes and rodata 48 bytes. Physical results follow below.

## COM6 results

The candidate was flashed at 921600 and fully read back at 115200, including the OTA selector. Both writes match; the original application slot remains preserved. The owned host booted VALID with no automatic Bluedroid restoration and passed the existing 37 cold codec cases. No new boot tests were added.

Native YouTube LHDC 192 kHz / S24 / stereo / fixed 900 kbps was restored after reconnect (which had selected 48 kHz/automatic quality). Commanded play was 12.218 seconds with an independent 14-second pause guard. Volume93/EQ 2/8/12 / flags 12 were observed during playback. This was a different segment of the loaded music track, not an identical-content randomized benchmark.

| Measurement | Prior image | Fair-sharing candidate |
|---|---:|---:|
| Accepted = rendered = DMA source frames | 2,372,160 | 2,566,592 |
| Initial DMA shortage frames | 562,048 | 516,480 |
| Late recovered frames | 55,936 | 197,632 |
| Unfilled DMA frames | 506,112 | 318,848 |
| Descriptor errors | 0 | 0 |
| Core1 maximum local render interval, us | 12,334 | 4,130 |
| DSP publication-to-retirement maximum, us | 5,649 | 1,787 |
| DSP helper dispatch maximum, us | 5,115 | 797 |
| Host poll maximum wall time, us | 2,261 | 4,481 |

The candidate has 37.0% fewer unfilled frames in this one comparison, but still fails continuity. Host wall time includes preemption by the newly equal-priority renderer and increased; zero ingress rejects alone does not qualify control latency. Neither run records encoded-queue drops, decode rejects, PCM delivery failure/timeouts, RTP sequence breaks or snapshot loss. The bounded detail trace lost records (candidate worker_trace_lost = 4513); it is not a complete packet timeline.

Sparse wire-RTP observations also show timestamp advances beyond the normal 960 samples per one-frame packet while sequence numbers remain continuous. For example, candidate sequence 2 has timestamp 960, while sequence 2602 has 2792640: an additional 295680 source-clock ticks. Prior-image observations reach 483840 additional ticks. These are gaps already represented on arrival, consistent with upstream pressure; these records do not prove the source's reason. Payloads observed in the candidate reach 566 bytes, without oversize rejection, in the retained 1800-byte × 8 decoder queue. The packet rate/size corresponds to the phone's fixed 900 kbps mode, not proof of the 1000-kbps goal.

## LDAC phone-volume regression

Native YouTube, LDAC 96 kHz / stereo with the phone's 32-bit input selection: 12.068 seconds commanded playback, independent 14-second pause guard, ten real volume-down keys then ten volume-up keys. Independent Android readback confirms110 ->10 ->110. Accepted, rendered and DMA source frames all equal 1,432,832. Initial shortage, late recovery, unfilled frames, descriptor errors, encoded drops and decode rejects remain zero. Observed DSP controls retain EQ 2/8/12 / flags 12 and return to volume 93. The actual key spacing was approximately 233–261 ms down and 99–146 ms up, not a claimed 20 ms stress test. LDAC maximum bitrate, all codecs and audible pop elimination remain unqualified.

The user's metronome test is in native YouTube. Searching for 'metronome 120 bpm' returned a blank results area on repeated fresh UI snapshots. No metronome video was selected or claimed tested. The existing music player was restored paused; phone volume remains 110/150.

The candidate remains an explicit development selection on COM6, with the experiment default OFF in source. Controller priority 23 remains highest. Full-rate LHDC, quantified control latency, encrypted BLE coexistence, acoustics and the complete feature/stress gates are open. Do not publish this as a qualified main/release replacement.
