# PCM span copies and renderer comparison — 2026-11-05

## Compared source

The complete owned dbt_audio.c body and public contract were read, including SPSC publication, borrowed input, conversion, render blocks, clock control, DMA take and stop fade. Reference BTstack is pinned at 9edb5d82ea6ec4466a393415d65030929330acdf and is not integrated.

| Boundary | Compared source | Finding |
|---|---|---|
| Fractional rendering | BTstack src/btstack_resample.c:44–114; owned dbt_resampler.c:34–145 | The reference uses 16-bit linear interpolation. This product keeps full 32-bit PCM and the existing 96/48/24-tap Q30 FIR. The reference is not an equivalent replacement for the selected filter. |
| Rate estimation | BTstack src/btstack_sample_rate_compensation.c:45–114; owned dbt_audio.c clock functions | Reference periodically smooths source-rate/ratio observations. Owned control separately tracks occupancy, backpressure, underruns and finite slew. No reference algorithm is copied. |
| Working baseline | released 0d85a975, bt_audio_sink/main/audio/audio_pipeline.h:245–328 | Its driftDrop/driftInsert are explicitly zero. It processes DSP then emits a fixed chunk; it does not run the new continuous FIR stage. This is a material workload difference, not evidence of a Bluetooth radio bitrate change. |
| Bounded owner scheduling | probe_output.c:659–860 | Each render call is capped to 128 frames. Runnable refill continues toward a finite watermark; idle waits use notifications/deadlines. CPU cost can still exceed the audio deadline despite bounded execution. Full output lifecycle review remains incomplete. |

Historical render evidence exceeded the 96 kHz 128-frame deadline (1732.91 us versus 1333.33 us). That evidence predates the current candidate. Current sustained headroom is not established. The FIR/DSP cost remains open.

## Change

PCM submission now copies a whole accepted input block in one or two ring spans instead of looping per frame and per byte. It still rejects the entire block on insufficient capacity, copies before release publication and never modifies the immutable external decoder view. Input capacity is a power of two and a submission is at most that capacity. Byte totals and ordering are identical.

DSP block output and DMA take now copy contiguous page/ring spans. For a valid cursor, every nonempty iteration copies at least one frame and stops at the lesser of the page edge, ring edge and remaining count. At most 65 spans cover the maximum legal full-ring transfer; normal <=128-frame operations need at most three when a partial final page gives distinct page and ring edges. Physical cursors still wrap independently of uint32 publication counters. Missing output is zeroed after copied frames, then the original full-block read/counter publication occurs.

The submit/backpressure/render counters have one writer for each open stream, verified at every owning-source write site. Open/reset already require owners quiescent. Their increments use atomic relaxed load/store instead of atomic read/modify/write retry loops. Other owners remain atomic readers; control-writer serialization and SPSC acquire/release publication are unchanged. Borrowed and copied input modes cannot be active together.

No task, queue, PCM format, sample rate, codec setting, FIR coefficient, rounding, DSP state or affinity changes. This is bounded CPU copying, not DMA offload or a measured speed guarantee.

## Compiler and memory evidence

Strict C11 Clang analysis passes for both block-DSP and scalar-DSP configurations. Full ESP-IDF 6.1 ccache compilation passes with stable source fingerprints. Linked Xtensa submission previously contained the byte loop and two S32C1I retry sites; the new function has neither. DMA take remains in IRAM at 0x40088d5c. Its copy/zero calls resolve to ROM memcpy 0x4000c2c8 and memset 0x4000c44c, with literal pointers in IRAM. They do not add flash-code calls to the cache-disabled path.

Whole-image text increases by 100 bytes (866743→866843): IRAM text grows 24 bytes (122231→122255), flash text grows 76 bytes, DRAM data remains 23893, aggregate data remains 396577 and BSS remains 50300. Thus there is a small instruction-SRAM cost despite unchanged BSS/data. The submit and DMA-take compiler frames each increase 32→48 bytes; renderer frame remains 1120 bytes. These figures exclude ROM-callee stack use. ISR/producer stack headroom under load remains unqualified. Function/text size and instruction changes are not elapsed-time measurements. No task stack allocation is increased.

The IDF 6.1 public i2s_channel_tune_rate contract permits tuning only while the channel is READY/stopped. Its implementation takes a mutex with portMAX_DELAY and changes the APLL clock source. It is not adopted as a live nonblocking substitute for the FIR. No undocumented register tuning is introduced.

## Qualification

No unit, packet-injection, playback, app, pairing or timing campaign is invoked under the user's test hold. Flash readback and ordinary boot evidence are recorded separately. Existing automatic codec startup checks do not exercise span wrapping, ISR refill under load or clock/DSP headroom. Full-ring/partial-page/counter-wrap/concurrency and sustained-audio qualification remain open. The complete stack comparison and master goal remain unfinished.
