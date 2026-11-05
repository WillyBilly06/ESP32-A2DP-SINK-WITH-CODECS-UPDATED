# Finite processed-PCM target experiment

The measured maximum-rate path cannot allocate its 64 ms processed-PCM queue.
The new `-PcmTargetMs 16` or `32` switch permits finite full-product measurements
with a smaller queue. The default stays 64 ms. Non-default values require async
PCM loans, an unconfirmed finite boot and real I2S output; checksum-only mode
and persistent selection are rejected. This is a buffering experiment, not a
qualified replacement for the prior jitter tolerance.

No sample-rate, source bit depth, codec quality, integer DSP arithmetic, FIR
taps, PCM precision, peripheral or codec is removed or reduced. DMA remains
three descriptors of 128 stereo/S32 frames. Output target/prefill and adaptive
clock occupancy follow the selected duration; the AVDTP delay estimate uses
the same target. Every periodic I2S snapshot and offline image admission reports
the actual compiled target.

| 192 kHz target | PCM pages | PCM payload | PCM + DMA payload |
| --- | ---: | ---: | ---: |
| 64 ms default | 25 | 102400 bytes | 105472 bytes |
| 32 ms experiment | 13 | 53248 bytes | 56320 bytes |
| 16 ms experiment | 7 | 28672 bytes | 31744 bytes |

The smaller target changes buffering latency and jitter headroom. Allocation
success alone cannot establish continuity, clock stability, BLE/control
headroom, NVS coexistence or original jitter tolerance. Those gates remain
required. No performance or physical output pass is claimed by the local tests.

The actual provider/render ownership suite passes at all three targets under
ASan/UBSan. It verifies exact allocation geometry, configured delay, all input
formats, partial cleanup, ticket retirement/cancellation, stale IDs, DMA faults,
carry and restart fencing. The default copied-output tests also pass. Eight
actual CMake configurations, sixteen actual image-admission cases and two
early CLI rejections cover mode isolation, receipt/cache mismatches, missing/
overridden/undefined compile definitions and unreceipted short targets.
Six actual deployment-guard cases additionally require explicit finite RF or
pipeline capture with baseline return; a bare probe deployment cannot leave a
short-target experiment running.

```powershell
python native_bt/tools/test_probe_output_loan.py
python native_bt/tools/test_pcm_target_mode.py
powershell -File native_bt/tools/build_probe.ps1 -PcmTargetMs 16 -PcmLoans -DecoderCore 0 -AsyncAudio -WithProductIo -WithRfcomm -WithLhdc -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir '<LHDC checkout>'
```

Builds use ccache. The full product passes source/image/layout admission and
the limited IRAM callback audit. Exact hardware evidence follows.

## COM6 maximum-rate result

[The full RF result](PCM_TARGET16_RF_RESULTS_2026_11_04.json) confirms memory
admission and running 192 kHz I2S with a 31744-byte PCM/DMA payload. Firmware
publishes LHDC CIE `00ff3a050000354c0102110000`; Android freshly confirms
192 kHz, 24-bit stereo and the 1000 kbps target. Android does not report actual
bitrate. The observed aggregate media-payload size is 628 bytes per packet;
that observation alone is not a measured encoded bitrate. No sample-quality
setting was lowered to admit this path.

The last CRC-valid I2S record has 13694784 source frames sent to DMA, 306218
callbacks, zero descriptor errors, maximum EOF gap 704 us and callback peak
40 us. Allocation failures, stale receipts and PCM faults/timeouts remain zero.
The cumulative, coherently published PCM checksum differs from the checksum
of an all-zero byte stream of the same length. This establishes nonzero decoded
PCM, not a full correctness or acoustic test.

Continuity **fails**: 25501376 underrun frames, 26418 encoded-queue drops,
13393 decoder resets and 42 core0 idle-starvation watchdog warnings occur.
The latest loan record has 59012 begun, 59011 retired and one still active;
it does not establish quiescent retirement at capture end. Maximum observed
loan hold is 16072 us. No core1 idle-starvation or interrupt-watchdog warning
is observed. Per-stage counters have separate bounded observation windows and
must not be summed as one atomic snapshot.

The render work records approximately 10.575 elapsed us per output frame,
including preemption, versus the 5.208 us/frame playback budget. Its maximum
call span is 14532 us. This is not a CPU-utilization measurement or an intrinsic
kernel timing. Decoder/render scheduling and integer-DSP/FIR work still need
optimization; a smaller PCM target has not solved throughput or proven jitter
equivalence. Live free internal heap reaches a recorded minimum of 53680 bytes
and largest-block observations include 49152 bytes; those do not establish
CPU or control-response headroom.

Deep Bass (+8/-1/-2) and Balanced restoration (0/0/0) are observed in the app UI
during streaming. Source inspection confirms that this UI updates optimistically.
No run-local wire acknowledgment or per-update NVS-commit timing is captured,
so these observations must not be promoted to storage/EQ coexistence proof.

The 300-second capture ends below its byte limit (887431 bytes). It contains
1486 CRC/time-valid periodic records and 45 rejected interleaved/truncated
records; unchecked one-off/trace records remain separate. Released baseline
returns with System ready and no fatal marker, and phone playback is verified
paused. The 64 ms default remains unchanged. This finite experiment is not a
production replacement, continuous-playback pass or complete feature-parity
qualification.
