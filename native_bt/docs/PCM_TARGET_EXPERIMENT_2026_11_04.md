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

Builds use ccache. Exact full-product hardware admission, processing and
continuity results remain pending at this checkpoint.
