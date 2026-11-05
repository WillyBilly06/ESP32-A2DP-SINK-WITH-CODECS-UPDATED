# Integrated word-DSP synthetic pipeline result

The [bound result](WORD_PIPELINE_RESULTS_2026_11_05.json) identifies the image,
ELF, source manifest and complete COM6 capture. Ten owning inputs were copied
and hash-checked before further work. The trial uses the actual 1800-byte x8
queue, two-step worker budget, compact copied PCM input, 16 ms processed output,
block DSP and word quantization. Radio, BLE and storage are absent.

This pipeline does **not** pass. It accepts12 packets and completes4 of96 before
the eight encoded slots fill and one media drop stops the test at65.084 ms.
Decoder/worker fault and PCM-timeout counters remain zero. No allocation failure
is reported. Renderer payload is39936 bytes; observed minimum free internal heap
is172204 bytes in this controller-off image. Core1/render task stack low-water
observations are4980/3376 bytes. Tracked allocations return to zero on cleanup.

I2S starts. The active snapshot reports4352 accepted sample frames,3456 rendered,
896 input-ready,3072 output-ready and384 DMA source frames. The reported counters
balance, but are separate reads rather than a coherent multi-thread snapshot.
Three EOFs have a667 us maximum gap and no descriptor errors or underrun frames.
The diagnostic `continuity:true` field therefore applies only to about2 ms of
DMA source observation before stop. It is not a sustained playback pass.

Profiling including stop drain spans4363 PCM sample frames. It attributes about
4.78 us/frame to conversion/resampling and9.49 us/frame to DSP, plus tail/control
work. Initial coefficient/volume ramps dominate this short run; the canonical
ramp prefix remains unchanged by the steady-only word optimization. These
aggregate wall/core-cycle spans include preemption and must not be substituted
for the isolated steady-kernel comparison. The checksum/decode correctness gate
is incomplete because the required96 packets were not completed; the result
does not establish a decoded-PCM mismatch.

The preceding [kernel comparison](DSP_WORD_TARGET_RESULTS_2026_11_05.json) still
establishes the approximately10% steady full-EQ cost reduction with exact output
and state. That narrow result does not certify complete pipeline headroom. The
next DSP work must address the ramp path and the remaining conversion/resampling
cost without shortening transitions, lowering precision, or hiding packet loss.
The baseline is selected and boots cleanly after the finite diagnostic.
