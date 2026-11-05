# Trace the first maximum-LHDC drop

The first recorded loss is a **software rejection at decoder-queue admission**,
not a packet that was already missing when the host received it. This analysis
uses the exact 16 ms/core0-decoder RF image and archived ELF. Current owning
sources match that image's source manifest. [Machine-readable evidence](DROP_ROOT_CAUSE_RESULTS_2026_11_04.json)
retains the first events, validated periodic counters and decoded stacks.

## First-loss chain

| Stage | Observation and scope |
| --- | --- |
| VHCI ingress | Zero rejected callbacks/ingress packets recorded, including the final valid snapshot. This does not rule out radio retries or all transport latency. |
| Host RTP admission | Input sequence and SSRC break counters remain zero. Sequence 13 arrived with expected sequence 13. |
| Decoder input queue | At ESP timer **89.086152 s**, `queue_full` ordinal 1 rejects sequence **13**, payload 628 bytes, because all **8** slots are occupied. |
| Worker sequence handling | At **89.184884 s**, the worker sees sequence **14** while expecting **13**. Its sequence-gap branch resets the decoder. That gap is downstream of the recorded host rejection. |
| Renderer | Aggregate render wall time is approximately **10.575 us/output frame**, including preemption, against **5.208 us/frame** at 192 kHz. |
| I2S | DMA is running at 192 kHz, with zero descriptor errors, callback peak 40 us and maximum EOF gap 704 us. Processed-source starvation nevertheless produces underrun frames. |

The first CRC-valid nonzero drop-counter snapshot corroborates 73 queue-full
drops, no input sequence/SSRC breaks and no decode rejects, PCM failures or
timeouts. Individual `media_trace` records have no CRC; the periodic records
used here pass the capture integrity/time checks. Counter snapshots have
separate observation windows, not one coherent instant.
The queue-full record's `expected` member is currently populated from the
incoming sequence itself; it is not an independent ordering check. The
separate input-sequence-break counter supplies the upstream-order evidence.

The exact rejecting branch is `dbt_codec_worker_media()` in
`codecs/common/dbt_codec.c`: it checks `written - media_read >= depth`, increments
`queue_full_drops`/`media_drops`, emits `DBT_CODEC_QUEUE_FULL`, and returns
`DBT_NO_SPACE`. The later `deferred_step()` detects the resulting worker gap
and invokes reset. Increasing queue depth alone cannot repair a sustained
consumer-throughput deficit.

## Why slots stay occupied

The renderer is priority 6 and the decoder is priority 5, both on core0 in this
image. Controller/host work remains on core1. Each decoded LHDC frame contains
960 samples, due every 5 ms at 192 kHz. The worker lends at most 256 PCM frames
per ticket and cannot overwrite/reuse its raw decoder buffer until the renderer
retires those views. The frame's media slot is retained until its complete PCM
output has retired and the codec batch finishes.

`pcm_retire()` returns pending while the renderer owns a ticket;
`dbt_codec_worker_runnable()` then reports no runnable decoder work. This is
correct lifetime fencing, not a spin wait. With both tasks on one core and one
raw frame in flight, however, decoding and rendering are serialized. Waking
the higher-priority renderer can also preempt the decoder before its PCM-begin
call returns. An asynchronous API alone does not create parallel execution.

The renderer runs `dbt_audio_render()` -> resampler -> `fixed_dsp_process()` ->
biquad filters. The sustained wall-time budget failure and occupied media slots
explain the software queue overflow. The PCM admission hook also hashes PCM
bytes for diagnostics; its extra cost has not been isolated and must not be
claimed as the dominant cause.

## Watchdog and interrupt attribution

All 42 watchdog backtraces are decoded against the **archived tested ELF**:

- 28 contain `dbt_audio_render()` with DSP/FIR work.
- 13 contain codec-worker/LHDC decode or PCM instrumentation work.
- One samples the render scheduling/critical-section path.

These are biased watchdog-triggered samples, not CPU-utilization percentages.
The task names printed before the cross-core backtrace are separate samples;
the task can change before the interrupted PC is captured. Stack classification
therefore uses decoded PCs rather than assigning each stack to that earlier
task-list name.

The common `esp_crosscore_isr` / `_xt_lowint1` frames are the SDK's requested
backtrace path (`REASON_PRINT_BACKTRACE`), not evidence that an application ISR
is the original packet-loss site. The first CRC-valid public watchdog hook is
at **94.081257 s** on the same ESP timer, **4.995105 s after the first rejection**.
The SDK's printed millisecond log timestamp uses a different offset and is not
mixed into that subtraction. The hook executes on core1 while core0 is the
reported idle-starved core; those are different concepts.

Watchdog printing can amplify later stalls, but it did not produce the first
recorded rejection. The first queue/underrun observations also preceded the
manual Deep Bass UI action. No first-underrun event timestamp was latched, so
this capture cannot prove the exact ordering between the very first PCM
underrun and the first queue rejection.

Full decoded output remains locally archived as `watchdog-symbols.json` and
`watchdog-symbols.txt` beside the exact ELF. For example:

```powershell
xtensa-esp32-elf-addr2line -a -f -i -e native_bt/_build/hardware/runs/1793851239846868100-b8b5c1ed52c8/native_bt_probe.elf 0x40173A3E 0x40089100 0x4012D706 0x400E6C9A
```

The next fixes must reduce DSP/FIR work and remove avoidable decoder/render
serialization while preserving PCM ownership, precision and control ramps.
Adding sleeps, suppressing watchdogs or merely enlarging the queue would not
address this measured service-rate deficit. No fix or complete feature parity
is claimed by this trace analysis.
