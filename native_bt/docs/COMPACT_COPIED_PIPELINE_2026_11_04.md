# Compact copied input and independent core placement

The PCM-loan path retains a decoder frame until the renderer consumes its
views. On core0 this serializes decoder and renderer work; moving the decoder
alone still leaves that raw-buffer lifetime dependency. This finite experiment
uses the existing nonblocking copied-input path with a smaller ring, so the
decoder may reuse its buffer while core0 renders an owned copy.

`-PcmCopied` selects a nominal 4 ms input ring rounded to a power of two, with
a 256-frame minimum matching the codec worker's bounded transfer quantum.
It excludes `-PcmLoans`, requires async I2S and forces unconfirmed finite boot.
Short PCM targets and block DSP accept either explicit input experiment.
The existing 32 ms copied-input and 64 ms output defaults are unchanged.

| 192 kHz/stereo/S32 mode, page-sized output | Input payload | Output payload | DMA payload | Total |
| --- | ---: | ---: | ---: | ---: |
| Default copied input / 64 ms output | 65536 | 102400 | 3072 | 171008 bytes |
| Loan input / 16 ms output | 0 | 28672 | 3072 | 31744 bytes |
| Compact copied input / 16 ms output | 8192 | 28672 | 3072 | 39936 bytes |

At 192 kHz the rounded compact capacity is 1024 frames (5.333 ms). Lower
sample rates retain the minimum quantum even when it represents more than
4 ms. This is not an encoded-jitter replacement or an equivalence claim for
the original 64 ms target. No bit depth, sample rate, codec quality or DSP
precision is reduced.

The producer attempts one whole-block copy and returns `BUSY` without partial
publication if there is insufficient room. The existing renderer notification
wakes it after consumption; it does not poll or wait inside the PCM callback.
Stream-control tickets and teardown fencing retain the existing async copied
path. No new task or queue is added.

## Core arrangement under test

The full-product candidate uses `-DecoderCore 1 -LoggerCore 1`; rendering stays
on core0. Controller/host stay on core1. Host priority 6 remains above decoder
priority 5; logger stays priority 1 and renderer priority 6. The independently
selectable logger avoids placing diagnostics on an overloaded renderer core.
Omitting `-LoggerCore` retains its previous opposite-decoder default.

The receipt, CMake cache and actual `probe_audio.c` definition must agree on
logger placement. Compact input is similarly bound to provider/client flags.
Every I2S record reports `pcm_input_mode` (0 legacy copy, 1 loans, 2 compact
copy), actual input capacity and allocated input bytes. These fields reset
their live allocation quantities on close.

## Local evidence and open gates

Scalar and block-DSP provider builds pass ASan/UBSan without signed-overflow
wrapping. Each checks 24 rate/format/channel geometries, input-floor behavior,
whole-frame copy followed by immediate decoder-source overwrite, whole-block
backpressure, wrap, decoder wakeup and teardown. A full 960-frame LHDC output
fits; after it is copied the source may change independently. Full-ring retry
becomes accepted only after actual renderer consumption frees enough space.

Nine CMake configurations, ten actual compact-mode checks, six logger checks,
two actual finite-deployment checks and two early CLI rejections pass. Existing
loan/short-target/block mode checks pass; loan provider tests pass at all three
targets. The full all-codec/product build passes source/image/layout and the
limited IRAM callback audit. The I2S record's conservative 752-character bound
fits the existing 1024-byte formatter.

```powershell
python native_bt/tools/test_compact_copy.py
python native_bt/tools/test_compact_mode.py
powershell -File native_bt/tools/build_probe.ps1 -PcmCopied -BlockDsp -PcmTargetMs 16 -DecoderCore 1 -LoggerCore 1 -AsyncAudio -WithProductIo -WithRfcomm -WithLhdc -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir '<LHDC checkout>'
```

This adds a copy and 8 KiB at maximum rate to permit overlap. Its throughput,
CPU balance, actual stacks, BLE/control behavior and long-run jitter effects
need physical measurement. It does not make the over-budget renderer faster
by itself. No full-rate continuity or feature-parity pass is claimed.
