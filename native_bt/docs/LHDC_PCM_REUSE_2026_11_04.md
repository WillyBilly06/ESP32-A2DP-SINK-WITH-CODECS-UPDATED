# Reuse retired LHDC transform storage for PCM

The single-slot LHDC worker no longer appends a separate interleaved PCM buffer
to its configuration allocation. At emission, both channels have completed
IMDCT and overlap-add. The remaining float PCM sources are `pcm_mid` and
`ch_pcm` (`mdct_in`); `mdct_out` has no remaining reader. The existing final
conversion now writes directly into that retired transform buffer.

This reuses storage without an additional copy, PCM format change or arithmetic
change. The frame parser obtains rate/depth/channels from the configured CIE,
so packet headers cannot resize or replace the buffer. Open validates its
allocated transform size and PCM capacity. The single-slot static assertion
remains. No new state field or allocation is introduced.

The immutable PCM loan borrows this buffer only after conversion. The existing
worker fences loan retirement before a subsequent frame, reset, close or
reconfiguration can reuse/free it. The copied provider still copies before
returning. No callback waits for another owner.

## Local verification

Eighty real-codec deferred/loan cases pass with ASan and owned-code UBSan in
both the retained-buffer reference and reuse build. Every sample byte and
timestamp matches synchronous decoding, and all 80 complete PCM hashes match
between builds. Tests cover fragmentation, batching, a simulated 150 ms pause,
suspend/drop/reconfiguration with live loans and all 37 configurations plus
the 192 kHz/S24/1000 kbps fixture. The mock now hashes each outstanding loan at
admission and at every completion poll to detect mutation before retirement.

Tracked peak reductions are 960 bytes for the 44.1/48 kHz S16 fixtures,
3840 bytes for 96 kHz S24 and 7680 bytes for 192 kHz S24. Other codec peaks are
unchanged. These paired host measurements are not physical free-heap or CPU
headroom measurements. `DBT_LHDC_KEEP_DEDICATED_PCM` retains the preceding
allocation for explicit comparison only.

```powershell
python native_bt/tools/test_probe_codec_check.py --deferred --pcm-loans --sanitize --lhdc-esp32-paths --lhdc-keep-dedicated-pcm --lhdc-source '<LHDC checkout>'
python native_bt/tools/test_probe_codec_check.py --deferred --pcm-loans --sanitize --lhdc-esp32-paths --lhdc-source '<LHDC checkout>'
```

Target codec/hash/deadline validation, full-product memory admission, DSP/I2S
continuity and Bluetooth/BLE coexistence remain separate open gates. The prior
paired-FIR experiment was rejected for speed; this allocation change makes no
DSP speedup claim.
