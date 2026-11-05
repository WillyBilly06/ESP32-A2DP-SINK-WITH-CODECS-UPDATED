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

## COM6 decoder validation

[The exact target result](LHDC_PCM_REUSE_TARGET_RESULTS_2026_11_04.json) passes
all 37 codec configurations and three complete maximum-LHDC runs. Each decodes
96 packets/92160 frames with unchanged FNV 4008775619 and 512-frame prefix
639236599. All 252 measured packets meet the isolated 5 ms deadline, including
validation: mean 4583–4584 us, worst 4884 us. This is not a decoder speedup.

The tracked peak falls from 59832 to 52152 bytes, confirming the 7680-byte
saving on ESP32. Minimum free internal heap after decoder allocation rises
from 224528 to 232208 bytes in the isolated diagnostic. These are source-bound
decoder measurements, not full-product free heap. Released 1.0.4 firmware
returns with System ready and no fatal boot marker.

The build with all codecs/product/security/RFCOMM passes source/image/layout
and the limited IRAM callback audit (six protected symbols, five direct
functions, four indirect calls unproven). Full-product memory admission,
DSP/I2S continuity and Bluetooth/BLE coexistence remain separate open gates.
The prior paired-FIR experiment was rejected for speed; this allocation change
makes no DSP speedup claim.

## Full host/Redmagic follow-up

[The separate RF result](RF_LHDC_PCM_REUSE_RESULTS_2026_11_04.json) still fails
maximum output admission. Android freshly confirms LHDC V5, 192 kHz, 24-bit
stereo and the 1000 kbps target; Android does not report measured bitrate.
Firmware records the matching requested rate/container, then fails a 4096-byte
allocation with 6656 bytes free and largest block 1984. No CRC-valid 192 kHz
running-I2S snapshot follows. This is a failed maximum-rate startup, not DAC
playback. Current allocation telemetry does not identify how many pages had
already succeeded, so this result cannot quantify the remaining byte deficit
or attribute changed fragmentation solely to the saving.

The preceding LHDC48 epoch uses DMA and retires all 9642 PCM loans, with zero
active/cancelled loans, stale receipts, PCM faults/timeouts or descriptor errors.
Its 23079 IRQ callbacks take at most 45 us. Audio nevertheless fails: 638736
underrun frames, 1332 encoded-queue drops, 15428 us maximum render span and
18095 us maximum loan hold. Input sequence/SSRC breaks remain zero in the last
epoch snapshot. Retained 48 kHz counters after teardown are not 192 kHz output.
The later reconnect also returns to 48 kHz; its counters form another epoch.

Across the finite capture, 15 core0 idle-starvation watchdog warnings occur;
core1 has none. Watchdog task samples name `native_render` eleven times and
`native_decoder` four times. Both tasks are configured on core0 in this image;
these samples are not utilization measurements or proof that a different
placement meets deadlines. DSP/kernel throughput remains independently open.

The capture reaches its 1 MiB byte limit before the 300-second maximum. It has
2130 CRC/time-valid periodic records; 18 interleaved/truncated records are
excluded and 1379 one-off/trace records are outside checksum coverage. Integrity
audit therefore fails the whole-capture gate, while retained individual
periodic records validate. No statistics reconstruct damaged records. The
helper observes released baseline return with no fatal marker, and phone
playback is verified paused. EQ controls were not exercised in this RF run.
