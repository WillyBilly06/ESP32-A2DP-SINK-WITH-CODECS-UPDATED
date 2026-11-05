# Remove the ESP32 decoder's unused overlap snapshot

The pinned LHDC source already compiles selector-retry out for classic ESP32.
Its `ov_save` buffer is read only by that excluded retry branch, but the
single-slot decoder still allocates it. The native port now omits that buffer
when `CONFIG_IDF_TARGET_ESP32` is true. Portable builds retain the snapshot and
retry; an explicit retained-buffer reference supports paired validation.
No retry/concealment behavior, codec setting, coefficient or sample operation
is removed from the current ESP32 path.

Two host runs execute the classic-ESP32 source conditionals across all 80 real
codec/loan cases with ASan and owned-code UBSan. Both retain every PCM byte and
timestamp against synchronous decoding. Their first-512-frame hashes match
each other. The tracked allocation reduction is 968 bytes at 44.1/48 kHz,
1928 bytes at 96 kHz and 3848 bytes at 192 kHz, including the owned eight-byte
allocation header. The maximum-rate payload saving is 3840 bytes (3.75 KiB).
Other codecs have unchanged tracked peaks. These are paired host results,
not ESP32 heap or real-time performance measurements.

```powershell
python native_bt/tools/test_probe_codec_check.py --deferred --pcm-loans --sanitize --lhdc-esp32-paths --lhdc-keep-unused-overlap --lhdc-source '<LHDC checkout>'
python native_bt/tools/test_probe_codec_check.py --deferred --pcm-loans --sanitize --lhdc-esp32-paths --lhdc-source '<LHDC checkout>'
```

The 6.1 configuration now explicitly uses `FREERTOS_IN_IRAM=n` and drops the
obsolete `FREERTOS_PLACE_FUNCTIONS_INTO_FLASH` setting. SDK 6.1 already defaults
ordinary task functions to flash, so this cleanup makes no RAM-saving claim.
ISR-in-flash remains disabled and I2S ISR IRAM safety remains enabled. The
placement tool checks linked addresses, accepts archived build folders, and
also follows the owned `finish_sent` callback helper. Six protected symbols and
five direct callback functions pass on the exact archived full-product ELF;
four indirect calls and vendor/ROM/cache behavior remain outside that proof.

The earlier loan-pipeline run accepted only 512 PCM frames before its small
encoded queue overflowed. It never started DMA or completed the 96-packet
correctness comparison. Host and target LHDC floating-point decoding already
use a numerical oracle; an unequal cross-CPU FNV hash alone cannot diagnose
corruption. The target maximum-codec diagnostic now records a separate 512-frame
prefix hash for a within-target comparison. The complete COM6 codec diagnostic
now passes all 37 configurations. Its three maximum LHDC runs each decode 96
packets/92160 frames, retaining full FNV 4008775619 and first-512-frame FNV
639236599. The latter matches the earlier pipeline prefix on the same target.
The independent numerical oracle also passes; no cross-CPU hash equality is
claimed.

The 252 measured maximum-rate packets have zero 5 ms deadline misses, including
PCM validation: mean 4584 us, maximum 4887 us. This is isolated decoder timing,
without controller, BLE, DSP or I2S. The target tracked peak falls from 63688 to
59832 bytes: 3840 payload bytes plus two eight-byte allocation headers. This
3856-byte tracked reduction is not a whole-product free-heap measurement.
[Exact results](LHDC_UNUSED_OVERLAP_RESULTS_2026_11_04.json) preserve source,
image, ELF, checks and baseline-return identity.

The separate [loan pipeline result](PCM_LOAN_PIPELINE_RESULTS_2026_11_04.json)
still fails continuity and complete correctness; no DMA starts. Maximum-product
SRAM admission, renderer throughput and full feature/stress qualification remain
open. All three finite trials returned to released firmware.
