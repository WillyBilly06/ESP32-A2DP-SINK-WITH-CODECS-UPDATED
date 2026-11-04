# Offline codec check for the ESP32 probe

`probe_codec_check(owner)` creates one 8192-byte task and returns false if any
selected case fails, leaks its tracked allocations, leaves less than 1024 bytes
of task stack, fails to start, or exceeds the 60-second wait. The caller must
check the result before enabling the controller. The `owner` argument remains
for compatibility. A timed-out task may finish later; the caller must still
treat the failed return as fatal for this boot.

Compile definitions `DBT_PROBE_SBC_BUILD`, `DBT_PROBE_OPUS_BUILD`,
`DBT_PROBE_AAC_BUILD`, `DBT_PROBE_APTX_BUILD`, `DBT_PROBE_LDAC_BUILD`, and
`DBT_PROBE_LHDC_BUILD` select the linked descriptors. Define only those whose
actual codec component is linked. The check opens and closes one descriptor and
configuration at a time. Its const input and reference samples live in flash.
It does not initiate Bluetooth operation.

The all-enabled run covers 16 Opus configurations (three packets each), one
SBC configuration, four AAC rate/channel configurations, standard aptX, aptX HD
and aptX LL at 44.1 and 48 kHz, six LDAC rate/channel-mode configurations,
and four LHDC configurations spanning 44.1 through 192 kHz and 16/24-bit output.
The LHDC vectors use valid encoded tone input; no 32-bit LHDC decode is checked
or implied. The generated header comes from bounded offline encoder fixtures and
previously validated host decoder output. Run
`python native_bt/tools/generate_probe_codec_vectors.py` to regenerate it after
the relevant fixture and decoder receipts exist; it verifies the source and
fixture hashes recorded by those receipts. Input hashes and case metadata are
written to `native_bt/_build/native_codecs/probe_codec_vectors.json`.

SBC, aptX, and Opus compare complete decoded PCM hashes. AAC, LDAC and LHDC
instead compare 24 selected sample values per case against host output with an
initial bound of 2% of each expected magnitude plus 256 S16 units or 2^24 S32
units. They also require a peak amplitude of at least one quarter of the host
reference and the expected frame counts and formats. The bounds accommodate
cross-architecture numerical variation; they have not yet been calibrated on an
ESP32. A failure needs investigation rather than an automatic tolerance change.
Sparse sample comparisons are not full-waveform or listening-quality proof.

Each `target_codec_case` JSON record reports the decoded format, packet and
callback counts, sample frames, maximum observed media-call duration, comparison
type and maximum checked error. The elapsed time includes self-test callback
and sample validation work. `tracked_peak_bytes` includes requested codec heap
allocations and the codec/test allocator headers, but excludes the allocator's
own metadata; `min_free_after_alloc_bytes` is the smallest free-internal-heap
snapshot observed after an allocation, not a global heap low-water mark. Task
stack low water is cumulative across the single task, not independent per case.

When Opus is enabled the existing `target_opus_selftest` record remains, with
`passed` and `frames:48` on success, for compatibility with the existing
flasher. The additional `target_codec_selftest` record reports overall selected
and passed configuration counts. Both records include `rf_test:false`.

`python native_bt/tools/test_probe_codec_check.py` compiles and executes the
actual check and codec implementations on the host, including failure injection.
Its FreeRTOS stack and timer values are stubs. `python
native_bt/tools/test_probe_codec_check_target.py` compiles the check as ESP32
objects for each single-codec selection and the combined selection and verifies
descriptor references. Neither test links a complete firmware, runs on target,
tests RF, or establishes real-time playback. Both invoke the workspace budget
guard before and after their builds.
