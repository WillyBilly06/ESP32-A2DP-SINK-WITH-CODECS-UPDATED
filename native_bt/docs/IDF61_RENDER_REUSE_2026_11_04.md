# Exact render reuse and rejected experiments — 2026-11-04

This checkpoint retains conversion/FIR changes with exact numerical behavior:

- Once FIR history is complete, read its samples directly without per-tap
  priming checks. Zero-remainder and fractional paths retain every coefficient,
  integer interpolation rule, tap and saturation operation. Incomplete history
  retains the original priming path. No buffer or state field is added.
- Reuse the already-converted current input pair when committing the first
  consumed sample to FIR history. Additional consumed samples and drain cases
  retain their existing behavior. Zero/one/two-sample clock steps remain covered.
- Aligned S32 little-endian input uses an alias-safe four-byte builtin copy with
  an explicit alignment check. Unaligned input and other formats retain byte
  decoding. The input allocator contract is not strengthened.

## Measured and rejected work

A fused stereo biquad experiment shared coefficient loads and identity checks
while retaining two exact sums and histories. All 18 target case pairs matched
PCM and complete state, but its DSP stages were slower. At 192 kHz/flags 9,
steady EQ/compensation increased from 3.192 to 3.388 us/frame, and crossover/
boost from 2.110 to 2.163. The fused helper was removed.

The same run's conversion/FIR span improved from about 5.051 to 4.279 us/frame
against the preceding scalar accumulation image. Those reuse/FIR/load changes
are retained. These are observed finite prefixes, not a sustained latency
qualification or an attribution of the gain to each individual change.

Image: c819a30c7f6e8f6533c16a683632d77210d16e78261602631ade6c41c932965f,
source 57eeb01d, run recorded in JSON.

A second experiment restored the scalar biquad and placed the hot scalar/FIR
functions in IRAM. Actual object sections confirmed placement. The controlled
192 kHz steady filter stages were about 3.198/2.117 us/frame, effectively the
same as the earlier 3.192/2.110. Conversion/FIR was about 4.339 us/frame. A
feature-equivalent full-product ELF comparison showed IRAM text rising from
121883 to 123191 bytes (+1308); DRAM data/bss remained 23685/31608. That linked
comparison also includes removal of the fused caller, so it is a combined text
cost, not pure section-placement attribution. With no material controlled speed
gain, the added placement was removed.

Image: 1013ce8636d4ddbdde17c217d840e4e478a0173c8239b544dc3547012b156f4f,
source b7cdbe45. The lower maximum render span (1883 us) is a single finite
prefix with other changes and does not establish an IRAM speedup.

Neither full-quality 192 kHz/S24/stereo/1001600-bit/s run passed: encoded queues
filled before output prefill completed, and DMA playback never started. The
last trial delivered 13248 PCM frames and rendered 5120 in 80074 us. Decoder/
audio buffers were reclaimed, no fatal boot marker occurred, and released
1.0.4-idf6.1 ota0 reached System ready after both trials. Only COM6 was used.

The selected tree uses the proven scalar DSP at Os and conversion/FIR at O3,
with the retained reuse changes and no fused stereo or added hot placement.
The selected all-codec/product/security/RFCOMM image compiles and passes offline
image/layout/source admission. That exact selected combination has not had a
new full hardware pipeline run; performance remains unqualified.

## Validation and next gate

179 portable tests pass. The wide filter/reference tests pass 300000 cases and
106496 rapid-control stereo frames. The FIR wide oracle passes 150000 full
output/state cases across all tap lengths and phases. O3 ASan/UBSan tests now
also exercise aligned and unaligned S16/S24/S32 mono/stereo input against the
integer-DSP numerical reference. Clock/resampler spectral results are unchanged.
These host checks do not establish audible quality or hardware continuity.

The compiler harness now uses the general arithmetic reference in internal-
stage mode, and the same configured algorithm at both Os/O3 in ordinary compiler-
comparison mode. Both host modes pass output/state checks. New compiler choices
must be measured on identical input; smaller code or fewer loads alone have not
predicted target speed reliably in these experiments.

The bundled public i2s_channel_tune_rate documentation allows READY state only,
not an active stream. Its APLL implementation disables/re-enables the source
and takes the channel mutex without a finite timeout. It was reviewed and not
used to bypass the software clock path or claim uninterrupted tuning.

Next: controlled FIR/compiler and producer-copy timing, then exact optimizations
with the same full-quality fixture. DSP and conversion still exceed their
combined frame budget. Controller/BLE/storage coexistence, complete profile/
security lifecycles, sustained fractional-clock/DMA operation and an equivalent-
workload SRAM advantage remain open. The master objective is incomplete.

[Raw image-bound results and selected build](IDF61_RENDER_REUSE_RESULTS.json).
