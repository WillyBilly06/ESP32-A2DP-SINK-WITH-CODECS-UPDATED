# Native audio clock correction

The audio rate negotiated by AVDTP remains the nominal codec rate. The output
render task calls `dbt_audio_clock_reset` after the I2S consumer is enabled and
calls `dbt_audio_clock_update` with monotonic milliseconds approximately every
500 ms. No I2S clock is periodically reconfigured, and no codec timestamp or
header is rewritten. A causal, fixed-point polyphase FIR advances by a bounded
correction to its input phase; it does not insert an exact duplicate or discard
a sample as a hard correction.

The first four seconds of clean playback establish an occupancy target from the
actual combined input/output queue, avoiding a fixed packet-duration assumption
for SBC, AAC, Opus, aptX, or later codecs. A slow integer occupancy filter and
bounded integral controller adjust the phase step by at most +/-500 ppm, with
slew limited to 50 ppm per second. The integrator stops at saturation and is
not trained on an underrun, decoder backpressure, missing progress or a long
scheduler gap. Backpressure with an advancing consumer can apply bounded
proportional relief for an overfull queue; this is not learned as clock drift.
The controller waits for two seconds of clean progress after a disturbance.
Opening a new format/rate and closing the output reset the controller, while
EQ and volume control snapshots retain their existing independent behavior.

`dbt_audio_clock_snapshot` provides bounded atomic telemetry. State is OFF,
CALIBRATING, TRACKING or RECOVERING. `ppm` is the currently applied fractional
correction; positive values consume source frames faster. `error_frames` is
queued input plus output frames minus the calibrated target. `observed_ppm` is
an eight-second ratio of accepted source frames to consumed output frames, and
`observed_valid` means that window had no detected loss/backpressure/stall. It
is subject to packet-boundary quantization and is a diagnostic estimate, not a
measurement of either oscillator in isolation. It is not injected into the
controller. The output snapshot carries the same values and validity indicators
for the low-priority logger.

`python native_bt/tools/test_audio_clock.py` runs a deterministic 600-second
counter model with packet-arrival jitter, both offset signs and all 13 DSP
rates from 8 to 192 kHz, explicitly including 96,010 and 95,991 frames/s at a
96-kHz nominal rate and +/-19 frames/s at 192 kHz. It checks settling, bounded
slew, small steady occupancy error, observed-rate quantization, clock wrap,
anti-windup, backpressure, recovery, scheduler jitter and rate-switch reset.
A separate finite 20-second test per case executes the actual fixed-point
render/consumer with 40-ms input bursts; another checks interpolation output
against an independent ramp calculation at both correction limits. Existing
PCM-format, fade, OOM and control-snapshot regressions were also run. These are
digital simulation/host results, not proof of phone clock behavior, RF,
continuous playback, I2S timing, acoustic quality, or on-device stack headroom.

On Linux, the same driver accepts `--sanitize` for a local ASan/UBSan build
and `--list-inputs` to enumerate the needed source files. The ordinary Windows
host run does not itself establish a sanitizer result.

## Polyphase resampler

The adaptive path uses 129 immutable Q30 coefficient rows per bank and linearly
interpolates adjacent rows using all 25 remaining Q32 phase bits. The 96-, 48-
and 24-tap banks serve rates up to 48 kHz, up to 96 kHz and above 96 kHz,
respectively. The 128-frame stereo history is 1,036 bytes on the tested ABI;
it is part of the audio object and is reset for each open. The generated banks
occupy 86,688 bytes of read-only data. Their per-row Q30 absolute-sum maxima,
including a conservative per-tap interpolation rounding allowance, are
2,776,196,764, 2,484,840,164 and 2,209,870,756. Each is below 2^32, so the
worst signed Q31 input cannot overflow the signed 64-bit accumulators. Output
is saturated to Q31; filter overshoot at full-scale transients can therefore
clip. Coefficients are generated offline by `tools/gen_resampler_coeffs.py`.

Causal group delay is 47, 23 or 11 input frames; at 44.1 kHz the largest delay
is about 1.07 ms. Startup extends the first sample into the history. After the decoder producer is quiescent, the stop path can extend the final
sample through the causal delay while the existing fade drains. The existing
fixed 20-ms stop fade can still close the output before a larger buffered queue
is fully consumed. The output-delay report includes the filter group delay. The bank
has a finite transition band, not an ideal brick-wall anti-alias filter.

At 48, 96 and 192 kHz the design performs about 9.22 million stereo sample MACs
per second, plus about 4.61 million coefficient interpolations per second;
44.1 kHz performs about 8.47 million stereo MACs. These are arithmetic counts,
not ESP32 cycle or deadline measurements.

Independent host sine projections at 1, 10, 18 and 20 kHz when below Nyquist,
across every supported DSP rate and 128 interpolated phases, measured a
minimum/maximum gain of -0.041328/+0.001970 dB, maximum gain variation across
phase of 0.001982 dB, and maximum phase error of 0.004100 degrees. A separate
8,192-frame moving-phase tone test at +/-500 ppm, after fitting out its
fundamental, measured worst residual -82.462 dBc and worst of the first four
phase-sweep sidebands -82.732 dBc. These are finite digital tests and are not a
claim of 24-bit transparency, complete alias rejection or acoustic fidelity.
The tests also cover phase wrap, full-scale accumulation bounds, drift,
backpressure, rebuffering and rate-switch reset.
