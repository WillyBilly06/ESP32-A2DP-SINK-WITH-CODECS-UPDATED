# Remove diagnostic stack scans from host status checks

Source inspection finds `probe_audio_poll` collecting the entire output snapshot
on every host iteration just to inspect `failed`. `probe_output_stats` also calls
`uxTaskGetStackHighWaterMark` for the render task, whose SDK implementation scans
the unused stack byte by byte. Main's 40 ms product-meter update similarly
collects all statistics just for `running`.

The candidate uses the existing cheap `probe_output_failed` fence (failed OR
request-fault atomics) in the profile poll and adds a one-atomic `running`
accessor for the product meter. It removes no diagnostic field: the low-priority
periodic logger keeps its complete snapshot and stack measurement. No priority,
affinity, codec, DSP, buffering or security policy changes.

Actual provider tests check all eight combinations of running/failed/request
fault. Cheap getters agree with the full statistics and trigger zero mocked
stack queries; the full snapshot still triggers its expected query. Both scalar
and block compact-provider modes pass ASan/UBSan with these checks, plus their
existing copy/ownership/format/teardown cases. Production provider tests pass
at 100/1000 Hz, and the real-codec async probe regression passes.

The phase-profiling RF image was already frozen before this candidate was edited;
its exact original main/audio/provider/header bytes are additionally archived
under `owning-sources`, each checked against the tested source manifest before
copying. Building this candidate cannot alter that running image or its ELF.
This keeps the live cost measurement separate from the candidate fix.

The target ccache build, source/image admission and limited IRAM audit pass.
The frozen [phase trial](HOST_PHASE_RF_RESULTS_2026_11_04.json) attributes 17.47%
of host samples to audio polling in its startup/fault window; other polling
categories also consume substantial time. That is not exact CPU utilization or
a per-function execution benchmark.

## Hardware comparison

[Exact results](HOT_STATUS_RF_RESULTS_2026_11_04.json) bind source 100d9dd3,
image 1b0b5c2b6000 and the archived ELF/source manifest. All 37 target codec
configurations pass. Android and the ESP32 separately confirm maximum LHDC,
192 kHz/S24/stereo with the 1000 kbps nominal target, before playback starts.
Actual encoded bitrate is unavailable.
The phone needs a second normal Connect attempt before the dashboard appears;
no bond is cleared. This run does not qualify repeated reconnect reliability.

In the coherent core1 startup/fault window 138.176522–149.233489 s, audio polling
accounts for **5.49% of 5888 host samples**, versus 17.47% in the previous trial's
window. Runtime polling is 20.01%, wait calculation 21.62%, GATT 11.41%, and
controller query/submission 4.09% combined. Main console logging has no sampled
ticks. The stack scan is removed from the two hot callers by source inspection;
the lower audio-poll share supports reduced cost there. Different source
positions, timelines and tick replay prohibit a controlled percentage-speedup
claim.

The same window samples host 53.25%, decoder 8.09%, controller 12.10%, logger
13.19% and idle 13.23% on core1. Core0 is 82.04% idle and 15.11% renderer.
These are interrupted-task samples with startup/post-fault time, not exact
CPU utilization. Substantial host work and producer starvation remain.

Continuity still fails: 193920 source frames reach DMA, 1264768 underrun frames,
one core1 idle-starvation watchdog and a descriptor-order fault after a 12087 us
EOF gap. No allocation failure, ingress rejection or upstream sequence break
is recorded. The interrupted watchdog stack is inside runtime channel receive;
it is not proof that one parser call monopolized execution. Do not hide this
failure by disabling watchdog or descriptor checks.

All complete periodic records in this capture pass individual CRC/time checks.
The finite helper correctly fails runtime health, then observes a clean released
baseline return. Phone pause is independently verified. Further work must
address fair host/decoder service and the remaining polling cost; the maximum
audio/headroom and feature-parity gates remain open.
