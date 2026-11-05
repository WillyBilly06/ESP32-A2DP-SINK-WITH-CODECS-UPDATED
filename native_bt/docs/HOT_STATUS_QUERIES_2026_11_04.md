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

Target build/admission, observed phase cost and hardware comparison are pending.
No measured performance or continuous-audio improvement is claimed yet.
