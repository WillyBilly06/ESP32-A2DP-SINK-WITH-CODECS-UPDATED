# AVRCP playback clock — 2026-11-08

The earlier normal LHDC trace retained 9792 unfilled frames near its end, with empty decoded-input and output queues at the first gap. The phone pause timestamp and firmware gap timestamp were in separate clock domains. That evidence did not justify labeling every gap as a pause tail.

The existing AVRCP snapshot now retains status_changed_us: the low 32 bits of esp_timer_get_time observed when a validated peer reply or notification changes playback status. Duplicate status replies retain the first-change timestamp. Metadata and volume publications do not change it. Disconnect and a new control session clear it with unknown status. No host scheduling or audio policy uses this diagnostic field. In particular, AVRCP pause does not replace AVDTP suspend or stop audio.

The background logger emits a separate CRC-protected avrcp_playback record when connected/status/timestamp/generation changes. Only the latest change is retained; rapid transitions can coalesce between logger reads, so this is not a complete AVRCP event history. It reads the existing bounded atomic snapshot. A busy snapshot reader does not mark the state as logged. The producer formats no UART output, allocates no memory, and creates no task or event queue. One public SDK clock read is added only on a peer status change.

Playback status is 0 stopped, 1 playing, 2 paused, 3 forward seek, 4 reverse seek, or 255 unknown/error. The timestamp is callback observation time, not phone action, over-the-air arrival or controller receipt time. Logger receipt can be delayed independently.

First-gap event_us and status_changed_us use the same low32 microsecond domain. Compare them with signed modular subtraction only within the 2^31 us half range (about 35.8 minutes), and match the current connected peer generation/stream. This labels an observed peer status around a gap; it does not prove that an entire gap count has the same cause or establish CPU headroom.

## Validation

The actual adapter/profile packet tests pass five groups in each of the audio-output OFF/ON builds: exchange, SDP, backpressure waits, volume bursts and playback-clock lifecycle. The clock case covers duplicate interim/GetPlayStatus replies, unrelated metadata/volume publication, low32 clock wrap, disconnect and stale-channel isolation. The paired owned-runtime exchange and pinned-disconnect test also passes, including repeated status/track re-registration across two successive tracks. Test shims supply the public SDK clock signature; target compile routing uses the actual ESP-IDF esp_timer header. Both affected host harnesses now compile objects through ccache with explicit -c, then link separately.

Candidate f2062fa445c6409fdc7b63ec9df756a9ff3baf2dc680d432b90a6b522d1cb351 builds with 10433 unchanged inputs and 1036 object compilation commands. Image admission and the owned ISR-reference audit pass. Compared with normal f3890656391e, linked BSS adds 24 B, IRAM/data do not increase and flash text adds 144 B. This is diagnostic storage, not a RAM or speed improvement. Heap/tick/task samplers remain OFF. Codec, PCM-buffer, priority/affinity, security and gain configuration are unchanged.

The candidate was flashed at 921600 and fully read back at 115200; all written regions match. The old application MD5 remained unchanged, ota1 is VALID and automatic Bluedroid fallback remains disabled. Existing cold checks pass 37 configurations /173 callbacks, tracked peak 56899 B, minimum free after allocation 106104 B and stack low-water 3604 B, with no boot watchdog/backtrace/idle-starvation marker.

| Normal-image excerpt | Music | Metronome |
|---|---:|---:|
| I2S duration upper bound |12.576 s|12.492 s|
| Initial /recovered /unfilled frames |0 /0 /0|7360 /1600 /5760|
| Controls ACK/readback/restoration observed before pause |No|Yes|
| Android LE request to connected callback |8032 ms|1224 ms|
| Minimum internal free heap |20280 B|20136 B|

Both excerpts report zero media drops, decoder faults, descriptor errors, watchdog events and host/worker/snapshot/UART trace loss. The music client completed its controls after pause, so its helper done=true is not presented as a during-playback control pass. Its peer PLAYING and PAUSED callbacks were observed at 131236599 us and 140888009 us. The LE link-added security marker was 463392 us after that PAUSED callback; the later bond-resolution job took 32776 us. This places the delay before the observed reconnect key-resolution job, without assigning it entirely to the phone or controller.

The metronome completed controls during playback but retained 5760 unfilled frames. Its first unfilled snapshot at 236308408 us had 768 input frames, zero output frames and 128 frames taken; decoder intent was wait_encoded_input, renderer intent was inside_audio_render and the helper state was idle. These are bounded first-gap observations, not complete scheduler execution history.

The metronome yielded a PLAYING callback at 225106614 us but no subsequent PAUSED update in the captured/final cache, while the sole YouTube media session reported PAUSED afterward. Therefore the new clock does **not** classify this gap as a pause tail. The actual paired-runtime test passes repeated play/pause and both status/track re-registration across two tracks; that host result does not prove why the Redmagic update is missing. More protocol-level evidence is needed before changing the notification state machine. Older f3890656391e gaps are not retrospectively reclassified from these different runs.

BDK Audio was restored to the foreground, YouTube was verified paused, and the verified COM6 logger remains at 921600. No audio-gap fix, 1 Mbps headroom or complete coexistence qualification is claimed. Machine-readable [results](AVRCP_PLAYBACK_CLOCK_RESULTS_2026_11_08.json) preserve both finite outcomes.

The first build compiled but failed the unchanged 512 MiB rebuildable-output gate at 512.111 MiB. The failed receipt remains. Completed logs and host-test objects were compressed losslessly and SHA-verified; SDK objects/ELF/headers and device evidence were retained. The later callback-clock build passes at 511.444 MiB. An intermediate host-loop-millisecond timestamp design was built offline and never flashed.

Evidence: native_bt/_build/experiments/avrcp-pause-clock-1794201907315. This contains source-bound host results, linked footprint, superseded/failed build receipts and lossless archive indexes.
