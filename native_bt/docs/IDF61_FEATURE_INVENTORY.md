# Feature-parity inventory against the released 6.1 application

Reference commit: `0d85a975ec079d954f1bdd3e94d2fed70848c252`. The released
application remains available throughout development. This table records
interfaces and gates, not completion claims for the replacement.

| Feature | Baseline implementation / dependencies | Owned path and outstanding parity gate |
|---|---|---|
| Controller, Classic + BLE coexistence | `esp-idf/components/bt`, dual-mode controller on core 1; one Classic ACL and one LE connection | HCI/VHCI, queue, ACL, receive-flow and runtime modules; sustained RF/credit/backpressure and recovery qualification |
| Pairing, bonds, encryption, permissions | SDK GAP/SSP/SMP plus NVS; app controls use the resulting links | Owned Classic security, SMP/session/distribution and NVS providers; all methods, UI policy, privacy, reconnect, power-loss and mandatory response tests |
| SDP and peer services | SDK server/client handling A2DP/AVRCP service records | Owned server/client and product Audio Source discovery routing compile; automatic peer policy and complete record/peer interoperability remain open; see [discovery scope](SDP_DISCOVERY_2026_11_05.md) |
| A2DP sink lifecycle | `main/bt/a2dp_sink_native.cpp`, patched SDK AVDTP and codec layer | Owned signaling/capabilities/acceptor/sink/media manager; real negotiation, start/suspend/reconfigure/close, loss and all-codec switching qualification |
| A2DP source | Source API/codec implementation remains in the SDK; no source initialization/PCM input entrypoint was found in the application | Do not silently exclude it: owned source profile/encoding parity must be resolved explicitly against the retained SDK capability |
| SBC / AAC / aptX / HD / LL / LDAC / Opus | Patched SDK codec adapters and libraries; native wrapper publishes decoded format | Standalone codec algorithms plus owned bounded adapters/worker. Preserve CIEs, framing, precision, configuration, lifetime and all variants; target outputs and real peers remain unqualified |
| LHDC V5 | Separate user decoder, not offered by the released Bluedroid application | Explicit additional master requirement: 192 kHz / stereo / 24-bit / 1000 kbps. Owned adapter exists; full frame correctness, actual negotiated CIE, cycle/deadline and coexistence proof required |
| AVRCP controller and target | Native wrapper playback/pause/stop/next/previous, metadata, notifications and absolute volume | Owned AVCTP/AVRCP control/bearer; complete external semantics, browsing binding/providers, delay and disconnect handling remain open |
| PCM, resampling and I2S | `audio/audio_pipeline.h`, `i2s_output.h`, internal Q1.31 ring, APLL, I2S1 GPIO27/25/26; 32-bit stereo DMA | Owned audio/resampler/output provider. Preserve every sample format/rate and bounded ownership; measure timestamps, clocks, packet loss, starvation, PCM/DMA faults and output continuity |
| EQ and routing | `dsp/dsp_processor.h`, 3-band EQ, presets, bass boost, crossover/full-range, low/high swap, volume and soft clipping | Fixed-point DSP kernel and render-owned controls exist. Final integration must be integer-only and click-free, with high-amplitude/rapid-transition and real flash-stall tests |
| Persistence | `storage/nvs_settings.h`, `audio` namespace, individual setting writes/commits | Owned NVS provider retains each accepted EQ commit without coalescing. Preserve name/LED/sounds/control schemas, errors, restart and durable ownership |
| Product BLE service | UUID suffixes `9000` service, `9001` command, `9002` status, `9003` meter under `12345678-1234-1234-1234-12345678…` | Owned ATT/GATT and product control. Verify exact bytes, CCCDs, MTU/fragmentation, permissions, indication confirmation, generation cleanup and all phone flows |
| EQ/control commands | `0x01` EQ, `0x02` preset, `0x03` controls; flags 1 bass boost, 2 output swap, 4 full-range | Owned protocol/shim tests exist. Keep signed values, preset behavior, immediate persistence and readback during maximum audio load |
| Name and identity | `0x04`, canonical shared Classic/BLE name; image and BLE firmware version both `APP_PROJECT_VER` | Owned GAP rename/product storage; verify transaction failure, UTF-8/name bounds, advertising cache, reconnect and metadata identity. Do not restore the former ` BLE` suffix |
| LED controls | `0x05` full settings, `0x06` effect, `0x07` brightness; 22 user effects, speed, two colors, five gradients, Off=brightness zero | Owned IO/LED paths exist but differ visually. Preserve/reuse baseline behavior until visual/timing parity is established; no matrix currently attached |
| Meters and beat feedback | Three raw level bytes, nominal 30 ms notifications; DSP analysis, peak/beat GPIO and LED inputs | Owned analysis/meter path; verify exact interpretation, stale/low-memory policy, latency and continued BLE progress with maximum codec load |
| Sound events and storage | `audio/sound_player.h`, overlay mixer, SPIFFS; startup/pairing/connected/max-volume WAV slots and mute | Owned sound/player/mixer/storage providers; preserve WAV validation, resampling, overlay/duck/fade, event timing and cleanup; audible/flash coexistence gate remains |
| Sound commands | `0x10` mute, `0x11` delete, `0x12/13/14` upload start/data/end, sequence wrapping, worker READY/COMPLETE/FAILED | Verify capacity, ordering, accepted-vs-completed semantics, disconnect cancellation, replacement failures and accurate presence bits |
| BLE OTA | `0x20/21/22/23` begin/data/end/abort; image validation, rollback, progress/terminal states, two main slots | Owned asynchronous OTA provider with selector/restart barriers. Retain exact-length/integrity checks, cancellation, stale-peer rejection, completion delivery and safe core-0 timer reboot dispatch with host admission/affinity/failure checks; native physical fault/power-loss/OTA testing pending |
| Recovery | Separate WiFi recovery image at `0x10000`, captive portal, saved WiFi and encrypted Google Drive update | Existing source/build path retained. Do not overwrite its partition or treat local build as physical recovery proof; current user waived its hardware test |
| Hardware controls | `input/encoder_controller.h`, seesaw I2C on GPIO23/22, buttons, volume/EQ/effects/brightness, click gestures | Owned input/provider path and queue tests; preserve mappings, debounce and pairing consent. No encoders attached for physical checks |
| Status/sync/lifecycle | `0xF0` full status, `0xFF` ping, setting notifications and status/error/progress response families | Verify every command against BDK 0.7.2, post-write/reconnect state, low-memory behavior, rapid cycling and terminal message reservations |
| App-local functionality | BDK discovery, codec preferences/readback, presets/import/export, light preview, themes, sound drafts, firmware picker | Keep firmware protocol and identity compatible. App transport tests do not prove peripheral execution; combined UI/hardware regression remains required |

The exact source files and lexical review leads are in `IDF61_AUDIT.json`.
The full requirements and acceptance scenarios are in `MASTER_GOAL_2026_11_04.md`.
Nothing in this inventory authorizes retiring the baseline or declaring a
profile complete from compilation or a finite simulated exchange.

## Released reconnect policy clarification

Baseline `0d85a975ec079d954f1bdd3e94d2fed70848c252` explicitly uses
`set_auto_reconnect(false)` in `bt_audio_sink/main/main.cpp:2296`; the native
sink still persists `a2dp/last_bda`. The owning files were verified clean.
Phone-initiated reconnect and rapid codec-switch handling are active baseline
behaviors. Speaker-initiated automatic reconnect remains an explicit new
master-goal requirement and still needs complete owned-stack integration.
