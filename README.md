# ESP32 Bluetooth audio — independent host stack

The current source uses **our own Bluetooth host**, written for the original dual-core ESP32 and **ESP-IDF 6.1**. It talks directly to Espressif's dual-mode Bluetooth controller through **HCI/VHCI**.

**The default build does not use the Bluedroid or NimBLE host, and does not wrap BTstack.** Espressif's controller, FreeRTOS, hardware drivers, storage and cryptographic primitives remain in use.

This is development firmware. Playback and controls have finite hardware observations, but full codec/feature parity, sustained LHDC V5 192 kHz / 24-bit / 1 Mbps operation, worst-case RAM and long-duration coexistence are still being qualified. See the [feature matrix](native_bt/docs/FEATURE_MATRIX.md) for implementation and verification scope.

## Host architecture

| Layer | Current implementation |
|---|---|
| Controller / radio | Espressif ESP32 BTDM controller, HCI over VHCI |
| HCI, ACL, command/packet credits, fragmentation | Independent C host in [native_bt/src](native_bt/src) |
| Classic GAP, pairing/bonds, SDP, L2CAP/ERTM | Independent host state machines |
| AVDTP / A2DP, codec negotiation, AVRCP | Independent host/profile implementation |
| BLE GAP, ATT/GATT server, SMP and bond integration | Independent host with the existing application services |
| Media | Bounded codec-worker queues → integer DSP/EQ → PCM/I2S DMA → DAC |

The build explicitly enables `CONFIG_BT_CONTROLLER_ONLY` and disables `CONFIG_BT_BLUEDROID_ENABLED` and `CONFIG_BT_NIMBLE_ENABLED`. The [build-boundary audit](native_bt/tools/check_workspace.py) rejects host/reference dependencies in the generated compilation database. The [default-build receipt](native_bt/docs/DEFAULT_HOST_BUILD_2026_11_08.json) records the initial default configuration and linked-symbol checks; the checkpoints below bind subsequent images.

Some standalone SBC/AAC/aptX/LDAC/Opus codec sources remain under the bundled SDK's historical `host/bluedroid/external/` directories. Those are codec-only dependencies; the Bluedroid host, BTC/BTA profiles and its A2DP/AVRCP runtime are not built into this application. The shared fixed-point DSP source remains under `bt_audio_sink/main/dsp/`.

## Build the custom-host firmware

Use the bundled ESP-IDF 6.1 SDK and its installed ESP32 toolchain, Python environment and **ccache**, with **PowerShell 7**. No PSRAM is required. The checked layout targets an original ESP32 with 8 MB flash and PCM5102A I2S output; board/peripheral definitions remain in the project.

From the repository root:

```powershell
pwsh -NoProfile -File .\build.ps1
```

`build_flash.bat` now selects the same custom-host build. Its historical name is retained, but it is **build-only** and does not forward arbitrary ESP-IDF flashing commands. `build_flash.bat build` also selects this preset.

The default preset enables the product integration, Classic and BLE, RFCOMM, asynchronous copied PCM, integer DSP, codec-aware rendering and the current core/priority policy. It includes SBC, AAC, aptX, aptX HD, aptX LL, LDAC, Opus and LHDC V5. Configuration/compile coverage is not a claim that every live scenario is qualified.

The **pinned portable LHDC V5 C snapshot is included** in [source_snapshot](native_bt/codecs/lhdcv5/source_snapshot), with [file digests and provenance](native_bt/codecs/lhdcv5/source_snapshot/SOURCE.json). A separate decoder checkout is no longer required. The audited port generator verifies its source/header pins and fails if they differ. An explicit source override is available for reviewed decoder work:

```powershell
pwsh -NoProfile -File .\build.ps1 -LhdcSourceDir C:\path\to\reviewed-decoder
```

Inspect the preset without building or accessing hardware:

```powershell
pwsh -NoProfile -File .\build.ps1 -PrintConfiguration
```

The resulting application is `native_bt/_build/esp32/native_bt_probe.bin`. Logging defaults to **921600 baud**. Formatting and UART transmission run in the low-priority background logger; runtime producers publish bounded records. Optional heap/tick/task sampling is disabled by the default preset.

The build does not open a serial port, flash or reset a board. Deployment must use the matching partition layout and preserve any existing firmware/data that the board needs. The repository's current [admission/deployment helper](native_bt/tools/deploy_audio_trial.py) is identity-bound to the development COM6 fixture; it is not a generic flasher for another board. Do not apply its device identity/offset assumptions to a different board.

## Current validation and remaining work

The [reconnect stack checkpoint](native_bt/docs/RECONNECT_STACK_2026_11_08.md) reduces only the SMP reconnect stack from 12 KiB to 8 KiB; fresh pairing and maintenance retain 12 KiB. Comparable diagnostic excerpts measured 8,836 bytes of stack margin before and 4,736 after, with minimum free heap 10,480 → 14,596 bytes. The optional sampler is disabled in the normal build.

The normal image at that checkpoint is readback-verified and passes the existing 37 cold codec configurations / 173 callbacks. One short native YouTube/LHDC192/S24/phone-fixed900 excerpt completed BLE EQ ACK/readback and original-subscription restoration before pause, with 20,304 bytes minimum free heap. It also retained 9,792 unfilled frames near the end; that result is not presented as underrun-free. See the checkpoint for exact counts and timing limits.

The [playback-clock follow-up](native_bt/docs/AVRCP_PLAYBACK_CLOCK_2026_11_08.md) adds a timestamp of peer playback-state changes to the existing snapshot (24 bytes BSS). At that checkpoint, image `f2062fa445c6` passed the cold checks. Its music excerpt had zero underruns but an 8.032-second BLE connection delay, with controls completing after pause. The metronome completed controls during playback but retained 5,760 unfilled frames; decoded data was available at its first miss. No peer PAUSED update was observed for that excerpt, so its gap phase remains unclassified.

COM6 now runs readback-verified image `297aae153578` with a [flash-guard observer and durable CCCD cache](native_bt/docs/FLASH_GUARD_CCCD_CACHE_2026_11_09.md). The observer confirms a flash cache-off/suspended-scheduler obstruction at an I2S miss with decoded PCM waiting. The 48-byte cache avoids redundant provider reads and verified no-op writes while retaining commits before changed-value ACKs. No audio buffer, task, codec quality or core/priority policy changes. The existing 37 cold codec configurations pass; logging remains 921600 baud.

A guarded LHDC192/S24/phone-fixed900 excerpt completes all 32 BLE descriptor ACKs and independent readbacks, EQ ACK/settings readback and original-subscription restoration before pause. It still retains **2,432 unfilled frames**, with a CRC-verified pre-pause miss during a cache-off interval. This is not an underrun fix or sustained 1Mbps proof. [Asynchronous A2DP coexistence-status integration](native_bt/docs/A2DP_COEX_2026_11_09.md) remains in place through the public SDK worker, without a Bluedroid host or private controller hook. Storage-induced rendering stalls remain the immediate audio issue.

The earlier [256-frame DMA redistribution experiment](native_bt/docs/DMA_REFILL_EXPERIMENT_2026_11_08.md) preserved the audio payload budget but failed with a confirmed pre-pause miss. Its code was removed; normal 192-frame DMA geometry remains.

These finite checks do not establish sustained continuity, acoustic quality or worst-case RAM. Earlier active PCM gaps and CPU1 watchdog faults remain relevant. Full 1 Mbps headroom and all codec/feature/security/lifecycle/OTA qualification remain open.

Preservation and verification status for BLE controls, uploads, OTA, security, lifecycle, peripheral integration and the codec variants is tracked in the [feature matrix](native_bt/docs/FEATURE_MATRIX.md) and [baseline inventory](native_bt/docs/IDF61_FEATURE_INVENTORY.md). Original working paths are retained for comparison; their existence is not presented as custom-host parity.

Companion application: [BDK Audio](https://github.com/WillyBilly06/BDK-AUDIO-APP).

## Older Bluedroid releases

These releases preserve the previous firmware and are separate from the current custom-host development source:

- [Firmware 1.0.4 / ESP-IDF 6.1](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/releases/tag/v1.0.4-idf6.1)
- [Firmware 1.0.3 / ESP-IDF 5.5.2](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/releases/tag/v1.0.3-idf5.5.2)
- [Previous Bluedroid README and build instructions](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/blob/v1.0.4-idf6.1/README.md)

The `bt_audio_sink/` application remains the historical Bluedroid baseline and supplies shared DSP/product resources. It is not the application selected by the root build launcher. The owned development image has no automatic return to Bluedroid.

## Repository map

- [native_bt/esp32_probe](native_bt/esp32_probe): current ESP32 application and controller integration.
- [native_bt/src](native_bt/src), [native_bt/include](native_bt/include): independent host, profiles, audio pipeline and application modules.
- [native_bt/codecs](native_bt/codecs): codec adapters, audited LHDC port and pinned source snapshot.
- [native_bt/tests](native_bt/tests), [native_bt/tools](native_bt/tools): protocol/audio fixtures, build checks and diagnostics.
- [native_bt/README.md](native_bt/README.md): detailed, dated engineering checkpoints.
- `esp-idf/`: bundled ESP-IDF 6.1 framework/controller and standalone codec dependencies.
- `bt_audio_sink/`: preserved prior application and shared resources.

Source notices and component-specific licensing/provenance remain with the corresponding code. Reference-stack code is not linked as a host replacement.
