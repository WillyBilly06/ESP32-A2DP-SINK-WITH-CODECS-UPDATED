# ESP-IDF 6.1 migration and BDK feature validation

This record distinguishes source preservation, local builds, app tests and device observations.

## SDK and scope

- Previous `main`: `f4eb12517438a486af9e33987114c4823af40aca`, patched ESP-IDF 5.5.2.
- New SDK: Espressif stable [v6.1](https://github.com/espressif/esp-idf/releases/tag/v6.1), commit `fff9895c82d744c7237be8847347bdd1b07c6643`.
- Exact upstream submodule revisions: [source manifest](esp-idf-v6.1-source.json). Sources/libraries are vendored as ordinary files; no SDK Git metadata is required to build this repository.
- Target: original ESP32, internal RAM, 8 MB flash. Existing partition offsets and NVS/SPIFFS layouts are retained.
- Bluetooth remains Bluedroid with the project's SBC, AAC, aptX, aptX-HD, aptX-LL, LDAC and Opus integrations. This does not add LHDC V5 or LE Audio playback to the original ESP32.
- The separate, uncommitted `development` checkout was preserved. This migration starts from remote `main` and does not publish the independent host-stack work.

## Changes

- Updated the complete SDK and its matching controller/WiFi/PHY and other dependencies, rather than changing version strings in the 5.5 tree.
- Carried forward the codec libraries, negotiation, decoding, memory-pressure and backpressure extensions. Integrated the 6.1 A2DP source metadata and typed codec callbacks, retaining upstream AVRCP handle bounds checks and L2CAP malformed-option checks.
- Moved custom codec build inputs to `esp-idf/components/bt/project_codecs.cmake` alongside the refactored upstream build.
- Replaced the removed Classic name setter, removed the legacy I2S include and port cast, and declared driver dependencies explicitly for both applications.
- Replaced removed Mbed TLS MD5 calls with ESP ROM MD5, and recovery AES-CBC calls with Espressif's AES API. Firmware containers and BLE command formats remain unchanged. AES failures now fail the update.
- Kept BLE cached EQ, control, name and LED status current after phone writes, including writes that intentionally suppress echo notifications. Reconnect/status requests therefore report the applied settings.
- Updated tracked SDK configurations, documented the separate recovery flash step, made the Windows helper portable, and enabled ccache in the build commands.

## Feature inventory and evidence

App reference: [BDK Audio](https://github.com/WillyBilly06/BDK-AUDIO-APP), commit `b11c84ee2a57c4baae64c040d0f53fa97e665cbf`; installed Android app 0.7.2 / version code 2030 on Redmagic NX809J.

| Feature | Source/configuration | Device evidence |
|---|---|---|
| Classic audio / reconnect / AVRCP volume and transport | Retained | Connected on Redmagic; volume/transport exercised during playback and OTA |
| SBC, AAC, aptX, aptX-HD, aptX-LL, LDAC, Opus | All compiled and enabled | Live switches: SBC, AAC, aptX, aptX-HD and LDAC passed with nonzero meters; LDAC 96 kHz also passed. Phone does not expose aptX-LL/Opus. |
| Bass/mid/treble EQ, presets and saved profiles | Commands and DSP retained | Device GATT checks passed; app unit tests passed |
| Bass boost, full-range stereo and output swap | Flag bits and DSP retained | Device GATT checks passed; app unit tests passed |
| Live meters and reactive previews | Meter UUID/payload and analysis retained | Nonzero live meters and controls passed during codec switching |
| All 22 user LED effects, brightness/Off, speed, two colors and 5 gradients | Renderer, controls and persistence retained | Device GATT checks passed; app unit tests passed; physical pixels require a connected matrix |
| Notification mute; startup/pairing/connected/max-volume WAV upload/delete | Four sound slots and transfer protocol retained | All four WAV slots uploaded/deleted during live A2DP; no analog measurement |
| Device rename, firmware identity, appearance and connection UI | Wire identity and app behavior retained | Device GATT checks passed; app unit tests passed |
| BLE OTA begin/data/end/abort and progress | Retained; MD5 backend migrated | Abort/retry, full transfer, COMPLETE, core-0 reboot, new-slot boot and version verified; both OTA slots exercised |
| WiFi recovery portal, credentials and encrypted OTA | Separate recovery image built; AES backend migrated | Build/fit verified; hardware run omitted at user request |
| Rotary encoders, buttons and beat GPIO | Enabled; source retained | Requires the corresponding connected hardware and physical actuation |
| Demo, profile save/import/export, sound drafts, app information/themes | App implementation unchanged | Relevant app unit tests passed; connected Sound/Light/Speaker and codec UI inspected |

Local validation: main application and WiFi recovery compile/link with ESP-IDF 6.1 / GCC 15.2 and ccache. The main image fits each 0x280000 OTA application slot; recovery fits its 0x100000 slot. ESP-IDF may warn that the main image does not fit the smaller recovery slot; it is flashed to `ota_0`, and the separate recovery binary is flashed at `0x10000`.

BDK tests: **174 passed** across protocol, session/brightness, sound transfer packets/controller, OTA packets/controller, preset import and profile mutations. These tests use app-side transports and do not constitute firmware/hardware validation.

Run the source/configuration compatibility check:

```powershell
python tools/check_feature_contract.py --baseline f4eb12517438a486af9e33987114c4823af40aca --bdk-app <BDK-AUDIO-APP-checkout>
```

## Reproducible Android peripheral checks

`tools/android/FeatureCheck.java` is a separate, temporary instrumentation app, not part of BDK Audio or the speaker firmware. Build it with `tools/android/build_check.ps1` using Android SDK 36 and a JDK. It uses normal Bluetooth permissions and requires an explicit speaker address.

`controls` checks characteristic discovery/subscriptions, ping, EQ endpoints on all three bands, all control flag combinations, all user LED effects/gradients, brightness/Off, name and notification mute, with status readback and restoration of the initial settings. `sounds` replaces/deletes all four sound slots and requires a restorable SPIFFS backup first. `ota` writes `files/firmware.bin`, tests abort, transfers the image and requires separate boot verification. Mode `status` only requests state. Remove the temporary test app after a finite validation run.

These checks establish protocol state and transfer outcomes. They cannot by themselves establish acoustic quality, exact physical LED output, encoder operation, every phone codec, or long-duration RF reliability.

## Final device results

[Machine-readable validation](ESP_IDF_6_1_VALIDATION.json) records the passing checks. The temporary Android test app was used with normal Bluetooth/companion permissions, and the original LDAC 96 kHz / 32-bit stereo phone tuple was restored. No hidden-API policy exemptions were used.

Additional defects found and fixed by device tests: sound presence is published before SOUND_COMPLETE; full LED packets persist the selected effect; OTA abort closes the IDF update handle and resumes audio; short/incomplete OTA images fail; successful OTA sends completion before a timer reboots on core 0. The 5.5-era CPU startup override was removed in favor of the upstream 6.1 startup sequence.

The release firmware is `1.0.4-idf6.1`. Image metadata and BLE now use the same ESP-IDF project-version setting so BDK's post-update version check can match. The previous custom FIRMWARE_VERSION setting is migrated by sdkconfig.rename.
