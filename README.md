<p align="center">
  <img src="https://img.shields.io/badge/ESP32-WROOM-000000?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP32"/>
  <img src="https://img.shields.io/badge/ESP--IDF-v6.1-E7352C?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP-IDF"/>
  <img src="https://img.shields.io/badge/Bluetooth-A2DP-0082FC?style=for-the-badge&logo=bluetooth&logoColor=white" alt="Bluetooth"/>
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" alt="License"/>
</p>

<p align="center">
  <b>ESP32 Bluetooth A2DP sink with an independent host stack and expanded codec support</b><br>
  <sub>LHDC V5 | LDAC | aptX HD | aptX | aptX LL | Opus | AAC | SBC</sub>
</p>

<p align="center">
  <i>ESP-IDF v6.1 build with an independently written Bluetooth host on Espressif's dual-mode controller over HCI/VHCI.</i><br>
  <a href="https://github.com/WillyBilly06/esp32-a2dp-sink-with-LDAC-APTX-AAC">View the original ESP-IDF 5.1.4 version</a>
</p>

---

# ESP32 A2DP Sink with LHDC V5, LDAC, aptX, Opus, AAC, and SBC

> **Branch:** `main` - independent Bluetooth host, development firmware. Runs on internal SRAM. No PSRAM is required.

This project turns an original dual-core ESP32 into a Bluetooth Classic A2DP audio receiver. The `main` branch uses its own Bluetooth host, written in C for ESP-IDF 6.1, which talks directly to Espressif's BTDM controller through HCI over VHCI. The Bluedroid and NimBLE hosts are not built, and BTstack is not wrapped.

Espressif's controller, FreeRTOS, hardware drivers, storage and cryptographic primitives are still used. Everything above HCI - ACL, L2CAP, SDP, AVDTP/A2DP, AVRCP, RFCOMM, ATT/GATT and SMP - is implemented in [`native_bt/`](native_bt).

> **Status:** this is development firmware. Playback and controls have been observed on hardware, but sustained continuity, BLE/flash coexistence, worst-case RAM and full feature parity with the Bluedroid releases are still being qualified. See [Validation Status](#validation-status). For a released Bluedroid build, use one of the [older releases](#branch-differences).

## Highlights

- Independent Bluetooth host on the ESP32 BTDM controller. No Bluedroid, NimBLE or BTstack host.
- Supports LHDC V5 up to 192 kHz / 24-bit, from an included, pinned decoder snapshot.
- Supports LDAC up to 96 kHz.
- Supports aptX HD, aptX, aptX LL, Android Opus, Opus 05, AAC, and SBC.
- Integer-only fixed-point DSP: 3-band EQ, bass boost, crossover, channel flip, bypass and volume ramps.
- Bounded codec-worker queues, a Q1.31 internal-RAM PCM ring and I2S DMA output.
- BLE product service for EQ, controls, metering, LED settings, sound upload and OTA.
- Optional 16x16 LED matrix, quad rotary encoder and buttons.
- One-command PowerShell build with ccache. The build never opens a serial port.

## Table of Contents

- [Branch Differences](#branch-differences)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Codec Support](#codec-support)
- [Building](#building)
- [Architecture Overview](#architecture-overview)
- [Memory and Flash Layout](#memory-and-flash-layout)
- [OTA Updates](#ota-updates)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Runtime Flow](#runtime-flow)
- [Validation Status](#validation-status)
- [Troubleshooting](#troubleshooting)
- [Compatibility](#compatibility)
- [Changes from the Bluedroid Releases](#changes-from-the-bluedroid-releases)
- [Credits](#credits)
- [License](#license)

## Branch Differences

| Area | Bluedroid releases (v1.0.x) | `main` branch |
|---|---|---|
| Bluetooth host | Patched ESP-IDF Bluedroid | Independent C host over HCI/VHCI |
| ESP-IDF | 5.5.2 (v1.0.3) or 6.1 (v1.0.4) | 6.1 |
| Application | `bt_audio_sink/` | `native_bt/esp32_probe/` |
| Codecs | SBC, AAC, aptX, aptX HD, aptX LL, LDAC, Opus | Same set plus LHDC V5 |
| Highest sample rate | 96 kHz (LDAC) | 192 kHz (LHDC V5) |
| Build | `idf.py build` / `idf.py flash` | `build.ps1` (PowerShell 7), build-only |
| Console | ESP-IDF default | 921600 baud, background logger |
| Maturity | Released firmware | Development firmware |

Released Bluedroid firmware:

- [Firmware 1.0.4 / ESP-IDF 6.1](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/releases/tag/v1.0.4-idf6.1)
- [Firmware 1.0.3 / ESP-IDF 5.5.2](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/releases/tag/v1.0.3-idf5.5.2)
- [Bluedroid README and build instructions](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/blob/v1.0.4-idf6.1/README.md)

Other branches:

- [`second-i2s-wm8805-spdif`](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/tree/second-i2s-wm8805-spdif) - Bluedroid build with an optional WM8805 secondary I2S / S/PDIF output.
- [`PSRAM-only`](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/tree/PSRAM-only) - original PSRAM-dependent implementation.

## Features

### Audio

- **Codec support:** LHDC V5, LDAC, aptX HD, aptX, aptX LL, Android Opus, Opus 05, AAC, and SBC.
- **LHDC V5 configuration:** 44.1, 48, 96 and 192 kHz at 16 or 24 bits.
- **LDAC configuration:** 44.1, 48, 88.2 and 96 kHz.
- **Codec-aware rendering:** bounded per-codec worker queues, with priming and queue depth tuned for high-rate LHDC.
- **Integer fixed-point DSP** (no heap allocation, no floating point):
  - 3-band EQ for bass, mid, and treble.
  - Bass boost shelf and volume-based bass compensation.
  - Crossover with low-pass and high-pass routing.
  - Channel flip, bypass, and ramped volume changes.
  - Block processing with IRAM placement for the hot path.
- **Q1.31 stereo PCM ring** in internal SRAM feeding I2S DMA.
- **Overlay sounds** for startup, connected, pairing, and max volume, with ducking and fades.
- **WAV upload, mute and delete** for overlay sounds over BLE.

### Visual

- 16x16 WS2812B LED matrix over SPI.
- 22 integer audio-reactive effects with meter and beat visualization.
- Lazy allocation of per-effect particle state.
- Pairing and volume overlays on the LED matrix.

### Hardware Control

- Seesaw-based quad rotary encoder over I2C.
- Per-detent EQ, volume with AVRCP sync, and relative LED effect and brightness control.
- Hardware buttons on GPIO 18, 21 and 19.
- Pairing gesture on the BOOT button (GPIO 0).

### Connectivity

- **Classic:** GAP, Secure Simple Pairing, bonds stored in NVS, SDP server, L2CAP Basic and ERTM.
- **A2DP sink:** AVDTP signaling, codec negotiation and ranking, delay reporting.
- **AVRCP:** control target/controller subset with metadata, absolute volume and pass-through.
- **RFCOMM / SPP:** single data channel, included in the default build.
- **BLE:** GAP, ATT/GATT server, SMP pairing and bonds, durable CCCD storage.
- **Product BLE service** (`12345678-1234-1234-1234-123456789000`) with command, status and meter characteristics.
- **BLE OTA** with staged write, readback and guarded restart.
- Companion Android app: [BDK Audio](https://github.com/WillyBilly06/BDK-AUDIO-APP).

## Hardware Requirements

- Original dual-core ESP32 with Bluetooth Classic, such as ESP32-WROOM-32.
- 8 MB flash. The included partition table is laid out for 8 MB.
- I2S DAC such as PCM5102A, ES9018, MAX98357A, or equivalent.
- Optional 16x16 WS2812B LED matrix.
- Optional Seesaw quad rotary encoder breakout and push buttons.

PSRAM is not required. LE Audio, LC3, LE 2M PHY and isochronous channels are outside the original ESP32 controller's capabilities.

## Pin Configuration

| Function | GPIO | Configurable through `menuconfig` |
|---|---:|---|
| I2S DATA | 26 | Yes |
| I2S BCK | 27 | Yes |
| I2S LRCK | 25 | Yes |
| WS2812B data (SPI2) | 4 | No |
| Button 1 | 18 | No |
| Button 2 | 21 | No |
| Effect button | 19 | No |
| Pairing (BOOT) | 0 | No |
| Encoder SDA (I2C1) | 23 | No |
| Encoder SCL (I2C1) | 22 | No |
| Console UART TX / RX | 1 / 3 | No |

The encoder is probed at I2C address `0x36`, then `0x07`, with Seesaw hardware and product ID checks. Check your board's actual wiring before enabling the product I/O.

## Codec Support

| Codec | Max bitrate | Sample rate | Bit depth | Status |
|---|---:|---:|---|---|
| LHDC V5 | 1000 kbps | Up to 192 kHz | 16 / 24-bit | In default build |
| LDAC | 990 kbps | Up to 96 kHz | Up to 24-bit | In default build |
| aptX HD | 576 kbps | 44.1 / 48 kHz | 24-bit | In default build |
| aptX | 352 kbps | 44.1 / 48 kHz | 16-bit | In default build |
| aptX LL | 352 kbps | 44.1 / 48 kHz | 16-bit | In default build |
| Android Opus / Opus 05 | Codec-dependent | 48 kHz | 16-bit | In default build |
| AAC-LC | Negotiated | 44.1 / 48 kHz | 16-bit | In default build |
| SBC | 328 kbps | Up to 48 kHz | 16-bit | In default build |
| LC3 / LC3plus | N/A | N/A | N/A | Not available for Bluetooth Classic A2DP |

Every codec is rendered through the Q1.31 pipeline into 32-bit I2S slots. All 37 cold codec configurations pass the on-device decode check. Live playback qualification for each codec is tracked in the [feature matrix](native_bt/docs/FEATURE_MATRIX.md). LHDC 32-bit has lifecycle coverage only and is not advertised.

LC3 is used for Bluetooth LE Audio over BLE. This project is a Bluetooth Classic A2DP sink on a controller without LE Audio support, so LC3 is not available in this playback path.

## Building

### Prerequisites

1. Windows with **PowerShell 7** (`pwsh`).
2. The bundled ESP-IDF v6.1 in `esp-idf/`, with its ESP32 toolchain, Python environment and ccache installed.
3. An ESP32 board for deployment. The build itself does not need one.

### Build Steps

Install the ESP-IDF tools once:

```powershell
cd esp-idf
.\install.ps1 esp32
cd ..
```

Then build from the repository root:

```powershell
pwsh -NoProfile -File .\build.ps1
```

The firmware is written to `native_bt/_build/esp32/native_bt_probe.bin`.

Useful options:

```powershell
# Print the build preset without building or touching hardware
pwsh -NoProfile -File .\build.ps1 -PrintConfiguration

# Build with a reviewed LHDC decoder checkout instead of the bundled snapshot
pwsh -NoProfile -File .\build.ps1 -LhdcSourceDir C:\path\to\reviewed-decoder

# Change the console baud rate (115200, 230400, 460800 or 921600)
pwsh -NoProfile -File .\build.ps1 -ConsoleBaud 115200
```

`build_flash.bat` runs the same build. Despite its name it is **build-only** and does not flash.

### Flashing

The build never opens a serial port, flashes or resets a board. Flash the image with a partition layout that matches [`partitions.csv`](native_bt/esp32_probe/partitions.csv), and keep any existing NVS and SPIFFS data your board needs. The deployment helper in `native_bt/tools/` is bound to the development board's identity and is not a generic flasher.

## Architecture Overview

```text
Bluetooth Classic A2DP source
  -> Espressif BTDM controller (HCI over VHCI)
  -> Independent host: HCI / ACL -> L2CAP -> AVDTP / A2DP sink
  -> Bounded codec-worker queue
  -> Codec decoder (LHDC V5, LDAC, aptX, Opus, AAC, SBC)
  -> Integer DSP, EQ and overlay mixer
  -> Q1.31 PCM ring
  -> I2S DMA -> DAC
```

| Layer | Implementation |
|---|---|
| Controller / radio | Espressif ESP32 BTDM controller, HCI over VHCI |
| HCI, ACL, credits, fragmentation | Independent C host in [`native_bt/src`](native_bt/src) |
| Classic GAP, pairing/bonds, SDP, L2CAP/ERTM | Independent host state machines |
| AVDTP / A2DP, codec negotiation, AVRCP | Independent host/profile implementation |
| BLE GAP, ATT/GATT server, SMP | Independent host with the product services |
| Media | Codec-worker queues -> integer DSP/EQ -> PCM/I2S DMA -> DAC |

The host owner loop does not decode audio. Encoded media is copied to the decoder worker, storage runs on background workers, the render task owns the DSP state, and the I2S callback only copies ready samples. The SDK configuration reserves CPU0 for codec execution and keeps the controller and background services on CPU1.

The build enables `CONFIG_BT_CONTROLLER_ONLY` and disables Bluedroid and NimBLE. A [build-boundary audit](native_bt/tools/check_workspace.py) rejects host or reference-stack dependencies in the compilation database. Some standalone codec sources remain under the SDK's `host/bluedroid/external/` directories; only the codecs are compiled, not the Bluedroid host.

## Memory and Flash Layout

The firmware runs from internal SRAM only. Stream storage is allocated when a stream starts and released when it closes. Minimum free heap is recorded per checkpoint in [`native_bt/docs/`](native_bt/docs); worst-case RAM is not yet qualified.

| Partition | Type | Offset | Size |
|---|---|---:|---:|
| `nvs` | data / nvs | `0x9000` | 24 KB |
| `phy_init` | data / phy | `0xF000` | 4 KB |
| `recovery` | app / ota_2 | `0x10000` | 1 MB |
| `ota_0` | app / ota_0 | `0x110000` | 2.5 MB |
| `ota_1` | app / ota_1 | `0x390000` | 2.5 MB |
| `otadata` | data / ota | `0x610000` | 8 KB |
| `spiffs` | data / spiffs | `0x612000` | 1.93 MB |

## OTA Updates

### BLE OTA

Firmware updates are sent over the product BLE service. The update path checks the trusted control identity, writes the image to the inactive OTA slot in stages with readback, and only then switches the boot selector and restarts. Bootloader app rollback is enabled.

### Recovery Slot

The partition table keeps the `recovery` slot used by the Bluedroid releases. Its WiFi recovery firmware source is in `bt_audio_sink/recovery/`.

### Security Notice

The AES-256 key in this repository is a placeholder value. Before distributing firmware, generate a unique key:

```bash
cd bt_audio_sink/tools
python encrypt_firmware.py --generate-key
```

Then update both of these files with the new key:

- `bt_audio_sink/tools/encrypt_firmware.py`
- `bt_audio_sink/recovery/main/recovery_main.cpp`

## Configuration

The default preset lives in [`build.ps1`](build.ps1). It enables the product integration, Classic and BLE, RFCOMM, all codecs, asynchronous copied PCM, integer block DSP, codec-aware rendering and the current core/priority policy. Optional heap, tick and task profilers are off.

I2S pins are set in **Native Bluetooth development output** in [`native_bt/esp32_probe/main/Kconfig.projbuild`](native_bt/esp32_probe/main/Kconfig.projbuild):

- `CONFIG_NATIVE_AUDIO_BCLK` (default 27)
- `CONFIG_NATIVE_AUDIO_LRCK` (default 25)
- `CONFIG_NATIVE_AUDIO_DATA` (default 26)

SDK defaults, core pinning and the console UART are in [`native_bt/esp32_probe/sdkconfig.defaults`](native_bt/esp32_probe/sdkconfig.defaults).

## Project Structure

```text
native_bt/
  esp32_probe/                  Current ESP32 application and controller integration
    main/                       App, audio, AVRCP, GATT, SMP, product I/O glue
    partitions.csv              8 MB partition table
    sdkconfig.defaults          SDK defaults
  src/, include/                Independent host, profiles and product modules
    dbt_hci / dbt_acl           HCI and ACL transport
    dbt_l2cap / dbt_ertm        L2CAP Basic and ERTM
    dbt_sdp                     SDP server and client
    dbt_avdtp / dbt_a2dp_*      AVDTP and A2DP sink negotiation
    dbt_avctp / dbt_avrcp       AVRCP control
    dbt_rfcomm_*                RFCOMM / SPP
    dbt_att / dbt_smp_*         ATT/GATT and SMP
    dbt_led / dbt_input         LED matrix and encoder/buttons
    dbt_sound / dbt_ota         Overlay sounds and OTA
  codecs/                       SBC, AAC, aptX, LDAC, Opus and LHDC V5 adapters
    lhdcv5/source_snapshot/     Pinned LHDC V5 decoder source with digests
  tests/, tools/                Host tests, build checks and diagnostics
  docs/                         Feature matrix and dated engineering checkpoints
  README.md                     Detailed engineering log
bt_audio_sink/                  Preserved Bluedroid application and shared DSP
  main/dsp/                     Fixed-point DSP shared with the native build
  recovery/                     WiFi recovery firmware
  tools/encrypt_firmware.py     AES-256-CBC firmware encryption helper
docs/                           ESP-IDF 6.1 migration and validation records
tools/                          Release packaging and feature-contract checks
esp-idf/                        Bundled ESP-IDF v6.1
build.ps1                       Default build entry point
build_flash.bat                 Windows build launcher (build-only)
```

## Runtime Flow

1. The controller starts in BTDM mode and the host takes over HCI through VHCI.
2. Bonds and settings load from NVS; the product service starts with the saved EQ and controls.
3. The host advertises its SDP records and A2DP sink endpoints for each enabled codec.
4. The source connects, pairs or reconnects, and the best mutual codec is negotiated.
5. Encoded media is copied into the bounded queue for the codec worker.
6. The decoder produces PCM; the render task applies DSP, mixes overlay sounds and fills the PCM ring.
7. The I2S callback copies ready samples to DMA.
8. BLE commands run on bounded workers. EQ changes commit to NVS immediately.

## Validation Status

Verified so far:

- All 37 cold codec configurations (173 callbacks) pass on the device.
- Readback-verified images have played LHDC V5 192 kHz / 24-bit excerpts with BLE control traffic.
- Actual-source sanitizer tests and host protocol tests pass.

Still open:

- Sustained, gap-free playback while BLE settings are written to flash.
- Full 1 Mbps LHDC headroom and worst-case RAM.
- Full codec, security, lifecycle, OTA and product parity with the Bluedroid releases.

The [feature matrix](native_bt/docs/FEATURE_MATRIX.md) and [`native_bt/README.md`](native_bt/README.md) record every checkpoint, including failed runs.

## Troubleshooting

### `build.ps1` will not start

The script requires PowerShell 7. Run it with `pwsh`, not Windows PowerShell 5.1.

### LHDC source error during build

The LHDC source directory must contain `decoder/lhdc_dec.c`. There is no decoder fallback. Leave `-LhdcSourceDir` unset to use the bundled snapshot.

### No serial output

The console runs at **921600 baud** by default. Set your monitor to match, or rebuild with `-ConsoleBaud 115200`.

### Bluetooth pairing issues on Linux

Clear the Bluetooth cache and pair again:

```bash
sudo rm -rf /var/lib/bluetooth/<adapter-mac>/cache/<device-mac>
```

### Short audio gaps when changing settings

Flash writes pause the cache while audio is playing, and the PCM buffer does not always cover that interval. This is a known open issue on `main`. Avoid rapid setting changes during critical listening.

## Compatibility

Expected codec behavior by common source platform:

| Platform | Expected codecs |
|---|---|
| Android | LHDC V5, LDAC, aptX HD, aptX, AAC, SBC, and Opus where source support exists |
| Windows 11 | aptX, AAC, SBC, LDAC where source support exists |
| macOS | AAC and SBC |
| iOS | AAC and SBC |
| Linux / BlueZ | Depends on BlueZ build and codec plugins, including Opus 05 |

LC3 is not expected on any Bluetooth Classic A2DP source because LC3 is part of Bluetooth LE Audio, not Classic A2DP.

## Changes from the Bluedroid Releases

- Replaced the Bluedroid host with an independent C host over HCI/VHCI.
- Added LHDC V5 up to 192 kHz / 24-bit, with a pinned, digest-checked decoder snapshot.
- Added Opus 05 alongside Android Opus.
- Moved codec decoding to bounded worker queues with codec-aware rendering.
- Switched DSP to integer block processing with IRAM placement.
- Reserved CPU0 for codec execution, with the controller and background services on CPU1.
- Added RFCOMM / SPP and L2CAP ERTM to the host.
- Added a durable CCCD cache that shares the EQ persistence worker.
- Replaced `idf.py` build steps with a single build-only `build.ps1` preset.
- Raised the console to 921600 baud with a background logger.

## Credits

- Espressif for ESP-IDF and the ESP32 Bluetooth controller.
- Codec library authors for libldac-dec, libfreeaptx-esp, Opus, and Helix AAC.
- LHDC V5 decoder from [LHDC-V5-Decoder](https://github.com/WillyBilly06/LHDC-V5-Decoder).
- Original ESP32-A2DP codec project by [cfint](https://github.com/cfint/ESP32-A2DP).

## License

MIT License. Bundled ESP-IDF, codec libraries and the LHDC V5 snapshot keep their own licenses and notices.
