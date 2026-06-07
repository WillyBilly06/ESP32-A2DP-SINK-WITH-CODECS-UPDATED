<p align="center">
  <img src="https://img.shields.io/badge/ESP32-WROOM-000000?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP32"/>
  <img src="https://img.shields.io/badge/ESP--IDF-v5.5.2-E7352C?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP-IDF"/>
  <img src="https://img.shields.io/badge/Bluetooth-A2DP-0082FC?style=for-the-badge&logo=bluetooth&logoColor=white" alt="Bluetooth"/>
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" alt="License"/>
</p>

<p align="center">
  <b>ESP32 Bluetooth A2DP sink with expanded codec support and internal-RAM operation</b><br>
  <sub>LDAC | aptX HD | aptX | aptX LL | Opus | AAC | SBC</sub>
</p>

<p align="center">
  <i>Updated ESP-IDF v5.5.2 build with a patched Bluetooth stack, native A2DP handling, and multi-codec decoding.</i><br>
  <a href="https://github.com/WillyBilly06/esp32-a2dp-sink-with-LDAC-APTX-AAC">View the original ESP-IDF 5.1.4 version</a>
</p>

---

# ESP32 A2DP Sink with LDAC, aptX, Opus, AAC, and SBC

> **Branch:** `main` - runs entirely on internal SRAM. No PSRAM is required.

This project turns a standard ESP32-WROOM module into a high-quality Bluetooth Classic A2DP audio receiver. It uses a patched ESP-IDF v5.5.2 Bluetooth stack with native multi-codec support for LDAC, aptX-HD, aptX, aptX-LL, Opus, AAC, and SBC.

The `main` branch is designed for internal RAM operation on common ESP32-WROOM boards. For the original PSRAM-dependent implementation, see the [`PSRAM-only`](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/tree/PSRAM-only) branch.

## Highlights

- Runs on ESP32-WROOM without PSRAM.
- Supports LDAC up to 96 kHz / 24-bit output.
- Supports Opus over A2DP at 48 kHz / 16-bit configuration.
- Supports aptX-HD, aptX, aptX-LL, AAC, and SBC.
- Uses a native ESP-IDF A2DP/AVRCP wrapper instead of Arduino audio libraries.
- Includes real-time EQ, crossover, bass compensation, soft clipping, and overlay sound mixing.
- Includes BLE control, metering, sound upload, and BLE OTA support.
- Includes WiFi recovery OTA for fallback firmware updates.

## Table of Contents

- [Branch Differences](#branch-differences)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Codec Support](#codec-support)
- [Building](#building)
- [Architecture Overview](#architecture-overview)
- [Memory Budget](#memory-budget)
- [OTA Updates](#ota-updates)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Runtime Flow](#runtime-flow)
- [Troubleshooting](#troubleshooting)
- [Compatibility](#compatibility)
- [Changes from PSRAM-only](#changes-from-psram-only)
- [Credits](#credits)
- [License](#license)

## Branch Differences

| Area | `PSRAM-only` branch | `main` branch |
|---|---|---|
| Memory target | ESP32 with PSRAM | ESP32 internal SRAM only |
| Recommended module | ESP32-WROVER | ESP32-WROOM |
| A2DP integration | External `ESP32-A2DP` plus `arduino-audio-tools` | Native ESP-IDF A2DP/AVRCP wrapper |
| Codecs | Original extended codec set | SBC, AAC, aptX, aptX-HD, aptX-LL, LDAC, Opus |
| LC3 / LC3plus | Present in the older codec set | Not available for Bluetooth Classic A2DP |
| 3D sound | Stage Presence 3D enabled | Removed to reduce RAM and CPU load |
| LED effects | Pre-allocated at boot | Lazy allocation for the active effect |
| Sound player | New static task per playback | Persistent task with zero heap cost per play |
| Audio buffering | Pool buffers in PSRAM | Q1.31 internal-RAM ring buffer with software ASRC |
| DSP analysis | Goertzel on every sample | Decimated analysis to reduce CPU load |

## Features

### Audio

- **Codec support:** LDAC, aptX-HD, aptX, aptX-LL, Opus, AAC, and SBC.
- **Opus configuration:** 48 kHz / 16-bit.
- **LDAC configuration:** up to 96 kHz / 24-bit output.
- **Native ESP-IDF A2DP path:** no Arduino audio runtime dependency.
- **Real-time DSP:**
  - 3-band parametric EQ for bass, mid, and treble.
  - Crossover split-ear mode with low-pass and high-pass routing.
  - Bass boost shelf and volume-based bass compensation.
  - Soft-clip limiter for zero-latency output protection.
  - Division-free biquad filters using ESP32 hardware reciprocal approximations.
- **Q1.31 stereo ring buffer** with software ASRC and A2DP backpressure.
- **Overlay mixer** for startup, pairing, connected, and max-volume sound effects.

### Visual

- 16x16 WS2812B LED matrix support with SPI/DMA output.
- 24 audio-reactive effects including spectrum analyzer, VU meter, fire, plasma, starfield, and rainbow.
- Lazy LED effect allocation to reduce internal RAM usage.
- EQ and volume overlay display on the LED matrix.
- Automatic demo mode when audio is idle.

### Hardware Control

- Adafruit 5752 quad rotary encoder support over I2C.
- Knobs for volume, bass, mid, treble, LED effect selection, and brightness.
- Multi-click support for play, pause, next, and previous.
- Hardware buttons for bass boost, channel flip, bypass, and effect cycling.
- Configurable beat detection LED.

### Connectivity

- BLE GATT unified protocol with command, status, and meter characteristics.
- Real-time 3-band level meters at selectable update rates.
- Full EQ, control, device name, LED, sound upload, and mute synchronization.
- BLE OTA for encrypted firmware updates.
- WiFi recovery mode for fallback OTA when the main firmware is not bootable.

## Hardware Requirements

- ESP32-WROOM-32 or another ESP32 variant with Bluetooth Classic support.
- I2S DAC such as PCM5102A, ES9018, MAX98357A, or equivalent.
- Optional 16x16 WS2812B LED matrix.
- Optional Adafruit Quad Rotary Encoder Breakout, product 5752.

PSRAM is not required for the `main` branch.

## Pin Configuration

| Function | GPIO | Configurable through `menuconfig` |
|---|---:|---|
| I2S DATA | 26 | Yes |
| I2S BCK | 27 | Yes |
| I2S LRCK | 25 | Yes |
| WS2812B data | 4 | Yes |
| Button 1 | 18 | Yes |
| Button 2 | 21 | Yes |
| Encoder SDA | 23 | Yes |
| Encoder SCL | 22 | Yes |

## Codec Support

| Codec | Max bitrate | Sample rate | Bit depth / output | Status |
|---|---:|---:|---|---|
| LDAC | 990 kbps | Up to 96 kHz | 24-bit output | Supported |
| aptX-HD | 576 kbps | 48 kHz | 24-bit output | Supported |
| aptX | 352 kbps | 48 kHz | 16-bit output | Supported |
| aptX-LL | 352 kbps | 48 kHz | 16-bit output | Supported |
| Opus | Codec-dependent | 48 kHz | 16-bit output | Supported |
| AAC | 256 kbps | 48 kHz | 16-bit output | Supported |
| SBC | 328 kbps | 48 kHz | 16-bit output | Supported |
| LC3 / LC3plus | N/A | N/A | N/A | Not available for Bluetooth Classic A2DP |

LC3 is not listed as an available A2DP codec because LC3 is used for Bluetooth LE Audio over BLE. This project is a Bluetooth Classic A2DP sink, so LC3 is not available in this playback path.

## Building

### Prerequisites

1. ESP-IDF v5.5.2. This repository includes the patched ESP-IDF tree in `esp-idf/`.
2. Python 3.11 or newer.
3. An ESP32 board connected over USB.

### Build Steps

```bat
cd esp-idf
install.bat
export.bat

cd ..\bt_audio_sink
idf.py build
idf.py -p COMXX flash monitor
```

Replace `COMXX` with the serial port for your ESP32.

On first flash, flash the recovery partition as well so the recovery button path is available:

```bat
idf.py -p COMXX flash
```

The repository also includes `build_flash.bat` for a Windows build-and-flash workflow.

## Architecture Overview

```text
Bluetooth Classic A2DP
  -> Patched ESP-IDF Bluetooth stack
  -> Native codec decoder
  -> Q1.31 stereo ring buffer
  -> audio_render task
  -> DSP, overlay mixer, and limiter
  -> I2S output
```

The Bluetooth decode path and render path are separated so the ESP32 can absorb Bluetooth jitter while keeping I2S output steady. Audio is converted into the internal Q1.31 processing format, processed by the DSP chain, mixed with overlay sounds, and written to the DAC through the ESP-IDF I2S driver.

## Memory Budget

| Component | Approximate size |
|---|---:|
| Audio ring buffer | 16 KB |
| DSP output buffer | 2 KB |
| Overlay mixer ring | 8 KB |
| I2S DMA buffers | 3 KB |
| LED framebuffer | 1 KB |
| BT/BLE stack | 90 KB |
| Free heap with LDAC | About 40 KB |
| Free heap with AAC / SBC | About 50 KB |
| Free heap with aptX / aptX-HD / aptX-LL | About 45 KB |

## OTA Updates

### BLE OTA

Firmware updates can be sent over BLE through the unified protocol. The main application receives encrypted firmware data and writes it to the OTA partition.

### WiFi Recovery OTA

If the main firmware is corrupted, hold **Button 1** during power-on to boot into the recovery partition. Recovery mode:

- Creates a WiFi access point named `ESP32-Recovery-Setup`.
- Serves a captive portal.
- Downloads encrypted firmware from Google Drive.
- Streams decrypt-and-flash without holding the full image in RAM.

### Security Notice

The AES-256 key in this repository is a placeholder value.

Before distributing firmware, generate a unique key:

```bash
cd tools
python encrypt_firmware.py --generate-key
```

Then update both of these files with the new key:

- `tools/encrypt_firmware.py`
- `recovery/main/recovery_main.cpp`

## Configuration

Run:

```bat
idf.py menuconfig
```

Then open **A2DP DSP BLE Configuration** to configure:

- I2S pins and default sample rate.
- GPIO buttons and beat LED.
- DSP block size and crossover frequencies.
- Beat detection thresholds.
- LED matrix size, GPIO, brightness, and frame rate.
- Rotary encoder I2C pins and address.
- Device name and firmware version.
- Audio buffer pool count and size.

## Project Structure

```text
bt_audio_sink/
  main/
    main.cpp                    App entry point
    bt/
      a2dp_sink_native.h/.cpp   Native ESP-IDF A2DP/AVRCP wrapper
    audio/
      audio_pipeline.h          Q1.31 ring buffer, ASRC, and render loop
      i2s_output.h              ESP-IDF I2S output
      overlay_mixer.h           Sound effect ducking and mixing
      sound_player.h            Streaming WAV resampler and persistent task
    dsp/
      dsp_processor.h           EQ, crossover, bass boost, and limiter
      biquad.h                  Biquad filters
      fast_math.h               Hardware reciprocal and fast math helpers
      goertzel.h                Frequency analysis
    ble/
      ble_unified.h             Single-service BLE GATT protocol
    led/
      led_controller.h          LED effect manager
      led_effects.h             Audio-reactive effects
      led_driver_spi.h          SPI/DMA WS2812B driver
    input/
      encoder_controller.h      Quad rotary encoder support
    ota/
      idf_update.h/.cpp         ESP-IDF OTA wrapper
    storage/
      nvs_settings.h            NVS persistence layer
    config/
      app_config.h              Centralized Kconfig aliases
  recovery/
    main/
      recovery_main.cpp         WiFi recovery OTA firmware
  tools/
    encrypt_firmware.py         AES-256-CBC firmware encryption helper
  ota_releases/                 Encrypted firmware output folder
esp-idf/                        Patched ESP-IDF v5.5.2 tree
build_flash.bat                 Windows build and flash helper
```

The external Arduino libraries from the `PSRAM-only` branch are not used in the `main` branch. The A2DP sink path is implemented through `main/bt/a2dp_sink_native.cpp`, and the codec integrations are patched into the ESP-IDF Bluetooth stack.

## Runtime Flow

1. Bluetooth Classic and BLE initialize.
2. The patched ESP-IDF stack advertises the enabled A2DP codecs.
3. The source device selects the best mutual codec.
4. The Bluetooth decoder produces PCM audio.
5. PCM is pushed into the Q1.31 ring buffer with backpressure.
6. The render task reads frames, applies DSP, mixes overlay sounds, and writes to I2S.
7. BLE status and meter notifications are throttled when needed to protect internal heap.

## Troubleshooting

### Bluetooth pairing issues on Linux

Clear the Bluetooth cache and pair again:

```bash
sudo rm -rf /var/lib/bluetooth/<adapter-mac>/cache/<device-mac>
```

### LED effects are not reactive at low volume

The firmware includes automatic gain control for LED analysis. If the LEDs still do not react, check the LED matrix wiring, power supply, and configured GPIO.

### AAC decoder fails to initialize

AAC does not require PSRAM in this branch. If initialization fails, check that the ESP32 still has sufficient free internal heap before A2DP connects.

### Audio stutters at LDAC 96 kHz

- Confirm that `CONFIG_FREERTOS_HZ=1000` is set.
- Reduce BLE meter update rate if needed.
- Check that the source device is not forcing an unstable radio condition.
- Keep the ESP32 close to the source device during testing.

## Compatibility

Expected codec behavior by common source platform:

| Platform | Expected codecs |
|---|---|
| Android | LDAC, aptX-HD, aptX, AAC, SBC, and Opus where source support exists |
| Windows 11 | aptX, AAC, SBC, LDAC where source support exists |
| macOS | AAC and SBC |
| iOS | AAC and SBC |
| Linux / BlueZ | Depends on BlueZ build and codec plugins |

LC3 is not expected on any Bluetooth Classic A2DP source because LC3 is part of Bluetooth LE Audio, not Classic A2DP.

## Changes from PSRAM-only

- Reworked memory allocation for internal SRAM operation.
- Replaced external Arduino audio libraries with a native ESP-IDF A2DP/AVRCP path.
- Added Opus support with 48 kHz / 16-bit configuration.
- Removed LC3 / LC3plus from the Classic A2DP codec list because LC3 is BLE Audio only.
- Removed Stage Presence 3D processing to reduce CPU and RAM pressure.
- Added analysis decimation to reduce DSP load at high sample rates.
- Switched the audio pipeline from PSRAM pool buffers to an internal-RAM Q1.31 ring buffer.
- Added software ASRC and A2DP backpressure.
- Fixed overlay sound ducking and mixing behavior.
- Switched sound playback to a persistent task.
- Changed LED effects to lazy allocation.
- Added BLE OTA for the main firmware and WiFi recovery OTA as a fallback path.

## Credits

- Espressif for ESP-IDF and the Bluetooth stack.
- Codec library authors for libldac-dec, libfreeaptx-esp, Opus, and Helix AAC.
- Original ESP32-A2DP codec project by [cfint](https://github.com/cfint/ESP32-A2DP).

## License

MIT License. See the license file for details.
