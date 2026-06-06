<p align="center">
  <img src="https://img.shields.io/badge/ESP32-WROOM-000000?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP32"/>
  <img src="https://img.shields.io/badge/ESP--IDF-v5.5.2-E7352C?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP-IDF"/>
  <img src="https://img.shields.io/badge/Bluetooth-A2DP-0082FC?style=for-the-badge&logo=bluetooth&logoColor=white" alt="Bluetooth"/>
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" alt="License"/>
</p>

<p align="center">
  <b>High-fidelity Bluetooth audio receiver with premium codec support</b><br>
  <sub>LDAC | aptX HD | aptX | AAC | SBC</sub>
</p>

<p align="center">
  <i>Updated version built on ESP-IDF v5.5.2 with native multi-codec decoding</i><br>
  <a href="https://github.com/WillyBilly06/esp32-a2dp-sink-with-LDAC-APTX-AAC">View original ESP-IDF 5.1.4 version</a>
</p>

---

# ESP32 A2DP Sink with LDAC, aptX, and AAC Codecs (Internal RAM Edition)

> **Branch:** `main` — runs entirely on internal SRAM. No PSRAM required.

High-quality Bluetooth audio receiver for ESP32 with native multi-codec decoding (LDAC, aptX-HD, aptX, AAC, SBC). Built on a patched ESP-IDF v5.5.2 Bluetooth stack. This branch is optimized for **internal RAM only** and works on standard ESP32-WROOM modules without external PSRAM.

For the original PSRAM-dependent version, see the [`PSRAM-only`](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/tree/PSRAM-only) branch.

## Table of Contents

- [What's Different in This Branch](#whats-different-in-this-branch)
- [Features](#features)
  - [Audio](#audio)
  - [Visual](#visual)
  - [Hardware Control](#hardware-control)
  - [Connectivity](#connectivity)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration (Default)](#pin-configuration-default)
- [Codec Support](#codec-support)
- [Building](#building)
  - [Prerequisites](#prerequisites)
  - [Build Steps](#build-steps)
- [Architecture Overview](#architecture-overview)
  - [Memory Budget (Internal RAM)](#memory-budget-internal-ram)
- [OTA Updates](#ota-updates)
  - [BLE OTA (Primary)](#ble-ota-primary)
  - [WiFi Recovery OTA (Fallback)](#wifi-recovery-ota-fallback)
  - [Security Notice](#security-notice)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [How It Works](#how-it-works)
- [Troubleshooting](#troubleshooting)
- [Compatibility](#compatibility)
- [Changes from `PSRAM-only` Branch](#changes-from-psram-only-branch)
- [Credits](#credits)
- [License](#license)

## What's Different in This Branch

| Feature | `PSRAM-only` Branch | `main` (this) |
|---|---|---|
| **Memory** | Requires PSRAM (WROVER) | Internal SRAM only (WROOM) |
| **A2DP Library** | External `ESP32-A2DP` + `arduino-audio-tools` | Native ESP-IDF A2DP/AVRCP wrapper |
| **Codecs** | All 8 codecs (incl. Opus, LC3plus) | SBC, AAC, aptX, aptX-HD, aptX-LL, LDAC |
| **3D Sound** | Stage Presence 3D enabled | **Removed** (saves ~2 KB + CPU) |
| **LED Effects** | All pre-allocated at boot | Lazy allocation (only current effect) |
| **Sound Player** | Spawns new static task per play | Persistent task (zero heap per play) |
| **Audio Pipeline** | Pool-based buffers in PSRAM | Q1.31 ring buffer in internal RAM + software ASRC |
| **DSP** | Goertzel on every sample | Analysis decimation (~4-9x less CPU at 96 kHz) |

## Features

### Audio
- **Codecs:** LDAC (96kHz/24-bit Hi-Res), aptX-HD, aptX, aptX-LL, AAC, SBC
- **Native ESP-IDF A2DP wrapper** — no external Arduino libraries, zero Arduino overhead
- **Real-time DSP:**
  - 3-band parametric EQ (bass/mid/treble shelving filters)
  - Crossover split-ear mode (LP on L channel, HP on R channel)
  - Bass boost shelf (+2 dB)
  - Volume-based bass compensation (equal-loudness contour, +3 dB at low volume)
  - Soft-clip limiter (tanh-style saturation, zero latency)
  - Division-free biquad filters using ESP32 hardware reciprocal approximations
- **Q1.31 stereo ring buffer** with software ASRC and A2DP backpressure for stable LDAC 96k streaming
- **Overlay mixer** for sound effects (startup, pairing, connected, max-volume) mixed seamlessly with BT audio

### Visual
- **16x16 WS2812B LED matrix** (256 LEDs) with SPI/DMA driver
- **24 audio-reactive effects** including spectrum analyzer, VU meter, fire, plasma, starfield, rainbow
- **Lazy effect allocation** — only the currently selected effect is allocated in RAM
- **EQ/Volume overlay display** on the LED matrix
- **Automatic demo mode** when audio is idle

### Hardware Control
- **Quad rotary encoder** support (Adafruit 5752 via I2C)
  - Volume, Bass, Mid, Treble knobs
  - LED effect selection
  - Brightness adjustment
  - Multi-click detection (play/pause/next/prev)
- **Hardware buttons** for bass boost, channel flip, bypass toggle, effect cycle
- **Beat detection LED** (GPIO-configurable)

### Connectivity
- **BLE GATT unified protocol** — single service with command/status/meter characteristics
  - Real-time 3-band level meters (30Hz / 60Hz / 100Hz)
  - Full EQ, control, name, LED settings sync
  - Sound upload and mute control
- **BLE OTA** — encrypted firmware updates via BLE using unified protocol
- **WiFi recovery mode** — hold Button 1 at boot to enter recovery partition with captive portal and encrypted streaming OTA from Google Drive

## Hardware Requirements

- **ESP32-WROOM-32** (or any ESP32 with Classic Bluetooth; PSRAM is **not required**)
- **I2S DAC** (PCM5102A, ES9018, MAX98357A, etc.)
- **16x16 WS2812B LED matrix** (optional, GPIO 4 by default)
- **Adafruit Quad Rotary Encoder Breakout** (Product 5752, optional)

## Pin Configuration (Default)

| Function      | GPIO | Configurable via `menuconfig` |
|---------------|------|-------------------------------|
| I2S DATA      | 26   | Yes                           |
| I2S BCK       | 27   | Yes                           |
| I2S LRCK      | 25   | Yes                           |
| LED Data      | 4    | Yes                           |
| Button 1      | 18   | Yes                           |
| Button 2      | 21   | Yes                           |
| Beat LED      | 5    | Yes                           |
| Encoder SDA   | 23   | Yes                           |
| Encoder SCL   | 22   | Yes                           |

## Codec Support

| Codec   | Max Bitrate | Sample Rate | Notes                         |
|---------|-------------|-------------|-------------------------------|
| LDAC    | 990 kbps    | 96 kHz      | Hi-Res, 24-bit output         |
| aptX-HD | 576 kbps    | 48 kHz      | 24-bit, Qualcomm              |
| aptX    | 352 kbps    | 48 kHz      | Low latency                   |
| aptX-LL | 352 kbps    | 48 kHz      | Ultra low latency             |
| AAC     | 256 kbps    | 48 kHz      | Apple devices; no PSRAM req   |
| SBC     | 328 kbps    | 48 kHz      | Universal fallback            |

> **Opus and LC3plus are currently disabled** — still under testing for stability on internal RAM builds.

## Building

### Prerequisites

1. ESP-IDF v5.5.2 (included as `esp-idf/` with patched Bluetooth stack + codec decoders)
2. Python 3.11+

### Build Steps

```bash
# Set up ESP-IDF environment (Windows)
cd esp-idf
.\install.bat
.\export.bat

# Build the project
cd ..\bt_audio_sink
idf.py build

# Flash to device
idf.py -p COMXX flash monitor
```

> On first flash, also flash the recovery partition so the recovery button works:
> ```bash
> idf.py -p COMXX flash
> ```

## Architecture Overview

```
Bluetooth A2DP (Core 1)
  └─> Native ESP-IDF decoder (SBC/AAC/aptX/LDAC)
      └─> Q1.31 stereo ring buffer (producer)
            └─> audio_render task (Core 0)
                  ├─> DSP (EQ / crossover / bass boost / soft clip)
                  ├─> Overlay mixer (sound effects duck + mix)
                  └─> I2S write (32-bit stereo, APLL clock)
```

### Memory Budget (Internal RAM)

| Component              | Approx. Size |
|------------------------|-------------|
| Audio ring buffer      | ~16 KB      |
| DSP output buffer      | ~2 KB       |
| Overlay mixer ring     | ~8 KB       |
| I2S DMA buffers        | ~3 KB       |
| LED framebuffer        | ~1 KB       |
| BT/BLE stack           | ~90 KB      |
| Free heap (LDAC)       | ~40 KB      |
| Free heap (AAC / SBC)  | ~50 KB      |
| Free heap (aptX / aptX-HD / aptX-LL) | ~45 KB |

## OTA Updates

### BLE OTA (Primary)
Firmware updates are sent over BLE using the unified protocol. The main app receives encrypted firmware directly via BLE and writes it to the OTA partition.

### WiFi Recovery OTA (Fallback)
If the main firmware is corrupted, hold **Button 1** during power-on to boot into the **recovery partition**:
- Creates a WiFi AP (`ESP32-Recovery-Setup`)
- Serves a captive portal web UI
- Downloads encrypted firmware from Google Drive
- Streams decryption+flash (no full-RAM download needed)

### Security Notice

**The AES-256 key in this repo is a placeholder (`0x00`).**

Before building or distributing firmware, generate your own key:

```bash
cd tools
python encrypt_firmware.py --generate-key
```

Then update BOTH files with your new key:
- `tools/encrypt_firmware.py` (Python `AES_KEY`)
- `recovery/main/recovery_main.cpp` (C++ `AES_KEY` array)

## Configuration

Use `idf.py menuconfig` → **A2DP DSP BLE Configuration**:

- I2S pins and default sample rate
- GPIO buttons and beat LED
- DSP block size and crossover frequencies
- Beat detection thresholds
- LED matrix size, GPIO, brightness, FPS
- Rotary encoder I2C pins and address
- Device name and firmware version
- Audio buffer pool count/size

## Project Structure

```
bt_audio_sink/
  main/
    main.cpp              # App entry point (BLE, A2DP, DSP, LED, encoders)
    core/                 # Alternative main.cpp (same features)
    bt/
      a2dp_sink_native.h/.cpp   # Native ESP-IDF A2DP/AVRCP wrapper
    audio/
      audio_pipeline.h    # Q1.31 ring buffer + ASRC + render loop
      i2s_output.h        # ESP-IDF v5 new I2S driver (32-bit stereo)
      overlay_mixer.h     # Sound effect duck/mix ring buffer
      sound_player.h      # Streaming WAV resampler + persistent task
    dsp/
      dsp_processor.h     # EQ, crossover, bass boost, soft clipper
      biquad.h            # Division-free biquad filters
      fast_math.h         # Hardware reciprocal / fast log
      goertzel.h          # Frequency analysis
    ble/
      ble_unified.h       # Single-service BLE GATT protocol
    led/
      led_controller.h    # Effect manager with lazy allocation
      led_effects.h       # 24 audio-reactive effects
      led_driver_spi.h    # SPI/DMA WS2812B driver
    input/
      encoder_controller.h # Quad rotary encoder via I2C
    ota/
      idf_update.h/.cpp   # ESP-IDF OTA wrapper
    storage/
      nvs_settings.h      # NVS persistence layer
    config/
      app_config.h        # Centralized Kconfig aliases
  recovery/
    main/
      recovery_main.cpp   # WiFi AP + captive portal + encrypted OTA
  tools/
    encrypt_firmware.py   # AES-256-CBC firmware encryption
  ota_releases/           # Encrypted firmware output folder
```

> **Note:** The external Arduino libraries `ESP32-A2DP` and `arduino-audio-tools` from the `PSRAM-only` branch are **not used** here. The A2DP sink is implemented natively in `main/bt/a2dp_sink_native.cpp`. The AAC decoder is **Helix** (from [arduino-libhelix](https://github.com/pschatzmann/arduino-libhelix)), patched into the ESP-IDF Bluetooth stack — it is not used as an Arduino library here.

## How It Works

1. **Bluetooth** initializes in dual mode (Classic BT + BLE) on Core 1
2. The patched ESP-IDF stack advertises all enabled codecs to the source
3. Source selects the best mutual codec and streams encoded audio
4. The **native A2DP wrapper** receives decoded PCM in the BT decoder task
5. PCM is pushed to a **Q1.31 stereo ring buffer** with A2DP backpressure
6. The **audio_render task** (Core 0) pops frames, runs DSP, mixes overlays, and writes to I2S
7. **Overlay sounds** (startup, max-volume, etc.) are streamed from SPIFFS and mixed without stopping BT audio
8. **BLE meter notifications** are throttled when internal heap is low to protect the BT stack

## Troubleshooting

### Bluetooth pairing issues on Linux
Clear the Bluetooth cache:
```bash
sudo rm -rf /var/lib/bluetooth/<adapter-mac>/cache/<device-mac>
```
Then re-pair.

### LED effects not reactive at low volume
The firmware includes automatic gain control for LED analysis. If LEDs still don't react, check the LED matrix wiring and GPIO configuration.

### AAC decoder fails to initialize
In this branch, AAC does **not** require PSRAM. If it fails, ensure your ESP32 has sufficient free internal heap (>20 KB) before A2DP connects.

### Audio stutters at LDAC 96kHz
- Make sure `CONFIG_FREERTOS_HZ=1000` is set (default in `sdkconfig.defaults`)
- Check that BLE meter notifications are not overwhelming the stack
- The ring buffer backpressure should automatically throttle the phone

## Compatibility

Tested with:
- Android phones (LDAC, aptX HD, aptX, AAC, SBC)
- Windows 11 (aptX, SBC, AAC, LDAC)
- macOS (AAC, SBC)
- iOS (AAC, SBC)
- Linux (all enabled codecs with BlueZ except LC3 and Opus)

## Changes from `PSRAM-only` Branch

- **Memory:** All allocations forced to internal SRAM (`MALLOC_CAP_INTERNAL`) — PSRAM-only used `MALLOC_CAP_SPIRAM`
- **A2DP:** Replaced external `ESP32-A2DP` + `arduino-audio-tools` with native ESP-IDF wrapper
- **Codecs:** Disabled Opus and LC3plus (Still in optimizing process)
- **3D Sound:** Removed Stage Presence 3D processor
- **DSP:** Added Goertzel/peak-meter analysis decimation to reduce CPU load at high sample rates
- **Audio Pipeline:** Switched from pool-based PSRAM buffers to Q1.31 internal RAM ring buffer with software ASRC and backpressure
- **Overlay Mixer:** Fixed duck gain wrap bug (int16→int32) for seamless sound effect ducking
- **Sound Player:** Switched from spawning a new static task per play to a persistent task with zero heap cost per play
- **LED:** Lazy effect allocation instead of pre-allocating all effects at boot
- **OTA:** Using BLE to do OTA for main code, fallback to Wifi OTA if main firmware is corrupted

## Credits

- ESP-IDF Bluetooth stack by Espressif
- Codec libraries (patched into ESP-IDF): libldac-dec, libfreeaptx-esp, Helix AAC (arduino-libhelix)
- Original ESP32-A2DP codec project by [cfint](https://github.com/cfint/ESP32-A2DP)

## License

MIT License - See LICENSE file for details.
