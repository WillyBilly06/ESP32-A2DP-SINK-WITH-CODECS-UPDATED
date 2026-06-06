# ESP32 A2DP Sink with LDAC, aptX, and AAC Codecs (Internal RAM Edition)

> **Branch:** `internal-ram-only` — runs entirely on internal SRAM. No PSRAM required.

High-quality Bluetooth audio receiver for ESP32 with native multi-codec decoding (LDAC, aptX-HD, aptX, AAC, SBC). Built on a patched ESP-IDF v5.5.2 Bluetooth stack. This branch is optimized for **internal RAM only** and works on standard ESP32-WROOM modules without external PSRAM.

For the original PSRAM-dependent version, see the `main` branch.

## What's Different in This Branch

| Feature | `main` Branch | `internal-ram-only` (this) |
|---|---|---|
| **Memory** | Requires PSRAM (WROVER) | Internal SRAM only (WROOM) |
| **A2DP Library** | External `ESP32-A2DP` + `arduino-audio-tools` | Native ESP-IDF A2DP/AVRCP wrapper |
| **Codecs** | All 8 codecs (incl. Opus, LC3plus) | SBC, AAC, aptX, aptX-HD, aptX-LL, LDAC |
| **3D Sound** | Stage Presence 3D enabled | **Removed** (saves ~2 KB + CPU) |
| **LED Effects** | All pre-allocated at boot | Lazy allocation (only current effect) |
| **Sound Player** | Spawns task per play | Persistent task + pre-allocated buffers |
| **Audio Pipeline** | Basic buffering | Q1.31 ring + software ASRC + backpressure |
| **DSP** | Standard | Division-free, analysis decimation |

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
| AAC     | 256 kbps    | 48 kHz      | Apple devices; no PSRAM req |
| SBC     | 328 kbps    | 48 kHz      | Universal fallback            |

> **Opus and LC3plus are disabled** in this branch to free internal RAM for LDAC/AAC decode.

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
idf.py set-target esp32
idf.py build

# Flash to device
idf.py -p COM3 flash monitor
```

> On first flash, also flash the recovery partition so the recovery button works:
> ```bash
> idf.py -p COM3 flash
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
| Free heap at runtime   | ~15-25 KB   |

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

> **Note:** `components/ESP32-A2DP/` and `components/arduino-audio-tools/` from the original version are **not used** in this branch. The A2DP sink is implemented natively in `main/bt/a2dp_sink_native.cpp`.

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
- Windows 11 (aptX, SBC, AAC)
- macOS (AAC, SBC)
- iOS (AAC, SBC)
- Linux (all enabled codecs with BlueZ)

## Changes from `main` Branch

- **Memory:** All allocations forced to internal SRAM (`MALLOC_CAP_INTERNAL`)
- **A2DP:** Replaced external Arduino libraries with native ESP-IDF wrapper
- **Codecs:** Disabled Opus and LC3plus to save decoder RAM
- **3D Sound:** Removed Stage Presence 3D processor
- **DSP:** Added division-free math, analysis decimation, soft clipper
- **Audio Pipeline:** New Q1.31 ring buffer with software ASRC and backpressure
- **Overlay Mixer:** Added ducking/gain ramping for seamless sound effects
- **Sound Player:** Persistent playback task + streaming resampler (zero heap per play)
- **LED:** Lazy effect allocation instead of pre-allocating all effects
- **BLE:** Unified single-service protocol replacing legacy multi-characteristic design
- **OTA:** Added BLE OTA path in addition to WiFi recovery
- **Recovery:** Recovery partition with button-boot and captive portal

## Credits

- ESP-IDF Bluetooth stack by Espressif
- Codec libraries (patched into ESP-IDF): libldac-dec, libfreeaptx-esp, FDK-AAC
- Original ESP32-A2DP codec project by [cfint](https://github.com/cfint/ESP32-A2DP)

## License

MIT License - See LICENSE file for details.
