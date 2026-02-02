<p align="center">
  <img src="https://img.shields.io/badge/ESP32-WROVER-000000?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP32"/>
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

## Table of Contents

- [Features](#features)
- [Supported Codecs](#supported-codecs)
- [Hardware Requirements](#hardware-requirements)
- [Repository Structure](#repository-structure)
- [Building from Source](#building-from-source)
- [OTA Updates](#ota-updates)
- [BLE GATT Services](#ble-gatt-services)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)
- [License](#license)

---

## Features

### Audio Processing

- Hi-Res LDAC streaming up to 96kHz/24-bit
- aptX HD/aptX/aptX-LL for low latency
- AAC for Apple device compatibility
- LC3plus and Opus codec support
- Real-time DSP processing with optimized biquad filters
- 3-band parametric EQ (bass, mid, treble)
- Stage Presence 3D audio enhancement with bass centering and stereo widening
- Division-free DSP using fast reciprocal approximations for ESP32 optimization

### Visual Feedback

- 16x16 LED Matrix with SPI/DMA driver
- 24 audio-reactive LED effects including spectrum analyzer, VU meter, fire, plasma, starfield, rainbow patterns, and more
- Audio AGC (Automatic Gain Control) with fixed floor normalization for LED reactivity at all volume levels
- Volume/EQ overlay display
- Pairing mode LED animation
- Startup animation with fixed brightness

### Hardware Control

- Quad Rotary Encoder support (Adafruit 5752)
- Volume, Bass, Mid, Treble controls
- LED effect selection via encoder
- Brightness adjustment mode
- Multi-click detection (play/pause/next/prev)

### Connectivity

- BLE GATT remote control
- Real-time level meters
- OTA firmware updates via BLE or Android app
- Encrypted OTA with AES-256-CBC
- WiFi OTA Recovery system

---

## Supported Codecs

| Codec | Bitrate | Sample Rate | Latency | Use Case |
|:------|:-------:|:-----------:|:-------:|:---------|
| LDAC | 990 kbps | 96 kHz | ~200ms | Hi-Res listening |
| aptX HD | 576 kbps | 48 kHz | ~150ms | High quality |
| aptX | 352 kbps | 48 kHz | ~120ms | CD quality |
| aptX-LL | 352 kbps | 48 kHz | ~40ms | Gaming/Video |
| AAC | 256 kbps | 44.1 kHz | ~150ms | Apple devices |
| LC3plus | Variable | 48 kHz | ~20ms | BT LE Audio |
| Opus | Variable | 48 kHz | ~20ms | Open codec |
| SBC | 328 kbps | 44.1 kHz | ~200ms | Universal |

---

## Hardware Requirements

| Component | Requirement | Notes |
|:----------|:------------|:------|
| MCU | ESP32-WROVER or ESP32-WROOM | Original ESP32 only (not S2/S3/C3) |
| PSRAM | Optional (8MB recommended) | Enables larger buffers for LDAC/AAC |
| Flash | 8MB | Enables OTA dual partition |
| DAC | I2S compatible | PCM5102, MAX98357A, etc. |
| Encoder | Adafruit Quad Rotary (5752) | Optional - I2C control interface |
| LED Matrix | 16x16 WS2812B | Optional - SPI/DMA driven |

### GPIO Assignments

| GPIO | Function |
|:-----|:---------|
| GPIO 25 | I2S DATA |
| GPIO 26 | I2S BCK |
| GPIO 27 | I2S LRCK |
| GPIO 18 | LED Matrix Data |
| GPIO 23 | I2C SDA (Encoder) |
| GPIO 22 | I2C SCL (Encoder) |

### Quad Rotary Encoder (Adafruit 5752)

| Encoder | Function | Button Action |
|:--------|:---------|:--------------|
| Volume (Green) | Adjust volume | 1-click: Play/Pause, 2-click: Next, 3-click: Previous |
| Bass (Red) | Adjust bass EQ | Click: Enter brightness mode |
| Mid (Blue) | Adjust mid EQ | Click: Enter Bluetooth pairing mode |
| Treble (Yellow) | Adjust treble EQ | Click: Enter effect selection mode |

### PSRAM vs Non-PSRAM Mode

| Mode | Audio Buffers | OTA Buffer | LED Stack | Best For |
|:-----|:-------------|:-----------|:----------|:---------|
| With PSRAM | 48 x 8KB (384KB) | 16KB | 8KB | LDAC 96kHz, AAC |
| Without PSRAM | 16 x 2KB (32KB) | 4KB | 4KB | SBC, aptX |

---

## Repository Structure

```
ESP32-A2DP-SINK-WITH-CODECS-UPDATED/
├── README.md                 # This file
├── bt_audio_sink/            # Main ESP32 project
│   ├── main/                 # Application source code
│   ├── components/           # ESP32-A2DP, arduino-audio-tools
│   ├── recovery/             # OTA recovery partition
│   └── tools/                # Firmware encryption tools
└── esp-idf/                  # ESP-IDF v5.5.2 with patched Bluetooth stack
```

---

## Building from Source

### Step 1: Clone This Repository

```bash
git clone https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED.git
cd ESP32-A2DP-SINK-WITH-CODECS-UPDATED
```

### Step 2: Set Up ESP-IDF Environment

**Windows:**
```cmd
cd esp-idf
install.bat
export.bat
```

**Linux/macOS:**
```bash
cd esp-idf
./install.sh
. ./export.sh
```

### Step 3: Build and Flash

**Windows:**
```cmd
cd ..\bt_audio_sink
idf.py build
idf.py -p COMXX (Replace this with your ESP32 COM) flash monitor
```

**Linux/macOS:**
```bash
cd ../bt_audio_sink
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## OTA Updates

The firmware supports encrypted OTA updates via Google Drive.

### OTA Performance

| Metric | Value |
|:-------|:------|
| Firmware Size | ~1,320 KB |
| Download + Flash Time | ~1 minute 10 seconds |
| Encryption | AES-256-CBC |

### Encrypting Firmware for OTA

1. Generate a new AES key:
```bash
cd bt_audio_sink/tools
python encrypt_firmware.py --generate-key
```

2. Update the key in:
   - `tools/encrypt_firmware.py` (AES_KEY variable)
   - `recovery/main/recovery_main.cpp` (AES_KEY array)

3. Build and encrypt:
```bash
idf.py build
python tools/encrypt_firmware.py build/bt_audio_sink.bin --version 1.1.0
```

4. Upload `ota_releases/1.1.0.enc` to Google Drive and share with "Anyone with link"

---

## BLE GATT Services

Control your audio device remotely via Bluetooth Low Energy:

| Service | UUID Prefix | Description |
|:--------|:------------|:------------|
| Level Meters | 0x0042 | Real-time L/R audio levels (0-80) |
| Control | 0x0046 | Play, Pause, Volume, Mute |
| Equalizer | 0x0048 | Bass, Mid, Treble (-12 to +12 dB) |
| Device Name | 0x0050 | Read/write Bluetooth device name |
| OTA Control | 0x0054 | Firmware update control |
| OTA Data | 0x0056 | Firmware binary transfer |
| LED Control | 0x0058 | Effect selection, brightness |

---

## Configuration

Key sdkconfig settings for optimal performance:

```ini
# Flash and Memory
CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_SPIRAM_SPEED_80M=y

# CPU Performance
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y

# Bluetooth Codecs (built into patched ESP-IDF 5.5.2)
CONFIG_BTDM_CTRL_MODE_BTDM=y
CONFIG_BT_A2DP_ENABLE=y
CONFIG_BT_A2DP_LDAC_DECODER=y
CONFIG_BT_A2DP_APTX_DECODER=y
CONFIG_BT_A2DP_AAC_DECODER=y
CONFIG_BT_A2DP_OPUS_DECODER=y
CONFIG_BT_A2DP_LC3PLUS_DECODER=y
```

### DSP Configuration

The DSP processor includes:

- 3-band Parametric EQ: Bass (80Hz), Mid (1kHz), Treble (8kHz)
- Stage Presence 3D: Bass frequencies (<180Hz) centered mono, mids/highs widened 2.2x
- LED Audio Boost: Up to 100x gain for low volume LED reactivity
- Fast Math: Division-free operations using hardware reciprocal approximations

---

## Troubleshooting

**Bluetooth pairing issues on Linux**

Clear the Bluetooth cache:
```bash
sudo rm -rf /var/lib/bluetooth/<adapter-mac>/cache/<device-mac>
```
Then re-pair the device.

**LED effects not reactive at low volume**

The firmware includes automatic gain control that boosts audio to 70% minimum for LED effects. If LEDs still don't react, check the LED matrix wiring and GPIO configuration.

**AAC decoder fails to initialize**

Ensure PSRAM is enabled and running at 80MHz. Check that your ESP32 board has PSRAM (WROVER, not WROOM).

---

## Credits

This project builds upon the excellent work of the open-source community:

| Project | Author | Description |
|:--------|:-------|:------------|
| [ESP32-A2DP](https://github.com/cfint/ESP32-A2DP/tree/v5.1-a2dp_codecs) | cfint/pschatzmann | A2DP library |
| [arduino-audio-tools](https://github.com/cfint/arduino-audio-tools/tree/v5.1-a2dp_codecs) | cfint/pschatzmann | Audio Processing |
| [libldac-dec](https://github.com/cfint/libldac-dec) | cfint | LDAC decoder |
| [libfreeaptx-esp](https://github.com/cfint/libfreeaptx-esp) | cfint | aptX decoder for ESP32 |
| [ESP-IDF](https://github.com/espressif/esp-idf) | Espressif | Official ESP32 development framework |
| [Original Project](https://github.com/WillyBilly06/esp32-a2dp-sink-with-LDAC-APTX-AAC) | WillyBilly06 | ESP-IDF 5.1.4 version |



### Codec Libraries (integrated into patched ESP-IDF)

- libldac-dec - LDAC decoder
- libfreeaptx - aptX/aptX-HD decoder
- LibHelix-AAC - AAC decoder
- libopus - Opus decoder
- liblc3 - LC3 decoder

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
