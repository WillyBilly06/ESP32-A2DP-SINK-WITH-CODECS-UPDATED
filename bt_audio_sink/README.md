# ESP32 A2DP Sink with LDAC, aptX, and AAC Codecs (ESP-IDF 5.5.2)

High-quality Bluetooth audio receiver for ESP32 with support for advanced audio codecs including LDAC, aptX, aptX-HD, and AAC. Built on ESP-IDF v5.5.2 with a patched Bluetooth stack for native codec decoding.

For the original ESP-IDF 5.3 version, see: https://github.com/WillyBilly06/esp32-a2dp-sink-with-LDAC-APTX-AAC

## Features

- Native multi-codec decoding (LDAC, aptX HD, aptX, aptX LL, AAC, LC3plus, Opus, SBC)
- Hi-Res audio support up to 96kHz/24-bit with LDAC
- I2S audio output for PCM5102A or similar DAC
- BLE control interface for volume, EQ, and settings
- OTA firmware updates via Google Drive with AES-256-CBC encryption
- WS2812B LED status indicator with effects
- NVS-based persistent settings storage
- DSP processing (EQ, bass boost, filters)

## Hardware Requirements

- ESP32-WROOM-32 or ESP32-WROVER module (Classic Bluetooth required)
- I2S DAC (PCM5102A, ES9018, MAX98357A, etc.)
- WS2812B LED strip (optional)
- Rotary encoder with I2C Seesaw controller (optional)

## Pin Configuration

| Function | GPIO |
|----------|------|
| I2S DATA | 25   |
| I2S BCK  | 26   |
| I2S LRCK | 27   |
| LED Data | 18   |

## Codec Support

| Codec   | Max Bitrate | Sample Rate | Notes |
|---------|-------------|-------------|-------|
| LDAC    | 990 kbps    | 96 kHz      | Hi-Res, Sony |
| aptX-HD | 576 kbps    | 48 kHz      | 24-bit, Qualcomm |
| aptX    | 352 kbps    | 48 kHz      | Low latency |
| aptX-LL | 352 kbps    | 48 kHz      | Ultra low latency |
| AAC     | 256 kbps    | 48 kHz      | Apple devices |
| LC3plus | Variable    | 48 kHz      | BT LE Audio |
| Opus    | Variable    | 48 kHz      | Open codec |
| SBC     | 328 kbps    | 48 kHz      | Universal fallback |

## Building

### Prerequisites

1. ESP-IDF v5.5.2 (included as `esp-idf-5.5.2/` with patched Bluetooth stack)
2. Python 3.11+

### Build Steps

```bash
# Set up ESP-IDF environment (Windows)
cd esp-idf-5.5.2
.\install.bat
.\export.bat

# Build the project
cd ..\bt_audio_sink
idf.py set-target esp32
idf.py build

# Flash to device
idf.py -p COM3 flash monitor
```

## OTA Updates

The firmware supports encrypted OTA updates via Google Drive.

### OTA Performance

- File size: ~1,320 KB
- Download and flash time: ~1 minute 10 seconds (via WiFi)

### Creating Encrypted Firmware

1. Generate your own AES-256 key:
```bash
cd tools
python encrypt_firmware.py --generate-key
```

2. Copy the generated key to:
   - `tools/encrypt_firmware.py` (AES_KEY variable)
   - `recovery/main/recovery_main.cpp` (AES_KEY array)

3. Build the firmware and encrypt:
```bash
idf.py build
python tools/encrypt_firmware.py build/bt_audio_sink.bin --version 1.1.0
```

4. Upload `ota_releases/1.1.0.enc` to Google Drive and share with "Anyone with link"
5. Update `latest.txt` with the version and file ID

## Project Structure

```
bt_audio_sink/
  main/
    main.cpp              # Application entry point
    audio/                # Audio pipeline and I2S output
    ble/                  # BLE GATT server for control
    config/               # Application configuration
    dsp/                  # DSP processing (EQ, filters)
    input/                # Encoder input handling
    led/                  # LED effects and driver
    ota/                  # OTA update handling
    storage/              # NVS settings storage
  components/
    ESP32-A2DP/           # A2DP sink library with codec support
    arduino-audio-tools/  # Audio tools library
  recovery/               # Recovery partition firmware
  tools/
    encrypt_firmware.py   # OTA encryption tool
```

## Configuration

Build-time configuration is available via `idf.py menuconfig` under "BT Audio Sink Configuration":

- Device name
- I2S pins
- LED configuration
- OTA settings
- Firmware version

## How It Works

1. Bluetooth initializes in Classic BT mode as A2DP sink
2. All enabled codecs are advertised to source devices
3. Source device (phone/PC) selects the best mutual codec
4. Source sends encoded audio stream
5. Patched ESP-IDF decodes audio internally to PCM
6. Decoded PCM is output via I2S to DAC

## Compatibility

Tested with:
- Android phones (LDAC, aptX HD, aptX, AAC, SBC)
- Windows 11 (aptX, SBC, AAC)
- macOS (AAC, SBC)
- iOS (AAC, SBC)
- Linux (all codecs with BlueZ)

## Changes from Original Version

- Updated to ESP-IDF v5.5.2 (from v5.3)
- Cleaned up unused legacy code
- Improved build system compatibility
- Updated component dependencies

## Components

This project uses:

- [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP) - A2DP sink implementation
- [arduino-audio-tools](https://github.com/pschatzmann/arduino-audio-tools) - Audio utilities

## License

MIT License - See LICENSE file for details.

## Credits

- ESP-IDF Bluetooth stack by Espressif
- Codec libraries: libldac-dec, libfreeaptx, FDK-AAC, libopus, liblc3
