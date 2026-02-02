# ESP32 A2DP Sink with LDAC, aptX, and AAC Codecs (ESP-IDF 5.5.2)

High-quality Bluetooth audio receiver for ESP32 with support for advanced audio codecs including LDAC, aptX, aptX-HD, and AAC. Built on ESP-IDF v5.5.2 with a patched Bluetooth stack for native codec decoding.

For the original ESP-IDF 5.3 version, see: https://github.com/WillyBilly06/esp32-a2dp-sink-with-LDAC-APTX-AAC

## Repository Structure

- **bt_audio_sink/** - Main ESP32 project
- **esp-idf/** - ESP-IDF v5.5.2 with patched Bluetooth stack

## Features

- Native multi-codec decoding (LDAC, aptX HD, aptX, aptX LL, AAC, LC3plus, Opus, SBC)
- Hi-Res audio support up to 96kHz/24-bit with LDAC
- I2S audio output for PCM5102A or similar DAC
- BLE control interface for volume, EQ, and settings
- OTA firmware updates via Google Drive with AES-256-CBC encryption
- WS2812B LED status indicator with effects
- NVS-based persistent settings storage
- DSP processing (EQ, bass boost, filters)

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

## Hardware Requirements

- ESP32-WROOM-32 or ESP32-WROVER module (Classic Bluetooth required)
- I2S DAC (PCM5102A, ES9018, MAX98357A, etc.)
- WS2812B LED strip (optional)

## Pin Configuration

| Function | GPIO |
|----------|------|
| I2S DATA | 25   |
| I2S BCK  | 26   |
| I2S LRCK | 27   |
| LED Data | 18   |

## Quick Start

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

## OTA Updates

- File size: ~1,320 KB
- Download and flash time: ~1 minute 10 seconds (via WiFi)
- Encryption: AES-256-CBC

Generate your own encryption key:
```bash
cd bt_audio_sink/tools
python encrypt_firmware.py --generate-key
```

## Compatibility

Tested with:
- Android phones (LDAC, aptX HD, aptX, AAC, SBC)
- Windows 11 (aptX, SBC, AAC)
- macOS (AAC, SBC)
- iOS (AAC, SBC)
- Linux (all codecs with BlueZ)

## License

MIT License

## Credits

- ESP-IDF Bluetooth stack by Espressif
- Codec libraries: libldac-dec, libfreeaptx, FDK-AAC, libopus, liblc3
- [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP)
- [arduino-audio-tools](https://github.com/pschatzmann/arduino-audio-tools)
