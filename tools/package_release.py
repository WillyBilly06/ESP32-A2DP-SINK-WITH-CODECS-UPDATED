"""Package known ESP32 build outputs; never reads device flash or NVS backups."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import shutil
import zipfile
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--build", type=Path, required=True)
    p.add_argument("--recovery-build", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--version", required=True)
    p.add_argument("--sdk", required=True)
    p.add_argument("--commit", required=True)
    a = p.parse_args()
    root = Path(__file__).resolve().parents[1]
    a.out.mkdir(parents=True, exist_ok=True)
    sources = {
        "firmware.bin": a.build / "bt_audio_sink.bin",
        "recovery.bin": a.recovery_build / "recovery.bin",
        "bootloader.bin": a.build / "bootloader/bootloader.bin",
        "partition-table.bin": a.build / "partition_table/partition-table.bin",
        "ota_data_initial.bin": a.build / "ota_data_initial.bin",
    }
    for name, source in sources.items():
        assert source.is_file(), source
        shutil.copy2(source, a.out / name)
    image = (a.out / "firmware.bin").read_bytes()
    assert image[0] == 0xE9 and len(image) <= 0x280000
    assert (a.out / "recovery.bin").stat().st_size <= 0x100000
    version = image[48:80].split(b"\0", 1)[0].decode()
    assert version == a.version, (version, a.version)
    source = (root / "bt_audio_sink/recovery/main/recovery_main.cpp").read_text()
    key_block = re.search(r"AES_KEY\[[^\]]*\]\s*=\s*\{(.*?)\}", source, re.S).group(1)
    key = bytes(int(v, 16) for v in re.findall(r"0x([0-9A-Fa-f]{2})", key_block))
    assert len(key) == 32
    iv = os.urandom(16)
    pad = 16 - len(image) % 16
    encryptor = Cipher(algorithms.AES(key), modes.CBC(iv)).encryptor()
    encrypted = iv + encryptor.update(image + bytes([pad]) * pad) + encryptor.finalize()
    decryptor = Cipher(algorithms.AES(key), modes.CBC(iv)).decryptor()
    assert (decryptor.update(encrypted[16:]) + decryptor.finalize())[:-pad] == image
    (a.out / "firmware.enc").write_bytes(encrypted)
    manifest = {
        "firmware_version": a.version, "esp_idf": a.sdk, "source_commit": a.commit,
        "target": "esp32", "flash_size": "8MB", "psram_required": False,
        "offsets": {"bootloader.bin": "0x1000", "partition-table.bin": "0x8000", "recovery.bin": "0x10000", "firmware.bin": "0x110000", "ota_data_initial.bin": "0x610000"},
        "files": {name: {"bytes": (a.out / name).stat().st_size, "sha256": hashlib.sha256((a.out / name).read_bytes()).hexdigest()} for name in [*sources, "firmware.enc"]},
    }
    (a.out / "build-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (a.out / "flash_windows.bat").write_text('''@echo off
setlocal
if "%~1"=="" (
  echo Usage: flash_windows.bat COM6
  exit /b 2
)
cd /d "%~dp0"
python -m esptool --chip esp32 --port %~1 --baud 115200 write_flash --flash_mode dio --flash_freq 80m --flash_size 8MB 0x1000 bootloader.bin 0x8000 partition-table.bin 0x10000 recovery.bin 0x110000 firmware.bin 0x610000 ota_data_initial.bin
exit /b %errorlevel%
''')
    (a.out / "README.txt").write_text(f'''BDK ESP32 firmware {a.version}, ESP-IDF {a.sdk}
Source commit: {a.commit}

For original ESP32 / 8 MB flash / internal RAM; no PSRAM required.
USB: install esptool, extract this archive, run flash_windows.bat COMXX.
This writes the repository partition table, bootloader, OTA selection and both
application images. Back up existing firmware/data before changing layouts.
NVS and SPIFFS data are not included or deliberately erased by this bundle.
BLE OTA: select firmware.bin. firmware.enc uses the repository's legacy
IV16 + AES-256-CBC/PKCS7 container and configured key.
Recovery is a separate image at 0x10000; main is at 0x110000.
See release notes for the exact validation scope and known limitations.
''')
    checksum = "".join(f"{hashlib.sha256(f.read_bytes()).hexdigest()}  {f.name}\n" for f in sorted(a.out.iterdir()) if f.is_file() and f.name != "SHA256SUMS.txt")
    (a.out / "SHA256SUMS.txt").write_text(checksum)
    archive = a.out.parent / f"BDK-{a.version}-flash-bundle.zip"
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(a.out.iterdir()):
            if f.is_file():
                z.write(f, f.name)
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
    print(json.dumps({"archive": str(archive), "firmware_bytes": len(image), "manifest": str(a.out / "build-manifest.json")}))


if __name__ == "__main__":
    main()
