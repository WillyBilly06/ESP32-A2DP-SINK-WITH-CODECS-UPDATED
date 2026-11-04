"""Offline image/repository-layout check. Does not satisfy device admission.

The separate validate_local_image.py and deployment helpers retain their
physical-backup gates. This check never creates those hardware receipts.
"""
from pathlib import Path
import hashlib
import io
import json
import struct
import sys
from esptool.bin_image import ESP32FirmwareImage
from source_snapshot import verify_manifest

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "_build/esp32"
sys.path.insert(0, str(ROOT.parent / "esp-idf/components/partition_table"))
from gen_esp32part import PartitionTable

status = json.loads((BUILD / "local-build-status.json").read_text(encoding="utf-8-sig"))
assert status["passed"] and status["state"] == "complete"
count = verify_manifest(BUILD / "local-build-sources.json", status["source_manifest_sha256"])
blob = (BUILD / "native_bt_probe.bin").read_bytes()
assert hashlib.sha256(blob).hexdigest() == status["image_sha256"]
config_bytes = (BUILD / "config/sdkconfig.h").read_bytes()
assert hashlib.sha256(config_bytes).hexdigest() == status["config_sha256"]
config = config_bytes.decode()
for setting in ("CONFIG_BT_CONTROLLER_ONLY 1", "CONFIG_BTDM_CTRL_MODE_BTDM 1", "CONFIG_FREERTOS_HZ 1000", "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE 1"):
    assert "#define " + setting in config, setting
assert "#define CONFIG_BT_BLUEDROID_ENABLED 1" not in config
assert "#define CONFIG_BT_NIMBLE_ENABLED 1" not in config
image = ESP32FirmwareImage(io.BytesIO(blob))
image.verify()
assert image.chip_id == 0 and image.min_rev_full <= 301 <= image.max_rev_full
assert image.flash_mode == 2 and image.flash_size_freq == 0x3f
position, checksum = 24, 0xef
for _ in range(blob[1]):
    assert position + 8 <= len(blob)
    _, length = struct.unpack_from("<II", blob, position)
    position += 8
    assert position + length <= len(blob)
    for value in blob[position:position + length]:
        checksum ^= value
    position += length
checksum_at = position + ((15 - position) % 16)
assert blob[checksum_at] == checksum
end = checksum_at + 1
assert image.append_digest and hashlib.sha256(blob[:end]).digest() == blob[end:end + 32]
assert end + 32 == len(blob)
compiled = (BUILD / "partition_table/partition-table.bin").read_bytes()
assert hashlib.sha256(compiled).hexdigest() == status["partition_sha256"]
expected = PartitionTable.from_csv((ROOT.parent / "bt_audio_sink/partitions.csv").read_text()).to_binary()
assert compiled == expected
slot = next(p for p in PartitionTable.from_binary(compiled) if p.name == "ota_1")
assert slot.offset == 0x390000 and slot.size == 0x280000 and len(blob) <= slot.size
report = {"passed": True, "scope": "offline image integrity, ESP32 revision, source/config binding and repository layout fit only",
          "hardware_accessed": False, "deployment_admitted": False, "runtime_qualified": False,
          "image_sha256": status["image_sha256"], "image_bytes": len(blob), "build_inputs": count,
          "partition_offset": slot.offset, "partition_bytes": slot.size, "remaining_bytes": slot.size - len(blob),
          "next_device_gate": "fresh target identity/partition/backup admission; existing hardware validator is unchanged"}
out = ROOT / "_build/audit"
out.mkdir(parents=True, exist_ok=True)
(out / "repository-image-validation.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
