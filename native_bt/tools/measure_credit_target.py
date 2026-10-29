"""Read credit-channel metadata sizes from the linked ESP32 DWARF, without hardware."""
import hashlib
import json
from pathlib import Path
try:
    from elftools.elf.elffile import ELFFile
except ImportError:
    raise SystemExit("Run this tool with the configured ESP-IDF Python environment (pyelftools)")

ROOT = Path(__file__).resolve().parents[1]


def main():
    elf = ROOT / "_build/esp32/native_bt_probe.elf"
    wanted = {"dbt_le_channel", "dbt_channels_t", "dbt_runtime_t", "dbt_l2cap_tx_t", "dbt_credit_t"}
    found = {}
    with elf.open("rb") as handle:
        dwarf = ELFFile(handle).get_dwarf_info()
        for unit in dwarf.iter_CUs():
            for die in unit.iter_DIEs():
                name = die.attributes.get("DW_AT_name")
                if name is None or not isinstance(name.value, bytes):
                    continue
                name = name.value.decode(errors="replace")
                if name not in wanted:
                    continue
                actual = die
                while actual and "DW_AT_byte_size" not in actual.attributes and "DW_AT_type" in actual.attributes:
                    actual = actual.get_DIE_from_attribute("DW_AT_type")
                if actual and "DW_AT_byte_size" in actual.attributes:
                    size = actual.attributes["DW_AT_byte_size"].value
                    assert name not in found or found[name] == size, name
                    found[name] = size
    assert set(found) == wanted
    report = {"scope": "linked ESP32 ABI metadata sizes; no physical heap/CPU/RF measurement",
              "metadata_bytes_target_abi": found,
              "active_le_state_requested_bytes": found["dbt_le_channel"],
              "inactive_le_state_allocated_bytes": 0,
              "runtime_includes_channels_and_host": True,
              "allocator_overhead_included": False,
              "probe_elf_sha256": hashlib.sha256(elf.read_bytes()).hexdigest()}
    (ROOT / "_build/host/credit_target.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
