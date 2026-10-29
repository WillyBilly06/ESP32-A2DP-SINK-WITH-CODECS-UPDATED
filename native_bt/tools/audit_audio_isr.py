"""Check owned I2S ISR roots and direct global references in the actual target ELF."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    build = ROOT/"_build/esp32"
    rows = json.loads((build/"compile_commands.json").read_text())
    compiler = Path(re.match(r'^\s*(\S+)', next(r["command"] for r in rows if r["file"].endswith("probe_output.c")))[1])
    nm, objdump = [compiler.with_name(compiler.name.replace("gcc", tool)) for tool in ["nm", "objdump"]]
    elf = build/"native_bt_probe.elf"
    symbols = {}
    for line in subprocess.check_output([str(nm), "-n", "-S", str(elf)], text=True).splitlines():
        fields = line.split()
        if len(fields) == 4:
            symbols.setdefault(fields[3], []).append(int(fields[0], 16))

    def internal_code(address):
        return 0x40000000 <= address < 0x40070000 or 0x40080000 <= address < 0x400a0000

    evidence = {}
    for source, function in [("probe_output.c", "sent"), ("dbt_audio.c", "dbt_audio_take"), ("dbt_audio.c", "dbt_audio_fade")]:
        row = next(r for r in rows if Path(r["file"]).name == source)
        obj = Path(row["directory"])/row["output"]
        table = subprocess.check_output([str(objdump), "-t", str(obj)], text=True)
        section = next(line.split()[3] for line in table.splitlines() if line.split() and line.split()[-1] == function)
        assert section.startswith(".iram"), (function, section)
        roots = [address for address in symbols.get(function, []) if 0x40080000 <= address < 0x400a0000]
        assert len(roots) == 1, function
        relocations = subprocess.check_output([str(objdump), "-r", str(obj)], text=True)
        active = False
        references = {}
        for line in relocations.splitlines():
            if line.startswith("RELOCATION RECORDS FOR ["):
                active = line == "RELOCATION RECORDS FOR ["+section+"]:"
                continue
            fields = line.split()
            if not active or len(fields) != 3 or not fields[1].startswith("R_XTENSA_"):
                continue
            name = fields[2].split("+")[0]
            if name.startswith("."):
                continue
            addresses = symbols.get(name, [])
            assert len(addresses) == 1 and internal_code(addresses[0]), (function, name, addresses)
            references[name] = hex(addresses[0])
        evidence[function] = {"address": hex(roots[0]), "object_section": section, "direct_global_references": references}
    config = (build/"config/sdkconfig.h").read_text()
    assert "#define CONFIG_I2S_ISR_IRAM_SAFE 1" in config
    assert "#define CONFIG_FREERTOS_PLACE_FUNCTIONS_INTO_FLASH 1" not in config
    assert any(0x3ff80000 <= address < 0x40000000 for address in symbols["output"])
    report = {"passed": True, "scope": "owned ISR roots/direct references and configuration; hardware flash continuity still requires testing",
              "roots": evidence, "context_internal_sram": True,
              "firmware_sha256": hashlib.sha256((build/"native_bt_probe.bin").read_bytes()).hexdigest()}
    (ROOT/"_build/host/audio-isr.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
