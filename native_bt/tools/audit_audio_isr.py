"""Check owned I2S ISR roots and direct global references in the actual target ELF."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    build = ROOT/"_build/esp32"
    receipt = ROOT/"_build/host/audio-isr.json"
    receipt.parent.mkdir(parents=True, exist_ok=True)
    receipt.write_text(json.dumps({"passed":False,"state":"checking","hardware_accessed":False})+"\n")
    rows = json.loads((build/"compile_commands.json").read_text())
    compiler = Path(re.match(r'^\s*(\S+)', next(r["command"] for r in rows if r["file"].endswith("probe_output.c")))[1])
    nm, objdump = [compiler.with_name(compiler.name.replace("gcc", tool)) for tool in ["nm", "objdump"]]
    elf = build/"native_bt_probe.elf"
    symbols = {}
    for line in subprocess.check_output([str(nm), "-n", "-S", str(elf)], text=True).splitlines():
        fields = line.split()
        if len(fields) == 4:
            symbols.setdefault(fields[3], []).append(int(fields[0], 16))
        elif len(fields) == 3 and fields[1].upper() in ("A", "T", "D", "B", "R"):
            # SDK ROM linker symbols can have no size field.
            symbols.setdefault(fields[2], []).append(int(fields[0], 16))

    def internal_code(address):
        return 0x40000000 <= address < 0x40070000 or 0x40080000 <= address < 0x400a0000

    cache = (build/"CMakeCache.txt").read_text()
    evidence = {}
    checked_roots = [("probe_output.c", "sent"), ("dbt_audio.c", "dbt_audio_take"), ("dbt_audio.c", "dbt_audio_fade")]
    if "DBT_PROBE_STAY_NATIVE:BOOL=ON" in cache:
        checked_roots += [("probe_output.c", name) for name in ("finish_sent", "refill_dispatch_needed") if name in symbols]
    for source, function in checked_roots:
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
    getter = None
    if "DBT_PROBE_STAY_NATIVE:BOOL=ON" in cache:
        name = "xTaskGetCurrentTaskHandle"
        addresses = symbols.get(name, [])
        assert len(addresses) == 1 and 0x40080000 <= addresses[0] < 0x400a0000, ("IRQ dispatch getter must be linked in IRAM", addresses)
        row = next(r for r in rows if Path(r["file"]).name == "tasks.c")
        obj = Path(row["directory"])/row["output"]
        table = subprocess.check_output([str(objdump), "-t", str(obj)], text=True)
        section = next(line.split()[3] for line in table.splitlines() if line.split() and line.split()[-1] == name)
        relocations = subprocess.check_output([str(objdump), "-r", str(obj)], text=True)
        getter_sections = {section, ".literal."+name}
        active = False; references = {}
        for line in relocations.splitlines():
            if line.startswith("RELOCATION RECORDS FOR ["):
                active = any(line == "RELOCATION RECORDS FOR ["+part+"]:" for part in getter_sections); continue
            fields = line.split()
            if not active or len(fields) != 3 or not fields[1].startswith("R_XTENSA_"): continue
            symbol = fields[2].split("+")[0]
            if symbol.startswith("."): continue
            values = symbols.get(symbol, [])
            assert len(values) == 1, (name, symbol, values)
            if symbol == "pxCurrentTCBs":
                assert 0x3ff80000 <= values[0] < 0x40000000, (name, symbol, values)
            else:
                # The public getter restores the local mask through an SDK
                # absolute ROM symbol; it does not acquire the kernel lock.
                assert symbol == "_xtos_set_intlevel" and 0x40000000 <= values[0] < 0x40070000, (name, symbol, values)
            references[symbol] = hex(values[0])
        assert set(references) == {"pxCurrentTCBs", "_xtos_set_intlevel"}, references
        getter = {"address":hex(addresses[0]),"object_section":section,"linked_iram":True,"direct_global_references":references,"placement":"public SDK getter through linker fragment"}
    dispatch = None
    if "DBT_PROBE_DISPATCH_TRACE:BOOL=ON" in cache:
        row = next(r for r in rows if Path(r["file"]).name == "probe_dispatch_trace.c")
        obj = Path(row["directory"])/row["output"]
        table = subprocess.check_output([str(objdump), "-t", str(obj)], text=True)
        entries = {f[-1]:f for line in table.splitlines() if len(f:=line.split()) == 6 and f[2] in ("F","O")}
        hot = ("task_role","record","probe_dispatch_switch","probe_dispatch_mark","probe_dispatch_eof","probe_dispatch_freeze")
        hot_sections = set(); functions = {}
        for name in hot:
            section = entries[name][3]; assert section.startswith(".iram"), (name,section)
            hot_sections.update((section,section+".literal"))
            values = [x for x in symbols.get(name,[]) if 0x40080000 <= x < 0x400a0000]
            assert len(values)==1,(name,values)
            functions[name] = {"address":hex(values[0]),"section":section}
        data_sections = set(); data = {}
        for name in ("trace","roles","rings"):
            section = entries[name][3]; assert section.startswith(".dram"),(name,section)
            values = [x for x in symbols.get(name,[]) if 0x3ff80000 <= x < 0x40000000]
            assert len(values)==1,(name,values)
            data_sections.add(section);data[name] = hex(values[0])
        relocations = subprocess.check_output([str(objdump),"-r",str(obj)],text=True)
        active = False;references = {}
        for line in relocations.splitlines():
            if line.startswith("RELOCATION RECORDS FOR ["):
                active = any(line=="RELOCATION RECORDS FOR ["+s+"]:" for s in hot_sections);continue
            f=line.split()
            if not active or len(f)!=3 or not f[1].startswith("R_XTENSA_"):continue
            target=f[2].split("+")[0]
            if target.startswith("."):
                assert target in hot_sections or target in data_sections,("trace cache dependency",target)
            else:
                values=symbols.get(target,[])
                assert len(values)==1 and internal_code(values[0]),("trace external dependency",target,values)
                references[target]=hex(values[0])
        kernel = next(r for r in rows if Path(r["file"]).name == "tasks.c")
        kernel_obj = Path(kernel["directory"])/kernel["output"]
        kernel_table = subprocess.check_output([str(objdump),"-t",str(kernel_obj)],text=True)
        section = next(l.split()[3] for l in kernel_table.splitlines() if l.split() and l.split()[-1]=="vTaskSwitchContext")
        values=symbols.get("vTaskSwitchContext",[])
        assert len(values)==1 and 0x40080000 <= values[0] < 0x400a0000,values
        kernel_reloc = subprocess.check_output([str(objdump),"-r",str(kernel_obj)],text=True)
        active=False;hook_calls=0
        for line in kernel_reloc.splitlines():
            if line.startswith("RELOCATION RECORDS FOR ["):
                active=line=="RELOCATION RECORDS FOR ["+section+"]:";continue
            f=line.split()
            if active and len(f)==3 and f[1]=="R_XTENSA_ASM_EXPAND" and f[2]=="probe_dispatch_switch":hook_calls+=1
        assert hook_calls>=1,"Task selection did not call the admitted observer"
        dispatch={"functions":functions,"internal_data":data,"direct_sdk_references":references,
                  "kernel_switch":hex(values[0]),"kernel_hook_calls":hook_calls,
                  "scope":"observer IRAM code/literal/data closure and pinned kernel hook; not the full SDK callgraph or runtime overhead proof"}
    config = (build/"config/sdkconfig.h").read_text()
    assert "#define CONFIG_I2S_ISR_IRAM_SAFE 1" in config
    assert "#define CONFIG_FREERTOS_PLACE_FUNCTIONS_INTO_FLASH 1" not in config
    assert any(0x3ff80000 <= address < 0x40000000 for address in symbols["output"])
    report = {"passed": True, "scope": "owned ISR roots/direct references, required SDK getter dependencies and configuration; not a whole callgraph or runtime proof",
              "roots": evidence, "dispatch_getter": getter, "frozen_dispatch": dispatch, "context_internal_sram": True,
              "firmware_sha256": hashlib.sha256((build/"native_bt_probe.bin").read_bytes()).hexdigest()}
    receipt.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
