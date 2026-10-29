"""Measure portable AVDTP metadata with the configured ESP32 C compiler, no flash."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    build = ROOT / "_build/esp32"
    rows = json.loads((build / "compile_commands.json").read_text())
    units = {Path(row["file"]).stem: row for row in rows
             if Path(row["file"]).stem in ["dbt_avdtp", "dbt_avdtp_bearer", "dbt_avdtp_caps",
                                           "dbt_avdtp_sink", "dbt_a2dp_sbc"]}
    assert len(units) == 5, "Build the probe's complete AVDTP/sink library first"
    row = units["dbt_avdtp"]
    compiler = Path(re.match(r'^\s*("[^"]+"|\S+)', row["command"])[1].strip('"'))
    assert compiler.name in ["xtensa-esp32-elf-gcc", "xtensa-esp32-elf-gcc.exe"]
    nm = compiler.with_name(compiler.name.replace("gcc", "nm"))
    destination = ROOT / "_build/host"
    source, obj = destination / "avdtp_abi.c", destination / "avdtp_abi.o"
    types = {"signal": "dbt_avdtp_t", "sink_including_signal": "dbt_avdtp_sink_t", "bearer": "dbt_avdtp_bearer_t",
             "capabilities": "dbt_avdtp_caps_t", "media_view": "dbt_avdtp_media_t"}
    source.write_text('#include "dbt_avdtp_bearer.h"\n#include "dbt_avdtp_sink.h"\n' +
        "".join("const unsigned char measure_"+name+"[sizeof("+ctype+")]={0};\n"
                for name, ctype in types.items()))
    subprocess.run([str(compiler), "-std=c11", "-Os", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "include"), "-c", str(source), "-o", str(obj)], check=True)
    output = subprocess.check_output([str(nm), "-S", "--defined-only", str(obj)], text=True)
    sizes = {}
    for line in output.splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[3].startswith("measure_"):
            sizes[fields[3][8:]] = int(fields[1], 16)
    assert sizes.keys() == types.keys()
    objects = {}
    for name, unit in units.items():
        path = Path(unit["directory"]) / unit["output"]
        original = Path(unit["file"])
        assert path.stat().st_mtime_ns >= original.stat().st_mtime_ns, name + " object is stale"
        symbols = subprocess.check_output([str(nm), "--defined-only", str(path)], text=True)
        assert "dbt_avdtp_" in symbols or "dbt_a2dp_sbc_validate" in symbols
        objects[name] = sha(path)
    elf = build / "native_bt_probe.elf"
    linked = subprocess.check_output([str(nm), "--defined-only", str(elf)], text=True)
    paths = [ROOT / "src" / (name + ".c") for name in units]
    paths += list((ROOT / "include").glob("dbt_avdtp*.h"))
    report = {
        "scope": "ESP32 ABI and compiled library objects; no profile/RF/audio validation",
        "metadata_bytes": sizes,
        "compiled_objects_sha256": objects,
        "linked_in_probe": any(line.split()[-1].startswith("dbt_avdtp_") for line in linked.splitlines() if line.split()),
        "probe_image_sha256": sha(build / "native_bt_probe.bin"),
        "source_sha256": {path.relative_to(ROOT).as_posix(): sha(path) for path in paths},
    }
    (destination / "avdtp_target.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items() if not key.endswith("sha256")}, indent=2))


if __name__ == "__main__":
    main()
