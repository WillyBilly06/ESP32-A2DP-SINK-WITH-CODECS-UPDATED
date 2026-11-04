"""Source inventory and compiler-measured static layout, not runtime/RF proof."""
from pathlib import Path
import ctypes
import hashlib
import json
import os
import re
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent
BUILD = ROOT / "_build/esp32"
OUT = ROOT / "_build/audit"


def windows_argv(command):
    count = ctypes.c_int()
    parse = ctypes.windll.shell32.CommandLineToArgvW
    parse.argtypes = [ctypes.c_wchar_p, ctypes.POINTER(ctypes.c_int)]
    parse.restype = ctypes.POINTER(ctypes.c_wchar_p)
    pointer = parse(command, ctypes.byref(count))
    if not pointer:
        raise RuntimeError("Cannot parse the actual target compiler command")
    try:
        return list(pointer[:count.value])
    finally:
        ctypes.windll.kernel32.LocalFree(pointer)


def sections(compiler, elf):
    tool = compiler.parent / "xtensa-esp32-elf-size.exe"
    text = subprocess.check_output([str(tool), "-A", str(elf)], text=True)
    return {name: int(size) for name, size in re.findall(r"^(\.[^ ]+)\s+(\d+)\s+\d+", text, re.M)}


def main():
    if os.name != "nt":
        raise RuntimeError("This target-layout extraction uses the Windows ESP32 build commands")
    commands = json.loads((BUILD / "compile_commands.json").read_text())
    entry = next(row for row in commands if Path(row["file"]).name == "probe_main.c")
    args = windows_argv(entry["command"])
    args = args[:args.index("-o")]
    actual = args[1] if Path(args[0]).stem.lower() == "ccache" else args[0]
    compiler = Path(actual)
    OUT.mkdir(parents=True, exist_ok=True)
    source = ROOT / "tests/audit/layout.c"
    obj, binary = OUT / "layout.o", OUT / "layout.bin"
    command = args + ["-I" + str(ROOT / "codecs/common"), "-I" + str(WORKSPACE / "bt_audio_sink/main/dsp"), "-o", str(obj), "-c", str(source)]
    subprocess.run(command, cwd=entry["directory"], check=True)
    subprocess.run([str(compiler.parent / "xtensa-esp32-elf-objcopy.exe"), "-j", ".rodata.dbt_audit_layout", "-O", "binary", str(obj), str(binary)], check=True)
    names = ["pointer_bytes", "queue_metadata_and_storage", "host_including_queue", "acl_manager", "runtime_including_host",
             "att_server", "att_connection", "ertm_engine", "codec_decoder_metadata", "audio_including_dsp",
             "secure_connections_session", "ingress_capacity", "max_h4_packet", "control_reserve", "max_links", "codec_worker_jobs"]
    data = binary.read_bytes()
    assert len(data) == 4 * len(names)
    layout = dict(zip(names, struct.unpack("<" + "I" * len(names), data)))
    inventory = {}
    folders = [ROOT / p for p in ("src", "include", "codecs", "esp32_probe", "tests")]
    folders += [WORKSPACE / "bt_audio_sink/main", WORKSPACE / "bt_audio_sink/recovery/main"]
    patterns = {"allocation_calls": r"\b(?:malloc|calloc|realloc|free)\s*\(", "while_loops": r"\bwhile\s*\(",
                "event_waits": r"\b(?:xQueueReceive|ulTaskNotifyTake|xTaskNotifyWait)\s*\(",
                "indefinite_wait_arguments": r"\bportMAX_DELAY\b"}
    for folder in folders:
        for path in sorted(folder.rglob("*")):
            if not path.is_file() or path.suffix not in (".c", ".cpp", ".h", ".inc"):
                continue
            raw = path.read_bytes()
            text = raw.decode("utf-8", errors="replace")
            inventory[path.relative_to(WORKSPACE).as_posix()] = {"sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw),
                "lines": len(text.splitlines()), **{name: len(re.findall(pattern, text)) for name, pattern in patterns.items()}}
    candidate_elf = BUILD / "native_bt_probe.elf"
    baseline_elf = Path(os.environ.get("DBT_BASELINE_ELF", r"D:\bt-idf61\bt_audio_sink\build-idf61\bt_audio_sink.elf"))
    report = {"scope": "source inventory, lexical review leads and target-compiler static sizes only; no runtime RAM or performance advantage proven",
              "baseline_commit": "0d85a975ec079d954f1bdd3e94d2fed70848c252", "owned_source_origin": "f313fce94220a21066375e97b0d266d92418f96a",
              "target_layout": layout, "candidate_sections": sections(compiler, candidate_elf), "baseline_sections": sections(compiler, baseline_elf),
              "files": inventory, "manual_review_required": ["Callback and owner execution bounds", "Every blocking result queue and ownership invariant",
                  "Immediate NVS flash stalls while rendering", "Required A2DP source/SDP client/browsing role parity", "Target PSA and all-codec execution",
                  "LHDC 192 kHz/24-bit/1000 kbps cycle budget and BLE/control headroom", "Power-loss, security, RF and long-duration stress"]}
    target = ROOT / "docs/IDF61_AUDIT.json"
    target.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"files_inventoried": len(inventory), "target_layout": layout, "report": str(target)}, indent=2))


if __name__ == "__main__":
    main()
