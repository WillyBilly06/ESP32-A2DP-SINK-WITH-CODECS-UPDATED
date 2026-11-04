"""Cross-check current portable test receipts and linked/compiled target scope."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def normalize(hashes):
    return {key.replace("\\", "/").replace("../bt_audio_sink/main/dsp/", "dsp/"): value
            for key, value in hashes.items()}


def main():
    output = ROOT / "_build/host"
    windows = json.loads((output / "results.json").read_text())
    linux = json.loads((output / "linux_results.json").read_text())
    spec = importlib.util.spec_from_file_location("host_inventory", ROOT / "tools/test_host.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    expected = len(module.TESTS) + 2  # FIFO and three-owner audio stresses.
    assert windows["passed"] and linux["passed"]
    assert len(windows["tests"]) == linux["normal_groups"] == expected
    assert normalize(windows["source_sha256"]) == normalize(linux["source_sha256"])
    for name, digest in windows["source_sha256"].items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, name
    build = ROOT / "_build/esp32"
    nm = next(line.split("=", 1)[1] for line in (build / "CMakeCache.txt").read_text().splitlines()
              if line.startswith("CMAKE_NM:FILEPATH="))

    def symbols(path):
        result = subprocess.run([nm, "--format=posix", str(path)], capture_output=True, text=True, check=True)
        return [line.split() for line in result.stdout.splitlines() if line.split()]

    names = {parts[0] for parts in symbols(build / "native_bt_probe.elf")}
    assert {"dbt_channels_receive", "dbt_channels_prepare_data"} <= names
    objects = list((build / "esp-idf/native_bt").rglob("dbt_credit.c.obj"))
    channels = list((build / "esp-idf/native_bt").rglob("dbt_channels.c.obj"))
    assert len(objects) == len(channels) == 1
    for source in [ROOT / "src/dbt_channels.c", ROOT / "include/dbt_channels.h"]:
        assert source.stat().st_mtime_ns <= channels[0].stat().st_mtime_ns, str(source)
    mutable = [p[0] for p in symbols(objects[0]) if len(p) > 1 and p[1] in "BbDdCcSs"]
    assert not mutable
    report = {"scope": "offline source checkpoint; no flash or connection validation",
              "windows_groups": expected, "linux_groups": expected,
              "linux_asan_ubsan": linux["address_undefined_passed"],
              "linux_tsan": linux["thread_sanitizer_passed"],
              "current_sources_match_both_test_receipts": True,
              "credit_engine_mutable_static_symbols": len(mutable),
              "credit_engine_compiled_in_component_archive": True,
              "credit_engine_linked_in_probe": any(name.startswith("dbt_credit_") for name in names),
              "basic_channel_changes_linked_in_probe": True,
              "probe_bin_sha256": hashlib.sha256((build / "native_bt_probe.bin").read_bytes()).hexdigest(),
              "probe_elf_sha256": hashlib.sha256((build / "native_bt_probe.elf").read_bytes()).hexdigest(),
              "sources_sha256": windows["source_sha256"]}
    (output / "offline_layer_audit.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "sources_sha256"}, indent=2))


if __name__ == "__main__":
    main()
