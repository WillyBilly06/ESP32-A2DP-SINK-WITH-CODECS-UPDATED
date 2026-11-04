"""Prepare a source-only draft checkpoint with exact offline evidence; never flashes."""
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
NATIVE = ROOT / "native_bt"
MANIFEST = NATIVE / "_build/host/github-offline-manifest.json"


def verify_index():
    entries = json.loads(MANIFEST.read_text())
    request = "".join(":" + entry["path"] + "\n" for entry in entries).encode()
    result = subprocess.run(["git", "cat-file", "--batch"], input=request, capture_output=True, cwd=ROOT, check=True)
    stream = io.BytesIO(result.stdout)
    for entry in entries:
        header = stream.readline().split()
        assert len(header) == 3 and header[1] == b"blob", entry["path"]
        assert stream.read(int(header[2])) == entry["content"].encode(), entry["path"]
        assert stream.read(1) == b"\n"
    assert not stream.read()
    print("All", len(entries), "staged source/evidence files match the manifest")


def main():
    subprocess.run([sys.executable, str(NATIVE / "tools/audit_offline_layers.py")], check=True)
    proof = json.loads((NATIVE / "_build/host/offline_layer_audit.json").read_text())
    target = json.loads((NATIVE / "_build/host/credit_target.json").read_text())
    assert target["probe_elf_sha256"] == proof["probe_elf_sha256"]
    assert proof["credit_engine_linked_in_probe"] and proof["current_sources_match_both_test_receipts"]
    public = {**proof, "target_abi": target,
              "replacement_ready": False, "all_required_features_complete": False,
              "this_checkpoint_flashed": False, "this_checkpoint_rf_tested": False,
              "equivalent_feature_advantage_proven": False,
              "historical_hardware_evidence": "HOST_EVIDENCE.json (different firmware identities)",
              "remaining": ["ERTM/Streaming/enhanced credit modes", "BLE SMP/privacy/bonds",
                            "remaining host/profile and codec integration", "product BLE control/OTA",
                            "post-audit external-peer/all-codec RF qualification", "equivalent-feature resource/performance benchmarks"]}
    public["sources_sha256"] = {key.replace("\\", "/"): value for key, value in proof["sources_sha256"].items()}
    (NATIVE / "docs/OFFLINE_EVIDENCE.json").write_text(json.dumps(public, indent=2) + "\n", encoding="utf-8")
    names = json.loads((NATIVE / "tools/public_sources.json").read_text())
    assert names == sorted(set(names))
    entries = []
    for name in names:
        path = ROOT / name
        path.resolve().relative_to(ROOT)
        assert name.startswith(("native_bt/", "bt_audio_sink/main/dsp/")) or name == "reference/README.md"
        assert "_build" not in path.parts and not path.is_symlink()
        data = path.read_bytes()
        assert len(data) <= 1048576, name
        content = data.decode("utf-8")
        assert "PRIVATE" + " KEY-----" not in content, name
        entries.append({"path": name, "mode": "100644", "type": "blob", "content": content})
    for key, expected in proof["sources_sha256"].items():
        source = (NATIVE / key.replace("\\", "/")).resolve()
        assert source.relative_to(ROOT).as_posix() in names, key
        assert hashlib.sha256(source.read_bytes()).hexdigest() == expected, key
    MANIFEST.write_text(json.dumps(entries), encoding="utf-8")
    print(json.dumps({"source_files": len(entries), "utf8_bytes": sum(len(e["content"].encode()) for e in entries),
                      "private_captures_included": False, "reference_implementation_included": False,
                      "hardware_qualification_claimed": False}))


if __name__ == "__main__":
    if sys.argv[1:] == ["--verify-index"]:
        verify_index()
    else:
        main()
