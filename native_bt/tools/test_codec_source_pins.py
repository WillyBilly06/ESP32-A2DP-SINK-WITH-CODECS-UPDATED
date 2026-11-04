"""Test canonical source pinning without changing any SDK/decoder source."""
from pathlib import Path
import hashlib
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "_build/source_pin_checks"


def load(name):
    path = ROOT / "codecs" / name / "prepare_port.py"
    spec = importlib.util.spec_from_file_location("dbt_pin_" + name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    source = ROOT.parent / "esp-idf/components/bt/host/bluedroid/external/libaac-lc/libhelix_aac/aacdec.c"
    data = source.read_bytes().replace(b"\r\n", b"\n")
    module = load("aac")
    OUT.mkdir(parents=True, exist_ok=True)
    hashes = []
    for name, value in (("lf", data), ("crlf", data.replace(b"\n", b"\r\n"))):
        input_file = OUT / (name + ".c")
        input_file.write_bytes(value)
        report = module.generate(input_file, OUT / name)
        assert report["source_sha256"] == hashlib.sha256(data).hexdigest()
        hashes.append(report["generated_sha256"])
    assert hashes[0] == hashes[1]
    previous = (OUT / "lf/aacdec_guarded.c").read_bytes()
    bad = OUT / "mutated.c"
    bad.write_bytes(data + b'\n#error "unreviewed test input"\n')
    try:
        module.generate(bad, OUT / "lf")
    except RuntimeError as error:
        assert "Unreviewed Helix AAC source" in str(error)
    else:
        raise AssertionError("Unreviewed C source accepted")
    assert (OUT / "lf/aacdec_guarded.c").read_bytes() == previous
    result = {"passed": True, "scope": "source-pinning equivalence/rejection, not codec PCM execution",
              "lf_crlf_generated_identical": True, "mutated_source_rejected": True, "prior_output_preserved": True}
    (OUT / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
