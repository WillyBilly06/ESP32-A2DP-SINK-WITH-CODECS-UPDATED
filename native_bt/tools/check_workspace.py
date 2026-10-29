"""Read-only size/build-boundary audit. Never deletes user files."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent


def tree_bytes(path):
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file() and not item.is_symlink()) if path.exists() else 0


def main():
    reference = WORKSPACE / "reference"
    build = ROOT / "_build"
    sizes = {"reference_mib": round(tree_bytes(reference) / 1048576, 3),
             "build_mib": round(tree_bytes(build) / 1048576, 3)}
    errors = []
    if sizes["reference_mib"] > 96:
        errors.append("Reference exceeds 96 MiB; inspect sparse checkout before fetching or building")
    if sizes["build_mib"] > 512:
        errors.append("Generated outputs exceed 512 MiB; inspect/reuse existing build tree")
    for cmake in ROOT.rglob("CMakeLists.txt"):
        if "_build" in cmake.relative_to(ROOT).parts:
            continue
        content = cmake.read_text(encoding="utf-8").lower()
        if "btstack" in content or "reference/" in content or "file(glob" in content:
            errors.append(f"Unexpected reference dependency or source glob: {cmake.relative_to(ROOT)}")
    commands = build / "esp32/compile_commands.json"
    if commands.exists():
        entries = json.loads(commands.read_text(encoding="utf-8"))
        for entry in entries:
            normalized = str(entry).replace("\\", "/").lower()
            # The existing SDK's standalone Apache SBC codec is a codec-only
            # dependency. Its protocol wrapper/common host headers remain
            # prohibited. No SDK/reference implementation is copied here.
            normalized = normalized.replace("/host/bluedroid/external/sbc/decoder/srce/", "/standalone_sbc/src/")
            normalized = normalized.replace("/host/bluedroid/external/sbc/decoder/include", "/standalone_sbc/include")
            if "/reference/" in normalized or "/btstack/" in normalized or "/host/bluedroid/" in normalized or "/host/nimble/" in normalized:
                errors.append(f"Disallowed host/reference source or include in probe: {entry['file']}")
    report = {**sizes, "passed": not errors, "errors": errors,
              "scope": "disk and dependency-path check; not proof of implementation provenance or RF functionality"}
    print(json.dumps(report, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
