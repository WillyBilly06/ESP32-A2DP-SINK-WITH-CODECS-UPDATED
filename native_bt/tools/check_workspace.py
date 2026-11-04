"""Read-only size/build-boundary audit. Never deletes user files."""
import json
import re
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent


def tree_bytes(path):
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file() and not item.is_symlink()) if path.exists() else 0


def main():
    reference = WORKSPACE / "reference"
    build = ROOT / "_build"
    reference_bytes=tree_bytes(reference)
    build_bytes=tree_bytes(build)
    # Physical captures, exact tested images/build receipts and the user's
    # original flash/NVS backup are retained evidence, not disposable compiler
    # intermediates. Keep their footprint visible without encouraging deletion
    # of the only record of a hardware test to make a build-size gate green.
    evidence_bytes=tree_bytes(build/'hardware')
    output_bytes=max(0,build_bytes-evidence_bytes)
    sizes = {"reference_mib": round(reference_bytes / 1048576, 3),
             "build_mib": round(build_bytes / 1048576, 3),
             "hardware_evidence_mib":round(evidence_bytes / 1048576,3),
             "rebuildable_mib":round(output_bytes / 1048576,3)}
    errors = []
    if reference_bytes > 96*1048576:
        errors.append("Reference exceeds 96 MiB; inspect sparse checkout before fetching or building")
    if output_bytes > 512*1048576:
        errors.append("Rebuildable outputs exceed 512 MiB; inspect/reuse existing build tree; retained hardware evidence is reported separately")
    for cmake in ROOT.rglob("CMakeLists.txt"):
        if "_build" in cmake.relative_to(ROOT).parts:
            continue
        content = cmake.read_text(encoding="utf-8").lower()
        if "btstack" in content or "reference/" in content or "file(glob" in content:
            errors.append(f"Unexpected reference dependency or source glob: {cmake.relative_to(ROOT)}")
    commands = build / "esp32/compile_commands.json"
    opus_root=WORKSPACE/'esp-idf/components/bt/host/bluedroid/external/opus'
    recipe=ROOT/'esp32_probe/components/opus_decoder/CMakeLists.txt'
    allowed_opus=set(re.findall(r'\$\{opus_root\}/([^"\n]+\.c)',recipe.read_text())) if recipe.exists() else set()
    if commands.exists():
        entries = json.loads(commands.read_text(encoding="utf-8"))
        for entry in entries:
            unit=Path(entry['file']).resolve()
            if opus_root.resolve() in unit.parents and unit.relative_to(opus_root.resolve()).as_posix() not in allowed_opus:
                errors.append(f"Unlisted standalone Opus source: {entry['file']}")
            normalized = str(entry).replace("\\", "/").lower()
            # The existing SDK's standalone Apache SBC codec is a codec-only
            # dependency. Its protocol wrapper/common host headers remain
            # prohibited. No SDK/reference implementation is copied here.
            normalized = normalized.replace("/host/bluedroid/external/sbc/decoder/srce/", "/standalone_sbc/src/")
            normalized = normalized.replace("/host/bluedroid/external/sbc/decoder/include", "/standalone_sbc/include")
            normalized=re.sub(r"/host/bluedroid/external/opus(?=/|[\"'\s])", "/standalone_opus", normalized)
            for codec_path in ('libaac-lc/libhelix_aac','libfreeaptx/main','libldac-dec'):
                normalized=re.sub('/host/bluedroid/external/'+re.escape(codec_path)+r"(?=/|[\"'\s])", '/standalone_codec',normalized)
            if "/reference/" in normalized or "/btstack/" in normalized or "/host/bluedroid/" in normalized or "/host/nimble/" in normalized:
                errors.append(f"Disallowed host/reference source or include in probe: {entry['file']}")
    report = {**sizes, "passed": not errors, "errors": errors,
              "scope": "disk and dependency-path check; not proof of implementation provenance or RF functionality"}
    print(json.dumps(report, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
