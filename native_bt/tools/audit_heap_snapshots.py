"""Read saved boot evidence only. Does not reset, flash or connect to a board."""
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "_build/hardware"


def snapshot(name, capture):
    receipt_path = EVIDENCE / (name + ".json")
    receipt = json.loads(receipt_path.read_text())
    capture_path = EVIDENCE / capture
    data = capture_path.read_bytes()
    text = data.decode("utf-8", errors="replace")
    match = re.search(r"ELF file SHA256:\s*([0-9a-f]+)", text)
    if not match or len(match[1]) < 8 or not receipt["candidate_elf_sha256"].startswith(match[1]):
        raise RuntimeError("Saved boot ELF marker does not match its receipt: " + name)
    if not receipt["boot_ready_marker"] or not all(item["readback_matches"] for item in receipt["writes"]):
        raise RuntimeError("Missing verified deployment/boot evidence: " + name)
    return text, {"receipt": receipt_path.name, "capture": capture_path.name,
                  "firmware_sha256": receipt["candidate_sha256"],
                  "elf_sha256": receipt["candidate_elf_sha256"],
                  "capture_sha256": hashlib.sha256(data).hexdigest()}


def main():
    # Archive names are deliberate. boot_capture inside older receipts names a
    # reusable COM6-probe-trial.bin that subsequent trials can overwrite.
    audio_text, audio = snapshot("COM6-audio-trial", "COM6-audio-trial.bin")
    native_text, native = snapshot("COM6-probe-parser", "COM6-probe-parser.txt")
    if "LIVE_DEVICE host=bluedroid" not in audio_text:
        raise RuntimeError("Audio boot host identity missing")
    stages = []
    for stage, free, largest, minimum in re.findall(
            r"HEAP\s+(\w+)\s+free=\s*(\d+) KB\s+largest=\s*(\d+) KB\s+min_ever=\s*(\d+) KB", audio_text):
        stages.append({"stage": stage, "free_kib_floor": int(free),
                       "largest_kib_floor": int(largest), "minimum_kib_floor": int(minimum)})
    if not stages or stages[-1]["stage"] != "after_bt_ready":
        raise RuntimeError("Incomplete audio boot heap checkpoints")
    markers = [json.loads(line) for line in native_text.splitlines()
               if line.startswith('{"probe":"controller_identified"')]
    if len(markers) != 1 or markers[0]["profiles_ready"] is not False:
        raise RuntimeError("Unexpected native identification checkpoint")
    native.update({key: markers[0][key] for key in ["internal_free", "internal_largest", "profiles_ready", "host_bytes", "runtime_bytes"]})
    native["checkpoint"] = "controller_identified; no active stream; incomplete product features"
    audio.update({"host": "bluedroid", "stages": stages})
    idle = re.search(r"LIVE1[^\r\n]*started=0 active=0[^\r\n]*heap=(\d+) largest=(\d+)", audio_text)
    if idle:
        audio["first_observed_idle"] = {"internal_free_bytes": int(idle[1]), "largest_bytes": int(idle[2])}
    report = {"scope": "historical saved hardware observations; not a live measurement or equivalent-feature benchmark",
              "audio_application": audio, "owned_probe": native,
              "btstack": {"free_kib_reported_by_user": [120, 130], "matched_build_evidence": False},
              "resource_advantage_proven": False}
    destination = ROOT / "_build/host/heap_audit.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
