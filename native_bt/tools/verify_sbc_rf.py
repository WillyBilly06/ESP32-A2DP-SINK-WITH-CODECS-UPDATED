"""Bind finite SBC RF/PCM observations to their exact firmware and UART window."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HARDWARE = ROOT / "_build/hardware"


def read(name):
    return json.loads((HARDWARE/name).read_text(encoding="utf-8"))


def main():
    source = read("linux-native-sbc.json")
    firmware = read("COM6-probe-trial.json")
    window = read("sbc-rf-COM6-115200.json")
    assert source["return_code"] == 0 and source["result"]["passed"]
    assert source["firmware"]["candidate_sha256"] == firmware["candidate_sha256"]
    assert firmware["boot_ready_marker"] and all(item["readback_matches"] for item in firmware["writes"])
    assert firmware["original_application_preserved"] and not window["byte_limit_reached"]
    result = source["result"]
    assert window["start_unix"] < result["start_unix"] and window["end_unix"] > result["end_unix"]
    capture = HARDWARE/"sbc-rf-COM6-115200.txt"
    rows = []
    for line in capture.read_text(encoding="utf-8").splitlines():
        if line.startswith('{"probe":"sbc_pcm",'):
            rows.append(json.loads(line))
    assert rows and max(row["frames"] for row in rows) == result["requested_frames"]
    assert all(row["drops"] == 0 and row["resets"] == 0 and row["i2s_output"] is False for row in rows)
    final = rows[-1]
    assert final["frames"] == result["sent_frames"] == result["requested_frames"]
    assert final["bytes"] == result["expected_pcm_bytes"]
    assert final["fnv1a32"] == result["expected_pcm_fnv1a32"]
    assert final["media_packets"] == result["sent_media_packets"] and final["resident"] == 0
    assert result["att_writes"] >= 100 and result["att_readbacks"] >= 10 and result["notifications"] >= 100
    report = {
        "scope": "finite encrypted AVDTP/SBC-to-PCM counters with concurrent kernel LE ATT; no I2S/Firefox/acoustic qualification",
        "passed": True, "firmware": source["firmware"],
        "sample_rate": result["rate"], "channels": result["channels"], "codec_cie": result["codec_cie"],
        "fixture_sha256": result["fixture_sha256"],
        "source_media_packets": result["sent_media_packets"], "source_frames": result["sent_frames"],
        "pcm_frames": final["frames"], "pcm_bytes": final["bytes"], "pcm_fnv1a32": final["fnv1a32"],
        "expected_rolling_fingerprint_matched": True,
        "fingerprint_limit": "FNV-1a-32 is a rolling diagnostic, not a cryptographic proof of every PCM byte",
        "traffic_seconds": result["traffic_seconds"], "att_writes": result["att_writes"],
        "att_readbacks": result["att_readbacks"], "notifications": result["notifications"],
        "recorded_worker_drops": 0, "recorded_sequence_resets": 0,
        "maximum_reported_worker_allocation_bytes": max(row["resident"] for row in rows),
        "worker_allocation_after_close": final["resident"],
        "uart_complete_window": True, "uart_capture_sha256": hashlib.sha256(capture.read_bytes()).hexdigest(),
        "i2s_output": False, "firefox_playback": False, "acoustic_test": False,
        "all_codec_qualification": False, "generic_dual_mode_corruption_resolved": False,
    }
    (HARDWARE/"sbc-rf-validation.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
