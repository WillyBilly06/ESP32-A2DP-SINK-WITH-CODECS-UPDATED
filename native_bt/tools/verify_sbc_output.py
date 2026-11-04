"""Bind the finite native SBC/I2S diagnostic to its source and firmware receipts."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HARDWARE = ROOT / "_build/hardware"


def read(name):
    return json.loads((HARDWARE / name).read_text(encoding="utf-8"))


def main():
    source = read("linux-native-sbc-output.json")
    archive = "COM6-probe-sbc-output.json"
    firmware = read(archive if (HARDWARE/archive).exists() else "COM6-probe-trial.json")
    window = read("sbc-output-COM6-115200.json")
    result = source["result"]
    assert source["return_code"] == 0 and result["passed"]
    assert source["firmware"]["candidate_sha256"] == firmware["candidate_sha256"]
    assert firmware["boot_ready_marker"] and firmware["original_application_preserved"]
    assert all(item["readback_matches"] for item in firmware["writes"])
    assert not window["byte_limit_reached"]
    assert window["start_unix"] < result["start_unix"] < result["end_unix"] < window["end_unix"]
    capture = HARDWARE / "sbc-output-COM6-115200.txt"
    rows = [json.loads(line) for line in capture.read_text(encoding="utf-8").splitlines()
            if line.startswith('{"probe":')]
    pcm = [row for row in rows if row["probe"] == "sbc_pcm"]
    output = [row for row in rows if row["probe"] == "native_i2s"]
    assert pcm and output
    assert all(row["drops"] == row["resets"] == 0 and row["i2s_output"] for row in pcm)
    final, final_output = pcm[-1], output[-1]
    assert final["frames"] == result["sent_frames"] == result["requested_frames"]
    assert final["bytes"] == result["expected_pcm_bytes"]
    assert final["fnv1a32"] == result["expected_pcm_fnv1a32"]
    assert final["media_packets"] == result["sent_media_packets"] and final["resident"] == 0
    assert any(row["running"] and row["dma_eofs"] for row in output)
    assert all(row["failed"] == row["descriptor_errors"] == 0 for row in output)
    assert final_output["accepted_frames"] == final["frames"] * 128
    assert final_output["running"] == final_output["requested_bytes"] == 0
    assert final_output["dma_source_frames"] == final_output["rendered_frames"]
    # The sender deliberately waits 250 ms after its last packet before Suspend.
    # Preserve terminal silence counters; never relabel them as zero underruns.
    assert (not final_output["underrun_frames"] or
            final_output["first_underrun_accepted_frames"] == final_output["accepted_frames"])
    assert result["att_writes"] >= 100 and result["att_readbacks"] >= 10
    assert result["notifications"] >= 100 and result["delay_reports"] > 0
    report = {
        "scope": "finite raw encrypted SBC source plus kernel ATT; native integer DSP/I2S digital counters",
        "passed": True, "firmware": source["firmware"],
        "sample_rate": result["rate"], "channels": result["channels"],
        "codec_cie": result["codec_cie"], "fixture_sha256": result["fixture_sha256"],
        "traffic_seconds": result["traffic_seconds"],
        "frames_per_packet": result["frames_per_packet"],
        "source_media_packets": result["sent_media_packets"], "sbc_frames": final["frames"],
        "pcm_bytes": final["bytes"], "pcm_fnv1a32": final["fnv1a32"],
        "fingerprint_limit": "32-bit rolling diagnostic, not cryptographic equality of every PCM byte",
        "att_writes": result["att_writes"], "att_readbacks": result["att_readbacks"],
        "notifications": result["notifications"], "delay_reports": result["delay_reports"],
        "last_reported_delay_100us": result["last_reported_delay_100us"],
        "delay_limit": "queue estimate, not measured physical latency",
        "recorded_decoder_drops": 0, "recorded_sequence_resets": 0,
        "maximum_reported_worker_requested_bytes": max(row["resident"] for row in pcm),
        "maximum_reported_output_requested_bytes": max(row["requested_bytes"] for row in output),
        "allocation_limit": "excludes allocator overhead, static metadata, task stacks and controller/SDK",
        "final_output": final_output,
        "underrun_limit": "first recorded only after all input accepted; sender has a deliberate 250 ms terminal pause",
        "uart_complete_window": True,
        "uart_capture_sha256": hashlib.sha256(capture.read_bytes()).hexdigest(),
        "firefox_playback": False, "acoustic_test": False, "physical_i2s_capture": False,
        "physical_nvs_stress": False, "all_codec_qualification": False,
        "generic_dual_mode_corruption_resolved": False,
    }
    (HARDWARE / "sbc-output-validation.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
