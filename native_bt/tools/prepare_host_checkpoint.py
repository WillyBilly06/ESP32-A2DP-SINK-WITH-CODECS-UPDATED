"""Prepare a source-only review manifest; never reads flash dumps or SSH artifacts."""
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
NATIVE = ROOT / "native_bt"


def main():
    names = [
        "native_bt/.gitignore", "native_bt/CMakeLists.txt", "native_bt/README.md",
        "native_bt/esp32_probe/CMakeLists.txt", "native_bt/esp32_probe/sdkconfig.defaults",
        "native_bt/esp32_probe/main/probe_main.c", "native_bt/esp32_probe/main/CMakeLists.txt",
        "native_bt/esp32_probe/main/probe_gatt.c", "native_bt/esp32_probe/main/probe_gatt.h",
        "native_bt/esp32_probe/main/probe_security.c", "native_bt/esp32_probe/main/probe_security.h",
        "native_bt/esp32_probe/main/probe_echo.c", "native_bt/esp32_probe/main/probe_echo.h",
        "native_bt/esp32_probe/main/probe_sdp.c", "native_bt/esp32_probe/main/probe_sdp.h",
        "native_bt/esp32_probe/main/probe_ingress.c", "native_bt/esp32_probe/main/probe_ingress.h",
        "native_bt/esp32_probe/main/probe_audio.c", "native_bt/esp32_probe/main/probe_audio.h",
        "native_bt/esp32_probe/main/probe_output.c", "native_bt/esp32_probe/main/probe_output.h",
        "native_bt/esp32_probe/main/Kconfig.projbuild",
        "native_bt/esp32_probe/components/native_bt/CMakeLists.txt",
        "native_bt/esp32_probe/components/lhdcv5_decoder/CMakeLists.txt",
        "native_bt/esp32_probe/components/sbc_decoder/CMakeLists.txt",
        "native_bt/tools/test_host.py", "native_bt/tools/test_stack_linux.py",
        "native_bt/tools/check_workspace.py", "native_bt/tools/build_probe.ps1",
        "native_bt/tools/requirements-linux.txt", "native_bt/tools/generate_fixed_dsp_tables.py",
        "native_bt/tools/measure_avdtp_target.py",
        "native_bt/tools/test_sbc_linux.py", "native_bt/tools/verify_sbc_rf.py",
        "native_bt/tools/verify_sbc_output.py", "native_bt/tools/audit_audio_isr.py",
        "native_bt/tools/verify_firefox_output.py",
        "native_bt/tests/sbc_decoder_probe.c",
        "native_bt/docs/architecture.md", "native_bt/docs/PROGRESS.md",
        "native_bt/docs/RELEASE_READINESS.md", "native_bt/docs/ATT_GATT.md", "native_bt/docs/SDP.md",
        "native_bt/docs/CLASSIC_SECURITY.md", "native_bt/docs/RX_FLOW.md", "reference/README.md",
        "native_bt/docs/AVDTP.md",
        "native_bt/docs/AVDTP_SINK.md",
        "native_bt/docs/SBC_PIPELINE.md",
        "native_bt/docs/AUDIO_OUTPUT.md",
        "native_bt/codecs/sbc/README.md",
        "bt_audio_sink/main/dsp/fixed_dsp.c", "bt_audio_sink/main/dsp/fixed_dsp.h",
        "bt_audio_sink/main/dsp/fixed_dsp_tables.inc",
    ]
    for directory, pattern in [("include", "*.h"), ("src", "*.c"), ("tests", "test_*.c"),
                               ("codecs/lhdcv5", "dbt_lhdcv5.*")]:
        names += [path.relative_to(ROOT).as_posix() for path in (NATIVE / directory).glob(pattern)]
    names += [path.relative_to(ROOT).as_posix() for path in (NATIVE/"codecs/sbc").rglob("*")
              if path.is_file() and path.suffix in [".c", ".h", ".py"]]
    entries = []
    hashes = {}
    for name in sorted(names):
        path = ROOT / name
        if "_build" in path.parts or path.is_symlink():
            raise RuntimeError("Generated output or link in source manifest")
        raw = path.read_bytes()
        content = raw.decode("utf-8")
        if "PRIVATE KEY-----" in content or "192.168.63.129" in content:
            raise RuntimeError("Private material requires review: " + name)
        hashes[name] = hashlib.sha256(raw).hexdigest()
        entries.append({"path": name, "mode": "100644", "type": "blob", "content": content})
    normal = json.loads((NATIVE / "_build/host/results.json").read_text())
    linux = json.loads((NATIVE / "_build/host/linux_results.json").read_text())
    hardware = json.loads((NATIVE / "_build/hardware/COM6-probe-trial.json").read_text())
    live = json.loads((NATIVE / "_build/hardware/linux-gatt-live.json").read_text())
    sdp = json.loads((NATIVE / "_build/hardware/linux-sdp-live.json").read_text())
    classic = json.loads((NATIVE / "_build/hardware/linux-classic-bond-for-soak.json").read_text())
    dual = json.loads((NATIVE / "_build/hardware/linux-dual-rx-flow-1k-soak.json").read_text())
    short_dual = json.loads((NATIVE / "_build/hardware/linux-dual-rx-flow-1k-traced.json").read_text())
    classic_only = json.loads((NATIVE / "_build/hardware/linux-classic-only-control.json").read_text())
    isolation = json.loads((NATIVE / "_build/hardware/receive-path-isolation-rx-flow.json").read_text())
    bluez_dual = json.loads((NATIVE / "_build/hardware/linux-dual-mode.json").read_text())
    restored = json.loads((NATIVE / "_build/hardware/linux-bt-lab-restore.json").read_text())
    audio = json.loads((NATIVE / "_build/hardware/COM6-audio-trial.json").read_text())
    avdtp = json.loads((NATIVE / "_build/host/avdtp_target.json").read_text())
    sbc = json.loads((NATIVE / "_build/codecs/sbc_results.json").read_text())
    sbc_rf = json.loads((NATIVE / "_build/hardware/sbc-rf-validation.json").read_text())
    sbc_output = json.loads((NATIVE / "_build/hardware/sbc-output-validation.json").read_text())
    isr = json.loads((NATIVE / "_build/host/audio-isr.json").read_text())
    firefox = json.loads((NATIVE / "_build/hardware/firefox-output-validation.json").read_text())
    reconnect = json.loads((NATIVE / "_build/hardware/linux-profile-reconnect-fixed.json").read_text())
    assert avdtp["linked_in_probe"] and avdtp["sbc_decoder_linked"] and not avdtp["bluedroid_init_linked"]
    assert avdtp["probe_image_sha256"] == hardware["candidate_sha256"]
    assert sbc["passed"] and sbc["address_undefined_sanitizers"] and sbc["thread_sanitizer"]
    assert sbc_rf["passed"] and sbc_output["passed"]
    assert isr["passed"] and isr["firmware_sha256"] == hardware["candidate_sha256"]
    assert firefox["browser_workload_passed"] and firefox["firmware"]["candidate_sha256"] == hardware["candidate_sha256"]
    assert reconnect["return_code"] == 0 and reconnect["result"]["passed"]
    assert reconnect["firmware"]["candidate_sha256"] == hardware["candidate_sha256"]
    # Historical counter/output runs keep their own image identities.
    for observed, name in [(sbc_rf, "COM6-probe-sbc-counters.json"), (sbc_output, "COM6-probe-sbc-output.json")]:
        archived = json.loads((NATIVE / "_build/hardware" / name).read_text())
        assert observed["firmware"]["candidate_sha256"] == archived["candidate_sha256"]
    for key, digest in sbc["source_sha256"].items():
        assert hashlib.sha256((ROOT/key).read_bytes()).hexdigest() == digest, key
    for key, digest in avdtp["source_sha256"].items():
        assert hashlib.sha256((NATIVE / key).read_bytes()).hexdigest() == digest, key
    assert live["return_code"] == 0 and live["result"]["passed"]
    assert sdp["return_code"] == 0 and sdp["result"]["passed"]
    assert classic["return_code"] == 0 and classic["result"]["passed"]
    # Live observations retain their own firmware identity: do not relabel an
    # earlier LE/pairing/SDP image as the latest diagnostic build.
    for observed in [live, sdp, classic, dual, bluez_dual, short_dual, classic_only]:
        assert len(observed["firmware"]["candidate_sha256"]) == 64
        assert observed["firmware"]["boot_ready_marker"]
    assert restored["return_code"] == 0 and restored["result"]["restored"]
    assert (NATIVE / "_build/hardware/linux-bt-lab-restore.json").stat().st_mtime_ns > (
        NATIVE / "_build/hardware/linux-bt-lab-enable.json").stat().st_mtime_ns
    assert audio["boot_ready_marker"] and all(item["readback_matches"] for item in audio["writes"])
    assert (NATIVE / "_build/hardware/COM6-audio-trial.json").stat().st_mtime_ns > (
        NATIVE / "_build/hardware/COM6-probe-trial.json").stat().st_mtime_ns
    assert hashlib.sha256((NATIVE / "_build/esp32/native_bt_probe.bin").read_bytes()).hexdigest() == hardware["candidate_sha256"]
    assert normal["passed"] and linux["passed"] and hardware["boot_ready_marker"]
    # Evidence must refer to the exact tested sources, not merely an older pass.
    for key, digest in normal["source_sha256"].items():
        source = (NATIVE / key.replace("\\", "/")).resolve()
        assert hashlib.sha256(source.read_bytes()).hexdigest() == digest, key
    for key, digest in linux["uploaded_sha256"].items():
        if key.startswith(("src/", "include/", "tests/")):
            assert hashlib.sha256((NATIVE / key).read_bytes()).hexdigest() == digest, key
    markers = {}
    for line in (NATIVE / "_build/hardware/COM6-probe-trial.txt").read_text(encoding='utf-8').splitlines():
        if line.startswith('{"probe":'):
            value = json.loads(line)
            markers[value["probe"]] = value
    identified = markers["controller_identified"]
    def observation(value, fields):
        return {"firmware": value["firmware"], **{key: value["result"][key] for key in fields}}
    traffic_fields=["scope", "passed", "stage", "classic_socket_security", "both_connected_initial", "classic_failure",
        "error", "classic_completed_packets", "classic_completed_bytes", "received_notifications", "completed_gatt_writes",
        "completed_gatt_readbacks", "reliable_write_completed", "final_connected", "target_connected_after_cleanup",
        "elapsed_seconds", "traffic_elapsed_seconds", "overlap_seconds", "classic_sizes", "classic_rtt_max_ms",
        "gatt_write_rtt_median_ms", "gatt_write_rtt_max_ms"]
    def traffic(value):
        return {"firmware":value["firmware"],**{key:item for key,item in value["result"].items() if key in traffic_fields}}
    evidence = {
        "scope": "independent host development milestone; no production profiles/audio qualification",
        "native_groups": len(normal["tests"]), "all_native_groups_passed": True,
        "replacement_ready": False, "hardware_reliability_qualified": False,
        "linux_address_undefined_sanitizers": linux["address_undefined_passed"],
        "linux_thread_sanitizer": linux["thread_sanitizer_passed"],
        "audio_three_owner_frames": linux["audio_three_owner_frames"],
        "concurrent_packets": linux["concurrent_packets"],
        "avdtp_foundation": {"scope": "owned signaling/acceptor/SBC worker with experimental integer DSP/I2S output",
            "host_tested": True, "esp32_library_compiled": True,
            "linked_in_probe": True, "rf_tested": True, "audio_tested": False,
            "production_profile_ready": False, "all_requested_codecs_integrated": False,
            "sink_acceptor_lifecycle_host_tested": True,
            "sbc_configuration_validator_host_tested": True,
            "actual_decoder_worker_integrated": True,
            "paired_runtime_route_tested": True, "media_channel_manager_integrated": True,
            "i2s_output_integrated": True, "audio_sink_sdp_advertised": True,
            "metadata_bytes_target_abi": avdtp["metadata_bytes"],
            "source_sha256": avdtp["source_sha256"]},
        "sbc_decoder_tests": {key: value for key, value in sbc.items() if key != "source_sha256"},
        "sbc_rf_pcm_diagnostic": sbc_rf,
        "sbc_raw_source_output_diagnostic": sbc_output,
        "audio_isr_audit": isr,
        "firefox_output_diagnostic": firefox,
        "ordinary_profile_reconnect": {"firmware":reconnect["firmware"], **reconnect["result"]},
        "target": {"chip": "ESP32-D0WD-V3 revision 3.1", "controller_hci_version": 8,
                   "identification_commands_completed": 8, "gap_setup_commands_completed": 9,
                   "host_bytes": identified["host_bytes"], "runtime_bytes_including_host": identified["runtime_bytes"],
                   "probe_image_bytes": hardware["writes"][0]["bytes"],
                   "probe_image_sha256": hardware["candidate_sha256"],
                   "image_readback_matches": all(item["readback_matches"] for item in hardware["writes"]),
                   "original_application_preserved": hardware["original_application_preserved"],
                   "gatt_metadata_bytes": 784,
                   "classic_security_metadata_bytes": markers["classic_security_probe_ready"]["metadata_bytes"],
                   "sdp_metadata_bytes": markers["sdp_ready"]["metadata_bytes"],
                   "receive_flow": markers["rx_flow_ready"],
                   "ingress_fingerprint_tracing_enabled": "ingress_trace" in markers,
                   "configured_flash_frequency_mhz": 80,
                   "independent_le_rf_link_tested": True, "classic_rf_link_tested": True,
                   "simultaneous_classic_le_connected_observed": all(dual["result"].get("both_connected_initial", {}).values())
                       and bool(dual["result"].get("both_connected_initial")),
                   "simultaneous_traffic_passed": dual["result"]["passed"], "product_profiles_ready": False,
                   "equivalent_feature_comparison_complete": False},
        "live_ble_diagnostic": observation(live, [
            "scope", "connect_method", "services_resolved", "request_writes", "periodic_readbacks",
            "additional_lengths", "reliable_write_bytes", "notifications", "elapsed_seconds",
            "write_rtt_median_ms", "write_rtt_max_ms", "notification_gap_max_ms"]),
        "live_classic_pairing": observation(classic, ["scope", "passed", "elapsed_seconds", "paired", "bonded",
            "bredr_paired", "disconnected", "bond_retained_after_cleanup"]),
        "live_sdp": {"firmware": sdp["firmware"], **sdp["result"]},
        "latest_dual_diagnostic": {"requested_gatt_writes":1000,**traffic(dual)},
        "earlier_short_dual_pass": traffic(short_dual),
        "classic_only_control": observation(classic_only,["scope","passed","classic_socket_security",
            "classic_completed_packets","classic_completed_bytes","classic_sizes","le_disconnected_observations","le_connection_used"]),
        "failing_receive_sample": {key:isolation[key] for key in ["scope","firmware","failure","callback_sample",
            "callback_pdu_offset","expected_callback_fingerprint","zero_byte_callback_fingerprint","source_after_owner_equal","unresolved"]},
        "bluez_gatt_dual_diagnostic": {"firmware": bluez_dual["firmware"], **{key: value for key, value in bluez_dual["result"].items()
            if key in ["scope", "passed", "stage", "error", "completed_gatt_writes", "final_connected"]}},
        "test_vm_original_bluetooth_daemon_restored": True,
        "board_restored_to_bluedroid_audio_trial": True,
        "restored_audio_image_sha256": audio["candidate_sha256"],
        "source_sha256": hashes,
    }
    public = NATIVE / "docs/HOST_EVIDENCE.json"
    public.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
    entries.append({"path": public.relative_to(ROOT).as_posix(), "mode": "100644", "type": "blob",
                    "content": public.read_bytes().decode("utf-8")})
    manifest = NATIVE / "_build/host/github_manifest.json"
    manifest.write_text(json.dumps(entries), encoding="utf-8")
    print(json.dumps({"source_files": len(entries), "utf8_bytes": sum(len(e["content"].encode()) for e in entries),
                      "private_captures_included": False, "reference_implementation_included": False}))


def verify_index():
    entries = json.loads((NATIVE / "_build/host/github_manifest.json").read_text(encoding="utf-8"))
    request = "".join(":" + entry["path"] + "\n" for entry in entries).encode()
    result = subprocess.run(["git", "cat-file", "--batch"], input=request, capture_output=True, cwd=ROOT, check=True)
    stream = io.BytesIO(result.stdout)
    for entry in entries:
        header = stream.readline().split()
        assert len(header) == 3 and header[1] == b"blob", entry["path"]
        blob = stream.read(int(header[2]))
        assert blob == entry["content"].encode(), entry["path"]
        assert stream.read(1) == b"\n"
    assert not stream.read()
    print("All", len(entries), "staged public files match the reviewed manifest")


if __name__ == "__main__":
    if sys.argv[1:] == ["--verify-index"]:
        verify_index()
    else:
        main()
