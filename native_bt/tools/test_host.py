"""Compile and execute the actual portable C core. One reusable output directory."""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
TESTS = [
    "hci_framing", "command_bounds", "queue_wrap_and_copy",
    "control_reserve_preserves_order", "bootstrap_separate_pools",
    "bootstrap_shared_pool", "transport_and_credit_gates",
    "status_is_not_completion", "deadlines_wrap_and_credit_stall",
    "rejection_and_short_return", "requires_dual_mode_features",
    "rx_overrun_fails_closed", "bounded_dispatch_and_order",
    "bad_ingress_and_hardware_error", "decode_generated_lengths",
    "acl_shared_and_separate_credits", "acl_disconnect_and_handle_reuse",
    "acl_completed_event_transaction", "acl_admission_and_encryption",
    "acl_link_limits_and_enhanced_le",
    "l2cap_complete_pdu_zero_allocation", "l2cap_fragmented_and_interleaved",
    "l2cap_reassembly_limits_and_oom", "l2cap_disconnect_and_deadline",
    "l2cap_tx_backpressure_and_roundtrip", "l2cap_tx_credit_stall_and_stale_generation",
    "fixed_dsp_flat_and_integer_boundaries", "fixed_dsp_eq_transition_continuity",
    "fixed_dsp_controls_and_full_scale", "fixed_dsp_meter_and_db",
    "channels_outgoing_and_asymmetric_mtu", "channels_incoming_and_duplicate_requests",
    "channels_option_errors_hints_and_continuation", "channels_response_continuation_and_mtu_retry",
    "channels_timers_pending_and_unsent_responses", "channels_multi_command_backpressure_and_malformed",
    "channels_generation_and_pinned_bytes", "channels_information_security_fixed_and_le",
    "channels_reconfiguration_and_disconnect",
    "channels_limits_ids_and_protocol_rejections", "channels_timer_wrap_and_late_pending_bound",
    "commands_copy_credit_status_and_rejection", "commands_queue_timeout_abort_and_late_response",
    "runtime_paired_channel_and_fragmented_data", "runtime_dual_bearer_credit_fairness",
    "runtime_disconnect_pins_and_partial_expiry", "runtime_control_cross_link_scheduling",
    "gap_setup_and_explicit_visibility", "gap_security_negative_replies_and_queue_bounds",
    "gap_connection_admission_is_not_establishment",
    "att_database_validation_and_link_lifecycle", "att_discovery_mixed_uuid_and_groups",
    "att_read_blob_multiple_and_base_uuid", "att_mtu_bounds_and_errors",
    "att_security_and_notify_only_protection", "att_deferred_write_ack_and_stale_completion",
    "att_prepare_execute_atomic_and_cancel", "att_prepare_limits_oom_and_deferred_execute",
    "att_cccd_per_client_bonds_and_indications", "att_cccd_queued_transaction_and_error",
    "att_buffer_pins_oom_and_timeouts", "att_command_rules_and_bad_dynamic_read",
    "att_runtime_bearer_fragmentation_and_disconnect",
    "att_generated_requests_and_resource_reclamation",
    "classic_key_classification_and_configuration", "classic_pairing_policy_and_cached_key",
    "classic_ssp_persistence_and_encryption_gates", "classic_manual_confirmation_and_rejection",
    "classic_pin_passkey_and_timeout", "classic_oob_lazy_storage_and_bounds",
    "classic_key_rejection_and_short_key", "classic_cached_auth_and_verified_key_size",
    "classic_store_failure_timeout_and_forget", "classic_disconnect_generation_and_store_completion",
    "classic_rotation_waits_for_new_cipher_and_commit", "classic_stale_cipher_query_and_command_backpressure",
    "sensitive_queue_and_hci_buffer_lifetime",
    "runtime_channel_close_returns_borrowed_data",
    "classic_command_collision_waits_for_actual_events",
    "channels_incoming_waits_for_verified_security", "channels_security_duplicates_failure_and_timeout",
    "channels_outgoing_security_wait_and_cipher_gate",
    "sdp_database_and_elements", "sdp_uuid_search_and_network_order",
    "sdp_attributes_golden_and_nil_omission", "sdp_request_validation",
    "sdp_search_continuation_and_limits", "sdp_attribute_continuation_exact_allocations",
    "sdp_continuation_binding_and_database", "sdp_combined_lists_and_empty_match",
    "sdp_generated_inputs_and_output_bounds",
    "sdp_runtime_fragmentation_and_lazy_reply", "sdp_runtime_oom_backpressure_and_disconnect",
    "runtime_1021_byte_fragments_with_le_interleave",
    "classic_ssp_defers_local_security_demand",
    "rx_flow_setup_limits_and_capabilities", "rx_flow_acks_ignore_credits_and_retain_transport_bytes",
    "rx_flow_disconnect_generations_and_error_response", "runtime_rx_flow_counts_consumed_classic_and_le_packets",
    "runtime_sco_framing_status_and_explicit_route",
    "avdtp_control_correlation_and_lazy_storage", "avdtp_fragmented_duplex_and_transport_pins",
    "avdtp_receive_sequence_limits_and_abort", "avdtp_deadlines_wrap_and_pinned_timeout",
    "avdtp_oom_retries_and_response_priority", "avdtp_unknown_signals_and_command_backpressure",
    "avdtp_rtp_media_headers_and_bounds", "avdtp_generated_packets_reclaim_storage",
    "avdtp_runtime_fragmentation_credit_stall_and_ble", "avdtp_runtime_disconnect_and_stale_channel",
    "avdtp_reply_lengths_and_fragment_capacity", "avdtp_simultaneous_local_and_remote_transactions",
    "avdtp_capabilities_configuration_and_alternatives", "avdtp_capability_format_errors_and_generated_tlvs",
    "avdtp_runtime_reconfigured_mtu_fails_closed",
    "avdtp_sink_discovery_flash_caps_and_validation", "avdtp_sink_configuration_worker_and_admission_order",
    "avdtp_sink_start_suspend_close_and_media_gate", "avdtp_sink_ordered_list_prefix_and_rejection",
    "avdtp_sink_reconfigure_rollback_and_fatal_cleanup", "avdtp_sink_disconnect_waits_for_worker_drop",
    "avdtp_sink_abort_idle_invalid_and_active", "avdtp_sink_oom_refusal_and_resource_rejection",
    "avdtp_sink_deadlines_media_wrap_and_late_receipts", "avdtp_sink_delay_reporting_and_close_deadline",
    "a2dp_sbc_selection_errors_and_all_fields", "avdtp_sink_generated_requests_leave_no_allocations",
    "avdtp_sink_paired_runtime_media_channels_and_ble",
    "avdtp_sink_abort_supersedes_pending_worker_safely",
    "sbc_golden_crc_and_negotiated_format", "sbc_packet_atomic_validation_and_timestamps",
    "sbc_fragment_lifecycle_sequence_wrap_and_lazy_storage", "sbc_fragment_errors_deadlines_and_oom",
    "sbc_generated_inputs_keep_output_and_allocations_bounded",
    "a2dp_acceptor_admission_routing_and_retirement",
    "a2dp_acceptor_abrupt_close_and_readmission",
    "a2dp_acceptor_configured_acl_loss",
    "audio_formats_match_integer_dsp_reference", "audio_whole_blocks_wrap_and_silence_accounting",
    "audio_oom_cleanup_and_control_snapshot", "audio_adaptive_fraction_bounds_and_clock_limit",
    "audio_consumer_fade_exact_duration_and_full_scale",
]


def build(output):
    sources = sorted((ROOT / "src").glob("*.c")) + sorted((ROOT / "tests").glob("test_*.c"))
    sources += [ROOT.parent / "bt_audio_sink/main/dsp/fixed_dsp.c"]
    compiler = os.environ.get("CC") or shutil.which("clang") or shutil.which("cc")
    if not compiler and os.name == "nt":
        candidate = Path(os.environ["ProgramFiles"]) / "LLVM/bin/clang.exe"
        if candidate.is_file():
            compiler = str(candidate)
    if not compiler:
        raise RuntimeError("Install a C11 compiler or set CC to its executable path")
    args = [compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "include"),
            "-I", str(ROOT.parent / "bt_audio_sink/main/dsp")]
    if os.name == "nt":
        # No dependency on a Visual Studio installation or CRT startup code.
        args += ["-ffreestanding", "-fno-builtin", "-fno-stack-protector",
                 "-fuse-ld=lld", "-nostdlib", "-shared", "-Wl,/noentry"]
    else:
        args += ["-shared", "-fPIC"]
    args += [str(path) for path in sources] + ["-o", str(output)]
    subprocess.run(args, check=True, cwd=ROOT)
    return sources


def concurrent_stress(lib, count=20000):
    lib.stress_produce.argtypes = [ctypes.c_uint32]
    lib.stress_produce.restype = ctypes.c_int
    lib.stress_consume.restype = ctypes.c_int
    lib.stress_begin()
    errors = []
    stop = threading.Event()
    deadline = time.monotonic() + 20

    def produce():
        for sequence in range(count):
            while not stop.is_set():
                status = lib.stress_produce(sequence)
                if status == 0:
                    break
                if status != 3 or time.monotonic() > deadline:
                    errors.append(f"producer status={status} sequence={sequence}")
                    stop.set()
                    return
                time.sleep(0)
            if stop.is_set():
                return

    def consume():
        received = 0
        while received < count and not stop.is_set():
            status = lib.stress_consume()
            if status == 0:
                received += 1
            elif status != 1 or time.monotonic() > deadline:
                errors.append(f"consumer status={status} received={received}")
                stop.set()
            else:
                time.sleep(0)

    workers = [threading.Thread(target=produce, daemon=True), threading.Thread(target=consume, daemon=True)]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join(22)
    if any(worker.is_alive() for worker in workers):
        stop.set()
        errors.append("thread deadline exceeded")
    if errors:
        raise RuntimeError("; ".join(errors))
    return count


def audio_concurrent_stress(lib):
    if lib.audio_stress_begin():
        raise RuntimeError("Audio stress initialization failed")
    lib.audio_stress_produce.argtypes = [ctypes.c_uint32]
    lib.audio_stress_rendered.restype = ctypes.c_uint32
    errors = []
    stopped = threading.Event()
    deadline = time.monotonic() + 20

    def producer():
        for sequence in range(0, 16384, 64):
            while not stopped.is_set():
                result = lib.audio_stress_produce(sequence)
                if result == 0:
                    break
                if result != 5 or time.monotonic() > deadline:
                    errors.append("PCM producer failed: " + str(result)); stopped.set(); return
                time.sleep(0)

    def renderer():
        while not stopped.is_set() and lib.audio_stress_rendered() < 16384:
            if not lib.audio_stress_render():
                if time.monotonic() > deadline:
                    errors.append("Render deadline"); stopped.set(); return
                time.sleep(0)

    def consumer():
        blocks = 0
        while not stopped.is_set() and blocks < 256:
            result = lib.audio_stress_consume()
            if result == 0:
                blocks += 1
            elif result != 1 or time.monotonic() > deadline:
                errors.append("DMA consumer/reference mismatch: " + str(result)); stopped.set(); return
            else:
                time.sleep(0)

    workers = [threading.Thread(target=fn, daemon=True) for fn in [producer, renderer, consumer]]
    for worker in workers: worker.start()
    for worker in workers: worker.join(22)
    if any(worker.is_alive() for worker in workers):
        raise RuntimeError("Audio owner thread did not stop")
    if lib.audio_stress_end(): errors.append("Audio counters or allocation cleanup failed")
    if errors: raise RuntimeError("; ".join(errors))
    if lib.audio_snapshot_begin(): raise RuntimeError("Snapshot stress initialization failed")
    finished = threading.Event()

    def write_controls():
        for n in range(10000):
            if lib.audio_snapshot_write(n % 25 - 12):
                errors.append("Snapshot writer failed"); break
        finished.set()

    control = threading.Thread(target=write_controls, daemon=True); control.start()
    while not finished.is_set():
        if lib.audio_snapshot_read(): errors.append("Torn control tuple"); break
    control.join(10)
    if control.is_alive(): raise RuntimeError("Snapshot owner did not stop")
    if lib.audio_snapshot_end(): errors.append("Snapshot cleanup failed")
    if errors: raise RuntimeError("; ".join(errors))
    return 16384


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=ROOT / "_build/host")
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()
    # Keep artifacts inside the one project-owned tree even on accidental invocation.
    output_dir.relative_to(ROOT / "_build")
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / ("native_bt_tests.dll" if os.name == "nt" else "native_bt_tests.so")
    sources = build(output)
    lib = ctypes.CDLL(str(output))
    results = []
    for name in TESTS:
        function = getattr(lib, "test_" + name)
        function.restype = ctypes.c_int
        result = function()
        results.append({"name": name, "passed": result == 0, "failed_line": result or None})
        print(f"{'PASS' if result == 0 else 'FAIL'} {name}" + (f" line {result}" if result else ""), flush=True)
    try:
        packets = concurrent_stress(lib)
        results.append({"name": "concurrent_spsc", "passed": True, "packets": packets})
        print(f"PASS concurrent_spsc ({packets} packets on two threads)", flush=True)
    except RuntimeError as error:
        results.append({"name": "concurrent_spsc", "passed": False, "error": str(error)})
    try:
        frames = audio_concurrent_stress(lib)
        results.append({"name": "audio_three_owners_and_controls", "passed": True, "frames": frames})
        print(f"PASS audio_three_owners_and_controls ({frames} frames, separate producer/render/consumer)", flush=True)
    except RuntimeError as error:
        results.append({"name": "audio_three_owners_and_controls", "passed": False, "error": str(error)})
    lib.test_host_bytes.restype = ctypes.c_uint32
    lib.test_queue_bytes.restype = ctypes.c_uint32
    report = {
        "scope": "native C behavioral tests only; no firmware or RF validation",
        "passed": all(item["passed"] for item in results),
        "host_bytes_native_abi": lib.test_host_bytes(),
        "queue_bytes_native_abi": lib.test_queue_bytes(),
        "tests": results,
        "source_sha256": {str(Path(os.path.relpath(path, ROOT))): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in sources + sorted((ROOT / "include").glob("*.h")) +
                          list((ROOT.parent / "bt_audio_sink/main/dsp").glob("fixed_dsp.*")) +
                          [ROOT.parent / "bt_audio_sink/main/dsp/fixed_dsp_tables.inc"]},
    }
    (output_dir / "results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"{sum(item['passed'] for item in results)}/{len(results)} passed; host object {report['host_bytes_native_abi']} bytes (native ABI)")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
