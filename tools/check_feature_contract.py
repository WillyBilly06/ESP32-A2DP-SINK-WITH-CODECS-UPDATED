"""Check retained firmware protocol/configuration against a Git baseline and BDK.

This is a source/configuration check, not a hardware or audio-quality test.
"""
from pathlib import Path
import argparse
import json
import re
import subprocess


def constants(source, namespace):
    body = re.search(r"namespace " + namespace + r"\s*\{(.*?)\n\}", source, re.S).group(1)
    return dict((name, int(value, 16)) for name, value in
                re.findall(r"constexpr uint8_t\s+(\w+)\s*=\s*(0x[0-9A-Fa-f]+)", body))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--bdk-app", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    rel = "bt_audio_sink/main/ble/ble_unified.h"
    before = subprocess.check_output(["git", "-C", str(root), "show", f"{args.baseline}:{rel}"], text=True)
    after = (root / rel).read_text()
    assert '"%s BLE"' not in after, "Different Classic/BLE names can make Android's device name flicker"
    report = {"scope": "source and generated configuration only", "baseline": args.baseline}
    for namespace in ("BleCmd", "BleResp", "BleError"):
        expected, actual = constants(before, namespace), constants(after, namespace)
        assert actual == expected, (namespace, expected, actual)
        report[namespace] = actual
    # Every command still reaches a firmware dispatch branch.
    for command in report["BleCmd"]:
        assert f"case BleCmd::{command}:" in after, f"Missing command handler: {command}"
    if args.bdk_app:
        app = (args.bdk_app / "lib/protocol/speaker_protocol.dart").read_text()
        configured = (root / "bt_audio_sink/main/config/app_config.h").read_text()
        uuids = re.findall(r"12345678-1234-1234-1234-12345678900[0-3]", app)
        assert len(set(uuids)) == 4
        for uuid in set(uuids):
            assert uuid in configured, f"Missing configured UUID: {uuid}"
        report["bdk_commit"] = subprocess.check_output(["git", "-C", str(args.bdk_app), "rev-parse", "HEAD"], text=True).strip()
    config = (root / "bt_audio_sink/sdkconfig").read_text()
    features = ["BT_ENABLED", "BT_CLASSIC_ENABLED", "BT_BLUEDROID_ENABLED", "BT_BLE_ENABLED", "BT_A2DP_ENABLE",
                "BT_A2DP_AAC_DECODER", "BT_A2DP_APTX_DECODER", "BT_A2DP_LDAC_DECODER", "BT_A2DP_OPUS_DECODER",
                "LED_MATRIX_ENABLE", "ENCODER_ENABLE"]
    for feature in features:
        assert f"CONFIG_{feature}=y" in config, f"Disabled feature: {feature}"
    assert "CONFIG_FREERTOS_HZ=1000" in config
    assert "CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y" in config
    assert "CONFIG_DSP_OUT_FRAMES=256" in config  # DSP is always compiled; there is no DSP_ENABLE symbol.
    report["enabled_features"] = features
    report["result"] = "PASS"
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
