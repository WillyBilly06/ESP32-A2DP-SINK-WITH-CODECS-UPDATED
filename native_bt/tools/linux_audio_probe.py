"""Read-only Linux audio snapshot, optionally reconnecting the identified project peer.

Local authentication is interactive and never written to a file or command line.
The remote collector runs from stdin and leaves no source/capture files on Linux.
"""
import argparse
import getpass
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time


def remote_probe(args):
    import dbus

    os.environ["XDG_RUNTIME_DIR"] = "/run/user/" + str(os.getuid())
    os.environ["DBUS_SESSION_BUS_ADDRESS"] = "unix:path=" + os.environ["XDG_RUNTIME_DIR"] + "/bus"

    def run(command, timeout=8):
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
            return {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr[:2000]}
        except Exception as error:
            return {"error": str(error)}

    def managed():
        bus = dbus.SystemBus()
        interface = dbus.Interface(bus.get_object("org.bluez", "/"), "org.freedesktop.DBus.ObjectManager")
        return interface.GetManagedObjects(timeout=8)

    def target(objects):
        matches = [(str(path), values["org.bluez.Device1"]) for path, values in objects.items()
                   if "org.bluez.Device1" in values and
                   str(values["org.bluez.Device1"].get("Address", "")).upper() == args.address.upper()]
        if len(matches) != 1:
            raise RuntimeError("Expected exactly one BlueZ peer with the requested address")
        path, properties = matches[0]
        uuids = [str(value).lower() for value in properties.get("UUIDs", [])]
        if (str(properties.get("Name", "")) != "BDK SPEAKER" or
            "0000110b-0000-1000-8000-00805f9b34fb" not in uuids or
            "12345678-1234-1234-1234-123456789000" not in uuids):
            raise RuntimeError("Peer does not match this project's name + audio-sink + control-service identity")
        return path, properties

    objects = managed()
    path, properties = target(objects)
    report = {"user": getpass.getuser(), "hostname": os.uname().nodename,
              "kernel": os.uname().release, "mode": args.mode,
              "scope": "existing-firmware Linux baseline; independent host not deployed",
              "target": {"path": path, "address": args.address, "name": str(properties["Name"]),
                         "paired": bool(properties.get("Paired", False)),
                         "trusted": bool(properties.get("Trusted", False)),
                         "connected_before": bool(properties.get("Connected", False))}}
    if args.mode == "connect" and not report["target"]["connected_before"]:
        if not report["target"]["paired"] or not report["target"]["trusted"]:
            raise RuntimeError("Connect mode requires the existing paired and trusted project peer")
        report["connect_result"] = run(["bluetoothctl", "connect", args.address], timeout=20)
        time.sleep(2)
        objects = managed()
        path, properties = target(objects)
    report["target"]["connected_after"] = bool(properties.get("Connected", False))
    report["transports"] = []
    for transport_path, interfaces in objects.items():
        values = interfaces.get("org.bluez.MediaTransport1")
        if values is not None and str(values.get("Device", "")) == path:
            report["transports"].append({
                "path": str(transport_path), "uuid": str(values.get("UUID", "")),
                "codec_id": int(values.get("Codec", 0)),
                "configuration_hex": bytes(values.get("Configuration", [])).hex(),
                "state": str(values.get("State", "")),
                "delay_100us": int(values.get("Delay", 0)),
            })
    for kind in ["cards", "sinks", "sink-inputs"]:
        result = run(["pactl", "--format=json", "list", kind])
        try:
            report[kind] = [value for value in json.loads(result["stdout"])
                            if "bluez" in value.get("name", "").lower() or
                            "firefox" in str(value.get("properties", {})).lower()]
        except Exception:
            report[kind] = result
    report["audio_server"] = run(["pactl", "info"])
    result = run(["pw-dump"])
    try:
        keys = ["device.name", "node.name", "device.description", "api.bluez5.address",
                "api.bluez5.codec", "bluez5.codec", "api.bluez5.profile",
                "device.profile.name", "audio.rate", "audio.channels", "media.class"]
        report["bluez_nodes"] = [{"id": value.get("id"),
                                   "properties": {key: properties[key] for key in keys if key in properties}}
                                  for value in json.loads(result["stdout"])
                                  if (properties := value.get("info", {}).get("props", {})) and
                                  "bluez" in str(properties).lower()]
    except Exception as error:
        report["bluez_nodes"] = {"error": str(error)}
    session = dbus.SessionBus()
    report["firefox_players"] = []
    for name in session.list_names():
        if str(name).startswith("org.mpris.MediaPlayer2.") and "firefox" in str(name).lower():
            interface = dbus.Interface(session.get_object(name, "/org/mpris/MediaPlayer2"),
                                       "org.freedesktop.DBus.Properties")
            values = interface.GetAll("org.mpris.MediaPlayer2.Player", timeout=5)
            report["firefox_players"].append({"bus": str(name),
                                              "status": str(values.get("PlaybackStatus", "")),
                                              "position_us": int(values.get("Position", 0))})
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host")
    parser.add_argument("--user")
    parser.add_argument("--address", required=True)
    parser.add_argument("--mode", choices=["inspect", "connect"], default="inspect")
    parser.add_argument("--remote", action="store_true")
    args = parser.parse_args()
    if args.remote:
        print(json.dumps(remote_probe(args), indent=2))
        return 0
    if not args.host or not args.user:
        parser.error("--host and --user are required locally")
    root = Path(__file__).resolve().parents[1]
    dependency_dir = root / "_build/python_deps"
    if dependency_dir.is_dir():
        sys.path.insert(0, str(dependency_dir))
    try:
        import paramiko
    except ImportError:
        parser.error("Install tools/requirements-linux.txt into _build/python_deps")
    known_hosts = root / "_build/linux_known_hosts"
    if not known_hosts.is_file():
        parser.error("Verify the VM identity and add its SSH key to _build/linux_known_hosts first")
    password = getpass.getpass("Linux SSH password (session only): ")
    client = paramiko.SSHClient()
    client.load_host_keys(str(known_hosts))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(args.host, username=args.user, password=password,
                       look_for_keys=False, allow_agent=False,
                       timeout=8, auth_timeout=10, banner_timeout=8)
        password = None
        command = "python3 - " + shlex.join(["--remote", "--address", args.address, "--mode", args.mode])
        stdin, stdout, stderr = client.exec_command(command, timeout=40)
        stdin.write(Path(__file__).read_text(encoding="utf-8"))
        stdin.flush()
        stdin.channel.shutdown_write()
        raw = stdout.read(1048577)
        errors = stderr.read(16000).decode("utf-8", errors="replace")
        code = stdout.channel.recv_exit_status()
        if code != 0 or len(raw) > 1048576:
            raise RuntimeError(f"Remote collector failed ({code}): {errors}")
        report = json.loads(raw)
        output = root / "_build/linux/latest.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        summary = {key: report[key] for key in ["user", "hostname", "mode", "scope", "target", "transports", "firefox_players"]}
        summary["card_names"] = [value.get("name") for value in report.get("cards", []) if isinstance(value, dict)]
        print(json.dumps(summary, indent=2))
        print("Full snapshot: " + str(output))
    finally:
        password = None
        client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
