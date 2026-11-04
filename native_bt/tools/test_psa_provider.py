"""Execute the actual PSA software provider and owned adapters on the host.

No ESP32, controller, radio or physical pairing access. Target hardware drivers
remain a separate validation gate. One reusable output tree and source receipt.
"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "_build/crypto_psa"


def digest(paths):
    return {str(p.relative_to(ROOT.parent)).replace("\\", "/"): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def main():
    env = os.environ.copy()
    if os.name == "nt":
        vswhere = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
        installation = subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"], text=True).strip()
        vcvars = Path(installation) / "VC/Auxiliary/Build/vcvars64.bat"
        if not vcvars.is_file():
            raise RuntimeError("Visual Studio C++ toolchain is required for the Windows PSA host checks")
        values = subprocess.check_output('call "' + str(vcvars) + '" >nul && set', shell=True, text=True)
        env.update(dict(line.split("=", 1) for line in values.splitlines() if "=" in line and not line.startswith("=")))
        compiler = Path(os.environ["ProgramFiles"]) / "LLVM/bin/clang-cl.exe"
        cmake = Path.home() / ".espressif/tools/cmake/3.30.2/bin/cmake.exe"
        ninja = Path.home() / ".espressif/tools/ninja/1.12.1/ninja.exe"
        binary = OUT / "dbt_psa_host_checks.exe"
    else:
        compiler = Path(os.environ.get("CC") or shutil.which("cc"))
        cmake = Path(shutil.which("cmake"))
        ninja = Path(shutil.which("ninja"))
        binary = OUT / "dbt_psa_host_checks"
    provider = ROOT.parent / "esp-idf/components/mbedtls/mbedtls/tf-psa-crypto"
    inputs = [Path(__file__).resolve(), ROOT / "include/dbt_crypto.h", ROOT / "include/dbt_sha256.h",
              ROOT / "src/esp32/dbt_smp_worker_mbedtls.c", ROOT / "src/dbt_smp_crypto.c"]
    inputs += sorted((ROOT / "include").glob("*.h"))
    inputs += sorted((ROOT / "tests/crypto_psa").glob("*"))
    inputs += sorted(p for p in provider.rglob("*") if p.is_file() and p.suffix in (".c", ".h"))
    before = digest(inputs)
    OUT.mkdir(parents=True, exist_ok=True)
    commands = [[str(cmake), "-S", str(ROOT / "tests/crypto_psa"), "-B", str(OUT), "-G", "Ninja",
                 "-DCMAKE_C_COMPILER=" + str(compiler), "-DCMAKE_MAKE_PROGRAM=" + str(ninja), "-DCMAKE_BUILD_TYPE=Release"],
                [str(cmake), "--build", str(OUT), "--parallel", "8"], [str(binary)]]
    ccache = shutil.which("ccache")
    if not ccache and os.name == "nt":
        available = sorted((Path.home() / ".espressif/tools/ccache").rglob("ccache.exe"))
        if available:
            ccache = str(available[-1])
    if ccache:
        commands[0].append("-DCMAKE_C_COMPILER_LAUNCHER=" + ccache)
    output = ""
    for command in commands:
        process = subprocess.run(command, env=env, capture_output=True, text=True)
        output += process.stdout + process.stderr
        if process.returncode:
            (OUT / "run.log").write_text(output)
            raise RuntimeError(output[-7000:])
    stable = before == digest(inputs)
    report = {"passed": stable, "scope": "host software PSA provider and actual owned worker; not ESP32 hardware, RF or pairing",
              "source_stable": stable, "checks": [line for line in output.splitlines() if line.startswith("PASS ")],
              "source_sha256": before}
    (OUT / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    (OUT / "run.log").write_text(output)
    print("\n".join(report["checks"]))
    if not stable:
        raise RuntimeError("PSA check inputs changed during compilation or execution")


if __name__ == "__main__":
    main()
