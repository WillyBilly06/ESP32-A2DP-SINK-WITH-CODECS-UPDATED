"""Test the optional probe's generated LHDC raw-session allocation lifecycle."""
import ctypes
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT.parent.parent / "LHDC-V5-Decoder-main/decoder"
OUT = ROOT / "_build/lhdc_probe_port"


def run(args, **kwargs):
    result = subprocess.run(args, capture_output=True, text=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f"command failed {args}\n{result.stdout[-3000:]}\n{result.stderr[-3000:]}")


def main():
    run([sys.executable, str(ROOT / "codecs/lhdcv5/prepare_port.py"),
         "--source", str(SOURCE), "--output", str(OUT)])
    env = os.environ.copy()
    if os.name == "nt":
        vc = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat")
        data = subprocess.check_output(f'call "{vc}" >nul && set', shell=True, text=True)
        env.update(dict(line.split('=', 1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    compiler = shutil.which("clang") or r"C:\Program Files\LLVM\bin\clang.exe"
    suffix = ".dll" if os.name == "nt" else ".so"
    binary = OUT / ("session_probe" + suffix)
    sources = [ROOT / "tests/codecs/lhdcv5_session_probe.c", ROOT / "codecs/lhdcv5/dbt_lhdcv5.c"]
    sources += [OUT / name for name in ("lhdc_dec.c", "lhdc_entropy_dec.c", "lhdc_imdct.c", "lhdc_sns_synth.c")]
    sources += [SOURCE / name for name in ("lhdc_bit_reader.c", "lhdc_diag_config.c", "lhdc_tables.c")]
    args = [compiler, "-std=c11", "-O2", "-fwrapv", "-shared", "-DLHDC_HOST_BUILD",
            "-DDBT_LHDC_NO_HOST_WINFILE", "-DDBT_LHDC_ALLOC_TEST"]
    if os.name != "nt":
        args += ["-fPIC"]
    for directory in (SOURCE, ROOT / "include", ROOT / "codecs/lhdcv5"):
        args += ["-I", str(directory)]
    run(args + [str(p) for p in sources] + ([] if os.name == "nt" else ["-lm"]) + ["-o", str(binary)], env=env)
    lib = ctypes.CDLL(str(binary))
    for name in ("probe_live", "probe_peak", "probe_calls"):
        getattr(lib, name).restype = ctypes.c_size_t
    lib.probe_fail_in.argtypes = [ctypes.c_size_t]
    observations = []
    for rate in (44100, 48000, 96000, 192000):
        lib.probe_peak_reset()
        before = lib.probe_calls()
        assert lib.probe_open(rate, 0) == 0
        count = lib.probe_calls() - before
        resident, open_peak = lib.probe_live(), lib.probe_peak()
        assert resident > 0 and lib.probe_open(rate, 1) != 0
        lib.probe_close(1)
        assert lib.probe_live() == resident
        lib.probe_close(0)
        assert lib.probe_live() == 0
        for i in range(1, count + 1):
            lib.probe_fail_in(i)
            assert lib.probe_open(rate, 0) != 0, (rate, i, count)
            assert lib.probe_live() == 0, (rate, i, lib.probe_live())
        lib.probe_fail_in(0)
        observations.append(dict(rate=rate, allocations=count, live_bytes=resident, open_peak_bytes=open_peak))
    assert lib.probe_open(96000, 0) == 0
    lib.probe_peak_reset()
    assert lib.probe_open(192000, 0) == 0
    switch_peak = lib.probe_peak()
    assert switch_peak == max(observations[2]["live_bytes"], observations[3]["open_peak_bytes"])
    lib.probe_close(0)
    assert lib.probe_live() == 0
    print(json.dumps(dict(passed=True, scope="raw-session lifecycle only; no valid encoded frame decoded",
                          configurations=observations, switch_peak_bytes=switch_peak), indent=2))


if __name__ == "__main__":
    main()
