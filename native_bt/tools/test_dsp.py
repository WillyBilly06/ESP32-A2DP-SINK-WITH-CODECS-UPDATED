"""Spectral oracle for the actual integer DSP C core; desktop measurements only."""
import ast
import cmath
import ctypes
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    receipt = json.loads((ROOT / "_build/host/results.json").read_text())
    source = ROOT.parent / "bt_audio_sink/main/dsp/fixed_dsp.c"
    key = str(Path(os.path.relpath(source, ROOT)))
    if receipt["source_sha256"].get(key) != hashlib.sha256(source.read_bytes()).hexdigest():
        raise RuntimeError("Run test_host.py after changing the DSP source")
    library = ROOT / "_build/host" / ("native_bt_tests.dll" if os.name == "nt" else "native_bt_tests.so")
    lib = ctypes.CDLL(str(library))
    lib.dsp_test_buffer.argtypes = [ctypes.c_uint32, ctypes.c_int8, ctypes.c_int8, ctypes.c_int8,
                                     ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_int32), ctypes.c_uint32]
    lib.dsp_test_buffer.restype = ctypes.c_int
    lib.dsp_test_state_bytes.restype = ctypes.c_uint32
    spec = importlib.util.spec_from_file_location("fixed_coefficients", ROOT / "tools/generate_fixed_dsp_tables.py")
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    results = []
    for rate in [44100, 48000, 96000, 192000]:
        frames = rate * 2 // 5
        first = frames // 2
        for gains in [(0, 0, 0), (12, -8, 6), (-12, 12, -12)]:
            for frequency in [30, 1000, 10000]:
                angle = 2 * math.pi * frequency / rate
                amplitude = 0.05 * 2147483648
                values = [round(amplitude * math.sin(angle*i)) for i in range(frames)]
                source_data = (ctypes.c_int32 * frames)(*values)
                output = (ctypes.c_int32 * frames)()
                if lib.dsp_test_buffer(rate, *gains, source_data, output, frames):
                    raise RuntimeError("DSP refused a supported rate")
                sine_sum = cosine_sum = 0.0
                for i in range(first, frames):
                    sine_sum += output[i] * math.sin(angle*i)
                    cosine_sum += output[i] * math.cos(angle*i)
                actual = 2 * math.hypot(sine_sum, cosine_sum) / (frames-first) / amplitude
                z = cmath.exp(-1j * angle)
                expected = 1+0j
                for gain, kind, center, scale in zip(gains, ["low", "peak", "high"], [150, 1000, 6000], [0.5, 0.7, 0.7]):
                    c = ast.literal_eval(generator.coefficients(kind, rate, center, gain*scale).replace("{", "[").replace("}", "]"))
                    b0, b1, b2, a1, a2 = [value / (1 << 28) for value in c]
                    expected *= (b0+b1*z+b2*z*z)/(1+a1*z+a2*z*z)
                error_db = 20 * math.log10(actual / abs(expected))
                results.append({"rate": rate, "gains": gains, "tone_hz": frequency,
                                "magnitude_error_db": error_db, "passed": abs(error_db) <= 0.1})
    report = {"scope": "native CPU numerical behavior; not an ESP32 timing or audible-pop qualification",
              "passed": all(value["passed"] for value in results),
              "cases": len(results), "max_magnitude_error_db": max(abs(value["magnitude_error_db"]) for value in results),
              "state_bytes": lib.dsp_test_state_bytes(), "results": results,
              "tables_sha256": hashlib.sha256((source.parent / "fixed_dsp_tables.inc").read_bytes()).hexdigest()}
    destination = ROOT / "_build/dsp/results.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ["passed", "cases", "max_magnitude_error_db", "state_bytes", "scope"]}, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
