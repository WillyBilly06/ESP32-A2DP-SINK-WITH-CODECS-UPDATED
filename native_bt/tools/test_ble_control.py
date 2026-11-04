"""Offline product ATT and NVS-shim tests; no device or network access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent
OUT = ROOT / '_build/ble_control'
CHECK = ROOT / 'tools/check_workspace.py'
NAMES = 'queue hci host acl l2cap credit ertm ertm_engine channels rx_flow runtime gap att att_bearer ble_control'.split()
SCHEMA_SOURCES = [ROOT / 'tests/ble_control/schema.c', ROOT / 'src/esp32/dbt_ble_nvs.c'] + [ROOT / f'src/dbt_{name}.c' for name in NAMES]
PROTOCOL_SOURCES = [ROOT / 'tests/ble_control/protocol.c', ROOT / 'src/dbt_ble_control.c']
INCLUDES = [ROOT / 'tests/ble_control/shim', ROOT / 'include']
INCLUDE_PATTERN = re.compile(r'^\s*#\s*include\s*["<]([^">]+)[">]', re.MULTILINE)


def inputs():
    """Resolve local transitive includes without needing a compiler or SDK."""
    pending = SCHEMA_SOURCES + PROTOCOL_SOURCES + [Path(__file__).resolve(), CHECK]
    found = set()
    while pending:
        path = pending.pop().resolve()
        if path in found:
            continue
        if not path.is_file():
            raise FileNotFoundError(str(path))
        found.add(path)
        if path.suffix not in ('.c', '.h'):
            continue
        for name in INCLUDE_PATTERN.findall(path.read_text(encoding='utf-8')):
            for directory in [path.parent, *INCLUDES]:
                candidate = (directory / name).resolve()
                if candidate.is_file() and candidate.is_relative_to(WORKSPACE):
                    pending.append(candidate)
                    break
    return sorted(found, key=lambda p: p.relative_to(WORKSPACE).as_posix())


def hashes(paths):
    return {p.relative_to(WORKSPACE).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list-inputs', action='store_true', help='Print workspace-root-relative inputs as JSON and exit.')
    parser.add_argument('--sanitize', action='store_true', help='Use Linux AddressSanitizer and UndefinedBehaviorSanitizer.')
    args = parser.parse_args()
    if args.list_inputs:
        paths = inputs()
        print(json.dumps({'workspace_relative_inputs': [p.relative_to(WORKSPACE).as_posix() for p in paths]}, indent=2))
        return 0

    OUT.mkdir(parents=True, exist_ok=True)
    result = {'schema': 1, 'passed': False, 'sanitize': args.sanitize,
              'scope': 'offline ATT/product parser and fake NVS/task queues; no target, RF, device or real NVS evidence',
              'source_sha256': {}, 'stage': 'preflight'}
    status = 1
    try:
        paths = inputs()
        before = hashes(paths)
        result['source_sha256'] = before
        subprocess.run([sys.executable, str(CHECK)], check=True)
        if args.sanitize and not sys.platform.startswith('linux'):
            raise RuntimeError('--sanitize requires Linux')
        env = os.environ.copy()
        if os.name == 'nt':
            vc = Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
            data = subprocess.check_output(f'call "{vc}" >nul && set', shell=True, text=True)
            env.update(dict(line.split('=', 1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
        cc = (shutil.which('clang') or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name == 'nt' else shutil.which('gcc')))
        if not cc:
            raise RuntimeError('C compiler not found')
        flags = ['-std=c11', '-O1', '-Wall', '-Wextra', '-Werror']
        if args.sanitize:
            flags += ['-g', '-fno-omit-frame-pointer', '-fsanitize=address,undefined', '-fno-sanitize-recover=all']
            env['ASAN_OPTIONS'] = 'detect_leaks=1:halt_on_error=1:abort_on_error=1'
            env['UBSAN_OPTIONS'] = 'halt_on_error=1:print_stacktrace=1'
        for name, sources, include_dirs in (('schema', SCHEMA_SOURCES, INCLUDES), ('protocol', PROTOCOL_SOURCES, [ROOT / 'include'])):
            exe = OUT / (name + ('.exe' if os.name == 'nt' else ''))
            result['stage'] = name + '_compile'
            subprocess.run([cc, *flags, *['-I' + str(x) for x in include_dirs], *map(str, sources), '-o', str(exe)], env=env, check=True)
            result['stage'] = name + '_run'
            subprocess.run([str(exe)], env=env, check=True)
        result['stage'] = 'source_stability'
        if hashes(paths) != before or inputs() != paths:
            raise RuntimeError('Source inputs changed during the test')
        result['stage'] = 'post_workspace_check'
        subprocess.run([sys.executable, str(CHECK)], check=True)
        result['passed'] = True
        result['stage'] = 'complete'
        status = 0
        print('ATT product UUIDs, CCCDs, worker queue, immediate NVS and stale generation passed')
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        result['failure_type'] = type(exc).__name__
        print(f'BLE control test failed at {result["stage"]}: {exc}', file=sys.stderr)
        try:
            subprocess.run([sys.executable, str(CHECK)], check=True)
        except (OSError, subprocess.CalledProcessError):
            result['post_workspace_check_failed'] = True
    finally:
        temporary = OUT / 'receipt.json.tmp'
        temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        temporary.replace(OUT / 'receipt.json')
    return status


if __name__ == '__main__':
    raise SystemExit(main())
