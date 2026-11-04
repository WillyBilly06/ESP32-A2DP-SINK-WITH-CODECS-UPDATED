"""Compile the isolated SMP mbedTLS worker with the existing ESP32 SDK config.

This is a compile-only check, not a firmware link or a target execution.
"""
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / '_build' / 'esp32'
OUT = ROOT / '_build' / 'smp'
subprocess.run([sys.executable, str(ROOT/'tools/check_workspace.py')], check=True)
try:
    commands = json.loads((BUILD/'compile_commands.json').read_text())
    entry = next(e for e in commands if e['file'].replace('\\','/').endswith('/mbedtls/library/aes.c'))
    command = entry['command']
    compiler = command.split(' -D', 1)[0].strip('"')
    includes = re.findall(r'-I"([^"]+)"', command)
    OUT.mkdir(parents=True, exist_ok=True)
    args = [compiler, '-std=c11', '-Wall', '-Wextra', '-Werror', '-DESP_PLATFORM',
            '-DMBEDTLS_CONFIG_FILE="mbedtls/esp_config.h"', '-I', str(ROOT/'include')]
    for include in includes:
        args += ['-I', include]
    args += ['-I', str(ROOT.parent/'esp-idf/components/nvs_flash/include')]
    for unit in ('smp','smp_crypto','smp_bond','smp_guard','smp_dist','smp_features','smp_sc','smp_bearer','smp_session','smp_worker_mbedtls','smp_nvs'):
        folder='src/esp32' if unit in ('smp_worker_mbedtls','smp_nvs') else 'src'
        subprocess.run(args+['-c', str(ROOT/f'{folder}/dbt_{unit}.c'), '-o', str(OUT/f'{unit}_esp32.o')], check=True)
    sizes=OUT/'smp_sizes_esp32.o'
    subprocess.run(args+['-c',str(ROOT/'tests/smp/target_sizes.c'),'-o',str(sizes)],check=True)
    nm=Path(compiler).with_name('xtensa-esp32-elf-nm.exe')
    symbols=subprocess.run([str(nm),'-S','--size-sort',str(sizes)],check=True,text=True,capture_output=True).stdout
    for line in symbols.splitlines():
        fields=line.split()
        if len(fields)==4 and fields[3].startswith('dbt_size_'):
            print(f'{fields[3]}: {int(fields[1],16)} bytes')
    print('Isolated ESP32 SMP units compile passed; no target link or execution.')
finally:
    subprocess.run([sys.executable, str(ROOT/'tools/check_workspace.py')], check=True)
