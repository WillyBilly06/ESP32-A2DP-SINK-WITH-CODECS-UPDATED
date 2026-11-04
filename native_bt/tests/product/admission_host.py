"""Bounded off-board admission and real SMP-adapter policy tests."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

root = Path(__file__).resolve().parents[2]
out = root / '_build/product'
out.mkdir(parents=True, exist_ok=True)
cc = shutil.which('clang') or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name == 'nt' else shutil.which('cc'))
common = [cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
          '-I' + str(root / 'tests/smp/probe_shim'), '-I' + str(root / 'include'),
          '-I' + str(root / 'esp32_probe/main'), '-I' + str(root.parent / 'bt_audio_sink/main/dsp')]
names = 'queue hci host acl l2cap credit ertm ertm_engine channels rx_flow runtime smp smp_crypto smp_bond smp_guard smp_dist smp_features smp_sc smp_bearer smp_session'.split()
security_source = (root / 'esp32_probe/main/probe_security.c').read_text(encoding='utf-8')
def function(signature):
    start = security_source.index(signature)
    brace = security_source.index('{', start)
    depth = 0
    for pos in range(brace, len(security_source)):
        if security_source[pos] == '{': depth += 1
        if security_source[pos] == '}':
            depth -= 1
            if not depth: return security_source[start:pos + 1]
    raise ValueError(signature)
(out / 'classic_admission_extracted.inc').write_text('\n'.join(function(x) for x in (
    'static void wipe(', 'static void copy(', 'static bool same(', 'static int cache_index(',
    'static int free_index(', 'static bool store(', 'static bool pairing(', 'bool probe_security_accept(', 'bool probe_security_quiescent('
)), encoding='utf-8')
for name, sources, flags in (
    ('admission', [root / 'tests/product/admission.c'], []),
    ('smp_admission_glue', [root / 'tests/product/smp_admission_glue.c'] + [root / f'src/dbt_{n}.c' for n in names], []),
    ('input_admission_glue', [root / 'tests/product/io_owner.c', root / 'esp32_probe/main/probe_product_io_owner.c'], ['-DDBT_PROBE_PRODUCT_SECURITY=1']),
    ('classic_product_glue', [root / 'tests/product/classic_admission_glue.c'], ['-DDBT_PROBE_PRODUCT_SECURITY=1', '-I' + str(out)]),
    ('classic_diagnostic_glue', [root / 'tests/product/classic_admission_glue.c'], ['-DDBT_PROBE_PAIRING_PEER_BYTES=1,2,3,4,5,6', '-I' + str(out)]),
):
    exe = out / (name + ('.exe' if os.name == 'nt' else ''))
    subprocess.run(common + flags + list(map(str, sources)) + ([] if os.name == 'nt' else ['-pthread']) + ['-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    print('PASS', name)
