"""Compile product admission glue with the existing ESP32 build flags only."""
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
entries = json.loads((root / '_build/esp32/compile_commands.json').read_text(encoding='utf-8'))
entry = next(e for e in entries if Path(e['file']).name == 'probe_main.c')
prefix = entry['command'].rsplit(' -o ', 1)[0]
out = root / '_build/product'
out.mkdir(parents=True, exist_ok=True)
for name in ('probe_admission.c', 'probe_security.c', 'probe_smp.c',
             'probe_product_io_owner.c', 'probe_main.c', '../../src/esp32/dbt_input_esp32.c', '../../src/dbt_runtime.c'):
    source = root / 'esp32_probe/main' / name
    command = prefix + ' -Werror -DDBT_PROBE_PRODUCT_SECURITY=1 -o "' + str(out / (source.stem + '_admission.obj')) + '" -c "' + str(source) + '"'
    result = subprocess.run(command, cwd=entry['directory'], text=True, capture_output=True)
    if result.returncode:
        raise SystemExit(name + ':\n' + result.stderr[-8000:])
    print('PASS target compile', name)
