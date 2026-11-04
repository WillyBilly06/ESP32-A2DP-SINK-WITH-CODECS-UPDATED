"""Compile product BLE adapter and provider using existing ESP32 flags."""
import json
from pathlib import Path
import subprocess
import sys
root=Path(__file__).resolve().parents[1]
out=root/'_build/ble_control'
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    entries=json.loads((root/'_build/esp32/compile_commands.json').read_text(encoding='utf-8'))
    entry=next(e for e in entries if Path(e['file']).name=='probe_main.c')
    prefix=entry['command'].rsplit(' -o ',1)[0];out.mkdir(parents=True,exist_ok=True)
    for source in (root/'esp32_probe/main/probe_gatt.c',root/'src/dbt_ble_control.c',root/'src/esp32/dbt_ble_nvs.c'):
        command=prefix+' -Werror -o "'+str(out/(source.stem+'.obj'))+'" -c "'+str(source)+'"'
        result=subprocess.run(command,cwd=entry['directory'],text=True,capture_output=True)
        if result.returncode: raise RuntimeError(f'{source.name}:\n{result.stderr[-5000:]}')
    print('ESP32 product BLE units compiled; no target link or hardware.')
finally:
    subprocess.run(check,check=True)
