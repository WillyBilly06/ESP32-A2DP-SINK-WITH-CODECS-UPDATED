"""Compile the isolated probe adapter using the existing target build flags."""
import json
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
database=root/'_build/esp32/compile_commands.json'
out=root/'_build/smp'
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    entries=json.loads(database.read_text(encoding='utf-8'))
    entry=next(e for e in entries if Path(e['file']).name=='probe_main.c')
    prefix=entry['command'].rsplit(' -o ',1)[0]
    source=root/'esp32_probe/main/probe_smp.c'
    out.mkdir(parents=True,exist_ok=True)
    command=prefix+' -Werror -o "'+str(out/'probe_smp.obj')+'" -c "'+str(source)+'"'
    subprocess.run(command,cwd=entry['directory'],check=True)
    print('ESP32 probe SMP adapter compile passed (no target link or hardware run).')
finally:
    subprocess.run(check,check=True)
