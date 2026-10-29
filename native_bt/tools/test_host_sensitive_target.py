"""Compile the modified host unit with existing ESP32 probe flags; no link."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
source=root/'src/dbt_host.c'; out=root/'_build/host_sensitive'
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    before=hashlib.sha256(source.read_bytes()).hexdigest()
    entries=json.loads((root/'_build/esp32/compile_commands.json').read_text(encoding='utf-8'))
    entry=next(e for e in entries if Path(e['file']).resolve()==source.resolve())
    prefix=entry['command'].rsplit(' -o ',1)[0]
    out.mkdir(parents=True,exist_ok=True)
    command=prefix+' -Werror -o "'+str(out/'dbt_host_esp32.o')+'" -c "'+str(source)+'"'
    subprocess.run(command,cwd=entry['directory'],check=True)
    if hashlib.sha256(source.read_bytes()).hexdigest()!=before:
        raise RuntimeError('Host source changed during target compile')
    print('ESP32 host sensitive-ingress unit compiled; no target link or hardware run.')
finally:
    subprocess.run(check,check=True)
