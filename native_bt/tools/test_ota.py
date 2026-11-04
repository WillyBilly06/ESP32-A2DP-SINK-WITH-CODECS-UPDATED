"""Offline OTA state/flash-shim test and ESP32 source compilation; no device."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

root=Path(__file__).resolve().parents[1];out=root/'_build/ota';check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    out.mkdir(parents=True,exist_ok=True);env=os.environ.copy()
    if os.name=='nt':
        vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    cc=shutil.which('clang') or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else shutil.which('gcc'))
    exe=out/('wire.exe' if os.name=='nt' else 'wire')
    subprocess.run([cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-I'+str(root/'include'),
                    str(root/'src/dbt_ota.c'),str(root/'src/dbt_ble_control.c'),str(root/'tests/ota/wire.c'),'-o',str(exe)],env=env,check=True)
    subprocess.run([str(exe)],env=env,check=True)
    exe=out/('selector_model.exe' if os.name=='nt' else 'selector_model')
    subprocess.run([cc,'-std=c11','-O1','-Wall','-Wextra','-Werror',str(root/'tests/ota/selector_model.c'),'-o',str(exe)],env=env,check=True)
    subprocess.run([str(exe)],env=env,check=True)
    exe=out/('esp_shim.exe' if os.name=='nt' else 'esp_shim')
    subprocess.run([cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-DCONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=1','-Dcalloc=ota_test_calloc',
                    '-I'+str(root/'tests/ota/shim'),'-I'+str(root/'include'),
                    str(root/'src/esp32/dbt_ota_esp.c'),str(root/'src/dbt_ota.c'),str(root/'src/dbt_ble_control.c'),
                    str(root/'tests/ota/esp_shim.c'),'-o',str(exe)],env=env,check=True)
    subprocess.run([str(exe)],env=env,check=True)
    entries=json.loads((root/'_build/esp32/compile_commands.json').read_text(encoding='utf-8'))
    entry=next(e for e in entries if Path(e['file']).name=='probe_main.c')
    prefix=entry['command'].rsplit(' -o ',1)[0]
    for source in (root/'src/dbt_ota.c',root/'src/esp32/dbt_ota_esp.c'):
        command=prefix+' -Werror -o "'+str(out/(source.stem+'.obj'))+'" -c "'+str(source)+'"'
        result=subprocess.run(command,cwd=entry['directory'],text=True,capture_output=True)
        if result.returncode:raise RuntimeError(source.name+':\n'+result.stderr[-8000:])
    source=root/'src/esp32/dbt_ota_esp.c'
    command=prefix+' -Werror -DCONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=1 -o "'+str(out/'dbt_ota_esp_rollback.obj')+'" -c "'+str(source)+'"'
    result=subprocess.run(command,cwd=entry['directory'],text=True,capture_output=True)
    if result.returncode:raise RuntimeError('rollback-enabled '+source.name+':\n'+result.stderr[-8000:])
    print('OTA state and flash-shim wire tests passed; ESP32 sources compiled, not linked or run.')
finally:
    subprocess.run(check,check=True)
