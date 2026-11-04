"""Offline BLE product mux tests and target-unit compile; no hardware."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
root=Path(__file__).resolve().parents[1];out=root/'_build/product';check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    out.mkdir(parents=True,exist_ok=True);env=os.environ.copy()
    if os.name=='nt':
        vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    cc=shutil.which('clang') or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else shutil.which('gcc'))
    exe=out/('product.exe' if os.name=='nt' else 'product')
    include=[root/'tests/product/shim',root/'tests/ota/shim',root/'include',root/'esp32_probe/main']
    sources=[root/'tests/product/product.c',root/'esp32_probe/main/probe_product.c',root/'esp32_probe/main/probe_product_runtime.c',root/'src/dbt_product_peer.c',
             root/'src/dbt_ota.c',root/'src/esp32/dbt_ota_esp.c',root/'src/dbt_ble_control.c',root/'src/dbt_boot_policy.c']
    subprocess.run([cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-Dcalloc=ota_test_calloc',
                    *['-I'+str(x) for x in include],*map(str,sources),'-o',str(exe)],env=env,check=True)
    subprocess.run([str(exe)],env=env,check=True)
    race=out/('peer_race.exe' if os.name=='nt' else 'peer_race')
    subprocess.run([cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-I'+str(root/'include'),
                    str(root/'tests/product/peer_race.c'),str(root/'src/dbt_product_peer.c'),
                    *([] if os.name=='nt' else ['-pthread']),'-o',str(race)],env=env,check=True)
    subprocess.run([str(race)],env=env,check=True)
    owner=out/('io_owner.exe' if os.name=='nt' else 'io_owner')
    subprocess.run([cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-I'+str(root/'include'),'-I'+str(root/'esp32_probe/main'),'-I'+str(root.parent/'bt_audio_sink/main/dsp'),
                    str(root/'tests/product/io_owner.c'),str(root/'esp32_probe/main/probe_product_io_owner.c'),'-o',str(owner)],env=env,check=True)
    subprocess.run([str(owner)],env=env,check=True)
    entries=json.loads((root/'_build/esp32/compile_commands.json').read_text(encoding='utf-8'))
    entry=next(e for e in entries if Path(e['file']).name=='probe_main.c');prefix=entry['command'].rsplit(' -o ',1)[0]
    for source in (root/'src/dbt_product_peer.c',root/'esp32_probe/main/probe_product.c',root/'esp32_probe/main/probe_product_runtime.c',root/'esp32_probe/main/probe_main.c'):
        command=prefix+' -Werror -o "'+str(out/(source.stem+'.obj'))+'" -c "'+str(source)+'"'
        result=subprocess.run(command,cwd=entry['directory'],text=True,capture_output=True)
        if result.returncode:raise RuntimeError(source.name+':\n'+result.stderr[-8000:])
    for source in (root/'esp32_probe/main/probe_main.c',root/'esp32_probe/main/probe_product_io_owner.c'):
        command=prefix+' -Werror -DDBT_PROBE_PRODUCT_IO=1 -DDBT_PROBE_SOUND_BUILD=1 -I"'+str(root.parent/'esp-idf/components/spiffs/include')+'" -o "'+str(out/(source.stem+'_io.obj'))+'" -c "'+str(source)+'"'
        result=subprocess.run(command,cwd=entry['directory'],text=True,capture_output=True)
        if result.returncode:raise RuntimeError(source.name+' IO:\n'+result.stderr[-8000:])
    print('BLE product authorization, OTA, durable name and stale-generation shims passed; ESP32 units compiled only.')
finally:
    subprocess.run(check,check=True)
