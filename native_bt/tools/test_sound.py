"""Offline owned sound-provider and overlay checks; no target/device access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/sound';CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]
sys.path.insert(0,str(ROOT/'tools'))
from test_native_codec_target_components import argv_windows,run

def main(sanitize=''):
    OUT.mkdir(parents=True,exist_ok=True);env=os.environ.copy()
    if os.name=='nt':
        vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        lines=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(x.split('=',1) for x in lines.splitlines() if '=' in x and not x.startswith('=')))
    cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
    if sanitize and os.name=='nt':env['PATH']=str(Path(cc).parent.parent/'lib/clang/22/lib/windows')+os.pathsep+env.get('PATH','')
    sanitizer=[f'-fsanitize={sanitize}','-fno-sanitize-recover=all','-fno-omit-frame-pointer'] if sanitize else []
    sources=[ROOT/'src/dbt_sound.c',ROOT/'src/dbt_sound_mix.c',ROOT/'src/esp32/dbt_sound_spiffs.c',
             ROOT/'include/dbt_sound.h',ROOT/'include/dbt_sound_esp32.h',ROOT/'tests/sound/test_sound.c',Path(__file__).resolve()]
    sources += [ROOT/'tests/sound/test_spiffs_shim.c',ROOT/'tests/sound/test_sound_saturation.c',ROOT/'tests/sound/test_sound_threads.c',
                ROOT/'tests/sound/test_sound_threads_linux.c',ROOT/'tools/test_sound_linux.py']+list((ROOT/'tests/sound/stubs').glob('*.h'))
    sources += [ROOT/'include/dbt_queue.h',ROOT/'_build/esp32/config/sdkconfig.h',
                ROOT.parent/'esp-idf/components/spiffs/include/esp_spiffs.h',
                ROOT.parent/'esp-idf/components/nvs_flash/include/nvs.h']
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    exe=OUT/('sound_test.exe' if os.name=='nt' else 'sound_test')
    host=[cc,'-std=c11','-O2','-Wall','-Wextra','-Werror']+sanitizer+['-I'+str(ROOT/'include'),str(sources[0]),str(sources[1]),str(sources[5]),'-o',str(exe)]
    subprocess.run(host,env=env,check=True,cwd=OUT)
    subprocess.run([str(exe)],env=env,check=True,cwd=OUT)
    shim=OUT/('spiffs_shim.exe' if os.name=='nt' else 'spiffs_shim')
    subprocess.run([cc,'-std=c11','-O2','-Wall','-Wextra','-Werror']+sanitizer+['-I'+str(ROOT/'tests/sound/stubs'),
                    '-I'+str(ROOT/'include'),str(ROOT/'tests/sound/test_spiffs_shim.c'),str(ROOT/'src/dbt_sound.c'),'-o',str(shim)],env=env,check=True,cwd=OUT)
    subprocess.run([str(shim)],env=env,check=True,cwd=OUT)
    sat=OUT/('saturation.exe' if os.name=='nt' else 'saturation')
    subprocess.run([cc,'-std=c11','-O2','-Wall','-Wextra','-Werror']+sanitizer+['-I'+str(ROOT/'include'),
                    str(ROOT/'tests/sound/test_sound_saturation.c'),'-o',str(sat)],env=env,check=True,cwd=OUT)
    subprocess.run([str(sat)],env=env,check=True,cwd=OUT)
    if os.name=='nt':
        threads=OUT/'sound_threads.exe'
        subprocess.run([cc,'-std=c11','-O2','-Wall','-Wextra','-Werror']+sanitizer+['-I'+str(ROOT/'include'),
                        str(sources[0]),str(sources[1]),str(ROOT/'tests/sound/test_sound_threads.c'),'-o',str(threads)],env=env,check=True,cwd=OUT)
        subprocess.run([str(threads)],env=env,check=True,cwd=OUT)
    commands=json.loads((ROOT/'_build/esp32/compile_commands.json').read_text())
    entry=next(x for x in commands if x['file'].replace('\\','/').endswith('/probe_main.c'))
    args=argv_windows(entry['command']);args=args[:args.index('-o')]
    args+=['-I'+str(ROOT/'include'),'-I'+str(ROOT.parent/'esp-idf/components/spiffs/include')]
    for source in sources[:3]:
        run(args+['-Werror','-fstack-usage','-o',str(OUT/(source.stem+'.o')),'-c',str(source)],cwd=entry['directory'])
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
    (OUT/('sanitized_results.json' if sanitize else 'results.json')).write_text(json.dumps({'passed':True,'sanitizers':sanitize,'scope':'host shim and ESP32 object compilation only; no device/filesystem writes',
        'host':'passed','target':'three objects compiled; no firmware link or target execution','sources':before},indent=2)+'\n')
    print('ESP32 sound objects compiled; no target link or execution')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--sanitize',choices=('undefined','address,undefined'),default='');options=parser.parse_args()
    subprocess.run(CHECK,check=True)
    try:main(options.sanitize)
    finally:subprocess.run(CHECK,check=True)
