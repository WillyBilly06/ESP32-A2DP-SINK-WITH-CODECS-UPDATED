"""Credential-free Linux ASan/UBSan sound and storage-shim tests."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
INPUTS=['native_bt/include/dbt_queue.h','native_bt/include/dbt_sound.h','native_bt/include/dbt_sound_esp32.h',
        'native_bt/src/dbt_sound.c','native_bt/src/dbt_sound_mix.c','native_bt/src/esp32/dbt_sound_spiffs.c',
        'native_bt/tests/sound/test_sound.c','native_bt/tests/sound/test_spiffs_shim.c',
        'native_bt/tests/sound/test_sound_saturation.c','native_bt/tests/sound/test_sound_threads_linux.c',
        'native_bt/tests/sound/stubs/esp_spiffs.h','native_bt/tests/sound/stubs/esp_timer.h',
        'native_bt/tests/sound/stubs/nvs.h','native_bt/tests/sound/stubs/unistd.h',
        'native_bt/tools/check_workspace.py','native_bt/tools/test_sound_linux.py']

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--list-inputs',action='store_true');args=parser.parse_args()
    if args.list_inputs:
        print('\n'.join(INPUTS));return
    if not sys.platform.startswith('linux'):raise SystemExit('Linux GCC is required; --list-inputs works anywhere')
    cc=shutil.which('gcc');
    if not cc:raise SystemExit('GCC is required')
    out=ROOT/'_build/sound_linux';out.mkdir(parents=True,exist_ok=True)
    paths=[ROOT.parent/p for p in INPUTS];before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    env=os.environ.copy();env['ASAN_OPTIONS']='detect_leaks=1:halt_on_error=1:abort_on_error=1';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
    base=[cc,'-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-pthread','-I'+str(ROOT/'include')]
    core=[str(ROOT/'src/dbt_sound.c'),str(ROOT/'src/dbt_sound_mix.c')]
    for name,source,extra,inc in (
        ('sound','test_sound.c',core,[]),('threads','test_sound_threads_linux.c',core,[]),
        ('saturation','test_sound_saturation.c',[],[]),('storage','test_spiffs_shim.c',[core[0]],['-I'+str(ROOT/'tests/sound/stubs')])):
        exe=out/name;subprocess.run(base+inc+[str(ROOT/'tests/sound'/source)]+extra+['-o',str(exe)],check=True,env=env)
        subprocess.run([str(exe)],check=True,env=env,timeout=60)
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha for p,sha in before.items())
    (out/'results.json').write_text(json.dumps({'passed':True,'scope':'Linux host ASan/UBSan and storage shim; no device access',
                                                'source_sha256':before},indent=2)+'\n')
    print('Linux sound ASan/UBSan and in-memory transaction-recovery tests passed')

if __name__=='__main__':
    if '--list-inputs' in sys.argv:main()
    else:
        check=[sys.executable,str(ROOT/'tools/check_workspace.py')];subprocess.run(check,check=True)
        try:main()
        finally:subprocess.run(check,check=True)
