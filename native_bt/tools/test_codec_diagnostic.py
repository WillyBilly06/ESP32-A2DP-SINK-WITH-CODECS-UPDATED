"""Execute real diagnostic entry/worker control with mocked IDF, no device access."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

root=Path(__file__).resolve().parents[1]
out=root/'_build/codec_diagnostic'
out.mkdir(parents=True,exist_ok=True)
env=os.environ.copy()
if os.name=='nt':
    vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
incs=[root/'tests/codec_diagnostic/shim',root/'tests/codecs/probe_audio_stubs',root/'include',root/'codecs/common',root/'esp32_probe/main']
tracked=[Path(__file__).resolve(),*list((root/'tests/codec_diagnostic').rglob('*')),
         *list((root/'esp32_probe/main').glob('probe_codec_*'))]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked if p.is_file()}
for test in ('return','worker'):
    lib=out/(test+('.dll' if os.name=='nt' else '.so'))
    args=[cc,'-std=c11','-O1','-g','-shared']+([] if os.name=='nt' else ['-fPIC'])
    args+=['-I'+str(p) for p in incs]+[str(root/f'tests/codec_diagnostic/{test}_host.c'),'-o',str(lib)]
    subprocess.run(args,env=env,check=True)
    loaded=ctypes.CDLL(str(lib))
    result=getattr(loaded,f'codec_diagnostic_{test}_tests')()
    assert result==0,f'{test} failed at C line {result}'
    print('PASS diagnostic '+test+' control; IDF/task/codec execution mocked')
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
(out/'results.json').write_text(json.dumps({'passed':True,'scope':'actual entry and worker control, mocked IDF; no target/RF evidence','source_sha256':before},indent=2)+'\n')
