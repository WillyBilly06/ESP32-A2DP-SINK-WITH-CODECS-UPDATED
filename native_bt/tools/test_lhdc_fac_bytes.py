"""Differential execution against the unmodified decoder's actual bit reader."""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--decoder',type=Path,required=True);p.add_argument('--sanitize',action='store_true');a=p.parse_args()
dec=a.decoder.resolve();out=root/'_build/lhdc_fac_bytes';out.mkdir(parents=True,exist_ok=True)
env=os.environ.copy()
if os.name=='nt':
    vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    matches=list((Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if matches:cache=str(sorted(matches)[-1])
assert cache,'ccache is required'
env.pop('CCACHE_DISABLE',None)
sources=[root/'tests/codecs/lhdc_fac_bytes.c',dec/'lhdc_bit_reader.c']
tracked=[*sources,root/'codecs/lhdcv5/dbt_lhdc_bits.h',Path(__file__).resolve(),*dec.glob('*.h')]
before={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in tracked}
objects=[]
for source in sources:
    obj=out/(source.stem+('-san' if a.sanitize else '')+'.obj');objects.append(obj)
    cmd=[cache,cc,'-std=c11','-O2','-g','-DLHDC_HOST_BUILD']+([] if os.name=='nt' else ['-fPIC'])
    if a.sanitize:cmd+=['-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-DDBT_LHDC_BITS_SANITIZER_MAIN=1']
    cmd+=['-I'+str(root/'codecs/lhdcv5'),'-I'+str(dec),'-c',str(source),'-o',str(obj)]
    subprocess.run(cmd,env=env,check=True)
if a.sanitize:
    binary=out/('fac_bytes_san.exe' if os.name=='nt' else 'fac_bytes_san')
    subprocess.run([cc,'-fsanitize=address,undefined',*map(str,objects),'-o',str(binary)],env=env,check=True)
    resource=Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    if os.name=='nt':
        runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
        if not runtime.is_file():runtime=Path(cc).parent/runtime.name
        assert runtime.is_file(),'The selected compiler ASan runtime is missing'
        # Application-directory loading selects the matching compiler runtime
        # even when an incompatible ASan DLL occurs elsewhere in Windows/PATH.
        shutil.copy2(runtime,out/runtime.name)
    env['PATH']=os.pathsep.join([str(Path(cc).parent),str(resource/'lib/windows'),env.get('PATH','')])
    output=subprocess.check_output([str(binary)],env=env,text=True,stderr=subprocess.STDOUT)
    cases=int(next(line.split('=',1)[1] for line in output.splitlines() if line.startswith('cases=')))
else:
    library=out/('fac_bytes.dll' if os.name=='nt' else 'fac_bytes.so')
    subprocess.run([cc,'-shared',*map(str,objects),'-o',str(library)],env=env,check=True)
    lib=ctypes.CDLL(str(library));line=lib.lhdc_fac_bytes_test();assert line==0,f'C test failed at line {line}'
    lib.lhdc_fac_bytes_cases.restype=ctypes.c_uint64;cases=lib.lhdc_fac_bytes_cases()
assert cases>=3000000
assert all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==h for path,h in before.items())
report={'passed':True,'scope':'actual bit-reader differential execution on host; not target timing or full codec execution','sanitizers':['address','undefined'] if a.sanitize else [],'cases':cases,'bytes_and_complete_reader_state_identical':True,'source_sha256':before}
(out/('sanitizer-results.json' if a.sanitize else 'results.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))
