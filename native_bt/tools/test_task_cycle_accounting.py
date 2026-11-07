"""Sanitize the actual tick sampler without accessing a device."""
import argparse,hashlib,json,os,pathlib,shutil,subprocess
parser=argparse.ArgumentParser();parser.add_argument("--runtime-stats",action="store_true",default=True);options=parser.parse_args()
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/task_cycle_accounting';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    choices=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if choices:cache=str(choices[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
source=ROOT/'tests/codecs/probe_task_cycles_test.c';stubs=ROOT/'tests/scheduler_stubs';main=ROOT/'esp32_probe/main'
tracked=[source,pathlib.Path(__file__).resolve()]+list(stubs.rglob('*.h'))+list(main.glob('probe_scheduler.*'))+list(main.glob('probe_task_*'))+[ROOT/'tests/codecs/probe_scheduler_test.c']
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
exe=OUT/('sampler.exe' if os.name=='nt' else 'sampler')
cmd=[cache,cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(stubs),str(source),'-o',str(exe)]
if options.runtime_stats:cmd+=['-DDBT_PROBE_TASK_RUNTIME_STATS=1','-DDBT_PROBE_CODEC_AWARE_RENDER=1','-DDBT_PROBE_TASK_CYCLE_ACCOUNTING=1']
if os.name=='nt':cmd+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
subprocess.run(cmd,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
subprocess.run([str(exe)],check=True,env=env,timeout=60)
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'runtime_observer':options.runtime_stats,'sanitizers':['address','undefined'],'signed_overflow_wrapping':False,'sources':before,'command':cmd,'scope':'actual sampler with model public RTOS hooks; target placement, real timing and sampling bias require physical measurement'},indent=2)+'\n')
