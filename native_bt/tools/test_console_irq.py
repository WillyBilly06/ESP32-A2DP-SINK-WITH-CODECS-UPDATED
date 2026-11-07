"""Sanitize the actual console initializer; no serial access."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/console_irq';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
env=os.environ.copy();vc=pathlib.Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=r'C:/Program Files/LLVM/bin/clang.exe';choices=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'));assert choices;cache=str(choices[-1]);env.pop('CCACHE_DISABLE',None)
source=ROOT/'tests/codecs/probe_console_test.c';tracked=[source,pathlib.Path(__file__).resolve(),ROOT/'esp32_probe/main/probe_console.c',ROOT/'esp32_probe/main/probe_console.h']+list((ROOT/'tests/console_stubs').rglob('*.h'));before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
commands=[]
resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
for mode in range(3):
    exe=OUT/f'console-{mode}.exe';cmd=[cache,cc,'-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt','-DCONSOLE_TEST_LINE_MODE='+str(mode),'-I'+str(ROOT/'tests/console_stubs'),'-I'+str(ROOT/'tests/scheduler_stubs'),str(source),'-o',str(exe)]
    subprocess.run(cmd,check=True,env=env);commands.append(cmd)
    r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=10);print(r.stdout.strip());print(r.stderr.strip());r.check_returncode()
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'sources':before,'commands':commands,'newline_modes':3,'sanitizers':['address','undefined'],'hardware_accessed':False},indent=2)+'\n')
