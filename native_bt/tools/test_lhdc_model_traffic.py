"""Compare actual generated and original FAC model updates under ASan/UBSan."""
import argparse,hashlib,json,os,pathlib,re,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--decoder',type=pathlib.Path,required=True);a=p.parse_args()
dec=a.decoder.resolve();OUT=ROOT/'_build/lhdc_model_traffic';OUT.mkdir(exist_ok=True);port=OUT/'port'
subprocess.run([sys.executable,str(ROOT/'codecs/lhdcv5/prepare_port.py'),'--source',str(dec),'--output',str(port)],check=True,stdout=subprocess.DEVNULL)
old=(dec/'lhdc_entropy_dec.c').read_text();new=(port/'lhdc_entropy_dec.c').read_text()
def function(text,name):
 m=re.search(r'^static[^\n]*\b'+name+r'\([^\n]*\)\n\{.*?^\}',text,re.M|re.S);assert m,name
 return m[0]
def struct(text):
 m=re.search(r'typedef struct \{\n\s+int\s+n;.*?\} fac_model_t;',text,re.S);assert m
 return m[0]
assert struct(old)==struct(new)
rescale=function(old,'fac_model_rescale');assert rescale==function(new,'fac_model_rescale')
init=function(old,'fac_model_init');assert init==function(new,'fac_model_init')
rescale=rescale.replace('\n{','\n{\n    ++model_rescales;',1)
header='#define LHDC_HOT\n#define LHDC_NOINLINE\n#define LHDC_ALWAYS_INLINE inline\n#define FAC_NSYM 3\n'
header+=struct(new)+'\n'+rescale+'\n'+init+'\n'
header+=function(old,'fac_model_update').replace('fac_model_update','reference_fac_model_update')+'\n'
header+=function(new,'fac_model_update').replace('fac_model_update','candidate_fac_model_update')+'\n'
(OUT/'lhdc_model_under_test.h').write_text(header)
env=os.environ.copy();vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(s.split('=',1) for s in values.splitlines() if '=' in s and not s.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe';cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:cache=str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1])
env.pop('CCACHE_DISABLE',None);source=ROOT/'tests/codecs/test_lhdc_model_traffic.c';exe=OUT/'model_traffic.exe';obj=OUT/'model_traffic.obj'
inputs=[source,pathlib.Path(__file__).resolve(),ROOT/'codecs/lhdcv5/prepare_port.py',dec/'lhdc_entropy_dec.c',port/'lhdc_entropy_dec.c',OUT/'lhdc_model_under_test.h'];before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in inputs}
flags=[cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fms-runtime-lib=dll','-D_DLL','-I'+str(OUT)]
subprocess.run([cache,*flags,'-c',str(source),'-o',str(obj)],check=True,env=env)
subprocess.run([*flags,str(obj),'-o',str(exe),'-Wl,/NODEFAULTLIB:libcmt'],check=True,env=env)
resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
if not runtime.is_file():runtime=pathlib.Path(cc).parent/runtime.name
assert runtime.is_file();shutil.copy2(runtime,OUT/runtime.name)
r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=120);(OUT/'stdout.txt').write_text(r.stdout);(OUT/'stderr.txt').write_text(r.stderr);print(r.stdout);print(r.stderr);r.check_returncode()
m=re.search(r'PASS (\d+) complete FAC model/history transitions; cancelled=(\d+); still_rescaled=(\d+)',r.stdout);assert m
assert all(hashlib.sha256(pathlib.Path(f).read_bytes()).hexdigest()==h for f,h in before.items())
(OUT/'results.json').write_text(json.dumps(dict(passed=True,transitions=int(m[1]),cancelled=int(m[2]),still_rescaled=int(m[3]),scope='actual original/generated FAC model, complete state/history and rescale-call equality; host only',sanitizers=['address','undefined'],source_sha256=before),indent=2)+'\n')
