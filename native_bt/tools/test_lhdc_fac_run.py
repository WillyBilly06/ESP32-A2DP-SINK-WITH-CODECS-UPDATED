"""Actual FAC range/model/output/trace comparison; ASan/UBSan, no hardware."""
import argparse,hashlib,json,os,pathlib,re,shutil,subprocess,sys
R=pathlib.Path(__file__).resolve().parents[1];p=argparse.ArgumentParser();p.add_argument('--decoder',type=pathlib.Path,required=True);a=p.parse_args();dec=a.decoder.resolve();out=R/'_build/lhdc_fac_run';out.mkdir(exist_ok=True);port=out/'port'
subprocess.run([sys.executable,str(R/'codecs/lhdcv5/prepare_port.py'),'--source',str(dec),'--output',str(port)],check=True,stdout=subprocess.DEVNULL)
old=(dec/'lhdc_entropy_dec.c').read_text();new=(port/'lhdc_entropy_dec.c').read_text()
def fn(t,name):
 m=re.search(r'^static[^\n]*\b'+name+r'\([^\n]*\)\n\{.*?^\}',t,re.M|re.S);assert m,name;return m[0]
def typ(t,name):
 m=re.search(r'typedef struct \{[^}]+\} '+name+';',t,re.S);assert m,name;return m[0]
for n in ('fac_model_rescale','fac_model_init','fac_model_update','fac_byte','fac_dec_init','fac_decode'):assert fn(old,n)==fn(new,n),n
header='#define FAC_NSYM 3\n#define FAC_TOTAL_BITS 15\n#define LHDC_HOT\n#define LHDC_NOINLINE\n#define LHDC_ALWAYS_INLINE inline\n#define LHDC_HOST_BUILD 1\n#define DBT_LHDC_FAC_RUN_TEST 1\n'
header+=typ(new,'fac_model_t')+'\n'+typ(new,'fac_dec_t')+'\n'
for n in ('fac_model_rescale','fac_model_init','fac_model_update','fac_byte','fac_dec_init','fac_decode'):
 code=fn(new,n)
 if n=='fac_model_rescale':code=code.replace('\n{','\n{\n    ++rescales;',1)
 header+=code+'\n'
header+='#include "dbt_lhdc_fac_run.h"\n';(out/'fac_run_under_test.h').write_text(header)
env=os.environ.copy();vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat');v=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in v.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe';cache=shutil.which('ccache',path=env.get('PATH')) or str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1]);env.pop('CCACHE_DISABLE',None)
source=R/'tests/codecs/test_lhdc_fac_run.c';exe=out/'fac_run.exe';obj=out/'fac_run.obj';inputs=[source,pathlib.Path(__file__).resolve(),R/'codecs/lhdcv5/dbt_lhdc_fac_run.h',R/'codecs/lhdcv5/prepare_port.py',dec/'lhdc_entropy_dec.c',port/'lhdc_entropy_dec.c',out/'fac_run_under_test.h'];before={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in inputs}
flags=[cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fms-runtime-lib=dll','-D_DLL','-I'+str(out),'-I'+str(R/'codecs/lhdcv5')]
subprocess.run([cache,*flags,'-c',str(source),'-o',str(obj)],check=True,env=env);subprocess.run([*flags,str(obj),'-o',str(exe),'-Wl,/NODEFAULTLIB:libcmt'],check=True,env=env)
resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());rt=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
if not rt.is_file():rt=pathlib.Path(cc).parent/rt.name
assert rt.is_file();shutil.copy2(rt,out/rt.name)
r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=120);(out/'stdout.txt').write_text(r.stdout);(out/'stderr.txt').write_text(r.stderr);print(r.stdout[-1500:]);print(r.stderr);r.check_returncode()
ref=re.search(r'BEGIN_TRACE_REF\n(.*?)END_TRACE_REF',r.stdout,re.S)[1];fast=re.search(r'BEGIN_TRACE_FAST\n(.*?)END_TRACE_FAST',r.stdout,re.S)[1];assert ref==fast
m=re.search(r'PASS (\d+) FAC call-boundary states / (\d+) symbols; fast=(\d+) padding=(\d+)',r.stdout);assert m
assert all(hashlib.sha256(pathlib.Path(f).read_bytes()).hexdigest()==h for f,h in before.items())
(out/'results.json').write_text(json.dumps(dict(passed=True,boundaries=int(m[1]),symbols=int(m[2]),fast_symbols=int(m[3]),fast_padding_bytes=int(m[4]),trace_identical=True,sanitizers=['address','undefined'],scope='complete FAC decoder/model/history/output/rescale counts, padding/truncated input and host trace equality; host only',source_sha256=before),indent=2)+'\n')
