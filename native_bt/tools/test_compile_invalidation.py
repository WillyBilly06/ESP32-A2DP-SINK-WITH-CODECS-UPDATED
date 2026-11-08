"""Real Ninja/ccache regression: backdated source and header must rebuild."""
import argparse,hashlib,importlib.util,json,os,pathlib,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args()
out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=False)
spec=importlib.util.spec_from_file_location('invalidate',ROOT/'tools/invalidate_compile_outputs.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:cache=str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1])
ninja=shutil.which('ninja',path=env.get('PATH'))
if not ninja:ninja=str(sorted((pathlib.Path.home()/'.espressif/tools/ninja').rglob('ninja.exe'))[-1])
env.pop('CCACHE_DISABLE',None);env['CCACHE_LOGFILE']=str(out/'ccache.log')
exe=out/('value.exe' if os.name=='nt' else 'value')
(out/'unit.c').write_text('#include "value.h"\nint main(void){return VALUE;}\n');(out/'value.h').write_text('#define VALUE 1\n')
(out/'compile_commands.json').write_text(json.dumps([{'directory':str(out),'file':str(out/'unit.c'),'output':'unit.obj'}]))
(out/'build.ninja').write_text(f'rule cc\n  command = "{cache}" "{cc}" -O2 -MMD -MF unit.obj.d -c unit.c -o unit.obj\n  depfile = unit.obj.d\n  deps = gcc\nrule link\n  command = "{cc}" unit.obj -o {exe.name}\nbuild unit.obj: cc unit.c\nbuild {exe.name}: link unit.obj\ndefault {exe.name}\n')
logs=[]
def build():
    r=subprocess.run([ninja],cwd=out,env=env,capture_output=True,text=True);logs.append(r.stdout+r.stderr);r.check_returncode()
def value():return subprocess.run([str(exe)],env=env).returncode
build();assert value()==1
# Restore a changed source with an older timestamp: unprotected Ninja skips it.
(out/'unit.c').write_text('#include "value.h"\nint main(void){return VALUE+1;}\n');os.utime(out/'unit.c',(1,1))
build();assert value()==1,'Negative control did not reproduce stale build'
before=hashlib.sha256((out/'unit.obj').read_bytes()).hexdigest();first=module.invalidate(out)
assert before==hashlib.sha256((out/'unit.obj').read_bytes()).hexdigest(),'Invalidation changed object content'
build();assert value()==2
# Headers have the same failure mode, even with a compiler dependency file.
(out/'value.h').write_text('#define VALUE 2\n');os.utime(out/'value.h',(1,1));build();assert value()==2
second=module.invalidate(out);build();assert value()==3
# Repeat compilation through ccache with unchanged contents.
module.invalidate(out);build();assert value()==3
assert 'cache_hit' in (out/'ccache.log').read_text(),'No ccache hit observed'
# Out-of-tree output is rejected before any timestamp is modified.
(out/'compile_commands.json').write_text(json.dumps([{'directory':str(out),'file':str(out/'unit.c'),'output':'../outside.obj'}]))
try:module.invalidate(out)
except ValueError:pass
else:raise AssertionError('Out-of-tree compiler output accepted')
(out/'ninja.log').write_text('\n'.join(logs))
(out/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'stale_source_negative_control':True,'stale_header_negative_control':True,'correct_results':[1,2,3],'ccache_hit':True,'object_content_preserved_by_invalidation':True,'out_of_tree_rejected':True,'first':first,'second':second},indent=2)+'\n')
print('PASS real Ninja stale-source/header negative controls, corrected compiled results, ccache hit, unchanged object bytes and path boundary')
