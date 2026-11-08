"""Actual old/new sensitive queue erasure plus host/SPSC regressions; ccache and ASan/UBSan."""
import argparse,hashlib,json,os,pathlib,re,shutil,subprocess,sys,gzip
ROOT=pathlib.Path(__file__).resolve().parents[1]
PIN='dcdb2478bddeb393b613952ba42485cd9af7f90fb82690c86c92847ebeb15032'
p=argparse.ArgumentParser()
p.add_argument('--baseline',type=pathlib.Path)
p.add_argument('--candidate',type=pathlib.Path,help='Optional archived experimental implementation; defaults to actual source')
p.add_argument('--out',type=pathlib.Path,required=True)
a=p.parse_args()
out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
report_path=out/'results.json'
report_path.write_text(json.dumps({'passed':False,'phase':'building','hardware_accessed':False})+'\n')
original=a.baseline.read_bytes() if a.baseline else subprocess.check_output(['git','show','528a27ee:native_bt/src/dbt_queue.c'],cwd=ROOT)
# Git normalizes CRLF; pin canonical source bytes, not worktree newline policy.
original=original.replace(b'\r\n',b'\n')
assert hashlib.sha256(original).hexdigest()==PIN
baseline=out/'reference_queue.c';baseline.write_bytes(original)
candidate=(a.candidate or ROOT/'src/dbt_queue.c').resolve()
current=candidate.read_text()
if current.count('*words++=0;')==1:
    bad=current.replace('*words++=0;','*words++=0xabad1deau;',1)
else:
    marker='bytes[slot(read+2u+i)]=0;'
    assert current.count(marker)==1,'Unrecognized erase implementation; review negative control'
    bad=current.replace(marker,'bytes[slot(read+2u+i)]=0xadu;',1)
mutant=out/'negative_queue.c';mutant.write_text(bad)
main=out/'sensitive_main.c'
main.write_text('''#include <stdio.h>
int host_sensitive_acl_and_events(void);
int host_sensitive_abort_and_borrow(void);
int host_sensitive_abort_with_queued_and_late(void);
int host_sensitive_error_paths(void);
int host_sensitive_control_reserve(void);
int main(void){
    int (*tests[])(void)={host_sensitive_acl_and_events,host_sensitive_abort_and_borrow,host_sensitive_abort_with_queued_and_late,host_sensitive_error_paths,host_sensitive_control_reserve};
    const char *names[]={"acl_and_events","abort_and_borrow","abort_with_queued_and_late","error_paths","control_reserve"};
    for(unsigned i=0;i<5;++i){int line=tests[i]();if(line){fprintf(stderr,"FAIL %s line%d\\n",names[i],line);return 1;}printf("PASS %s\\n",names[i]);}
    return 0;
}
''')
tracked=[candidate,ROOT/'src/dbt_host.c',ROOT/'src/dbt_hci.c',ROOT/'tests/host_sensitive/test_queue_wipe.c',
         ROOT/'tests/host_sensitive/ingress.c',ROOT/'tests/host_sensitive/threaded.c',pathlib.Path(__file__).resolve(),
         baseline,mutant,main,*[ROOT/'include'/n for n in ['dbt_queue.h','dbt_host.h','dbt_hci.h']]]
hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:cache=str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1])
env.pop('CCACHE_DISABLE',None)
flags=['-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT/'include')]
if os.name=='nt':flags+=['-fms-runtime-lib=dll','-D_DLL']
else:flags+=['-pthread']
commands=[];objects=[]
def compile_one(source,name,extra=()):
    obj=out/(name+('.obj' if os.name=='nt' else '.o'))
    cmd=[cache,cc,*flags,*extra,'-c',str(source),'-o',str(obj)]
    commands.append(cmd);subprocess.run(cmd,check=True,env=env);objects.append(obj);return obj
def link(name,inputs):
    exe=out/(name+('.exe' if os.name=='nt' else ''))
    cmd=[cc,*flags,*map(str,inputs),'-o',str(exe)]+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else [])
    commands.append(cmd);subprocess.run(cmd,check=True,env=env);return exe
renames=['-Ddbt_queue_'+n+'=reference_queue_'+n for n in ['init','push','push_sensitive','pop','used']]
ref=compile_one(baseline,'reference_queue',renames)
queue=compile_one(candidate,'queue')
harness=compile_one(ROOT/'tests/host_sensitive/test_queue_wipe.c','queue_cases')
diff=link('queue-differential',[queue,ref,harness])
bad=compile_one(mutant,'negative_queue')
negative=link('negative-queue',[bad,ref,harness])
host=compile_one(ROOT/'src/dbt_host.c','host')
hci=compile_one(ROOT/'src/dbt_hci.c','hci')
ingress=compile_one(ROOT/'tests/host_sensitive/ingress.c','ingress')
entry=compile_one(main,'sensitive_main')
sensitive=link('sensitive-host',[queue,host,hci,ingress,entry])
threaded=compile_one(ROOT/'tests/host_sensitive/threaded.c','threaded')
spsc=link('spsc',[queue,host,hci,threaded])
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
runs=[]
for name,exe,expect_success in [('queue',diff,True),('sensitive',sensitive,True),('spsc',spsc,True),('negative',negative,False)]:
    r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=120)
    (out/(name+'.stdout.txt')).write_text(r.stdout);(out/(name+'.stderr.txt')).write_text(r.stderr)
    print(r.stdout,end='');print(r.stderr,end='')
    if expect_success:r.check_returncode()
    else:assert r.returncode!=0 and 'FAIL line' in r.stderr,'Negative control did not detect changed erasure'
    runs.append({'name':name,'returncode':r.returncode,'expected_success':expect_success})
assert hashes=={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
retired=[]
# Preserve negative-control intermediates and every executable. Retire only
# passing AMD64 COFF intermediates; losslessly retain debug symbols.
if os.name=='nt':
    for obj in objects:
        if obj.name.startswith('negative_'):continue
        data=obj.read_bytes();assert int.from_bytes(data[:2],'little')==0x8664
        retired.append({'path':str(obj),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
    (out/'passing-intermediates.json').write_text(json.dumps(retired,indent=2)+'\n')
    for row in retired:pathlib.Path(row['path']).unlink()
compressed=[]
for pdb in out.glob('*.pdb'):
    raw=pdb.read_bytes();gz=gzip.compress(raw)
    assert gzip.decompress(gz)==raw
    target=pdb.with_suffix('.pdb.gz');target.write_bytes(gz)
    compressed.append({'path':str(pdb),'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest(),'gzip_bytes':len(gz),'lossless_verified':True})
    pdb.unlink()
(out/'lossless-symbols.json').write_text(json.dumps(compressed,indent=2)+'\n')
subprocess.run([sys.executable,str(ROOT/'tools/check_workspace.py')],check=True)
report={'passed':True,'hardware_accessed':False,'sanitizers':['address','undefined'],'baseline_commit':'528a27ee','baseline_canonical_sha256':PIN,'source_sha256':hashes,'runs':runs,'commands':commands,'scope':'actual queue and sensitive-host content/bounds/SPSC lifetime tests; not target timing or RF continuity'}
report_path.write_text(json.dumps(report,indent=2)+'\n')
