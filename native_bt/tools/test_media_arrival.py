"""Cursor-bound HCI arrival metadata, real host/SPSC, ASan/UBSan; no hardware."""
import argparse,gzip,hashlib,json,os,pathlib,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args()
out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
receipt=out/'results.json';receipt.write_text(json.dumps({'passed':False,'state':'building'})+'\n')
main=ROOT/'esp32_probe/main';source=main/'probe_arrival.c'
raw=source.read_text();key='s->queue_cursor+(uint32_t)s->wire_bytes+2u!=end'
assert raw.count(key)==1
mutant=out/'negative_arrival.c';mutant.write_text(raw.replace(key,'((void)end,0)',1))
sources=[source,*[ROOT/'src'/(n+'.c') for n in ['dbt_queue','dbt_hci','dbt_host']],ROOT/'tests/host_sensitive/test_arrival.c']
tracked=[*sources,main/'probe_arrival.h',*((ROOT/'include').glob('*.h')),pathlib.Path(__file__).resolve(),mutant]
before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:cache=str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1])
env.pop('CCACHE_DISABLE',None)
flags=['-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT/'include'),'-I'+str(main)]
flags+=['-fms-runtime-lib=dll','-D_DLL'] if os.name=='nt' else ['-pthread']
commands=[];objects=[]
def compile_one(s,name):
    obj=out/(name+('.obj' if os.name=='nt' else '.o'))
    cmd=[cache,cc,*flags,'-c',str(s),'-o',str(obj)];commands.append(cmd)
    subprocess.run(cmd,check=True,env=env);return obj
for i,s in enumerate(sources):objects.append(compile_one(s,'arrival-'+str(i)))
def link(name,objs):
    exe=out/(name+('.exe' if os.name=='nt' else ''))
    cmd=[cc,*flags,*map(str,objs),'-o',str(exe)]+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else [])
    commands.append(cmd);subprocess.run(cmd,check=True,env=env);return exe
positive=link('arrival',objects)
bad=compile_one(mutant,'negative_arrival')
negative=link('negative-arrival',[bad,*objects[1:]])
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
runs=[]
for name,exe,passed in [('positive',positive,True),('negative',negative,False)]:
    r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=120)
    (out/(name+'.stdout.txt')).write_text(r.stdout);(out/(name+'.stderr.txt')).write_text(r.stderr)
    print(r.stdout,end='');print(r.stderr,end='')
    if passed:r.check_returncode()
    else:assert r.returncode!=0 and 'FAIL arrival line' in r.stderr,'Cursor-negative control did not fail'
    runs.append({'name':name,'returncode':r.returncode,'expected_success':passed})
assert before=={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
retired=[]
if os.name=='nt':
    for obj in objects:
        b=obj.read_bytes();assert int.from_bytes(b[:2],'little')==0x8664
        retired.append({'path':str(obj),'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest()})
    (out/'passing-intermediates.json').write_text(json.dumps(retired,indent=2)+'\n')
    for obj in objects:obj.unlink()
compressed=[]
for pdb in out.glob('*.pdb'):
    b=pdb.read_bytes();g=gzip.compress(b);assert gzip.decompress(g)==b
    pdb.with_suffix('.pdb.gz').write_bytes(g)
    compressed.append({'path':str(pdb),'bytes':len(b),'gzip_bytes':len(g),'sha256':hashlib.sha256(b).hexdigest(),'lossless_verified':True})
    pdb.unlink()
(out/'lossless-symbols.json').write_text(json.dumps(compressed,indent=2)+'\n')
subprocess.run([sys.executable,str(ROOT/'tools/check_workspace.py')],check=True)
receipt.write_text(json.dumps({'passed':True,'hardware_accessed':False,'sanitizers':['address','undefined'],'source_sha256':before,'commands':commands,'runs':runs,'scope':'cursor/header/time association, overflow/rejection and actual host SPSC ownership; not target timing or RF'},indent=2)+'\n')
