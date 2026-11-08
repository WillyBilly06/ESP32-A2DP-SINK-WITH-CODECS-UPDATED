"""Actual main transport decisions and passive SDK-readiness snapshots; no hardware."""
import argparse,gzip,hashlib,json,os,pathlib,re,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args()
out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
(out/'results.json').write_text(json.dumps({'passed':False,'state':'building'})+'\n')
main=ROOT/'esp32_probe/main';source=main/'probe_vhci_ready.c';caller=main/'probe_main.c'
def function(text,name):
    m=re.search(r'^[^\n;]*\b'+re.escape(name)+r'\([^;\n]*\)\s*\{',text,re.M)
    assert m,name
    start=m.start();end=text.index('{',start)+1;depth=1
    while depth:
        if text[end]=='{':depth+=1
        elif text[end]=='}':depth-=1
        end+=1
    return text[start:end]
header=out/'readiness_under_test.h'
header.write_text('\n'.join(function(caller.read_text(),name) for name in ['observe_transport_ready','send_available','transport_ready','transport_send'])+'\n')
raw=source.read_text();key='atomic_store(&ready.flags,PROBE_VHCI_SUBMITTED);'
assert raw.count(key)==1
mutant=out/'negative_ready.c';mutant.write_text(raw.replace(key,'(void)0;',1))
sources=[source,ROOT/'src/dbt_hci.c',ROOT/'tests/host_sensitive/test_vhci_ready.c']
tracked=[*sources,main/'probe_vhci_ready.h',caller,*((ROOT/'include').glob('*.h')),pathlib.Path(__file__).resolve(),header,mutant]
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
flags=['-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT/'include'),'-I'+str(main),'-I'+str(out)]
flags+=['-fms-runtime-lib=dll','-D_DLL'] if os.name=='nt' else ['-pthread']
commands=[];objects=[]
def compile_one(s,name):
    obj=out/(name+('.obj' if os.name=='nt' else '.o'))
    cmd=[cache,cc,*flags,'-c',str(s),'-o',str(obj)];commands.append(cmd);subprocess.run(cmd,check=True,env=env);return obj
for i,s in enumerate(sources):objects.append(compile_one(s,'ready-'+str(i)))
def link(name,objs):
    exe=out/(name+('.exe' if os.name=='nt' else ''))
    cmd=[cc,*flags,*map(str,objs),'-o',str(exe)]+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else [])
    commands.append(cmd);subprocess.run(cmd,check=True,env=env);return exe
positive=link('ready',objects)
negative_object=compile_one(mutant,'negative_ready')
negative=link('negative-ready',[negative_object,*objects[1:]])
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
runs=[]
for name,exe,success in [('positive',positive,True),('negative',negative,False)]:
    r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=90)
    (out/(name+'.stdout.txt')).write_text(r.stdout);(out/(name+'.stderr.txt')).write_text(r.stderr)
    print(r.stdout,end='');print(r.stderr,end='')
    if success:r.check_returncode()
    else:assert r.returncode!=0 and 'FAIL readiness line' in r.stderr,'Stale-sample negative control not detected'
    runs.append({'name':name,'returncode':r.returncode,'expected_success':success})
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
    b=pdb.read_bytes();g=gzip.compress(b);assert gzip.decompress(g)==b;pdb.with_suffix('.pdb.gz').write_bytes(g)
    compressed.append({'path':str(pdb),'bytes':len(b),'gzip_bytes':len(g),'sha256':hashlib.sha256(b).hexdigest(),'lossless_verified':True});pdb.unlink()
(out/'lossless-symbols.json').write_text(json.dumps(compressed,indent=2)+'\n')
subprocess.run([sys.executable,str(ROOT/'tools/check_workspace.py')],check=True)
(out/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'sanitizers':['address','undefined'],'source_sha256':before,'commands':commands,'runs':runs,'scope':'actual main transport ready/send/callback decisions with SDK mock, bounded observations, concurrent snapshots; not controller-copy contract or target timing'},indent=2)+'\n')
