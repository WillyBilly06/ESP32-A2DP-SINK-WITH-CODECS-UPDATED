"""Actual bounded owner/credit observer with ASan/UBSan and a version-negative control."""
import argparse,gzip,hashlib,json,os,pathlib,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args()
out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
receipt=out/'results.json';receipt.write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+chr(10))
main=ROOT/'esp32_probe/main';source=main/'probe_owner_state.c';test=ROOT/'tests/host_sensitive/test_owner_state.c'
raw=source.read_text();key='s->coherent=!(before&1u) && before==atomic_load(&owner_state.version);';assert raw.count(key)==1
mutant=out/'negative_owner_state.c';mutant.write_text(raw.replace(key,'s->coherent=true;(void)before;',1))
tracked=[source,test,ROOT/'src/dbt_hci.c',main/'probe_owner_state.h',pathlib.Path(__file__).resolve(),*sorted((ROOT/'include').glob('*.h')),mutant]
before={str(s):hashlib.sha256(s.read_bytes()).hexdigest() for s in tracked};env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:/Program Files/LLVM/bin/clang.exe'
cache=shutil.which('ccache',path=env.get('PATH')) or str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1]);env.pop('CCACHE_DISABLE',None)
flags=['-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT/'include'),'-I'+str(main)]
if os.name=='nt':
    flags+=['-fms-runtime-lib=dll','-D_DLL'];resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
commands=[];runs=[];objects=[]
common=out/('hci.obj' if os.name=='nt' else 'hci.o');cmd=[cache,cc,*flags,'-c',str(ROOT/'src/dbt_hci.c'),'-o',str(common)];commands.append(cmd);subprocess.run(cmd,check=True,env=env);objects.append(common)
for name,bad in [('positive',False),('negative',True)]:
    obj=out/(name+('.obj' if os.name=='nt' else '.o'));exe=out/(name+('.exe' if os.name=='nt' else ''))
    extra=['-DOWNER_STATE_IMPLEMENTATION="'+str(mutant).replace(chr(92),'/')+'"'] if bad else []
    cmd=[cache,cc,*flags,*extra,'-c',str(test),'-o',str(obj)];commands.append(cmd);subprocess.run(cmd,check=True,env=env)
    link=[cc,*flags,str(common),str(obj),'-o',str(exe)]+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else []);commands.append(link);subprocess.run(link,check=True,env=env)
    r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=10);(out/(name+'.stdout.txt')).write_text(r.stdout);(out/(name+'.stderr.txt')).write_text(r.stderr);print(r.stdout,end='');print(r.stderr,end='')
    if bad:assert r.returncode!=0 and 'FAIL owner line' in r.stderr,'Version-negative control was not detected'
    else:r.check_returncode();objects.append(obj)
    runs.append({'name':name,'returncode':r.returncode,'expected_success':not bad})
assert before=={str(s):hashlib.sha256(s.read_bytes()).hexdigest() for s in tracked}
retired=[]
for obj in objects:
    b=obj.read_bytes();assert int.from_bytes(b[:2],'little')==0x8664 if os.name=='nt' else True
    retired.append({'path':str(obj),'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest()})
(out/'passing-intermediates.json').write_text(json.dumps(retired,indent=2)+chr(10))
for obj in objects:obj.unlink()
compressed=[]
for pdb in out.glob('*.pdb'):
    b=pdb.read_bytes();dest=pdb.with_suffix('.pdb.gz');dest.write_bytes(gzip.compress(b));assert gzip.decompress(dest.read_bytes())==b
    compressed.append({'path':str(pdb),'bytes':len(b),'gzip_bytes':dest.stat().st_size,'sha256':hashlib.sha256(b).hexdigest()});pdb.unlink()
(out/'lossless-symbols.json').write_text(json.dumps(compressed,indent=2)+chr(10))
subprocess.run([sys.executable,str(ROOT/'tools/check_workspace.py')],check=True)
receipt.write_text(json.dumps({'passed':True,'hardware_accessed':False,'sanitizers':['address','undefined'],'source_sha256':before,'commands':commands,'runs':runs,'scope':'actual owner/credit snapshot and single-boundary simulated preemption; not full host scheduling or RF timing'},indent=2)+chr(10))
