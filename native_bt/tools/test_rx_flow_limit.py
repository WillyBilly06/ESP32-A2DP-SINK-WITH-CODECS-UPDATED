"""Actual HCI partial completion limits, app capacity proof and runtime. No hardware."""
import argparse,gzip,hashlib,json,os,pathlib,shutil,subprocess,sys
p=argparse.ArgumentParser();p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args()
ROOT=pathlib.Path(__file__).resolve().parents[1];out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
receipt=out/'results.json';receipt.write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
check=[sys.executable,str(ROOT/'tools/check_workspace.py')];subprocess.run(check,check=True)
app=ROOT/'esp32_probe/main/probe_audio.c';text=app.read_text()
begin=text.index('bool probe_audio_rx_credit_ready(');end=text.index('\nstatic uint32_t media_frame_count(',begin)
(out/'actual_audio_limit.inc').write_text(text[begin:end])
codec=ROOT/'codecs/common/dbt_codec.c';codec_text=codec.read_text()
begin=codec_text.index('static void wake_owner(dbt_codec_worker_t *w)');end=codec_text.index(chr(10),begin)
wake=codec_text[begin:end]
def actual_function(signature):
    begin=codec_text.index(signature);end=codec_text.index('{',begin)+1;level=1
    while level:
        if codec_text[end]=='{':level+=1
        elif codec_text[end]=='}':level-=1
        end+=1
    return codec_text[begin:end]
begin=codec_text.index('typedef struct {\n    uint32_t timestamp,ssrc;');end=codec_text.index('_Static_assert',begin)
parts=[codec_text[begin:end],wake]+[actual_function(s) for s in ('static uint16_t record_stride(', 'static void release_page(', 'static media_t *read_slot(', 'uint16_t dbt_codec_worker_free_slots(', 'static void retire_slot(')]
(out/'actual_retirement.inc').write_text('#include <stddef.h>\n'+chr(10).join(parts))
names=['dbt_queue','dbt_hci','dbt_host','dbt_acl','dbt_l2cap','dbt_credit','dbt_ertm','dbt_ertm_engine','dbt_channels','dbt_rx_flow','dbt_runtime']
sources=[ROOT/'src'/(n+'.c') for n in names]+[ROOT/'tests/host_sensitive/test_rx_flow_limit.c']
tracked=sources+[app,codec,ROOT/'codecs/common/dbt_codec.h',ROOT/'tests/test_rx_flow.c',pathlib.Path(__file__).resolve()]+list((ROOT/'include').glob('*.h'))
hashes={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in tracked}
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    found=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if found:cache=str(found[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
base=[cache,cc,'-std=c11','-O1','-fwrapv','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT/'include'),'-I'+str(ROOT/'codecs/common'),'-I'+str(out)]
if os.name=='nt':base+=['-fms-runtime-lib=dll','-D_DLL']
commands=[];objects=[]
try:
    for i,src in enumerate(sources):
        obj=out/(str(i)+'.obj');cmd=base+['-c',str(src),'-o',str(obj)]
        subprocess.run(cmd,env=env,check=True);commands.append(cmd);objects.append(obj)
    exe=out/('partial-credit.exe' if os.name=='nt' else 'partial-credit')
    cmd=base+list(map(str,objects))+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else [])+['-o',str(exe)]
    subprocess.run(cmd,env=env,check=True);commands.append(cmd)
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    if os.name=='nt':env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
    result=subprocess.run([str(exe)],env=env,capture_output=True,text=True,timeout=30)
    (out/'stdout.txt').write_text(result.stdout);(out/'stderr.txt').write_text(result.stderr)
    print(result.stdout.strip());print(result.stderr.strip());result.check_returncode()
    assert all(hashlib.sha256(pathlib.Path(x).read_bytes()).hexdigest()==h for x,h in hashes.items()),'Source changed'
    pdb=exe.with_suffix('.pdb')
    if pdb.exists():
        data=pdb.read_bytes();identity=hashlib.sha256(data).hexdigest()
        compressed=pdb.with_name(pdb.name+'.'+identity[:12]+'.gz')
        if not compressed.exists():compressed.write_bytes(gzip.compress(data,compresslevel=9))
        assert gzip.decompress(compressed.read_bytes())==data,'PDB archive mismatch'
        (out/'pdb-archive.json').write_text(json.dumps({'original':str(pdb),'archive':str(compressed),'sha256':identity,'bytes':len(data)},indent=2)+chr(10));pdb.unlink()
    if os.name=='nt':
        retired=[]
        for obj in objects:
            b=obj.read_bytes();assert len(b)>=20 and int.from_bytes(b[:2],'little')==0x8664,'Unexpected object; retained'
            retired.append({'path':str(obj),'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest()})
        manifest=json.dumps({'passed_executable':str(exe),'objects':retired},indent=2)+chr(10)
        tag=hashlib.sha256(manifest.encode()).hexdigest()[:16];(out/(tag+'-retired.json')).write_text(manifest)
        for obj in objects:obj.unlink()
finally:subprocess.run(check,check=True)
receipt.write_text(json.dumps({'passed':True,'hardware_accessed':False,'sanitizers':['address','undefined'],'sources':hashes,'commands':commands,'stdout':result.stdout,'scope':'actual core and extracted app policy, legacy gate/setup regressions, partial count and borrowed TX lifetime,100000 mixed Classic/LE/fragment/deferred queue model steps; no radio timing qualification'},indent=2)+'\n')
