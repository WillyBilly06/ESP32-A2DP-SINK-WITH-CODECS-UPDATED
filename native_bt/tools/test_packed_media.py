"""Actual packed-worker ownership under ASan/UBSan; no device access."""
import gzip,hashlib,json,os,pathlib,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/packed_media'/str(__import__('time').time_ns());OUT.mkdir(parents=True)
receipt=OUT/'results.json';receipt.write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
sources=[ROOT/'tests/codecs/packed_media.c',ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c',ROOT/'tests/test_codec_deferred.c']
tracked=sources+[ROOT/'codecs/common/dbt_codec.h',pathlib.Path(__file__).resolve()]+list((ROOT/'include').glob('*.h'))
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
env=os.environ.copy();vc=pathlib.Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=r'C:/Program Files/LLVM/bin/clang.exe';cache=str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1]);env.pop('CCACHE_DISABLE',None)
base=[cache,cc,'-std=c11','-DDBT_TEST_PACKED_MEDIA=1','-O1','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fms-runtime-lib=dll','-D_DLL','-I'+str(ROOT/'include'),'-I'+str(ROOT/'codecs/common')]
commands=[];runs=[]
for mode in ('candidate','unsealed-tail-negative'):
    objects=[]
    actual=sources.copy()
    if mode!='candidate':
        text=sources[1].read_text();old='tail->size=UINT16_MAX;';assert text.count(old)==1
        mutated=OUT/'unsealed-tail.c';mutated.write_text(text.replace(old,'tail->size=0;'));actual[1]=mutated
    for i,source in enumerate(actual):
        obj=OUT/(mode+'-'+str(i)+'.obj');cmd=base+['-c',str(source),'-o',str(obj)];subprocess.run(cmd,env=env,check=True);objects.append(obj);commands.append(cmd)
    exe=OUT/(mode+'.exe');cmd=base[1:]+list(map(str,objects))+['-Wl,/NODEFAULTLIB:libcmt','-o',str(exe)];subprocess.run(cmd,env=env,check=True);commands.append(cmd)
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
    r=subprocess.run([str(exe)],env=env,capture_output=True,text=True,timeout=45);(OUT/(mode+'.stdout.txt')).write_text(r.stdout);(OUT/(mode+'.stderr.txt')).write_text(r.stderr);print(mode,r.stdout,r.stderr)
    if mode=='candidate':r.check_returncode()
    else:assert r.returncode and ('FAIL packed' in r.stderr or ('unsealed-tail.c:' in r.stderr and 'runtime error:' in r.stderr)),'Negative did not detect unsealed tail'
    runs.append({'mode':mode,'exit':r.returncode,'stdout':r.stdout,'stderr':r.stderr})
    assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    retired=[]
    for obj in objects:
        data=obj.read_bytes();retired.append({'path':str(obj),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
    (OUT/(mode+'-retired-objects.json')).write_text(json.dumps(retired,indent=2)+'\n')
    for obj in objects:obj.unlink()
receipt.write_text(json.dumps({'passed':True,'state':'complete','hardware_accessed':False,'sources':hashes,'commands':commands,'sanitizers':['address','undefined'],'runs':runs,'scope':'Actual SPSC worker and control/borrow lifetime;20000 concurrent host packets; not target/RF/codec fidelity proof'},indent=2)+'\n');print('ARTIFACTS',OUT)
