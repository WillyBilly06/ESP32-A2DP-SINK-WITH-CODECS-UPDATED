"""Actual storage reserve protocol and PCM pages with modeled events; no hardware access."""
import gzip,hashlib,json,os,pathlib,shutil,subprocess,sys,time,zipfile

root=pathlib.Path(__file__).resolve().parents[1]
out=root/'_build/storage_reserve'/str(time.time_ns());out.mkdir(parents=True)
receipt=out/'results.json';latest=root/'_build/storage_reserve/results.json'
result={'passed':False,'state':'building','hardware_accessed':False,
        'scope':'Actual storage reserve state machine, page ownership and PCM with modeled owner events; no SDK/IRQ/multithread/timing proof',
        'run_directory':str(out),'commands':[]}
def save():
    text=json.dumps(result,indent=2)+'\n';receipt.write_text(text);latest.write_text(text)
def run(command,**kwargs):
    result['commands'].append(command);save()
    return subprocess.run(command,env=env,check=True,**kwargs)
save();check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
env=os.environ.copy()
try:
    if os.name=='nt':
        vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
    cache=shutil.which('ccache',path=env.get('PATH'))
    if not cache:
        candidates=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
        if candidates:cache=str(candidates[-1])
    assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
    source=[root/'tests/audio/storage_reserve.c',root/'src/dbt_storage_reserve.c',root/'src/dbt_audio.c',root/'src/dbt_resampler.c',root.parent/'bt_audio_sink/main/dsp/fixed_dsp.c']
    tracked=source+[root/'tests/test_audio.c',root/'src/dbt_resampler_coeffs.inc',pathlib.Path(__file__).resolve(),*sorted((root/'include').glob('*.h')),*sorted((root.parent/'bt_audio_sink/main/dsp').glob('fixed_dsp*'))]
    pins={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    result['sources']=pins;save()
    flags=['-std=c11','-O1','-g','-fwrapv','-DDBT_AUDIO_BLOCK_DSP=1','-DFIXED_DSP_BLOCK_API=1','-Wall','-Wextra','-Werror',
           '-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer',
           '-I',str(root/'include'),'-I',str(root.parent/'bt_audio_sink/main/dsp')]
    if os.name=='nt':
        flags+=['-fms-runtime-lib=dll','-D_DLL']
        resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
        env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
    objects=[]
    for i,p in enumerate(source):
        obj=out/(str(i)+('.obj' if os.name=='nt' else '.o'));objects.append(obj)
        run([cache,cc,*flags,'-c',str(p),'-o',str(obj)])
    artifact=out/('storage-reserve.exe' if os.name=='nt' else 'storage-reserve')
    run([cc,*flags,*map(str,objects),*(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else ['-lm']),'-o',str(artifact)])
    result['state']='executing';save()
    executed=run([str(artifact)],capture_output=True,text=True,timeout=15)
    result['stdout']=executed.stdout;result['stderr']=executed.stderr
    print(executed.stdout.strip());print(executed.stderr.strip())
    assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in pins.items())
    result['artifact_sha256']=hashlib.sha256(artifact.read_bytes()).hexdigest()
    # Keep exact compiler objects and symbols losslessly, rather than allowing
    # completed host-test intermediates to consume the bounded build budget.
    compact=objects+([artifact.with_suffix('.pdb')] if artifact.with_suffix('.pdb').is_file() else [])
    manifest=[]
    with zipfile.ZipFile(out/'intermediates.zip','w',compression=zipfile.ZIP_DEFLATED) as archive:
        for p in compact:
            assert p.resolve().parent==out.resolve()
            raw=p.read_bytes();archive.writestr(p.name,raw)
            manifest.append({'name':p.name,'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest()})
    with zipfile.ZipFile(out/'intermediates.zip') as archive:
        for p,item in zip(compact,manifest):
            assert hashlib.sha256(archive.read(p.name)).hexdigest()==item['sha256']
    (out/'intermediates.json').write_text(json.dumps({'files':manifest,'lossless_verified':True},indent=2)+'\n')
    for p in compact:p.unlink()
    result.update(passed=True,state='complete',sanitizers=['address','undefined'],intermediates=manifest)
except Exception as exc:
    result.update(state='failed',failure=repr(exc));raise
finally:
    save();subprocess.run(check,check=True)
