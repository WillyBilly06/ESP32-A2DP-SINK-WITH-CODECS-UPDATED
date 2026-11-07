"""Run actual sound/output task logic with a mocked I2S device, off-board."""
import argparse,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];WS=ROOT.parent;OUT=ROOT/'_build/sound_output'
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]
sys.path.insert(0,str(ROOT/'tools'))
from test_native_codec_target_components import argv_windows,run

def main(sanitize=False,async_client=False,pcm_loans=False):
    global OUT
    if async_client:OUT=ROOT/'_build/sound_output_async'
    if pcm_loans:
        if not async_client:raise RuntimeError('PCM loans require the async sound client')
        OUT=ROOT/'_build/sound_output_async_pcm_loans'
    if os.name!='nt':raise RuntimeError('This driver currently uses Windows mock tasks')
    OUT.mkdir(parents=True,exist_ok=True);env=os.environ.copy()
    vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    lines=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in lines.splitlines() if '=' in x and not x.startswith('=')))
    cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
    harness='test_probe_sound_async.c' if async_client else 'test_probe_sound_output.c'
    sources=[ROOT/'tests/sound'/harness,ROOT/'esp32_probe/main/probe_output.c',
             ROOT/'esp32_probe/main/probe_sound.c',ROOT/'src/dbt_audio.c',ROOT/'src/dbt_resampler.c',
             ROOT/'src/dbt_sound.c',ROOT/'src/dbt_sound_mix.c',WS/'bt_audio_sink/main/dsp/fixed_dsp.c']
    tracked=sources+[ROOT/'include/dbt_audio.h',ROOT/'include/dbt_sound.h',ROOT/'esp32_probe/main/probe_output.h',
                     ROOT/'esp32_probe/main/probe_sound.h',Path(__file__).resolve()]
    if async_client:tracked.append(ROOT/'tests/sound/test_probe_sound_output.c')
    tracked+=list((ROOT/'tests/codecs/probe_audio_stubs').rglob('*.h'))+list((ROOT/'include').glob('*.h'))
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    inc=[ROOT/'tests/codecs/probe_audio_stubs',ROOT/'include',ROOT/'esp32_probe/main',WS/'bt_audio_sink/main/dsp']
    flags=[cc,'-std=c11','-O1','-g','-fwrapv','-DDBT_PROBE_SOUND_BUILD=1','-DDBT_OUTPUT_TEST_HOOKS=1','-DCONFIG_I2S_ISR_IRAM_SAFE=1',
           '-DCONFIG_NATIVE_AUDIO_BCLK=27','-DCONFIG_NATIVE_AUDIO_LRCK=25','-DCONFIG_NATIVE_AUDIO_DATA=26']
    if sanitize:flags+=['-fsanitize=undefined','-fno-sanitize-recover=all']
    if async_client:flags+=['-DDBT_PROBE_ASYNC_AUDIO=1']
    if pcm_loans:flags+=['-DDBT_PROBE_PCM_LOANS=1']
    cache=shutil.which('ccache',path=env.get('PATH'))
    if not cache:
        candidates=sorted((Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
        if candidates:cache=str(candidates[-1])
    assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
    objects=[]
    for index,source in enumerate(sources):
        obj=OUT/(str(index)+'.obj');subprocess.run([cache,*flags,*['-I'+str(x) for x in inc],'-c',str(source),'-o',str(obj)],env=env,check=True);objects.append(obj)
    exe=OUT/'integration.exe';subprocess.run(flags+list(map(str,objects))+['-o',str(exe)],env=env,check=True)
    subprocess.run([str(exe)],env=env,check=True,timeout=60)
    entries=json.loads((ROOT/'_build/esp32/compile_commands.json').read_text());entry=next(x for x in entries if x['file'].endswith('probe_output.c'))
    args=argv_windows(entry['command']);args=args[:args.index('-o')]+['-DDBT_PROBE_SOUND_BUILD=1','-Werror','-I'+str(ROOT/'include')]
    args=[arg for arg in args if not arg.startswith('-DDBT_PROBE_PCM_LOANS=')]
    if pcm_loans:args+=['-DDBT_PROBE_PCM_LOANS=1']
    if async_client:args+=['-DDBT_PROBE_ASYNC_AUDIO=1']
    for source in sources[1:4]:run(args+['-o',str(OUT/(source.stem+'.o')),'-c',str(source)],cwd=entry['directory'])
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha for p,sha in before.items())
    (OUT/('ubsan_results.json' if sanitize else 'results.json')).write_text(json.dumps({'passed':True,'sanitized':sanitize,'async_client':async_client,'pcm_loans':pcm_loans,'scope':'real render/output/sound code with host simulated tasks and I2S; ESP32 objects only',
                                                'sources':before},indent=2)+'\n')
    print('target sound/output objects compiled; no device execution')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--sanitize',action='store_true');parser.add_argument('--async-client',action='store_true');parser.add_argument('--pcm-loans',action='store_true');options=parser.parse_args()
    subprocess.run(CHECK,check=True)
    try:main(options.sanitize,options.async_client,options.pcm_loans)
    finally:subprocess.run(CHECK,check=True)
