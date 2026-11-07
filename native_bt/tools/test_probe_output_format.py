"""Offline production output-format test with real dbt_audio and mocked I2S."""
import argparse,ctypes,hashlib,json,os,pathlib,shutil,subprocess,sys
parser=argparse.ArgumentParser();parser.add_argument('--compact-copy',action='store_true');parser.add_argument('--block-dsp',action='store_true');parser.add_argument('--priority-order',action='store_true');parser.add_argument('--word-dsp',action='store_true');parser.add_argument('--refill-window',action='store_true');parser.add_argument('--assist-refill',action='store_true');options=parser.parse_args()
if options.assist_refill:
    options.refill_window=options.compact_copy=options.block_dsp=options.word_dsp=True
    options.priority_order=True
if options.word_dsp and not options.block_dsp:raise SystemExit('--word-dsp requires --block-dsp')
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;out=root/'_build/avrcp';out.mkdir(parents=True,exist_ok=True)
if options.assist_refill:
    receipt=out/'assist-refill-results.json'
    receipt.write_text(json.dumps({'passed':False,'hardware_accessed':False,'state':'building'})+'\n')
check=[sys.executable,str(root/'tools/check_workspace.py')];subprocess.run(check,check=True)
env=os.environ.copy()
try:
    if os.name=='nt':
        vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
    cache=shutil.which('ccache',path=env.get('PATH'))
    if not cache:
        matches=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
        if matches:cache=str(matches[-1])
    if not cache:raise RuntimeError('ccache required')
    env.pop('CCACHE_DISABLE',None)
    incs=[root/'tests/codecs/probe_audio_stubs',root/'include',root/'esp32_probe/main',ws/'bt_audio_sink/main/dsp']
    src=[root/'tests/codecs/probe_output_format.c',root/'src/dbt_audio.c',root/'src/dbt_resampler.c',ws/'bt_audio_sink/main/dsp/fixed_dsp.c']
    if options.assist_refill:src[0]=root/'tests/codecs/probe_output_assist_refill.c';src+=[root/'src/dbt_dsp_job.c']
    if options.assist_refill:
        tracked=src+[root/'esp32_probe/main/probe_output.c',root/'tests/codecs/probe_output_format.c',pathlib.Path(__file__).resolve()]
        source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked};runs=[]
    cmd=[cache,cc,'-std=c11','-O1','-g','-fwrapv','-shared','-DCONFIG_I2S_ISR_IRAM_SAFE=1','-DCONFIG_NATIVE_AUDIO_BCLK=26','-DCONFIG_NATIVE_AUDIO_LRCK=25','-DCONFIG_NATIVE_AUDIO_DATA=27']
    if options.refill_window:
        cmd.remove('-g')  # Host PDBs are unnecessary for this narrow invariant check.
        cmd+=['-DDBT_PROBE_STAY_NATIVE=1']
    if options.compact_copy:cmd+=['-DDBT_PROBE_PCM_COPIED=1','-DDBT_PROBE_ASYNC_AUDIO=1','-DDBT_PROBE_PCM_TARGET_MS=16']
    if options.block_dsp:cmd+=['-DDBT_AUDIO_BLOCK_DSP=1','-DFIXED_DSP_BLOCK_API=1','-DDBT_PROBE_BLOCK_DSP=1']
    if options.priority_order:cmd+=['-DDBT_PROBE_PRIORITY_ORDER=1','-DDBT_PROBE_DECODER_PRIORITY='+('21' if options.refill_window else '6')]
    if options.word_dsp:cmd+=['-DDBT_PROBE_WORD_DSP=1','-DFIXED_DSP_WORD_QUANT=1']
    if options.assist_refill:cmd+=['-DDBT_PROBE_DSP_ASSIST=1','-DDBT_PROBE_CODEC_AWARE_RENDER=1','-DDBT_OUTPUT_TEST_HOOKS=1','-DDBT_AUDIO_UNITY_COPY=1']
    if os.name!='nt':cmd+=['-fPIC']
    cmd+=sum((['-I',str(i)] for i in incs),[])+[str(s) for s in src]
    if options.assist_refill:
        cmd.remove('-shared')
        for s in src:cmd.remove(str(s))
        cmd+=['-DDBT_REFILL_STANDALONE=1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
        if os.name=='nt':
            cmd+=['-fms-runtime-lib=dll','-D_DLL']
            resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
            env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
    for hz in (100,1000):
        tag='assist-refill' if options.assist_refill else 'refill' if options.refill_window else 'compact-block' if options.compact_copy and options.block_dsp else 'compact' if options.compact_copy else 'block' if options.block_dsp else 'format'
        artifact=out/(f'probe-output-{tag}-{hz}.dll' if os.name=='nt' else f'probe-output-{tag}-{hz}.so')
        if options.assist_refill:
            artifact=out/(f'probe-output-{tag}-{hz}.exe' if os.name=='nt' else f'probe-output-{tag}-{hz}')
            objects=[]
            for i,s in enumerate(src):
                obj=out/f'assist-refill-{hz}-{i}.obj';objects.append(str(obj))
                subprocess.run(cmd+['-DDBT_TEST_TICK_HZ='+str(hz),'-c',str(s),'-o',str(obj)],env=env,check=True)
            link=cmd+objects+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else [])+['-o',str(artifact)]
            subprocess.run(link,env=env,check=True)
            result=subprocess.run([str(artifact)],env=env,capture_output=True,text=True,timeout=15)
            print(result.stdout.strip());print(result.stderr.strip());result.check_returncode()
            runs.append({'tick_hz':hz,'stdout':result.stdout,'stderr':result.stderr,'returncode':result.returncode})
            print(f'PASS sanitized busy-helper refill at {hz} Hz; no target timing proof')
            continue
        subprocess.run(cmd+['-DDBT_TEST_TICK_HZ='+str(hz),'-o',str(artifact)],env=env,check=True)
        lib=ctypes.CDLL(str(artifact))
        if options.refill_window:
            line=lib.probe_output_refill_windows()
            if line:raise SystemExit('FAIL focused refill window line '+str(line))
            print(f'PASS focused late refill ordering/expiry/wrap/stop/budget guards at {hz} Hz; mocked I2S, not RF/timing proof')
            continue
        line=(lib.probe_output_compact_copy if options.compact_copy else lib.probe_output_formats)()
        if line:raise SystemExit('FAIL production output PCM formats line '+str(line))
        line=lib.probe_output_descriptor_diagnostics()
        if line:raise SystemExit('FAIL distinct descriptor diagnostics line '+str(line))
        print(f'PASS '+('compact copied geometry, source reuse, backpressure and notifications' if options.compact_copy else 'production output 16/24/32-bit formats, bounds, teardown and nonzero event waits')+f' at {hz} Hz, mocked I2S')
finally:subprocess.run(check,check=True)
if options.assist_refill:
    assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in source_hashes.items())
    receipt.write_text(json.dumps({'passed':True,'hardware_accessed':False,'sanitizers':['address','undefined'],'signed_arithmetic_wrapping':True,'scope':'actual owner/helper/audio/DSP code with deterministic publication interleaving; no target timing or multithread proof','sources':source_hashes,'runs':runs},indent=2)+'\n')
