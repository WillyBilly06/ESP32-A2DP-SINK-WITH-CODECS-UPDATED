"""Offline production output-format test with real dbt_audio and mocked I2S."""
import argparse,ctypes,gzip,hashlib,json,os,pathlib,shutil,subprocess,sys
parser=argparse.ArgumentParser();parser.add_argument('--compact-copy',action='store_true');parser.add_argument('--block-dsp',action='store_true');parser.add_argument('--priority-order',action='store_true');parser.add_argument('--word-dsp',action='store_true');parser.add_argument('--refill-window',action='store_true');parser.add_argument('--assist-refill',action='store_true');parser.add_argument('--starvation-trace',action='store_true');parser.add_argument('--light-gap',action='store_true');parser.add_argument('--owner-state',action='store_true');options=parser.parse_args()
if options.light_gap and not options.starvation_trace:parser.error('--light-gap requires --starvation-trace')
if options.owner_state and not options.light_gap:parser.error('--owner-state requires --starvation-trace --light-gap')
if options.starvation_trace:
    if options.assist_refill:parser.error('Select one focused sanitized entry point')
    options.refill_window=options.compact_copy=options.block_dsp=options.word_dsp=options.priority_order=True
sanitized=options.assist_refill or options.starvation_trace
if options.assist_refill:
    options.refill_window=options.compact_copy=options.block_dsp=options.word_dsp=True
    options.priority_order=True
if options.word_dsp and not options.block_dsp:raise SystemExit('--word-dsp requires --block-dsp')
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;out=root/'_build/avrcp';out.mkdir(parents=True,exist_ok=True)
if sanitized:
    receipt=out/('owner-state-gap-results.json' if options.owner_state else 'light-gap-results.json' if options.light_gap else 'starvation-trace-results.json' if options.starvation_trace else 'assist-refill-results.json')
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
    if options.starvation_trace:incs.append(root/'tests/scheduler_stubs')
    src=[root/'tests/codecs/probe_output_format.c',root/'src/dbt_audio.c',root/'src/dbt_storage_reserve.c',root/'src/dbt_resampler.c',ws/'bt_audio_sink/main/dsp/fixed_dsp.c']
    if options.assist_refill:src[0]=root/'tests/codecs/probe_output_assist_refill.c';src+=[root/'src/dbt_dsp_job.c']
    if options.starvation_trace:src[0]=root/'tests/codecs/probe_output_starvation.c'
    if options.owner_state:src+=[root/'esp32_probe/main/probe_owner_state.c',root/'src/dbt_hci.c']
    if sanitized:
        tracked=src+[root/'esp32_probe/main/probe_output.c',root/'esp32_probe/main/probe_output.h',root/'tests/codecs/probe_output_format.c',pathlib.Path(__file__).resolve()]
        tracked += [root/'esp32_probe/main/probe_flash_guard.c',root/'esp32_probe/main/probe_flash_guard.h',root/'esp32_probe/main/probe_flash_guard.lf',ws/'esp-idf/components/freertos/FreeRTOS-Kernel/tasks.c']
        if options.owner_state:tracked+=[root/'esp32_probe/main/probe_owner_state.h',*sorted((root/'include').glob('*.h'))]
        source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked};runs=[]
    cmd=[cache,cc,'-std=c11','-O1','-fwrapv','-shared','-DCONFIG_I2S_ISR_IRAM_SAFE=1','-DCONFIG_NATIVE_AUDIO_BCLK=26','-DCONFIG_NATIVE_AUDIO_LRCK=25','-DCONFIG_NATIVE_AUDIO_DATA=27']
    if options.assist_refill or options.starvation_trace:cmd+=['-DDBT_PROBE_STORAGE_RESERVE=1']
    if options.refill_window:
        cmd+=['-DDBT_PROBE_STAY_NATIVE=1']
    if options.compact_copy:cmd+=['-DDBT_PROBE_PCM_COPIED=1','-DDBT_PROBE_ASYNC_AUDIO=1','-DDBT_PROBE_PCM_TARGET_MS=16']
    if options.block_dsp:cmd+=['-DDBT_AUDIO_BLOCK_DSP=1','-DFIXED_DSP_BLOCK_API=1','-DDBT_PROBE_BLOCK_DSP=1']
    if options.priority_order:cmd+=['-DDBT_PROBE_PRIORITY_ORDER=1','-DDBT_PROBE_DECODER_PRIORITY='+('21' if options.refill_window else '6')]
    if options.word_dsp:cmd+=['-DDBT_PROBE_WORD_DSP=1','-DFIXED_DSP_WORD_QUANT=1']
    if options.assist_refill:cmd+=['-DDBT_PROBE_DSP_ASSIST=1','-DDBT_PROBE_CODEC_AWARE_RENDER=1','-DDBT_OUTPUT_TEST_HOOKS=1','-DDBT_AUDIO_UNITY_COPY=1']
    if options.starvation_trace and not options.light_gap:cmd+=['-DDBT_PROBE_SCHEDULER_TRACE=1']
    if options.owner_state:cmd+=['-DPROBE_OWNER_STATE_TRACE=1']
    if options.owner_state or options.assist_refill:cmd+=['-DDBT_PROBE_HIGH_RATE_PCM_REBALANCE=1']
    if os.name!='nt':cmd+=['-fPIC']
    cmd+=sum((['-I',str(i)] for i in incs),[])+[str(s) for s in src]
    if sanitized:
        cmd.remove('-shared')
        for s in src:cmd.remove(str(s))
        cmd+=['-DDBT_REFILL_STANDALONE=1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
        if os.name=='nt':
            cmd+=['-fms-runtime-lib=dll','-D_DLL']
            resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
            env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
    for hz in (100,1000):
        tag='owner-state-gap' if options.owner_state else 'light-gap' if options.light_gap else 'starvation-trace' if options.starvation_trace else 'assist-refill' if options.assist_refill else 'refill' if options.refill_window else 'compact-block' if options.compact_copy and options.block_dsp else 'compact' if options.compact_copy else 'block' if options.block_dsp else 'format'
        artifact=out/(f'probe-output-{tag}-{hz}.dll' if os.name=='nt' else f'probe-output-{tag}-{hz}.so')
        if sanitized:
            artifact=out/(f'probe-output-{tag}-{hz}.exe' if os.name=='nt' else f'probe-output-{tag}-{hz}')
            objects=[]
            for i,s in enumerate(src):
                obj=out/f'{tag}-{hz}-{i}.obj';objects.append(str(obj))
                subprocess.run(cmd+['-DDBT_TEST_TICK_HZ='+str(hz),'-c',str(s),'-o',str(obj)],env=env,check=True)
            link=cmd+objects+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else [])+['-o',str(artifact)]
            subprocess.run(link,env=env,check=True)
            result=subprocess.run([str(artifact)],env=env,capture_output=True,text=True,timeout=15)
            print(result.stdout.strip());print(result.stderr.strip());result.check_returncode()
            # Preserve the tested binary and exact source/command evidence. Only
            # completed AMD64 COFF compiler intermediates are rebuildable here.
            if os.name=='nt':
                retired=[]
                for name in objects:
                    p=pathlib.Path(name);data=p.read_bytes()
                    assert len(data)>=20 and int.from_bytes(data[:2],'little')==0x8664,'Unexpected host object; retained'
                    retired.append({'path':str(p),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'coff_machine':'AMD64'})
                manifest=json.dumps({'passed_executable':str(artifact),'objects':retired},indent=2)+chr(10)
                identity=hashlib.sha256(manifest.encode()).hexdigest()[:16]
                (out/f'{tag}-{hz}-{identity}-retired.json').write_text(manifest)
                for name in objects:pathlib.Path(name).unlink()
            if os.name=='nt' and artifact.with_suffix('.pdb').exists():
                pdb=artifact.with_suffix('.pdb');raw=pdb.read_bytes();digest=hashlib.sha256(raw).hexdigest();dest=pdb.with_name(pdb.name+'.'+digest[:12]+'.gz');dest.write_bytes(gzip.compress(raw));assert gzip.decompress(dest.read_bytes())==raw
                (out/f'{tag}-{hz}-{digest[:12]}-symbols.json').write_text(json.dumps({'tested_executable':str(artifact),'executable_sha256':hashlib.sha256(artifact.read_bytes()).hexdigest(),'original':str(pdb),'archive':str(dest),'sha256':digest,'bytes':len(raw),'gzip_bytes':dest.stat().st_size,'lossless_verified':True},indent=2)+chr(10));pdb.unlink()
            runs.append({'tick_hz':hz,'stdout':result.stdout,'stderr':result.stderr,'returncode':result.returncode})
            print(f'PASS sanitized {tag} at {hz} Hz; no target timing proof')
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
if sanitized:
    assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in source_hashes.items())
    receipt.write_text(json.dumps({'passed':True,'hardware_accessed':False,'sanitizers':['address','undefined'],'signed_arithmetic_wrapping':True,'scope':'actual owner/audio/DSP code with deterministic virtual clock and interleavings; no target timing or multithread proof','sources':source_hashes,'runs':runs},indent=2)+'\n')
