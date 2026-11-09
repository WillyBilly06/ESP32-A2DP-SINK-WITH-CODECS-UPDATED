"""Actual bounded allocation hooks and deferred formatter; Windows model, no hardware."""
import hashlib,json,os,pathlib,shutil,subprocess,time
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/heap_alloc_trace'/str(time.time_ns());OUT.mkdir(parents=True)
result={'passed':False,'hardware_accessed':False,'scope':'actual filtered hook state machine and exact deferred formatter with model RTOS/heap; not target timing, full allocator coverage or RF qualification','commands':[]}
status=1
try:
    assert os.name=='nt','This harness uses Windows events/threads'
    main=ROOT/'esp32_probe/main';shim=ROOT/'tests/heap_alloc/shim'
    audio=(main/'probe_audio.c').read_text()
    start=audio.index('#ifdef DBT_PROBE_HEAP_ALLOC_TRACE\nstatic void log_heap_alloc_capture(')
    end=audio.index('\nstatic void log_loop(',start)
    span=audio[start:end];assert span.rstrip().endswith('#endif')
    (OUT/'actual_heap_logger.inc').write_text(span)
    sources=[main/'probe_heap_alloc_trace.c',ROOT/'tests/heap_alloc/probe_heap_alloc_test.c']
    tracked=sources+[main/'probe_heap_alloc_trace.h',main/'probe_audio.c',pathlib.Path(__file__).resolve()]+list(shim.rglob('*.h'))
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked};result['sources']=before
    env=os.environ.copy();vc=pathlib.Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    cc=r'C:/Program Files/LLVM/bin/clang.exe'
    choices=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'));assert choices
    cache=str(choices[-1]);env.pop('CCACHE_DISABLE',None)
    flags=['-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fms-runtime-lib=dll','-D_DLL','-DDBT_PROBE_HEAP_ALLOC_TRACE=1','-DCONFIG_HEAP_USE_HOOKS=1','-DPROBE_HEAP_ALLOC_TEST=1']
    includes=['-I'+str(x) for x in [shim,main,OUT]];objects=[]
    for i,source in enumerate(sources):
        obj=OUT/(str(i)+'.obj');cmd=[cache,cc,*flags,*includes,'-c',str(source),'-o',str(obj)]
        subprocess.run(cmd,env=env,check=True);result['commands'].append(cmd);objects.append(obj)
    exe=OUT/'heap-test.exe';cmd=[cc,*flags,*map(str,objects),'-Wl,/NODEFAULTLIB:libcmt','-lkernel32','-o',str(exe)]
    subprocess.run(cmd,env=env,check=True);result['commands'].append(cmd)
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());dll=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
    shutil.copy2(dll,OUT/dll.name);result['sanitizer_runtime_sha256']=hashlib.sha256(dll.read_bytes()).hexdigest()
    runs=[]
    for case in range(5):
        run=subprocess.run([str(exe),str(case)],env=env,capture_output=True,text=True,timeout=10)
        (OUT/f'case{case}.stdout.txt').write_text(run.stdout);(OUT/f'case{case}.stderr.txt').write_text(run.stderr)
        run.check_returncode();assert 'PASS bounded allocation sampler' in run.stdout
        runs.append(run.stdout)
    parsed=[json.loads(line) for line in runs[0].splitlines() if line.startswith('{')]
    assert len(parsed)==11
    capture=parsed[0];assert capture['probe']=='heap_alloc_capture' and capture['records']==10 and capture['row_storage_bytes']==1536 and capture['exhaustive'] is False
    allocations=[x for x in parsed if x['probe']=='heap_alloc_observation'];assert len(allocations)==4
    assert allocations[0]['bytes']==144 and allocations[0]['role']==6 and allocations[0]['entry_core']==1
    frees=[x for x in parsed if x['probe']=='heap_free_observation'];assert len(frees)==4
    assert not any('released_in_window' in x for x in parsed)
    assert capture['timestamps_are_callback_observations'] is True
    assert allocations[3]['role']==0 and allocations[3]['task_id']==0x22
    retire=parsed[-1];assert retire['probe']=='heap_task_retire' and retire['task_id']==0x22 and retire['minimum_bytes']==17000 and retire['stack_low_water_bytes']==1234
    sample=parsed[-2];assert sample['probe']=='heap_phase_memory' and sample['job_kind']==1
    assert [sample[k] for k in ['free_before','minimum_before','free_after','minimum_after']]==[30000,20000,24000,17000]
    assert all(hashlib.sha256(p.read_bytes()).hexdigest()==before[str(p)] for p in tracked)
    result.update(passed=True,sanitizers=['address','undefined'],cases=5,formatter_rows=parsed,logger_span_sha256=hashlib.sha256(span.encode()).hexdigest())
    status=0
except Exception as exc:
    result['error']=repr(exc);print(str(exc))
finally:
    (OUT/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(str(OUT/'results.json'))
print('PASS allocation sampler: 5 cases, exact formatter, ASan/UBSan' if status==0 else 'FAIL allocation sampler')
raise SystemExit(status)
