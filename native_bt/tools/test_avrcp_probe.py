import ctypes, hashlib, json, os, pathlib, re, shutil, subprocess, sys
root=pathlib.Path(__file__).resolve().parents[1]
out=root/'_build'/'avrcp'; out.mkdir(parents=True,exist_ok=True)
cc=os.environ.get('CC') or shutil.which('clang') or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
cache=shutil.which('ccache')
if not cache:
    candidates=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if candidates:cache=str(candidates[-1])
assert cache,'ccache required'
os.environ.pop('CCACHE_DISABLE',None)
result={'passed':False,'scope':'offline actual probe/profile wait policy, exchange and SDP; no RF, runtime timing or hardware evidence','commands':[],'groups':[]}
receipt=out/'probe-wait-results.json';receipt.write_text(json.dumps(result)+'\n')
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    flags=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry'] if os.name=='nt' else ['-shared','-fPIC']
    extra=[root/'tests/avrcp/runtime_freestanding.c'] if os.name=='nt' else []
    src=[root/'tests/avrcp/probe_integration.c']+[root/'src'/(n+'.c') for n in ('dbt_avctp','dbt_avctp_bearer','dbt_avrcp','dbt_avrcp_bearer','dbt_sdp','dbt_sdp_client','dbt_sdp_client_bearer','dbt_sdp_discovery')]+extra
    pending=src+[pathlib.Path(__file__).resolve()];tracked=set()
    while pending:
        source=pending.pop().resolve()
        if source in tracked:continue
        tracked.add(source)
        for name in re.findall(r'^\s*#include\s+"([^"]+)"',source.read_text(encoding='utf-8-sig'),re.M):
            for candidate in (source.parent/name,root/'include'/name,root/'esp32_probe/main'/name):
                if candidate.is_file():pending.append(candidate);break
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    for output in (False,True):
        artifact=out/(('probe-output' if output else 'probe')+('.dll' if os.name=='nt' else '.so'))
        define=['-DDBT_PROBE_AUDIO_OUTPUT=1'] if output else []
        command=[cache,cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-I',str(root/'include'),'-I',str(root/'esp32_probe/main')]+define+flags+[str(x) for x in src]+['-o',str(artifact)]
        subprocess.run(command,check=True);result['commands'].append(command)
        dll=ctypes.CDLL(str(artifact))
        for n in ('exchange','sdp','wait_backpressure','volume_burst'):
            line=getattr(dll,'probe_test_'+n)()
            if line: raise SystemExit('FAIL '+n+' output='+str(output)+' line '+str(line))
            result['groups'].append({'group':n,'audio_output':output,'passed':True})
            print('PASS',n,'output='+str(output))
    assert all(hashlib.sha256(p.read_bytes()).hexdigest()==h for p,h in ((pathlib.Path(k),v) for k,v in before.items()))
    result.update(passed=True,source_sha256=before,hardware_accessed=False)
finally:
    receipt.write_text(json.dumps(result,indent=2)+'\n')
    subprocess.run(check,check=True)
