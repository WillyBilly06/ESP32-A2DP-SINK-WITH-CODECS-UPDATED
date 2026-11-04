import ctypes, os, pathlib, shutil, subprocess, sys
root=pathlib.Path(__file__).resolve().parents[1]
out=root/'_build'/'avrcp'; out.mkdir(parents=True,exist_ok=True)
cc=os.environ.get('CC') or shutil.which('clang') or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    flags=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry'] if os.name=='nt' else ['-shared','-fPIC']
    extra=[root/'tests/avrcp/runtime_freestanding.c'] if os.name=='nt' else []
    src=[root/'tests/avrcp/probe_integration.c']+[root/'src'/(n+'.c') for n in ('dbt_avctp','dbt_avctp_bearer','dbt_avrcp','dbt_avrcp_bearer','dbt_sdp')]+extra
    for output in (False,True):
        artifact=out/(('probe-output' if output else 'probe')+('.dll' if os.name=='nt' else '.so'))
        define=['-DDBT_PROBE_AUDIO_OUTPUT=1'] if output else []
        subprocess.run([cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-I',str(root/'include'),'-I',str(root/'esp32_probe/main')]+define+flags+[str(x) for x in src]+['-o',str(artifact)],check=True)
        dll=ctypes.CDLL(str(artifact))
        for n in ('exchange','sdp'):
            line=getattr(dll,'probe_test_'+n)()
            if line: raise SystemExit('FAIL '+n+' output='+str(output)+' line '+str(line))
            print('PASS',n,'output='+str(output))
finally: subprocess.run(check,check=True)
