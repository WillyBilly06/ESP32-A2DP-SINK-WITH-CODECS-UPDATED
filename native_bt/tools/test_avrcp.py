import ctypes, os, pathlib, shutil, subprocess
root=pathlib.Path(__file__).resolve().parents[1]
out=root/'_build'/'avrcp'
out.mkdir(parents=True,exist_ok=True)
compiler=os.environ.get('CC') or shutil.which('clang') or shutil.which('cc')
if not compiler and os.name=='nt': compiler=r'C:\Program Files\LLVM\bin\clang.exe'
lib=out/('avrcp.dll' if os.name=='nt' else 'avrcp.so')
args=[compiler,'-std=c11','-O1','-Wall','-Wextra','-Werror','-I',str(root/'include')]
if os.name=='nt': args+=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry']
else: args+=['-shared','-fPIC']
args += [str(root/'src'/'dbt_avctp.c'),str(root/'src'/'dbt_avrcp.c'),str(root/'src'/'dbt_avrcp_browse.c'),str(root/'src'/'dbt_avctp_bearer.c'),str(root/'src'/'dbt_avrcp_bearer.c'),str(root/'tests'/'avrcp'/'test_avrcp.c'),'-o',str(lib)]
subprocess.run(args,check=True)
dll=ctypes.CDLL(str(lib))
for name in ('vector_and_fragments','notify_and_parallel','malformed_stale_busy','labels_time_and_ipid','aba_and_bad_notification','unit_discovery','metadata_continuation','browse_wire','truncated_and_bounded','bearer_borrow','notification_label_reserve','target_continuation','target_abort_and_expiry','control_reject_backpressure','controller_abort_quarantines','controller_interleave_aborts','browse_player_uid_lifecycle','browse_rejects_and_items','control_browse_ops_and_notifications','browse_addressed_scope_uid','not_implemented_echo','browse_large_bounded_sdu','continuation_rejected','browse_generated_and_wrap'):
    result=getattr(dll,'test_'+name)()
    if result: raise SystemExit(f'FAIL {name} line {result}')
    print('PASS',name)
