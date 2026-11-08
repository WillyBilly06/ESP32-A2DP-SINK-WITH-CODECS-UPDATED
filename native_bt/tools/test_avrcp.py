"""Actual AVRCP wire/ownership regression; per-object ccache, no hardware."""
import argparse,ctypes,hashlib,json,os,pathlib,shutil,subprocess
root=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--out',type=pathlib.Path,default=root/'_build/avrcp');a=p.parse_args()
out=a.out.resolve();out.relative_to((root/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
receipt=out/'avrcp-results.json';receipt.write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
compiler=os.environ.get('CC') or shutil.which('clang') or shutil.which('cc')
if not compiler and os.name=='nt':compiler=r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache')
if not cache:cache=str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1])
env=os.environ.copy();env.pop('CCACHE_DISABLE',None)
sources=[root/'src'/(name+'.c') for name in ['dbt_avctp','dbt_avrcp','dbt_avrcp_browse','dbt_avctp_bearer','dbt_avrcp_bearer']]+[root/'tests/avrcp/test_avrcp.c']
tracked=[*sources,*(root/'include').glob('*.h'),pathlib.Path(__file__).resolve()]
before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
flags=['-std=c11','-O1','-Wall','-Wextra','-Werror','-I',str(root/'include')]
if os.name=='nt':flags+=['-ffreestanding','-fno-builtin','-fno-stack-protector']
else:flags+=['-fPIC']
commands=[];objects=[]
for i,s in enumerate(sources):
    obj=out/('avrcp-'+str(i)+'.obj');objects.append(obj)
    command=[cache,compiler,*flags,'-c',str(s),'-o',str(obj)];commands.append(command);subprocess.run(command,check=True,env=env)
lib=out/('avrcp.dll' if os.name=='nt' else 'avrcp.so')
link=[compiler,*map(str,objects),'-shared','-o',str(lib)]+(['-fuse-ld=lld','-nostdlib','-Wl,/noentry'] if os.name=='nt' else [])
commands.append(link);subprocess.run(link,check=True,env=env)
dll=ctypes.CDLL(str(lib));names=('vector_and_fragments','notify_and_parallel','malformed_stale_busy','labels_time_and_ipid','aba_and_bad_notification','unit_discovery','metadata_continuation','browse_wire','truncated_and_bounded','bearer_borrow','notification_label_reserve','target_continuation','target_abort_and_expiry','control_reject_backpressure','controller_abort_quarantines','controller_interleave_aborts','browse_player_uid_lifecycle','browse_rejects_and_items','control_browse_ops_and_notifications','browse_addressed_scope_uid','not_implemented_echo','browse_large_bounded_sdu','continuation_rejected','browse_generated_and_wrap')
for name in names:
    result=getattr(dll,'test_'+name)()
    if result:raise SystemExit(f'FAIL {name} line {result}')
    print('PASS',name)
assert before=={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
receipt.write_text(json.dumps({'passed':True,'hardware_accessed':False,'scope':'actual AVRCP/AVCTP wire/state/borrow tests with runtime stand-ins; no RF','tests':names,'sources':before,'commands':commands},indent=2)+'\n')
