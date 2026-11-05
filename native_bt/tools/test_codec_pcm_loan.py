"""Sanitize actual worker PCM-loan acknowledgement and codec lifetime tests."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/codec_pcm_loan';OUT.mkdir(exist_ok=True)
report=OUT/'results.json';report.write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
names=['codec_deferred_pcm_backpressure_and_exact_order','codec_deferred_cancel_suspend_and_reconfigure',
       'codec_deferred_deadline_and_failure_retirement','codec_async_stream_completion_and_media_fence',
       'codec_async_stream_cancel_before_drop','codec_async_stream_deadline_and_receipt_reservation',
       'codec_async_pcm_completion_and_exact_order','codec_async_pcm_cancel_before_codec_lifecycle',
       'codec_async_pcm_deadline_and_stale_retirement','codec_async_pcm_failed_completion_and_busy_begin']
main=OUT/'main.c'
main.write_text('#include <stdio.h>\n'+''.join('int test_'+name+'(void);\n' for name in names)+
                'int main(void){int result;\n'+''.join('result=test_'+name+'();if(result){printf("FAIL '+name+' line %d\\n",result);return 1;}\n' for name in names)+
                'puts("PASS 10 worker groups: immutable PCM loans, retirement, controls, deadlines and stale identity");return 0;}\n')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    candidates=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if candidates:cache=str(candidates[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
sources=[ROOT/'tests/test_codec_deferred.c',ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c',main]
tracked=sources+list((ROOT/'include').glob('*.h'))+[ROOT/'codecs/common/dbt_codec.h',pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
exe=OUT/('loans.exe' if os.name=='nt' else 'loans')
cmd=[cache,cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT/'include'),'-I'+str(ROOT/'codecs/common'),*map(str,sources),'-o',str(exe)]
if os.name=='nt':cmd+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
subprocess.run(cmd,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());shutil.copy2(resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll',OUT/'clang_rt.asan_dynamic-x86_64.dll')
subprocess.run([str(exe)],check=True,env=env,timeout=60)
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
report.write_text(json.dumps({'passed':True,'groups':names,'sanitizers':['address','undefined'],'signed_overflow_wrapping':False,'sources':before,'command':cmd,'scope':'host worker/loan ownership model; no renderer/provider integration or hardware qualification'},indent=2)+'\n')
