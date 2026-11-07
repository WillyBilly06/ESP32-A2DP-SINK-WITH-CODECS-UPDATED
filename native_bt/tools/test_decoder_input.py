"""Sanitize actual 1800-byte worker admission, copy, saturation and wrap paths."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/decoder_input';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'hardware_accessed':False,'state':'building'})+'\n')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    choices=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if choices:cache=str(choices[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
names=['codec_1800_packet_boundaries_and_queue_wrap','codec_diagnostics_classify_local_and_upstream_loss','codec_queue_gates_and_copy','codec_lazy_lifecycle_and_rollback','codec_input_clock_snapshot_and_scope']
main=OUT/'main.c';main.write_text('#include <stdio.h>\n'+''.join('int test_'+n+'(void);\n' for n in names)+'int main(void){int result;\n'+''.join('result=test_'+n+'();if(result){printf("FAIL '+n+' line %d\\n",result);return 1;}\n' for n in names)+'puts("PASS decoder input: 1800-byte boundaries, eight slots, copy lifetime, queue saturation, counter wrap, RTP clock snapshot/scope and teardown");return 0;}\n')
sources=[ROOT/'tests/test_codec.c',ROOT/'codecs/common/dbt_codec.c',ROOT/'codecs/aac/dbt_aac_transport.c',ROOT/'src/dbt_avdtp_caps.c',main]
tracked=sources+list((ROOT/'include').glob('*.h'))+list((ROOT/'codecs/common').glob('*.h'))+list((ROOT/'codecs/aac').glob('*.h'))+[pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
exe=OUT/('decoder-input.exe' if os.name=='nt' else 'decoder-input')
cmd=[cache,cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT/'include'),'-I'+str(ROOT/'codecs/common'),'-I'+str(ROOT/'codecs/aac')]
if os.name=='nt':cmd+=['-fms-runtime-lib=dll','-D_DLL']
compile_commands=[];objects=[]
for i,source in enumerate(sources):
    obj=OUT/(str(i)+'.obj');command=cmd+['-c',str(source),'-o',str(obj)]
    subprocess.run(command,check=True,env=env);compile_commands.append(command);objects.append(str(obj))
cmd+=objects+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else [])+['-o',str(exe)]
subprocess.run(cmd,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());shutil.copy2(resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll',OUT/'clang_rt.asan_dynamic-x86_64.dll')
subprocess.run([str(exe)],check=True,env=env,timeout=60)
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'groups':names,'packet_payload_bytes':1800,'packet_slots':8,'sanitizers':['address','undefined'],'signed_overflow_wrapping':False,'sources':before,'command':cmd,'compile_commands':compile_commands,'scope':'actual portable worker with model codec; no RF or audio throughput proof'},indent=2)+'\n')
