"""Host ASan/UBSan for actual paged PCM ring code; never opens hardware."""
import hashlib,json,os,pathlib,shutil,subprocess

ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/audio_pages'
OUT.mkdir(exist_ok=True)
report_path=OUT/'sanitized-results.json'
report_path.write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'clang')
main=OUT/'main.c'
main.write_text('''#include <stdio.h>
int test_audio_paged_ring_capacity_wrap_mix_and_oom(void);
int test_audio_paged_render_matches_small_ring(void);
int main(void){
    int a=test_audio_paged_ring_capacity_wrap_mix_and_oom();
    int b=test_audio_paged_render_matches_small_ring();
    printf("paged ring capacity/OOM/wrap/mix=%d render comparison=%d\\n",a,b);
    return a||b;
}
''')
sources=[ROOT/'tests/test_audio.c',ROOT/'src/dbt_audio.c',ROOT/'src/dbt_resampler.c',ROOT.parent/'bt_audio_sink/main/dsp/fixed_dsp.c',main]
tracked=sources+list((ROOT/'include').glob('*.h'))+list((ROOT.parent/'bt_audio_sink/main/dsp').glob('fixed_dsp*'))+[ROOT/'src/dbt_resampler_coeffs.inc',pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
exe=OUT/('paged-asan.exe' if os.name=='nt' else 'paged-asan')
args=[cc,'-std=c11','-O1','-g','-fwrapv','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
if os.name=='nt':args+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
args+=['-I'+str(ROOT/'include'),'-I'+str(ROOT.parent/'bt_audio_sink/main/dsp'),*[str(p) for p in sources],'-o',str(exe)]
if os.name!='nt':args+=['-lm']
subprocess.run(args,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
    shutil.copy2(runtime,OUT/runtime.name)
subprocess.run([str(exe)],check=True,env=env,timeout=60)
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
report={'passed':True,'sanitizers':['address','undefined'],'scope':'host 65-point OOM unwind, requested extents ASan-poisoned, page/counter wrap, mixer and 16/24/32-bit all-sample render comparisons; no hardware','command':args,'sources':before,'hardware_accessed':False}
report_path.write_text(json.dumps(report,indent=2)+'\n')
print('PASS paged PCM ring ASan/UBSan with exact allocation bounds')
