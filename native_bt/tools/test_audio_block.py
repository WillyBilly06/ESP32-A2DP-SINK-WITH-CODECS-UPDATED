"""Compare block-render loans to the unchanged scalar copied-ring renderer."""
import argparse,hashlib,json,os,pathlib,re,shutil,subprocess
parser=argparse.ArgumentParser();parser.add_argument('--word-quant',action='store_true');options=parser.parse_args()
ROOT=pathlib.Path(__file__).resolve().parents[1];DSP=ROOT.parent/'bt_audio_sink/main/dsp';OUT=ROOT/('_build/audio_block_word' if options.word_quant else '_build/audio_block');OUT.mkdir(exist_ok=True)
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    matches=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if matches:cache=str(matches[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
names=sorted(set(re.findall(r'\b(dbt_audio_[a-zA-Z0-9_]+)\s*\(', (ROOT/'include/dbt_audio.h').read_text())))
wrapper=OUT/'block_render.c';wrapper.write_text('#define DBT_AUDIO_BLOCK_DSP 1\n'+''.join('#define '+n+' block_'+n+'\n' for n in names)+'#include "'+(ROOT/'src/dbt_audio.c').as_posix()+'"\n')
sources=[ROOT/'tests/audio/borrowed_input.c',ROOT/'src/dbt_audio.c',wrapper,ROOT/'src/dbt_resampler.c',DSP/'fixed_dsp.c']
tracked=sources+list((ROOT/'include').glob('*.h'))+list(DSP.glob('fixed_dsp*'))+[ROOT/'src/dbt_resampler_coeffs.inc',pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
base=[cache,cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-DFIXED_DSP_BLOCK_API=1','-DDBT_TEST_BLOCK_RENDER=1','-I'+str(ROOT/'include'),'-I'+str(DSP)]
if options.word_quant:base+=['-DFIXED_DSP_WORD_QUANT=1']
if os.name=='nt':base+=['-fms-runtime-lib=dll','-D_DLL']
commands=[];objects=[]
for i,source in enumerate(sources):
    obj=OUT/(str(i)+('.obj' if os.name=='nt' else '.o'));cmd=base+['-c',str(source),'-o',str(obj)]
    subprocess.run(cmd,check=True,env=env);commands.append(cmd);objects.append(obj)
exe=OUT/('block-render.exe' if os.name=='nt' else 'block-render');cmd=base+list(map(str,objects))+['-o',str(exe)]
if os.name=='nt':cmd+=['-Wl,/NODEFAULTLIB:libcmt']
subprocess.run(cmd,check=True,env=env);commands.append(cmd)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());shutil.copy2(resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll',OUT/'clang_rt.asan_dynamic-x86_64.dll')
run=subprocess.run([str(exe)],check=True,env=env,timeout=60,capture_output=True,text=True);print(run.stdout.strip())
assert 'PASS 108 borrowed/copied stream pairs' in run.stdout
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'word_quant':options.word_quant,'case_pairs':108,'input_frames_per_case':4096,'block_borrowed_vs_original_scalar_copied':True,'sanitizers':['address','undefined'],'sources':before,'commands':commands,'hardware_accessed':False,'scope':'actual block-render conversion/resampler/DSP path and loan lifetime versus original scalar copied path; no target performance'},indent=2)+'\n')
