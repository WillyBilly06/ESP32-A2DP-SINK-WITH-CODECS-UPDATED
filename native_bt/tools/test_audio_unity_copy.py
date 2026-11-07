"""Compare bounded unity PCM copies with the unchanged block renderer, no hardware."""
import argparse,hashlib,json,os,pathlib,re,shutil,subprocess
parser=argparse.ArgumentParser();parser.add_argument('--prepared-dsp',action='store_true');parser.add_argument('--dsp-lane',action='store_true');parser.add_argument('--early-input',action='store_true');options=parser.parse_args()
if options.early_input and (options.prepared_dsp or options.dsp_lane):parser.error('early input uses the local copied renderer, excluding prepared/borrowed DSP')
if options.dsp_lane:options.prepared_dsp=True
ROOT=pathlib.Path(__file__).resolve().parents[1];DSP=ROOT.parent/'bt_audio_sink/main/dsp';OUT=ROOT/('_build/audio_dsp_lane' if options.dsp_lane else '_build/audio_prepared_dsp' if options.prepared_dsp else '_build/audio_unity_copy');OUT=ROOT/'_build/audio_early_input' if options.early_input else OUT;OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    matches=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if matches:cache=str(matches[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
names=sorted(set(re.findall(r'\b(dbt_audio_[a-zA-Z0-9_]+)\s*\(', (ROOT/'include/dbt_audio.h').read_text())))
wrapper=OUT/'fast_render.c'
wrapper.write_text('#include <string.h>\nextern void *unity_test_copy(void *,const void *,size_t);\n#define memcpy unity_test_copy\n#define DBT_AUDIO_UNITY_COPY 1\n'+''.join('#define '+n+' fast_'+n+'\n' for n in names)+'#include "'+(ROOT/'src/dbt_audio.c').as_posix()+'"\n')
sources=[ROOT/'tests/audio/unity_copy.c',ROOT/'src/dbt_audio.c',wrapper,ROOT/'src/dbt_resampler.c',DSP/'fixed_dsp.c']
if options.dsp_lane:sources += [ROOT/'src/dbt_dsp_job.c']
tracked=sources+list((ROOT/'include').glob('*.h'))+list(DSP.glob('fixed_dsp*'))+[ROOT/'src/dbt_resampler_coeffs.inc',pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
base=[cache,cc,'-std=c11','-O3','-g','-fwrapv','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-DDBT_AUDIO_BLOCK_DSP=1','-DFIXED_DSP_BLOCK_API=1','-DFIXED_DSP_WORD_QUANT=1','-I'+str(ROOT/'include'),'-I'+str(DSP)]
if os.name=='nt':base+=['-fms-runtime-lib=dll','-D_DLL']
if options.early_input:base+=['-DDBT_TEST_EARLY_INPUT=1']
if options.prepared_dsp:base+=['-DDBT_TEST_PREPARED_DSP=1']
if options.dsp_lane:base+=['-DDBT_TEST_DSP_LANE=1']
commands=[];objects=[]
for i,source in enumerate(sources):
    obj=OUT/(str(i)+('.obj' if os.name=='nt' else '.o'));cmd=base+['-c',str(source),'-o',str(obj)]
    subprocess.run(cmd,check=True,env=env);commands.append(cmd);objects.append(obj)
exe=OUT/('unity-copy.exe' if os.name=='nt' else 'unity-copy');cmd=base+list(map(str,objects))+['-o',str(exe)]
if os.name=='nt':cmd+=['-Wl,/NODEFAULTLIB:libcmt']
subprocess.run(cmd,check=True,env=env);commands.append(cmd)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
result=subprocess.run([str(exe)],env=env,timeout=60,capture_output=True,text=True)
(OUT/'run.stdout.txt').write_text(result.stdout);(OUT/'run.stderr.txt').write_text(result.stderr)
print(result.stdout.strip());result.check_returncode()
assert 'PASS 192 unity/general stream pairs' in result.stdout and 'FAIL' not in result.stdout
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'early_input':options.early_input,'prepared_dsp':options.prepared_dsp,'dsp_lane':options.dsp_lane,'host_worker_thread':options.dsp_lane and os.name=='nt','stream_pairs':192,'input_frames_per_case':4096,'sanitizers':['address','undefined'],'scope':'every output word and DSP/resampler state versus original general block renderer; copied stereo S32 path exercised explicitly, other formats/phases unchanged; no target timing','sources':before,'commands':commands,'stdout':result.stdout,'hardware_accessed':False},indent=2)+'\n')
