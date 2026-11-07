"""Compare opt-in block DSP to canonical per-frame DSP, with ASan/UBSan."""
import argparse,hashlib,json,os,pathlib,re,shutil,subprocess
parser=argparse.ArgumentParser();group=parser.add_mutually_exclusive_group();group.add_argument('--transposed',action='store_true');group.add_argument('--word-quant',action='store_true');parser.add_argument('--word-ramp',action='store_true');parser.add_argument('--volume-ramp',action='store_true');parser.add_argument('--volume-compare',action='store_true');options=parser.parse_args()
if options.volume_compare:options.volume_ramp=True
if options.word_ramp:
    if options.transposed:parser.error('--word-ramp excludes --transposed')
    options.word_quant=True
ROOT=pathlib.Path(__file__).resolve().parents[1];DSP=ROOT.parent/'bt_audio_sink/main/dsp';OUT=ROOT/('_build/dsp_block_word_ramp' if options.word_ramp else '_build/dsp_block_word' if options.word_quant else '_build/dsp_block_transposed' if options.transposed else '_build/dsp_block');OUT.mkdir(exist_ok=True)
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
reference=OUT/'reference.c';reference.write_text('#undef FIXED_DSP_BLOCK_API\n#undef FIXED_DSP_WORD_QUANT\n#undef FIXED_DSP_WORD_RAMP\n#undef FIXED_DSP_VOLUME_RAMP_BLOCK\n#undef FIXED_DSP_VOLUME_RAMP_COMPARE\n'+''.join('#define fixed_dsp_'+n+' reference_'+n+'\n' for n in ['init','controls','process','zero_analysis','level_db_q8'])+'#include "'+(DSP/'fixed_dsp.c').as_posix()+'"\n')
sources=[DSP/'fixed_dsp.c',reference,ROOT/'tests/test_fixed_dsp_block.c']
tracked=sources+[pathlib.Path(__file__).resolve(),DSP/'fixed_dsp.h',DSP/'fixed_dsp_tables.inc']
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked};results=[]
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
for mode in ['proven','split_fallback']:
    commands=[];objects=[]
    base=[cache,cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(DSP),'-DFIXED_DSP_BLOCK_API=1']
    if options.transposed:base+=['-DFIXED_DSP_TRANSPOSED_BLOCK=1']
    if options.word_quant:base+=['-DFIXED_DSP_WORD_QUANT=1']
    if options.word_ramp:base+=['-DFIXED_DSP_WORD_RAMP=1']
    if options.volume_ramp:base+=['-DFIXED_DSP_VOLUME_RAMP_BLOCK=1']
    if options.volume_compare:base+=['-DFIXED_DSP_VOLUME_RAMP_COMPARE=1']
    if os.name=='nt':base+=['-fms-runtime-lib=dll','-D_DLL']
    for i,source in enumerate(sources):
        obj=OUT/(mode+'_'+str(i)+('.obj' if os.name=='nt' else '.o'));command=base+(['-DFIXED_DSP_FORCE_SPLIT_SUM=1'] if mode=='split_fallback' and i==0 else [])+['-c',str(source),'-o',str(obj)]
        subprocess.run(command,check=True,env=env);commands.append(command);objects.append(obj)
    exe=OUT/(mode+('.exe' if os.name=='nt' else ''));command=base+list(map(str,objects))+['-o',str(exe)]
    if os.name=='nt':command+=['-Wl,/NODEFAULTLIB:libcmt']
    subprocess.run(command,check=True,env=env);commands.append(command)
    run=subprocess.run([str(exe)],capture_output=True,text=True,check=True,env=env,timeout=120);print(run.stdout.strip())
    match=re.search(r'(\d+) blocks, (\d+) frames',run.stdout);assert match
    results.append({'mode':mode,'blocks':int(match[1]),'frames':int(match[2]),'commands':commands})
word_oracle=None
if options.word_quant:
    oracle=ROOT/'tests/test_dsp_word_quant.c';before[str(oracle)]=hashlib.sha256(oracle.read_bytes()).hexdigest()
    exe=OUT/('oracle.exe' if os.name=='nt' else 'oracle');command=[*base,str(oracle),'-o',str(exe)]
    if os.name=='nt':command+=['-Wl,/NODEFAULTLIB:libcmt']
    subprocess.run(command,check=True,env=env)
    run=subprocess.run([str(exe)],capture_output=True,text=True,check=True,env=env,timeout=60);print(run.stdout.strip())
    word_oracle={'cases':int(re.search(r'PASS (\d+) signed64',run.stdout)[1]),'command':command,'oracle':'signed division/remainder with floor correction and explicit range clamps'}
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'transposed':options.transposed,'word_quant':options.word_quant,'word_ramp':options.word_ramp,'volume_ramp':options.volume_ramp,'volume_compare':options.volume_compare,'word_oracle':word_oracle,'scope':'host exact PCM/state comparison to the existing per-frame implementation; no hardware timing','sanitizers':['address','undefined'],'signed_overflow_wrapping':False,'maximum_block_frames':128,'modes':results,'sources':before,'hardware_accessed':False},indent=2)+'\n')
