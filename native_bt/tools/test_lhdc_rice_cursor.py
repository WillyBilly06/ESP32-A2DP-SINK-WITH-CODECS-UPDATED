"""Compare actual Rice readers, full FAC/model state and malformed boundaries.

No hardware. The preserved baseline is required so an unrelated sibling edit
cannot silently change the comparison. ASan/UBSan cover the actual new code.
"""
import argparse,hashlib,json,os,pathlib,re,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser()
p.add_argument('--decoder',type=pathlib.Path,required=True)
p.add_argument('--baseline',type=pathlib.Path,required=True)
p.add_argument('--out',type=pathlib.Path,required=True)
p.add_argument('--encoder',type=pathlib.Path,default=pathlib.Path('D:/owned61-lhdc-encoder'))
a=p.parse_args();out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
(out/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
port=out/'port'
subprocess.run([sys.executable,str(ROOT/'codecs/lhdcv5/prepare_port.py'),'--source',str(a.decoder),'--output',str(port)],check=True,stdout=subprocess.DEVNULL)
old=a.baseline.read_text();new=(port/'lhdc_entropy_dec.c').read_text()
def function(text,name):
    match=re.search(r'^(?:static|LHDC_HOT)[^\n]*\b'+re.escape(name)+r'\(',text,re.M);assert match,name
    start=at=match.start();opening=text.index('{',at);depth=1;end=opening+1
    while depth:
        if text[end]=='{':depth+=1
        elif text[end]=='}':depth-=1
        end+=1
    return text[start:end]
for name in ('fac_model_init','fac_model_rescale','fac_model_update','fac_decode','s2_at'):
    assert function(old,name)==function(new,name),name+' unexpectedly changed'
prefix=new[new.index('#define FAC_TOTAL_BITS'):new.index('/* Decoder state (byte position')]
prefix=prefix.replace('static LHDC_HOT LHDC_ALWAYS_INLINE int s2_at(s2_lazy_t *z, int i)\n{',
                      'static LHDC_HOT LHDC_ALWAYS_INLINE int s2_at(s2_lazy_t *z, int i)\n{\n    ++peek_calls;')
assert '++peek_calls;' in prefix
reference=function(old,'rice_read_quotient')
coefficients=function(new,'rice_decode_coeffs_used')
(out/'rice_under_test.h').write_text(prefix+'\n#include "dbt_lhdc_rice.h"\n'+reference.replace('rice_read_quotient','reference_rice_read_quotient')+'\n'+coefficients+'\n'+function(new,'s2_state_after')+'\n'+function(new,'lhdc_entropy_decode_spectrum_ex2')+'\n')
(out/'rice_tape_reference.h').write_text(reference.replace('rice_read_quotient','reference_tape_rice').replace('s2_at','tape_at')+'\n'+coefficients.replace('rice_decode_coeffs_used','tape_coefficients').replace('rice_read_quotient','tape_rice')+'\n')
assert 'if (z.error) return LHDC_DEC_BITSTREAM_ERROR;' in new
assert 'z.eos = 0; z.error = 0;' in new
source=ROOT/'tests/codecs/test_lhdc_rice_cursor.c'
encoder_files=sorted([*(a.encoder/'src').glob('*.c'),*(a.encoder/'src').glob('*.h'),*(a.encoder/'include').glob('*.h'),*(a.encoder/'examples').glob('*.c'),a.encoder/'LICENSE',a.encoder/'README.md'],key=lambda f:f.relative_to(a.encoder).as_posix())
encoder_digest=hashlib.sha256()
for f in encoder_files:encoder_digest.update(f.relative_to(a.encoder).as_posix().encode()+b'\0');encoder_digest.update(f.read_bytes())
assert len(encoder_files)==17 and encoder_digest.hexdigest()=='6917725b0598e56af0f7ba20490cfadeb9b0910aefa822c4b71a0af726c0651a'
encoder_units=[a.encoder/'src/lhdcv5_arith.c',a.encoder/'src/lhdcv5_bitout.c',a.decoder/'lhdc_bit_reader.c']
tracked=[*encoder_files,*encoder_units,a.encoder/'src/lhdcv5_int.h',source,pathlib.Path(__file__).resolve(),ROOT/'codecs/lhdcv5/prepare_port.py',ROOT/'codecs/lhdcv5/dbt_lhdc_rice.h',a.baseline,port/'lhdc_entropy_dec.c',out/'rice_under_test.h',out/'rice_tape_reference.h']
before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    choices=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if choices:cache=str(choices[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
flags=['-D_CRT_SECURE_NO_WARNINGS','-DLHDC_HOST_BUILD=1','-I'+str(a.encoder/'src'),'-I'+str(a.encoder/'include'),'-I'+str(port),'-I'+str(a.decoder),'-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-const-variable','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(out),'-I'+str(ROOT/'codecs/lhdcv5')]
if os.name=='nt':flags+=['-fms-runtime-lib=dll','-D_DLL']
exe=out/('rice.exe' if os.name=='nt' else 'rice')
objects=[out/('rice_'+str(i)+'.obj') for i in range(1+len(encoder_units))]
commands=[[cache,cc,*flags,'-c',str(unit),'-o',str(obj)] for unit,obj in zip([source,*encoder_units],objects)]
commands.append([cc,*flags,*map(str,objects),'-o',str(exe)]+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else []))
for command in commands:subprocess.run(command,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
    assert runtime.is_file();shutil.copy2(runtime,out/runtime.name)
r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=120)
(out/'stdout.txt').write_text(r.stdout);(out/'stderr.txt').write_text(r.stderr)
print(r.stdout.strip());print(r.stderr.strip());r.check_returncode()
m=re.search(r'PASS (\d+) valid Rice/FAC/model/history transitions; overflow_rejects=(\d+); old_peeks=(\d+); new_peeks=(\d+)',r.stdout);assert m
assert before=={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
(out/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'valid_transitions':int(m[1]),'overflow_rejects':int(m[2]),'old_peeks':int(m[3]),'new_peeks':int(m[4]),'source_sha256':before,'commands':commands,'scope':'actual cached quotient versus preserved original, real FAC range/model/history state and exhaustive ternary tapes; invalid overflow has explicit rejection rather than undefined reference execution'},indent=2)+'\n')
