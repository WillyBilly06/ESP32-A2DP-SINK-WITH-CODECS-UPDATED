"""All float32 conversion patterns plus actual output/quality/overlap block comparison."""
import argparse,gzip,hashlib,json,os,pathlib,re,shutil,subprocess,sys
R=pathlib.Path(__file__).resolve().parents[1];p=argparse.ArgumentParser();p.add_argument('--decoder',type=pathlib.Path,required=True);p.add_argument('--baseline',type=pathlib.Path,required=True);a=p.parse_args();O=R/'_build/lhdc_s32_output';O.mkdir(exist_ok=True);port=O/'port'
(O/'results.json').write_text(json.dumps(dict(passed=False,state='building',hardware_accessed=False))+'\n')
subprocess.run([sys.executable,str(R/'codecs/lhdcv5/prepare_port.py'),'--source',str(a.decoder),'--output',str(port)],check=True,stdout=subprocess.DEVNULL)
old=a.baseline.read_text();new=(port/'lhdc_dec.c').read_text();macro=re.search(r'    #define LHDC_OUT_SAMPLE.*?while \(0\)',old,re.S)[0]
def body(t):
 return t[t.index('    float frame_peak = 0.0f;'):t.index('    /* Stats: concealed (garbage) frames')]
prefix='#include <math.h>\n#include <limits.h>\n#include <string.h>\n#include "lhdc_output_test_common.h"\n#define DBT_LHDC_QUALITY 1\n#define LHDC_HOT\n'
for name,text in [('reference',old),('candidate',new)]:
 code=prefix+('#include "dbt_lhdc_pcm_s32.h"\n' if name=='candidate' else macro+'\n')
 if name=='reference':code+='int32_t reference_sample(uint32_t bits){float v;memcpy(&v,&bits,4);int32_t out;LHDC_OUT_SAMPLE(out,v,-2147483648.0f,2147483647.0f);return out;}\nvoid reference_emit(const float *l,const float *r,int32_t *out,int frames,float scale){for(int i=0;i<frames;++i){LHDC_OUT_SAMPLE(out[2*i],l[i]*scale,-2147483648.0f,2147483647.0f);LHDC_OUT_SAMPLE(out[2*i+1],r[i]*scale,-2147483648.0f,2147483647.0f);}}\n#undef LHDC_OUT_SAMPLE\n'
 else:code+='int32_t candidate_sample(uint32_t bits){float v;memcpy(&v,&bits,4);return dbt_lhdc_s32_sample(v);}\nvoid candidate_emit(const float *l,const float *r,int32_t *out,int frames,float scale){if(scale>0.0f&&isfinite(scale)){(void)dbt_lhdc_scan_emit_s32(l,r,out,frames,scale);}else{for(int i=0;i<frames;++i){out[2*i]=dbt_lhdc_s32_sample(l[i]*scale);out[2*i+1]=dbt_lhdc_s32_sample(r[i]*scale);}}}\n'
 code+='static output_event_t *event;\n#define dbt_lhdc_frame_quality '+name+'_quality\nstatic void '+name+'_quality(float peak,unsigned clipped,int concealed,int nonfinite){memcpy(&event->peak,&peak,4);event->clipped=clipped;event->concealed=concealed;event->nonfinite=nonfinite;}\n'
 code+='typedef struct {int alloc_mdct_size;float *overlap_buf[2];} context_t;\nvoid '+name+'_frame(const float *out_l,const float *out_r,void *out_pcm,int samples_per_ch,int channels,int bytes_per_sample,float sc,output_event_t *result,float *ov0,float *ov1){\ncontext_t ctx={2*samples_per_ch,{ov0,ov1}},*dec=&ctx;int do_stereo=channels>=2;float clip_lim=bytes_per_sample==2?32767.0f:2147483647.0f;event=result;\n'+body(text)+'\n}\n'
 (O/(name+'.c')).write_text(code)
env=os.environ.copy();vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat');v=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in v.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe';cache=shutil.which('ccache',path=env.get('PATH')) or str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1]);env.pop('CCACHE_DISABLE',None)
source=R/'tests/codecs/test_lhdc_s32_output.c';inputs=[source,pathlib.Path(__file__).resolve(),R/'tests/codecs/lhdc_output_test_common.h',R/'codecs/lhdcv5/dbt_lhdc_pcm_s32.h',R/'codecs/lhdcv5/prepare_port.py',a.baseline,port/'lhdc_dec.c',O/'reference.c',O/'candidate.c'];before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in inputs};runs=[]
for sanitizer in (False,True):
 mode='sanitized' if sanitizer else 'exhaustive';flags=[cc,'-std=c11','-O3','-Wall','-Wextra','-Werror','-I'+str(R/'tests/codecs'),'-I'+str(R/'codecs/lhdcv5')]
 if sanitizer:flags+=['-g','-fsanitize=address,undefined,float-cast-overflow','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fms-runtime-lib=dll','-D_DLL']
 objs=[]
 for i,s in enumerate([source,O/'reference.c',O/'candidate.c']):
  obj=O/(mode+str(i)+'.obj');objs.append(obj);subprocess.run([cache,*flags,'-c',str(s),'-o',str(obj)],check=True,env=env)
 exe=O/(mode+'.exe');subprocess.run([*flags,*map(str,objs),'-o',str(exe),*(['-Wl,/NODEFAULTLIB:libcmt'] if sanitizer else [])],check=True,env=env)
 if sanitizer:
  resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());rt=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
  if not rt.is_file():rt=pathlib.Path(cc).parent/rt.name
  assert rt.is_file();shutil.copy2(rt,O/rt.name)
 r=subprocess.run([str(exe)]+(['--quick'] if sanitizer else []),capture_output=True,text=True,env=env,timeout=180);(O/(mode+'.stdout.txt')).write_text(r.stdout);(O/(mode+'.stderr.txt')).write_text(r.stderr);print(r.stdout,flush=True);print(r.stderr,flush=True);r.check_returncode();m=re.search(r'PASS (\d+) float patterns; (\d+) frame',r.stdout);assert m;runs.append(dict(sanitizers=sanitizer,float_patterns=int(m[1]),frame_cases=int(m[2])))
assert runs[0]['float_patterns']==4294967296
assert all(hashlib.sha256(pathlib.Path(f).read_bytes()).hexdigest()==h for f,h in before.items())
retired=[]
for mode in ('exhaustive','sanitized'):
 for i in range(3):
  obj=O/(mode+str(i)+'.obj');raw=obj.read_bytes();assert int.from_bytes(raw[:2],'little')==0x8664
  retired.append(dict(path=str(obj),bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()))
(O/'passing-intermediates.json').write_text(json.dumps(retired,indent=2)+chr(10))
for item in retired:pathlib.Path(item['path']).unlink()
symbols=[]
for pdb in O.glob('*.pdb'):
 raw=pdb.read_bytes();digest=hashlib.sha256(raw).hexdigest();dest=pdb.with_name(pdb.name+'.'+digest[:12]+'.gz');dest.write_bytes(gzip.compress(raw));assert gzip.decompress(dest.read_bytes())==raw
 symbols.append(dict(original=str(pdb),archive=str(dest),sha256=digest,bytes=len(raw),gzip_bytes=dest.stat().st_size,lossless_verified=True))
 (O/'lossless-symbols.json').write_text(json.dumps(symbols,indent=2)+chr(10));pdb.unlink()
subprocess.run([sys.executable,str(R/'tools/check_workspace.py')],check=True)
(O/'results.json').write_text(json.dumps(dict(passed=True,runs=runs,scope='all binary32 patterns; actual original/candidate frame output, quality events and overlap; host only',source_sha256=before),indent=2)+chr(10))
