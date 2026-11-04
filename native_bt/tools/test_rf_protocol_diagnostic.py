"""Actual PCM/stream callbacks in product and finite RF-isolation builds; host only."""
import hashlib,json,os,pathlib,re,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/rf_protocol_diagnostic';OUT.mkdir(exist_ok=True)
source=ROOT/'esp32_probe/main/probe_audio.c';text=source.read_text(encoding='utf-8')
def function(name):
 m=re.search(r'static dbt_result_t '+name+r'\(.*?\n\}',text,re.S);assert m,name
 return m.group(0)
start=text.index('#if defined(DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC) &&')
guard=text[start:text.index('#endif',start)+len('#endif')]
main=OUT/'main.c'
main.write_text('''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdatomic.h>
#define DBT_PROBE_AUDIO_OUTPUT 1
#define DBT_PROBE_PRODUCT_IO 1
#define DBT_PROBE_PRODUCT_SECURITY 1
'''+guard+'''
typedef int dbt_result_t;
enum {DBT_OK,DBT_BUSY,DBT_INVALID};
typedef int dbt_pcm_format_t;
static const int dbt_sbc_codec=1;
typedef struct {uint32_t rate;uint8_t sample_bytes,channels,cie_size;const int *ops;} dbt_codec_config_t;
static _Atomic uint32_t pcm_generation,pcm_frames,pcm_bytes,pcm_hash,pcm_rate,pcm_channels,pcm_codec,pcm_sample_bytes;
static unsigned pcm_calls,stream_calls;static bool busy;
#ifndef DBT_PROBE_ASYNC_AUDIO
enum {DBT_CODEC_PCM_BEGIN=15,DBT_CODEC_PCM_END=16};
static void timing_event(unsigned kind){(void)kind;}
#endif
static const void *expected_samples;static dbt_codec_config_t expected;
static dbt_result_t forward(const void *p,uint16_t n,uint8_t c,uint32_t r,dbt_pcm_format_t f){
 assert(p==expected_samples && n==64 && c==expected.channels && r==expected.rate && f==expected.sample_bytes);++pcm_calls;return busy?DBT_BUSY:DBT_OK;
}
dbt_result_t probe_output_try_pcm_format(const void *p,uint16_t n,uint8_t c,uint32_t r,dbt_pcm_format_t f){return forward(p,n,c,r,f);}
dbt_result_t probe_output_pcm_format(const void *p,uint16_t n,uint8_t c,uint32_t r,dbt_pcm_format_t f){return forward(p,n,c,r,f);}
dbt_result_t probe_output_stream_format(bool start,uint32_t r,uint8_t c,dbt_pcm_format_t f){assert(start && r==expected.rate && c==expected.channels && f==expected.sample_bytes);++stream_calls;return busy?DBT_BUSY:DBT_OK;}
'''+function('pcm')+'\n'+function('publish_stream')+'\n#ifndef DBT_PROBE_ASYNC_AUDIO\n'+function('stream')+'\n#endif\n'+'''
int main(void){
 uint8_t data[512];for(unsigned i=0;i<sizeof(data);i++)data[i]=(uint8_t)(i*37u+11u);
 uint32_t digest=0;const uint32_t rates[]={44100,48000,96000,192000};unsigned cases=0;
 for(unsigned b=2;b<=4;b++)for(unsigned c=1;c<=2;c++)for(unsigned r=0;r<4;r++){
  expected=(dbt_codec_config_t){rates[r],(uint8_t)b,(uint8_t)c,0,&dbt_sbc_codec};expected_samples=data;busy=false;
#ifdef DBT_PROBE_ASYNC_AUDIO
  assert(publish_stream(true,&expected)==DBT_OK);
#else
  assert(stream(NULL,true,&expected)==DBT_OK);
#endif
  assert(pcm(NULL,data,64,&expected,99)==DBT_OK);
  assert(atomic_load(&pcm_frames)==1 && atomic_load(&pcm_bytes)==b*c*64 && atomic_load(&pcm_rate)==rates[r] && atomic_load(&pcm_channels)==c && atomic_load(&pcm_sample_bytes)==b);
  digest^=atomic_load(&pcm_hash)+rates[r]+b*17+c;cases++;
 }
 assert(cases==24);
#ifdef DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC
 assert(!pcm_calls && !stream_calls);
#else
 assert(pcm_calls==24);
#ifndef DBT_PROBE_ASYNC_AUDIO
 assert(stream_calls==24);
#endif
#endif
 unsigned calls=pcm_calls;uint32_t frames=atomic_load(&pcm_frames);busy=true;
#ifdef DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC
 assert(pcm(NULL,data,64,&expected,0)==DBT_OK && pcm_calls==calls && atomic_load(&pcm_frames)==frames+1);
#else
 assert(pcm(NULL,data,64,&expected,0)==DBT_BUSY && pcm_calls==calls+1 && atomic_load(&pcm_frames)==frames);
#endif
 calls=pcm_calls;frames=atomic_load(&pcm_frames);
 assert(pcm(NULL,NULL,64,&expected,0)==DBT_INVALID && pcm(NULL,data,0,&expected,0)==DBT_INVALID);
 expected.channels=3;assert(pcm(NULL,data,64,&expected,0)==DBT_INVALID);expected.channels=2;expected.sample_bytes=1;assert(pcm(NULL,data,64,&expected,0)==DBT_INVALID);
 expected.sample_bytes=2;expected.rate=0;assert(pcm(NULL,data,64,&expected,0)==DBT_INVALID);
 assert(pcm_calls==calls && atomic_load(&pcm_frames)==frames);
 printf("%u\\n",digest);return 0;
}
''',encoding='utf-8')
env=os.environ.copy()
if os.name=='nt':
 vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
 data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
 env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'clang')
runtime=[]
if os.name=='nt':
 resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());shutil.copy2(resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll',OUT)
 runtime=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
digests={}
for mode,defines in [('product_sync',[]),('product_async_pcm',['-DDBT_PROBE_ASYNC_AUDIO=1']),('rf_checksum',['-DDBT_PROBE_RF_PROTOCOL_DIAGNOSTIC=1'])]:
 exe=OUT/(mode+('.exe' if os.name=='nt' else ''))
 subprocess.run([cc,'-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all',*runtime,*defines,str(main),'-o',str(exe)],env=env,check=True)
 digests[mode]=subprocess.check_output([str(exe)],env=env,text=True).strip()
assert len(set(digests.values()))==1,digests
guard_file=OUT/'guard.c';guard_file.write_text(guard+'\nint main(void){return 0;}\n')
base=['-DDBT_PROBE_RF_PROTOCOL_DIAGNOSTIC=1','-DDBT_PROBE_AUDIO_OUTPUT=1','-DDBT_PROBE_PRODUCT_IO=1','-DDBT_PROBE_PRODUCT_SECURITY=1']
invalid=[base+['-DDBT_PROBE_ASYNC_AUDIO=1'],base+['-DDBT_PROBE_KEEP_OWNED_ACTIVE=1']]+[[d for d in base if not d.startswith('-D'+feature+'=')] for feature in ('DBT_PROBE_AUDIO_OUTPUT','DBT_PROBE_PRODUCT_IO','DBT_PROBE_PRODUCT_SECURITY')]
for flags in invalid:
 result=subprocess.run([cc,'-std=c11','-fsyntax-only',*flags,str(guard_file)],env=env,capture_output=True,text=True)
 assert result.returncode and 'RF protocol diagnostic requires' in result.stderr
cmake=shutil.which('cmake',path=env.get('PATH'));assert cmake
cmake_text=(ROOT/'esp32_probe/CMakeLists.txt').read_text(encoding='utf-8').split('set(COMPONENTS main)')[0]
cmake_file=OUT/'guard.cmake';cmake_file.write_text(cmake_text)
full={x:'ON' for x in ('RF_PROTOCOL_DIAGNOSTIC','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_OUTPUT','AUDIO_TEST','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP')}
for change in ({},{'ASYNC_AUDIO':'ON'},{'KEEP_OWNED_ACTIVE':'ON'},{'PRODUCT_SECURITY':'OFF'},{'CODEC_DIAGNOSTIC_ONLY':'ON'}):
 options=full|change;result=subprocess.run([cmake,*[f'-DDBT_PROBE_{k}={v}' for k,v in options.items()],'-P',str(cmake_file)],env=env,capture_output=True,text=True)
 assert (result.returncode==0)==(not change),result.stdout+result.stderr
(OUT/'results.json').write_text(json.dumps({'passed':True,'callback_modes':3,'format_rate_cases_per_mode':24,'source_guards_rejected':5,'cmake_configurations':5,'pcm_digests':digests,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'scope':'actual PCM/stream callbacks, ASan/UBSan and actual pre-IDF CMake guards; normal forwarding/backpressure preserved, checksum mode omits I2S calls; host only, no RF/DAC or full application proof','hardware_accessed':False},indent=2)+'\n')
print('PASS RF protocol isolation: actual callbacks, identical PCM accounting, normal backpressure, source/CMake mode rejection; host only')
