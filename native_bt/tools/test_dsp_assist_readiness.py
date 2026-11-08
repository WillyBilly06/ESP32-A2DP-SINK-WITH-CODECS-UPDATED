"""Actual decoder-phase/admission checks. No SDK, phone or radio access."""
import hashlib,json,os,re,shutil,subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[1]
out=root/'_build/dsp_assist_readiness';out.mkdir(parents=True,exist_ok=True)
receipt=out/'results.json';receipt.write_text(json.dumps({'passed':False,'hardware_accessed':False})+'\n')
source=root/'esp32_probe/main/probe_audio.c';header=root/'esp32_probe/main/probe_audio.h'
text=source.read_text();head=header.read_text()
begin=text.index('probe_dsp_admission_t probe_audio_dsp_assist_admission(')
actual=text[begin:text.index('\nstatic TickType_t decoder_wait_ticks',begin)]
begin=head.index('typedef enum {\n    PROBE_DSP_NO_ASSIST')
enum=head[begin:head.index('} probe_dsp_admission_t;',begin)+len('} probe_dsp_admission_t;')]
state=re.search(r'enum \{ DECODER_COMPUTING=5u \};',text).group()
begin=text.index('typedef struct {\n    uint32_t began;')
phase=text[begin:text.index('static const char *const trace_names',begin)]
begin=text.index('static void timing_event(unsigned kind){')
timing=text[begin:text.index('\nstatic uint32_t trace_clock',begin)]
tracked=[source,header,root/'codecs/common/dbt_codec.h',Path(__file__).resolve()]
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
env=os.environ.copy()
if os.name=='nt':
    vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    matches=sorted((Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if matches:cache=str(matches[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
fixture=out/'readiness.c'
fixture.write_text(r'''#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stdio.h>
#include "dbt_codec.h"
static _Atomic uint32_t decoder_wait_kind,pcm_codec;
static int64_t clock_us;
static int64_t esp_timer_get_time(void){clock_us+=17;return clock_us;}
static void maximum(_Atomic uint32_t *v,uint32_t n){if(n>atomic_load(v))atomic_store(v,n);}
'''+enum+'\n'+state+'\n'+phase+'\n'+actual+'\n'+timing+r'''
int main(void){
    const unsigned reasons[]={0,1,2,3,4,5,UINT32_MAX,UINT32_MAX-1};
    unsigned cases=0;
    for(unsigned codec=0;codec<256;++codec)for(unsigned i=0;i<8;++i)for(unsigned reserve=0;reserve<2;++reserve){
        atomic_store(&pcm_codec,codec);atomic_store(&decoder_wait_kind,reasons[i]);
        unsigned expected=0;
#ifdef DBT_PROBE_LHDC_BUILD
        if(codec==9){if(reasons[i]==0 || reasons[i]==3)expected=1;else if(reasons[i]==5 && reserve)expected=2;}
#endif
        if((unsigned)probe_audio_dsp_assist_admission(reserve!=0)!=expected || atomic_load(&decoder_wait_kind)!=reasons[i]){
            fprintf(stderr,"codec=%u reason=%u reserve=%u expected=%u\n",codec,reasons[i],reserve,expected);return 1;
        }
        ++cases;
    }
    /* Compile and execute the actual producer hook; no timeline/profiler is enabled.
     * Reset, PCM copy and all END transitions must revoke busy-decode admission. */
    atomic_store(&pcm_codec,9);
    for(unsigned event=DBT_CODEC_DECODE_BEGIN;event<=DBT_CODEC_PCM_END;++event){
        atomic_store(&decoder_wait_kind,4);timing_event(event);
        unsigned expected=event==DBT_CODEC_DECODE_BEGIN?5:UINT32_MAX;
        if(atomic_load(&decoder_wait_kind)!=expected){fprintf(stderr,"phase=%u\n",event);return 2;}
    }
    atomic_store(&decoder_wait_kind,4);timing_event(UINT32_MAX);
    if(atomic_load(&decoder_wait_kind)!=4)return 3;
    timing_event(DBT_CODEC_DECODE_BEGIN);timing_event(DBT_CODEC_PCM_BEGIN);
    if(probe_audio_dsp_assist_admission(true)!=PROBE_DSP_NO_ASSIST)return 4;
    timing_event(DBT_CODEC_DECODE_BEGIN);timing_event(DBT_CODEC_RESET_BEGIN);
    if(probe_audio_dsp_assist_admission(true)!=PROBE_DSP_NO_ASSIST)return 5;
    printf("PASS actual admission:%u cases, actual decode/reset/PCM/end publication without profiler\n",cases);return 0;
}
''')
resource=Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
if os.name=='nt':env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
commands=[];runs=[]
for enabled in (False,True):
    tag='enabled' if enabled else 'disabled';obj=out/(tag+'.obj');exe=out/(tag+('.exe' if os.name=='nt' else ''))
    base=[cache,cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all',
        '-I'+str(root/'include'),'-I'+str(root/'codecs/common')]
    if os.name=='nt':base+=['-fms-runtime-lib=dll','-D_DLL']
    if enabled:base+=['-DDBT_PROBE_LHDC_BUILD=1']
    compile_command=base+['-c',str(fixture),'-o',str(obj)]
    subprocess.run(compile_command,env=env,check=True);commands.append(compile_command)
    link_command=base+[str(obj)]+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else [])+['-o',str(exe)]
    subprocess.run(link_command,env=env,check=True);commands.append(link_command)
    result=subprocess.run([str(exe)],env=env,check=True,capture_output=True,text=True,timeout=15)
    print(result.stdout.strip());runs.append({'lhdc_enabled':enabled,'admission_cases':4096,'stdout':result.stdout})
    if os.name=='nt':
        data=obj.read_bytes();assert len(data)>=20 and int.from_bytes(data[:2],'little')==0x8664,'Unexpected host object; retained'
        manifest=json.dumps({'passed_executable':str(exe),'object':str(obj),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()},indent=2)+chr(10)
        identity=hashlib.sha256(manifest.encode()).hexdigest()[:16]
        (out/(tag+'-'+identity+'-retired.json')).write_text(manifest);obj.unlink()
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
receipt.write_text(json.dumps({'passed':True,'hardware_accessed':False,'source_sha256':hashes[str(source)],'sources':hashes,
    'sanitizers':['address','undefined'],'commands':commands,'runs':runs,
    'scope':'actual admission and producer phase hook;8192 codec/reason/reserve cases plus phase revocation; no ESP32 scheduling qualification'},indent=2)+'\n')
