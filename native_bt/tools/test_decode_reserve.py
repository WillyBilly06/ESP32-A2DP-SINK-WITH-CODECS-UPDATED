"""Exercise the actual same-core decoder admission function with bounded ring states."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/decode_reserve';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
source=ROOT/'esp32_probe/main/probe_output.c';text=source.read_text();before=hashlib.sha256(source.read_bytes()).hexdigest()
def function(signature):
    start=text.index(signature);opening=text.index('{',start);depth=1;i=opening+1
    while depth:
        if text[i]=='{':depth+=1
        elif text[i]=='}':depth-=1
        i+=1
    return text[start:i]
prefix='''#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#define DBT_PROBE_CODEC_AWARE_RENDER 1
#define DBT_PROBE_PCM_COPIED 1
#define DBT_PROBE_STAY_NATIVE 1
#define DBT_PROBE_PCM_TARGET_MS 16
#define DMA_FRAMES 128u
typedef void *TaskHandle_t;
static unsigned observer_core,wakes;
static struct {
    _Atomic uint32_t running,failed,stop_requested,render_core,rate;
    _Atomic(TaskHandle_t) task;
    struct {struct {_Atomic uint32_t read,written;} input,output;} audio;
} output;
static unsigned xPortGetCoreID(void){return observer_core;}
static void xTaskNotifyGive(TaskHandle_t task){if(task)++wakes;}
#define CHECK(x) do{if(!(x)){printf("FAIL line %u\\n",(unsigned)__LINE__);return 1;}}while(0)
'''
main=r'''
static void rings(uint32_t input,uint32_t ready){
    uint32_t start=UINT32_MAX-37u;
    atomic_store(&output.audio.input.read,start);atomic_store(&output.audio.input.written,start+input);
    atomic_store(&output.audio.output.read,start);atomic_store(&output.audio.output.written,start+ready);
}
int main(void){
    atomic_store(&output.running,1);atomic_store(&output.task,(TaskHandle_t)1);
    atomic_store(&output.rate,192000);rings(960,256);
    CHECK(probe_output_decode_budget()==0 && wakes==1);
    /* At the formerly accepted boundary, the observed 3271us call needs
     * 629 frames. The old 256-frame reserve cannot cover it. */
    const uint32_t observed_frames=(3271u*192000u+999999u)/1000000u;
    CHECK(observed_frames==629u && observed_frames>256u);
    rings(960,1215);CHECK(probe_output_decode_budget()==0);
    rings(960,1216);CHECK(probe_output_decode_budget()==1 && 1216u-observed_frames>=256u);
    rings(960,3072);CHECK(probe_output_decode_budget()==1);
    rings(0,0);CHECK(probe_output_decode_budget()==1); /* decoder must supply data */
    const uint32_t rates[]={44100,48000,96000};
    for(unsigned i=0;i<3;++i){atomic_store(&output.rate,rates[i]);rings(1,255);CHECK(probe_output_decode_budget()==0);rings(1,256);CHECK(probe_output_decode_budget()==1);}
    atomic_store(&output.rate,192000);rings(960,0);unsigned old=wakes;
    observer_core=1;CHECK(probe_output_decode_budget()==2 && wakes==old);observer_core=0;
    atomic_store(&output.running,0);CHECK(probe_output_decode_budget()==2);atomic_store(&output.running,1);
    atomic_store(&output.failed,1);CHECK(probe_output_decode_budget()==2);atomic_store(&output.failed,0);
    atomic_store(&output.stop_requested,1);CHECK(probe_output_decode_budget()==2);atomic_store(&output.stop_requested,0);
    /* High rates remain bounded by the renderer's existing target. */
    for(uint32_t rate=96001;rate<=192000;rate+=997){atomic_store(&output.rate,rate);rings(1,pcm_target_frames(rate));CHECK(probe_output_decode_budget()==1);}
    printf("PASS actual decoder admission: 192k reserve, recorded deadline, empty input progress, lower-rate compatibility, counter wrap, owner/stop/failure fences and reachable targets\n");return 0;
}
'''
harness=OUT/'admission.c';harness.write_text(prefix+function('static uint32_t pcm_target_frames(')+'\n'+function('uint16_t probe_output_decode_budget(')+'\n'+main)
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
assert cache,'ccache required'
exe=OUT/('admission.exe' if os.name=='nt' else 'admission')
cmd=[cache,cc,'-O2','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all',str(harness),'-o',str(exe)]
subprocess.run(cmd,check=True,env=env)
run=subprocess.run([str(exe)],check=True,env=env,capture_output=True,text=True);print(run.stdout.strip())
assert hashlib.sha256(source.read_bytes()).hexdigest()==before
(OUT/'results.json').write_text(json.dumps({'passed':True,'source_sha256':before,'source':str(source),'scope':'actual extracted admission/target functions; simulated ring/core observations, not physical scheduling','command':cmd,'stdout':run.stdout,'hardware_accessed':False},indent=2)+'\n')
