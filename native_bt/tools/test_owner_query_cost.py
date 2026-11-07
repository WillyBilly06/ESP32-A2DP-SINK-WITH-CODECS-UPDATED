"""Test the actual optional owner profiler and call macros with a controlled clock.
No device access; interleavings are explicit, not an ESP32 scheduling model.
"""
import hashlib,json,os,pathlib,re,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/owner_query_cost';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
paths={name:ROOT/'esp32_probe/main'/name for name in ['probe_scheduler.h','probe_scheduler.c','probe_main.c','probe_audio.c']}
texts={name:path.read_text() for name,path in paths.items()}
hashes={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in paths.items()}
header=re.sub(r'^#include .*\n','',texts['probe_scheduler.h'],flags=re.M)
implementation=texts['probe_scheduler.c'].split('/* OWNER_COST_IMPLEMENTATION_BEGIN: actual implementation used by host tests. */',1)[1].split('/* OWNER_COST_IMPLEMENTATION_END */',1)[0]
main=texts['probe_main.c'];call_macro=main[main.index('#define HOST_CALL('):main.index('static int __attribute__')]
off_macro=main[main.index('#define HOST_COST_VALUE(',main.index('#else')):main.index('#define HOST_PRINTF printf')]
prefix='''#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
typedef int esp_err_t;
typedef void *TaskHandle_t;
#define DRAM_ATTR
#define CHECK(x) do{if(!(x)){printf("FAIL line %u\\n",(unsigned)__LINE__);return 1;}}while(0)
'''
enabled=r'''
static bool host_context=true;
static uint32_t clock_value,clock_reads,phase,change_snapshot;
bool probe_scheduler_host_context(void){return host_context;}
unsigned probe_scheduler_host_phase(unsigned next){unsigned old=phase;phase=next;return old;}
static uint32_t esp_cpu_get_cycle_count(void){++clock_reads;return clock_value;}
static void snapshot_fence(void);
#undef atomic_thread_fence
#define atomic_thread_fence(order) snapshot_fence()
'''
tests=r'''
static void snapshot_fence(void){if(change_snapshot){change_snapshot=0;atomic_fetch_add(&owner_cost_sequence,2u);}}
static unsigned expression_calls;
static uint32_t expression(void){++expression_calls;clock_value+=100u;return 23u;}
int main(void){
    probe_owner_cost_t s={.epoch=0x55},before=s;
    CHECK(PROBE_COST_SLOTS==29 && sizeof(owner_cost)==464);
    CHECK(!probe_scheduler_owner_cost_snapshot(0,&s) && !memcmp(&s,&before,sizeof s));
    uint32_t value=0;HOST_COST_VALUE(PROBE_COST_AUDIO_WAIT,value,expression());
    CHECK(value==23 && expression_calls==1 && clock_reads==0);
    probe_scheduler_owner_capture(true);CHECK(!probe_scheduler_owner_cost_finished_epoch(NULL));
    host_context=false;HOST_COST_VALUE(PROBE_COST_AUDIO_WAIT,value,expression());
    CHECK(expression_calls==2 && clock_reads==0);host_context=true;
    HOST_COST_VALUE(PROBE_COST_AUDIO_WAIT,value,expression());
    CHECK(probe_scheduler_owner_cost_snapshot(PROBE_COST_AUDIO_WAIT,&s));
    CHECK(s.epoch==3 && s.calls==1 && s.cycles==100 && s.min_cycles==100 && s.max_cycles==100);
    probe_owner_cost_mark_t m=probe_scheduler_owner_cost_begin();
    probe_scheduler_owner_cost_end(PROBE_COST_AUDIO_WAIT,m);
    CHECK(probe_scheduler_owner_cost_snapshot(PROBE_COST_AUDIO_WAIT,&s) && s.calls==2 && s.min_cycles==0 && s.max_cycles==100);
    clock_value=UINT32_MAX-7u;m=probe_scheduler_owner_cost_begin();clock_value=12;
    probe_scheduler_owner_cost_end(PROBE_COST_AUDIO_WAIT,m);
    CHECK(probe_scheduler_owner_cost_snapshot(PROBE_COST_AUDIO_WAIT,&s) && s.calls==3 && s.cycles==120);
    before=s;atomic_fetch_add(&owner_cost_sequence,1u);
    CHECK(!probe_scheduler_owner_cost_snapshot(PROBE_COST_AUDIO_WAIT,&s) && !memcmp(&s,&before,sizeof s));
    atomic_fetch_add(&owner_cost_sequence,1u);change_snapshot=1;
    CHECK(!probe_scheduler_owner_cost_snapshot(PROBE_COST_AUDIO_WAIT,&s) && !memcmp(&s,&before,sizeof s));
    CHECK(!probe_scheduler_owner_cost_snapshot(PROBE_COST_SLOTS,&s) && !probe_scheduler_owner_cost_snapshot(0,NULL));
    probe_scheduler_owner_cost_end(PROBE_COST_SLOTS,probe_scheduler_owner_cost_begin());
    m=probe_scheduler_owner_cost_begin();probe_scheduler_owner_capture(false);
    CHECK(probe_scheduler_owner_cost_finished_epoch(NULL)==3);clock_value+=500;
    probe_scheduler_owner_cost_end(PROBE_COST_AUDIO_WAIT,m);
    CHECK(probe_scheduler_owner_cost_snapshot(PROBE_COST_AUDIO_WAIT,&s) && s.calls==3);
    probe_scheduler_owner_capture(true);CHECK(!probe_scheduler_owner_cost_finished_epoch(NULL));
    HOST_COST_VALUE(PROBE_COST_RUNTIME_WAIT,value,expression());
    CHECK(probe_scheduler_owner_cost_snapshot(PROBE_COST_RUNTIME_WAIT,&s) && s.epoch==5 && s.calls==1 && s.cycles==100);
    before=s;CHECK(!probe_scheduler_owner_cost_snapshot(PROBE_COST_AUDIO_WAIT,&s) && !memcmp(&s,&before,sizeof s));
    m=probe_scheduler_owner_cost_begin();probe_scheduler_owner_capture(true);clock_value+=40;
    probe_scheduler_owner_cost_end(PROBE_COST_RUNTIME_WAIT,m);
    CHECK(probe_scheduler_owner_cost_snapshot(PROBE_COST_RUNTIME_WAIT,&s) && s.epoch==5 && s.calls==1);
    HOST_CALL(PROBE_HOST_RUNTIME,{HOST_CALL(PROBE_HOST_QUERY,clock_value+=9u);clock_value+=13u;});
    CHECK(phase==PROBE_HOST_OTHER);
    CHECK(probe_scheduler_owner_cost_snapshot(PROBE_COST_POLL_BASE+PROBE_HOST_RUNTIME,&s) && s.cycles==22 && s.calls==1 && s.epoch==7);
    CHECK(probe_scheduler_owner_cost_snapshot(PROBE_COST_POLL_BASE+PROBE_HOST_QUERY,&s) && s.cycles==9 && s.calls==1);
    /* Counter wrap preserves the independent seen bit and extrema. */
    atomic_store(&owner_cost[PROBE_COST_RUNTIME_WAIT].calls,UINT32_MAX);
    atomic_store(&owner_cost[PROBE_COST_RUNTIME_WAIT].cycles,UINT32_MAX-9u);
    HOST_COST_VALUE(PROBE_COST_RUNTIME_WAIT,value,expression());
    CHECK(probe_scheduler_owner_cost_snapshot(PROBE_COST_RUNTIME_WAIT,&s) && s.calls==0 && s.cycles==90 && s.min_cycles==100);
    probe_scheduler_owner_capture(false);uint32_t seen=0;CHECK(probe_scheduler_owner_cost_finished_epoch(&seen)==7 && (seen&(1u<<PROBE_COST_RUNTIME_WAIT)));
    atomic_fetch_add(&owner_cost_sequence,1u);seen=0x55;CHECK(!probe_scheduler_owner_cost_finished_epoch(&seen) && seen==0x55);atomic_fetch_add(&owner_cost_sequence,1u);
    change_snapshot=1;CHECK(!probe_scheduler_owner_cost_finished_epoch(&seen) && seen==0x55);
    atomic_store(&owner_cost_capture,UINT32_MAX-1u);probe_scheduler_owner_capture(true);
    CHECK(atomic_load(&owner_cost_capture)==1u);HOST_COST_VALUE(PROBE_COST_RUNTIME_WAIT,value,expression());
    CHECK(probe_scheduler_owner_cost_snapshot(PROBE_COST_RUNTIME_WAIT,&s) && s.epoch==1 && s.calls==1);
    puts("PASS actual profiler: capture/context gates, once-only expressions, nested inclusive calls, calibration floor, epoch/lifecycle exclusion, unsigned wraps, extrema, one-attempt coherent snapshots, fixed480-byte counters");return 0;
}
'''
offtest=r'''
static unsigned calls;
static uint32_t expression(void){++calls;return 17u;}
int main(void){uint32_t value=0;HOST_COST_VALUE(UNDEFINED_AND_DISCARDED_SLOT,value,expression());CHECK(value==17 && calls==1);puts("PASS trace-off macro evaluates expression once with no profiler storage or clock dependency");return 0;}
'''
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    found=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if found:cache=str(found[-1])
assert cache,'ccache required'
rows=[]
for mode in ['enabled','disabled']:
    source=OUT/(mode+'.c');exe=OUT/(mode+('.exe' if os.name=='nt' else ''))
    source.write_text(prefix+('#define DBT_PROBE_SCHEDULER_TRACE 1\n' if mode=='enabled' else '')+header+(enabled+implementation+call_macro+tests if mode=='enabled' else implementation+'\n#undef HOST_COST_VALUE\n'+off_macro+offtest))
    command=[cache,cc,'-O2','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all',str(source),'-o',str(exe)]
    subprocess.run(command,check=True,env=env)
    run=subprocess.run([str(exe)],check=True,env=env,capture_output=True,text=True);print(run.stdout.strip())
    rows.append({'mode':mode,'command':command,'stdout':run.stdout})
    if mode=='disabled':
        expanded=subprocess.check_output([cache,cc,'-E',str(source)],text=True,env=env)
        assert 'owner_cost_cell_t' not in expanded and 'esp_cpu_get_cycle_count' not in expanded
# Wiring guards protect each actual getter and freeze reporting outside playback.
assert main.count('HOST_WAIT_VALUE(PROBE_COST_')==15
assert 'probe_scheduler_owner_cost_end(PROBE_COST_WAIT_TOTAL,wait_mark)' in main
assert 'probe_scheduler_owner_capture(started)' in texts['probe_audio.c']
assert 'probe_scheduler_owner_cost_finished_epoch(&seen)' in texts['probe_audio.c']
assert hashes=={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in paths.items()}
(OUT/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'source_sha256':hashes,'rows':rows,'scope':'actual extracted optional profiler and macros; controlled clock/interleavings, not hardware timing'},indent=2)+'\n')
