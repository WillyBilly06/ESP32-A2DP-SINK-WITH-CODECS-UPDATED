"""Actual host trace/producer/logger tests; no ESP32/phone access."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/host_state_trace';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'hardware_accessed':False})+'\n')
source=ROOT/'esp32_probe/main/probe_audio.c';text=source.read_text()
types=text[text.index('typedef probe_audio_trace_record_t media_trace_t;'):text.index('static media_trace_ring_t media_trace_rings[2];')]
names=text[text.index('static const char *const trace_names[]='):text.index('static void maximum(')]
producer=text[text.index('static uint32_t trace_clock('):text.index('static void media_trace(')]
route_start=text.index('static void media_trace(');route=text[route_start:text.index('\n#if defined(DBT_PROBE_SCHEDULER_TRACE)',route_start)]
start=text.index('static void log_media_trace(void){');end=text.index('\n#ifdef DBT_PROBE_LHDC_BUILD',start)
logger=text[start:end];assert logger.rstrip().endswith('}')
(OUT/'actual_trace_logger.inc').write_text(types+names+producer+route+logger)
main=(ROOT/'esp32_probe/main/probe_main.c').read_text();blocks=[]
for kind,name,parameters in [('PROBE_HOST_CLASSIC_VISIBILITY','fixture_visibility','test_gap_t gap,uint8_t wanted_scan,bool discoverable,unsigned classic_origin'),('PROBE_HOST_GAP_STATE','fixture_gap','test_gap_t gap')]:
    marker=main.index('.kind='+kind);begin=main.rfind('#ifdef DBT_PROBE_AUDIO_TEST',0,marker);statement=main.index('HOST_PRINTF(',marker);finish=main.index(');',statement)+2
    block=main[begin:finish];assert '==DBT_UNSUPPORTED' in block and '#endif' in block
    blocks.append('static void '+name+'('+parameters+'){\n'+block+'\n}\n')
begin=main.index('                const dbt_runtime_failure_t *f=&runtime.failure;',main.index('else if (host->state == DBT_HOST_FAILED)'));finish=main.index('\n            }',begin)
blocks.append('static void fixture_failure(test_host_t *host,test_runtime_t runtime){\n'+main[begin:finish]+'\n}\n')
(OUT/'actual_main_records.inc').write_text(''.join(blocks))
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
    values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:/Program Files/LLVM/bin/clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    matches=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'));assert matches;cache=str(matches[-1])
env.pop('CCACHE_DISABLE',None)
includes=[ROOT/'include',ROOT/'codecs/common',ROOT/'esp32_probe/main',OUT]
sources=[ROOT/'esp32_probe/main/probe_audio_trace.c',ROOT/'tests/test_host_state_trace.c']
tracked=sources+[source,ROOT/'esp32_probe/main/probe_audio_trace.h',ROOT/'esp32_probe/main/probe_host_record.h',ROOT/'esp32_probe/main/probe_main.c',ROOT/'esp32_probe/main/probe_audio.h',ROOT/'codecs/common/dbt_codec.h',ROOT/'include/dbt_runtime.h',ROOT/'tests/host_trace_hook.h',pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
base=[cache,cc,'-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-include',str(ROOT/'tests/host_trace_hook.h')]
if os.name=='nt':base+=['-fms-runtime-lib=dll','-D_DLL']
base+=['-I'+str(p) for p in includes];commands=[];objects=[]
for i,path in enumerate(sources):
    obj=OUT/(str(i)+('.obj' if os.name=='nt' else '.o'));command=[*base,'-c',str(path),'-o',str(obj)]
    subprocess.run(command,env=env,check=True);commands.append(command);objects.append(obj)
exe=OUT/('host-state.exe' if os.name=='nt' else 'host-state');command=[cc,'-fsanitize=address,undefined',*map(str,objects),'-o',str(exe)]
if os.name=='nt':command+=['-fms-runtime-lib=dll','-Wl,/NODEFAULTLIB:libcmt']
subprocess.run(command,env=env,check=True);commands.append(command)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll';assert runtime.is_file()
    shutil.copy2(runtime,OUT/runtime.name);runtime_sha256=hashlib.sha256(runtime.read_bytes()).hexdigest()
    env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
run=subprocess.run([str(exe)],env=env,capture_output=True,text=True,timeout=15)
(OUT/'run.stdout.txt').write_text(run.stdout);(OUT/'run.stderr.txt').write_text(run.stderr)
print(run.stdout.strip());print(run.stderr.strip());run.check_returncode();assert 'PASS bounded shared trace' in run.stdout
parsed=[json.loads(line[len('FAILURE_JSON '):]) for line in run.stdout.splitlines() if line.startswith('FAILURE_JSON ')]
assert len(parsed)==2
expected_failed={'probe':'failed','fault':5,'controller_status':17,'pending_opcode':4660,'rx_rejected':0xfedcba98,'rx_flow_failed_opcode':64513,'event_us':0xfffffffe,'host_log_ordinal':1}
expected_parser={'probe':'parser_failure','at_ms':0xfffffff2,'origin':6,'result':3,'handle':291,'generation':0xabcdef01,'link_present':1,'cid':17767,'bytes':1800,'declared':4096,'type':2,'event':62,'pb':2,'bc':1,'code':11,'identifier':239,'host_stage':7,'host_packet_bytes':1029,'host_type':2,'host_event':19,'event_us':0xfffffffe,'host_log_ordinal':1}
assert parsed==[expected_failed,expected_parser],parsed
stalled=[json.loads(line[len('FAILURE_STALLED_JSON '):]) for line in run.stdout.splitlines() if line.startswith('FAILURE_STALLED_JSON ')]
assert stalled==[expected_failed|{'controller_status':99,'event_us':0xffffffff,'host_log_ordinal':2},expected_parser|{'handle':0,'event_us':0xffffffff,'host_log_ordinal':2}],stalled

assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==value for p,value in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'sanitizers':['address','undefined'],'signed_overflow_wrapping':False,'sanitizer_runtime_sha256':runtime_sha256 if os.name=='nt' else None,'scope':'actual queue plus exact main/producer/logger spans,24-field failure group and atomic publication: owner validation, immutable copies, mixed media/state, full/drop/no-clock, uint32 wrap, real JSON fields, stalled logger after slot release; no UART or MCU timing','failure_json':parsed,'stalled_failure_json':stalled,'sources':before,'commands':commands,'stdout':run.stdout},indent=2)+'\n')
