"""Actual owner deadline-collection differential control-flow test; no hardware."""
import argparse,gzip,hashlib,json,os,pathlib,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
PIN='09731553b89ab7d943c164c7f870fa9e43ad758210e7c2d73915c87a32cab649'
p=argparse.ArgumentParser();p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args()
out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
(out/'results.json').write_text(json.dumps({'passed':False,'state':'building'})+'\n')
source=ROOT/'esp32_probe/main/probe_main.c';wait_header=ROOT/'esp32_probe/main/probe_wait.h'
def chunk(s):
    s=s.replace('\r\n','\n')
    begin=s.index('        uint32_t deadline_now=now_ms();')
    end=s.index('#ifdef DBT_PROBE_AUDIO_TEST\n        probe_audio_owner_wait(wait_zeros);',begin)
    return s[begin:end]
reference=chunk(subprocess.check_output(['git','show','a48179be:native_bt/esp32_probe/main/probe_main.c'],cwd=ROOT,text=True))
assert hashlib.sha256(reference.encode()).hexdigest()==PIN
current=chunk(source.read_text());assert current.count('if(wait!=0){')==1
# Use the actual elapsed-wait helper, without unrelated FreeRTOS declarations.
s=wait_header.read_text()
start=s.index('static inline uint32_t probe_wait_remaining(')
end=s.index('\n}\n',start)+2
remaining=s[start:end]
defines='\n'.join('#define DBT_PROBE_'+n+' 1' for n in ['GATT_TEST','AUDIO_TEST','PRODUCT_SECURITY','SMP','AVRCP','RFCOMM','PRODUCT_RUNTIME','PRODUCT_IO','CLASSIC_SECURITY_TEST'])
names=['dbt_runtime_wait_ms','probe_ble_parameters_wait_ms','probe_admission_wait_ms','probe_smp_wait_ms','probe_avrcp_wait_ms','probe_pair_button_wait_ms','probe_rfcomm_wait_ms','dbt_gap_wait_ms','probe_product_runtime_wait_ms','probe_product_io_owner_wait_ms','probe_gatt_wait_ms','probe_security_wait_ms','probe_echo_wait_ms','probe_sdp_wait_ms','probe_audio_wait_ms']
mocks='\n'.join('#define '+n+'(...) query('+str(i)+')' for i,n in enumerate(names))
harness=r'''#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
__DEFINES__
static uint32_t values[15],counts[15],begin_clock,elapsed,clock_calls;
static bool smp_initialized,echo_initialized,product_runtime_ready,product_io_ready,restarting;
static uint32_t query(unsigned slot){++counts[slot];return values[slot];}
static uint32_t now_ms(void){return begin_clock+(clock_calls++?elapsed:0u);}
#define probe_restart_active(...) restarting
#define HOST_WAIT_VALUE(slot,destination,expression) do{(destination)=(expression);}while(0)
#define WAIT_TRACE(value,bit) do{if((value)==0)wait_zeros|=1u<<(bit);}while(0)
__MOCKS__
__REMAINING__
typedef struct{uint32_t wait,observed_zero_mask;} result_t;
static result_t original(void){
__REFERENCE__
    return(result_t){wait,wait_zeros};
}
static result_t candidate(void){
__CURRENT__
    return(result_t){wait,wait_zeros};
}
static uint32_t random_state=0xd3a5b79f;
static uint32_t next(void){random_state^=random_state<<13;random_state^=random_state>>17;random_state^=random_state<<5;return random_state;}
static unsigned long long cases;
static int check(unsigned guards){
    smp_initialized=(guards&1u)!=0;echo_initialized=(guards&2u)!=0;
    product_runtime_ready=(guards&4u)!=0;product_io_ready=(guards&8u)!=0;restarting=(guards&16u)!=0;
    bool enabled[15]={true,true,true,smp_initialized,echo_initialized,true,true,true,product_runtime_ready,product_io_ready,true,true,true,true,true};
    uint32_t expected=UINT32_MAX;
    for(unsigned i=0;i<15;++i)if(enabled[i]&&values[i]<expected)expected=values[i];
    if(restarting&&expected>10u)expected=10u;
    if(expected&&expected!=UINT32_MAX)expected=elapsed>=expected?0:expected-elapsed;
    memset(counts,0,sizeof counts);clock_calls=0;result_t a=original();uint32_t old_counts[15];memcpy(old_counts,counts,sizeof counts);
    memset(counts,0,sizeof counts);clock_calls=0;result_t b=candidate();
    if(a.wait!=expected||b.wait!=expected){fprintf(stderr,"FAIL wait case%llu old%u new%u expected%u\n",cases,a.wait,b.wait,expected);return 1;}
    for(unsigned i=0;i<15;++i){
        uint32_t wanted=values[0]==0?(i==0?1u:0u):old_counts[i];
        if(counts[i]!=wanted){fprintf(stderr,"FAIL query count case%llu slot%u got%u wanted%u\n",cases,i,counts[i],wanted);return 2;}
    }
    if(values[0]==0 && b.observed_zero_mask!=1u){fprintf(stderr,"FAIL observed-runtime mask\n");return 3;}
    ++cases;return 0;
}
int main(void){
    for(unsigned mask=0;mask<32768u;++mask)for(unsigned guards=0;guards<32u;++guards){
        for(unsigned i=0;i<15;++i)values[i]=(mask&(1u<<i))?0u:((mask+i)&1u?UINT32_MAX:i*31u+7u);
        begin_clock=UINT32_MAX-(mask&1023u);elapsed=(mask+guards)%1000u;
        if(check(guards))return 1;
    }
    for(unsigned n=0;n<100000u;++n){
        for(unsigned i=0;i<15;++i){uint32_t r=next();values[i]=(r&7u)==0?0u:(r&7u)==1?UINT32_MAX:r;}
        begin_clock=next();elapsed=next()%50000u;if(check(next()&31u))return 1;
    }
    printf("PASS %llu actual original/new owner collections: all32768zero masks,32guard states, elapsed/wrap, disabled profiles; same minimum;14queries skipped only when runtime0\n",cases);
    return 0;
}
'''
for token,value in [('__DEFINES__',defines),('__MOCKS__',mocks),('__REMAINING__',remaining),('__REFERENCE__',reference),('__CURRENT__',current)]:harness=harness.replace(token,value)
generated=out/'owner_wait.c';generated.write_text(harness)
negative=out/'negative_owner_wait.c';assert harness.count('if(wait!=0){')==1
negative.write_text(harness.replace('if(wait!=0){','if(1){',1))
tracked=[source,wait_header,pathlib.Path(__file__).resolve(),generated,negative]
before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:cache=str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1])
env.pop('CCACHE_DISABLE',None)
flags=['-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all']
if os.name=='nt':flags+=['-fms-runtime-lib=dll','-D_DLL']
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
commands=[];runs=[];positive_object=None
for name,src,success in [('owner',generated,True),('negative',negative,False)]:
    obj=out/(name+('.obj' if os.name=='nt' else '.o'));exe=out/(name+('.exe' if os.name=='nt' else ''))
    cmd=[cache,cc,*flags,'-c',str(src),'-o',str(obj)];commands.append(cmd);subprocess.run(cmd,check=True,env=env)
    cmd=[cc,*flags,str(obj),'-o',str(exe)]+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else []);commands.append(cmd);subprocess.run(cmd,check=True,env=env)
    r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=90)
    (out/(name+'.stdout.txt')).write_text(r.stdout);(out/(name+'.stderr.txt')).write_text(r.stderr);print(r.stdout,end='');print(r.stderr,end='')
    if success:r.check_returncode();positive_object=obj
    else:assert r.returncode!=0 and 'FAIL query count' in r.stderr,'Missing short-circuit negative control'
    runs.append({'name':name,'returncode':r.returncode,'expected_success':success})
assert before=={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
if os.name=='nt':
    b=positive_object.read_bytes();assert int.from_bytes(b[:2],'little')==0x8664
    (out/'passing-object.json').write_text(json.dumps({'path':str(positive_object),'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest()},indent=2)+'\n');positive_object.unlink()
compressed=[]
for pdb in out.glob('*.pdb'):
    b=pdb.read_bytes();g=gzip.compress(b);assert gzip.decompress(g)==b;pdb.with_suffix('.pdb.gz').write_bytes(g)
    compressed.append({'path':str(pdb),'bytes':len(b),'gzip_bytes':len(g),'sha256':hashlib.sha256(b).hexdigest(),'lossless_verified':True});pdb.unlink()
(out/'lossless-symbols.json').write_text(json.dumps(compressed,indent=2)+'\n')
subprocess.run([sys.executable,str(ROOT/'tools/check_workspace.py')],check=True)
(out/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'baseline_commit':'a48179be','baseline_chunk_sha256':PIN,'comparisons':1148576,'source_sha256':before,'runs':runs,'commands':commands,'scope':'actual owner collection control flow with mocked read-only getters; not protocol lifecycle or target timing'},indent=2)+'\n')
