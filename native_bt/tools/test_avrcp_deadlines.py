"""Actual AVRCP/AVCTP query differential test, ASan/UBSan, no hardware."""
import argparse,hashlib,json,os,pathlib,re,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
PINNED={"src_dbt_avctp.c.before":"90a902a3520c2338d81cce719ba08697f697a9929e8c812f3534b0bd0b661468","src_dbt_avctp_bearer.c.before":"92eca14ed0ff96d64b0c4a87c237eff0c62048466825287fa47d90188e288b58","src_dbt_avrcp.c.before":"4122b1e0f4673a55d38468e983ab4bd39b8e7904c958a01335665ae2bc08b071","src_dbt_avrcp_bearer.c.before":"7c33b50b3b417966a8225c6e078968bdc68d2799578c66523fcd840c529b5506"}
p=argparse.ArgumentParser();p.add_argument('--baseline-dir',type=pathlib.Path,required=True);p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args()
out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
(out/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
texts={}
for name,digest in PINNED.items():
    raw=(a.baseline_dir/name).read_bytes();assert hashlib.sha256(raw).hexdigest()==digest
    texts[name]=raw.decode().replace('\r\n','\n')
def function(s,name):
    m=re.search(r'^[^\n;]*\b'+re.escape(name)+r'\([^;\n]*\)\s*\{',s,re.M);assert m,name
    start=m.start();end=s.index('{',start)+1;depth=1
    while depth:
        if s[end]=='{':depth+=1
        elif s[end]=='}':depth-=1
        end+=1
    return s[start:end]
names={'expired':'reference_expired','due':'reference_due','matches':'reference_matches','current':'reference_current','paired':'reference_paired',
       'dbt_avctp_wait_ms':'reference_avctp_wait_ms','dbt_avrcp_wait_ms':'reference_avrcp_wait_ms',
       'dbt_avctp_bearer_wait_ms':'reference_avctp_bearer_wait_ms','dbt_avrcp_bearer_wait_ms':'reference_avrcp_bearer_wait_ms'}
parts=[]
for n in ('expired','due','matches','current','paired','dbt_avctp_wait_ms','dbt_avrcp_wait_ms','dbt_avctp_bearer_wait_ms','dbt_avrcp_bearer_wait_ms'):
    source=next(t for t in texts.values() if re.search(r'^[^\n;]*\b'+n+r'\([^;\n]*\)\s*\{',t,re.M))
    s=function(source,n)
    for old,new in names.items():s=re.sub(r'\b'+old+r'\b',new,s)
    if n=='dbt_avctp_wait_ms':s=s.replace('{','{\n    ++reference_transport_calls;',1)
    parts.append(s)
enums=re.findall(r'enum\s*\{[^}]+\};',texts['src_dbt_avrcp.c.before'])
reference=out/'reference_deadlines.h';reference.write_text('\n'.join(enums+parts)+'\n')
sources=[ROOT/'src'/x for x in ['dbt_avctp.c','dbt_avrcp.c','dbt_avctp_bearer.c','dbt_avrcp_bearer.c']]+[ROOT/'tests/avrcp/test_deadlines.c']
tracked=[*sources,*((ROOT/'include').glob('*.h')),ROOT/'tools/test_avrcp_deadlines.py',reference,*[a.baseline_dir/n for n in PINNED]]
before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:cache=str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1])
env.pop('CCACHE_DISABLE',None)
flags=['-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT/'include'),'-I'+str(out)]
if os.name=='nt':flags+=['-fms-runtime-lib=dll','-D_DLL']
commands=[];objects=[]
for i,s in enumerate(sources):
    obj=out/('deadline-'+str(i)+'.obj');objects.append(obj)
    extra=['-Ddbt_avctp_wait_ms=measured_avctp_wait_ms'] if s.name=='dbt_avctp.c' else []
    command=[cache,cc,*flags,*extra,'-c',str(s),'-o',str(obj)];commands.append(command)
    subprocess.run(command,check=True,env=env)
exe=out/('deadline.exe' if os.name=='nt' else 'deadline')
link=[cc,*flags,*map(str,objects),'-o',str(exe)]+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else []);commands.append(link)
subprocess.run(link,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=120)
(out/'stdout.txt').write_text(r.stdout);(out/'stderr.txt').write_text(r.stderr);print(r.stdout);print(r.stderr);r.check_returncode()
m=re.search(r'PASS (\d+) actual/reference deadline comparisons',r.stdout);assert m
assert before=={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
(out/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'comparisons':int(m[1]),'sanitizers':['address','undefined'],'source_sha256':before,'commands':commands,'baseline_transport_scans':2,'candidate_transport_scans':1,'scope':'actual queries compared with pinned original functions; no RF, CPU timing or full-protocol qualification'},indent=2)+'\n')
