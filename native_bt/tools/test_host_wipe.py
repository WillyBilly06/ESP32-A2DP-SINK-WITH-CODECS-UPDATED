"""Verify actual HCI scratch wipe across all alignments/tails, ASan/UBSan."""
import argparse,hashlib,json,os,pathlib,re,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--baseline',type=pathlib.Path,required=True);p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args()
out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
(out/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
source=ROOT/'src/dbt_host.c';old=a.baseline.read_text();new=source.read_text()
def function(s,name):
    m=re.search(r'^[^\n;]*\b'+name+r'\([^;\n]*\)\s*\{',s,re.M);assert m
    start=m.start();end=s.index('{',start)+1;depth=1
    while depth:
        if s[end]=='{':depth+=1
        elif s[end]=='}':depth-=1
        end+=1
    return s[start:end]
assert function(old,'sensitive')==function(new,'sensitive'),'Sensitive ACL/key classification changed'
word=re.search(r'typedef uint32_t dbt_wipe_word_t[^;]+;',new);assert word
reference=function(old,'wipe').replace('wipe(','reference_wipe(',1)
candidate=function(new,'wipe').replace('wipe(','candidate_wipe(',1)
harness=r'''#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
__FUNCTIONS__
static unsigned long long cases;
static int check(unsigned size,unsigned offset){
    size_t total=(size_t)size+64u;uint8_t *a=malloc(total),*b=malloc(total);
    if(!a||!b)return 1;
    for(size_t i=0;i<total;++i)a[i]=b[i]=(uint8_t)(i*137u+size+offset+17u);
    unsigned begin=16u+offset;
    reference_wipe(a+begin,size);candidate_wipe(b+begin,size);
    if(memcmp(a,b,total)){fprintf(stderr,"Different wipe size=%u offset=%u\n",size,offset);return 2;}
    for(size_t i=0;i<total;++i){uint8_t expected=i>=begin && i<begin+size?0:(uint8_t)(i*137u+size+offset+17u);
        if(b[i]!=expected){fprintf(stderr,"Boundary/zero mismatch size=%u offset=%u at%zu\n",size,offset,i);return 3;}}
    free(a);free(b);++cases;return 0;
}
int main(void){
    reference_wipe(NULL,0);candidate_wipe(NULL,0);
    for(unsigned offset=0;offset<8;++offset)for(unsigned size=0;size<=4096;++size)if(check(size,offset))return 1;
    const unsigned edges[]={8191,8192,8193,65532,65533,65534,65535,65543};
    for(unsigned offset=0;offset<8;++offset)for(unsigned n=0;n<sizeof(edges)/sizeof(edges[0]);++n)if(check(edges[n],offset))return 1;
    printf("PASS %llu exact original/new wipes; offsets0-7, all lengths0-4096, large/tail boundaries, unchanged guards and ACL/key classification; host only\n",cases);return 0;
}
'''.replace('__FUNCTIONS__',word[0]+'\n'+reference+'\n'+candidate)
generated=out/'wipe.c';generated.write_text(harness)
tracked=[source,a.baseline,pathlib.Path(__file__).resolve(),generated]
before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked};env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe';cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:cache=str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1])
env.pop('CCACHE_DISABLE',None)
flags=['-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
if os.name=='nt':flags+=['-fms-runtime-lib=dll','-D_DLL']
obj=out/'wipe.obj';exe=out/('wipe.exe' if os.name=='nt' else 'wipe')
compile_cmd=[cache,cc,*flags,'-c',str(generated),'-o',str(obj)]
link_cmd=[cc,*flags,str(obj),'-o',str(exe)]+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else [])
subprocess.run(compile_cmd,check=True,env=env);subprocess.run(link_cmd,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=60)
(out/'stdout.txt').write_text(r.stdout);(out/'stderr.txt').write_text(r.stderr);print(r.stdout);print(r.stderr);r.check_returncode()
assert before=={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
(out/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'sanitizers':['address','undefined'],'scope':'actual source erase policy/content/bounds; not target timing','sources':before,'commands':[compile_cmd,link_cmd]},indent=2)+'\n')
