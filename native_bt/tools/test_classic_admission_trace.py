"""Execute production admission callback with both policies and a full log queue."""
import os,pathlib,re,shutil,subprocess,json,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/classic_admission_trace';OUT.mkdir(exist_ok=True)
source=ROOT/'esp32_probe/main/probe_security.c';text=source.read_text(encoding='utf-8')
match=re.search(r'bool probe_security_accept\(.*?\n\}',text,re.S);assert match
main=OUT/'main.c';main.write_text('''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
static bool frozen,known,window,full;static unsigned logs;
static uint8_t allowed_peer[6]={1,2,3,4,5,6};
typedef struct {struct {uint8_t address[6];bool accepted,known,window;} admission;uint8_t event,local_record;} log_item_t;
static log_item_t captured;static void *log_queue=(void *)1;
static struct {uint32_t now_ms;} storage;static void *runtime_unused;
#ifdef DBT_PROBE_PRODUCT_SECURITY
#define runtime (&storage)
static bool probe_admission_frozen(void){return frozen;}
static bool probe_admission_window(uint32_t n){(void)n;return window;}
#else
static bool same(const uint8_t *a,const uint8_t *b){return !memcmp(a,b,6);}
#endif
static int cache_index(const uint8_t *a){(void)a;return known?0:-1;}
static void wipe(void *p,unsigned n){memset(p,0,n);}
static void copy(void *a,const void *b,unsigned n){memcpy(a,b,n);}
static int xQueueSend(void *q,const void *p,unsigned timeout){assert(q==log_queue && !timeout);++logs;captured=*(const log_item_t *)p;return !full;}
'''+match.group(0)+'''
int main(void){
 (void)storage;(void)runtime_unused;unsigned count=0;
 for(unsigned f=0;f<2;++f)for(unsigned k=0;k<2;++k)for(unsigned w=0;w<2;++w)for(unsigned q=0;q<2;++q){
  frozen=f;known=k;window=w;full=q;uint8_t peer[6]={1,2,3,4,5,6},cod[3]={0};if(!w)peer[0]=7;
#ifdef DBT_PROBE_PRODUCT_SECURITY
  bool expected=!frozen && (known || window);
#else
  bool expected=w;
#endif
  assert(probe_security_accept(0,peer,cod)==expected);
  assert(captured.local_record==1 && captured.admission.accepted==expected && captured.admission.known==known && !memcmp(captured.admission.address,peer,6));++count;
 }
 assert(logs==count && count==16);return 0;
}
''')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat');data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'clang')
for product in (False,True):
    exe=OUT/('product.exe' if product else 'peer.exe')
    args=[cc,'-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-Wno-unused-variable','-fsanitize=undefined','-fno-sanitize-recover=all',str(main),'-o',str(exe)]
    if product:args+=['-DDBT_PROBE_PRODUCT_SECURITY=1']
    subprocess.run(args,check=True,env=env);subprocess.run([str(exe)],check=True,env=env)
(OUT/'results.json').write_text(json.dumps({'passed':True,'cases':32,'scope':'actual callback with product-consent and exact-peer policies, optional log admission/full queue; host stubs, no hardware authentication proof','source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'hardware_accessed':False},indent=2)+'\n')
print('PASS classic admission logging preserves product/exact-peer decisions with full optional log queue')
