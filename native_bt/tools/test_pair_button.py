"""Exercise the actual local-consent GPIO state machine; no physical GPIO proof."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/pair_button';OUT.mkdir(exist_ok=True)
STUB=OUT/'stubs';(STUB/'freertos').mkdir(parents=True,exist_ok=True);(STUB/'driver').mkdir(exist_ok=True)
(STUB/'freertos/FreeRTOS.h').write_text('''#include <stddef.h>
typedef void *TaskHandle_t;typedef int BaseType_t;
#define pdFALSE 0
#define portYIELD_FROM_ISR() fake_yield()
void fake_yield(void);
''')
(STUB/'freertos/task.h').write_text('TaskHandle_t xTaskGetCurrentTaskHandle(void);\nvoid vTaskNotifyGiveFromISR(TaskHandle_t,BaseType_t *);\n')
(STUB/'esp_attr.h').write_text('#define IRAM_ATTR\n')
(STUB/'driver/gpio.h').write_text('''#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 1
#define ESP_INTR_FLAG_IRAM 1
#define GPIO_NUM_0 0
#define GPIO_MODE_INPUT 1
#define GPIO_PULLUP_ENABLE 1
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_ANYEDGE 3
typedef struct {uint64_t pin_bit_mask;int mode,pull_up_en,pull_down_en,intr_type;} gpio_config_t;
int gpio_config(const gpio_config_t *);
int gpio_install_isr_service(int);
int gpio_isr_handler_add(int,void (*)(void *),void *);
int gpio_get_level(int);
''')
main=OUT/'main.c';main.write_text('''#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include "probe_pair_button.h"
#include "driver/gpio.h"
static int scenario,level=1;static unsigned configs,wakes,yields,reads;
static TaskHandle_t task=(void *)1;static void (*interrupt)(void *);static void *argument;
TaskHandle_t xTaskGetCurrentTaskHandle(void){return task;}
void fake_yield(void){++yields;}
void vTaskNotifyGiveFromISR(TaskHandle_t t,BaseType_t *w){assert(t==(void *)1);++wakes;*w=1;}
int gpio_config(const gpio_config_t *c){++configs;assert(c->pin_bit_mask==1 && c->mode==GPIO_MODE_INPUT && c->pull_up_en==1 && !c->pull_down_en && c->intr_type==GPIO_INTR_ANYEDGE);return scenario==10?-1:0;}
int gpio_install_isr_service(int flags){assert(flags==ESP_INTR_FLAG_IRAM);return scenario==11?-1:scenario==13?ESP_ERR_INVALID_STATE:0;}
int gpio_isr_handler_add(int pin,void (*f)(void *),void *a){assert(pin==0);interrupt=f;argument=a;return scenario==12?-1:0;}
int gpio_get_level(int pin){assert(pin==0);++reads;return level;}
static void edge(int v){level=v;interrupt(argument);assert(wakes==yields);}
static void idle(uint32_t t){assert(!probe_pair_button_poll(t));}
int main(int argc,char **argv){
 scenario=argc>1?atoi(argv[1]):0;
 assert(!probe_pair_button_poll(0) && probe_pair_button_wait_ms(0)==UINT32_MAX);
 assert(!probe_pair_button_init(NULL) && !probe_pair_button_init((void *)2) && !configs);
 if(scenario>=10 && scenario<=12){assert(!probe_pair_button_init(task));idle(4000);assert(probe_pair_button_wait_ms(0)==UINT32_MAX);return 0;}
 if(scenario==1)level=0;
 assert(probe_pair_button_init(task) && probe_pair_button_init(task) && configs==1);
 task=(void *)2;idle(0);assert(probe_pair_button_wait_ms(0)==UINT32_MAX && !probe_pair_button_init(task));task=(void *)1;
 assert(!reads && probe_pair_button_wait_ms(0)==0);
 uint32_t base=scenario==4?UINT32_MAX-1000u:0;
 idle(base);assert(probe_pair_button_wait_ms(base)==30);idle(base+29);assert(!reads);idle(base+30);
 if(scenario==1){idle(base+4000);assert(probe_pair_button_wait_ms(base+4000)==UINT32_MAX);edge(1);idle(base+4100);idle(base+4130);base+=4200;}
 assert(probe_pair_button_wait_ms(base+30)==UINT32_MAX);
 unsigned previous=reads;idle(base+31);assert(reads==previous);
 edge(0);idle(base+100);idle(base+129);idle(base+130);assert(probe_pair_button_wait_ms(base+130)==3000);
 if(scenario==2){edge(1);idle(base+500);idle(base+530);idle(base+5000);assert(probe_pair_button_wait_ms(base+5000)==UINT32_MAX);return 0;}
 if(scenario==3){edge(1);edge(0);idle(base+600);idle(base+630);idle(base+3130);assert(probe_pair_button_wait_ms(base+3130)==500);assert(probe_pair_button_poll(base+3630));idle(base+4000);return 0;}
 if(scenario==5){level=1;idle(base+3130);idle(base+3160);assert(probe_pair_button_wait_ms(base+3160)==UINT32_MAX);return 0;}
 if(scenario==6){edge(1);edge(0);idle(base+3130);idle(base+3160);idle(base+6159);assert(probe_pair_button_poll(base+6160));return 0;}
 idle(base+3129);assert(probe_pair_button_poll(base+3130));idle(base+3131);idle(base+10000);assert(probe_pair_button_wait_ms(base+10000)==UINT32_MAX);
 edge(0);idle(base+10010);idle(base+10040);idle(base+14000);assert(probe_pair_button_wait_ms(base+14000)==UINT32_MAX);
 edge(1);idle(base+15000);idle(base+15030);edge(0);idle(base+15100);idle(base+15130);assert(probe_pair_button_poll(base+18130));return 0;
}
''')
env=os.environ.copy()
if os.name=='nt':
 vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
 data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
 env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'clang')
exe=OUT/('button.exe' if os.name=='nt' else 'button')
args=[cc,'-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all',f'-I{STUB}',f'-I{ROOT/"esp32_probe/main"}',str(main),str(ROOT/'esp32_probe/main/probe_pair_button.c'),'-o',str(exe)]
if os.name=='nt':
 args+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
 resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
 shutil.copy2(resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll',OUT)
subprocess.run(args,check=True,env=env)
for scenario in (*range(7),10,11,12,13):subprocess.run([str(exe),str(scenario)],check=True,env=env)
(OUT/'results.json').write_text(json.dumps({'passed':True,'scenarios':11,'sanitizers':['address','undefined'],'source_sha256':hashlib.sha256((ROOT/'esp32_probe/main/probe_pair_button.c').read_bytes()).hexdigest(),'scope':'actual production state machine with GPIO/task/IRQ stubs; boot-held, debounce, short/coalesced/repeated presses, wrap, release guard, owner and init failures; not physical consent or RF proof','hardware_accessed':False},indent=2)+'\n')
print('PASS local pairing button: 11 ASan/UBSan timing, ownership and failure scenarios')
