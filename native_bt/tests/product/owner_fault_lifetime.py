"""Host-only check of the real fault park and app_main failure exits."""
from pathlib import Path
import re
import shutil
import subprocess
import sys

root = Path(__file__).resolve().parents[2]
main = (root / 'esp32_probe/main/probe_main.c').read_text(encoding='utf-8')
match = re.search(r'static _Noreturn void owner_fault_park\(const char \*stage\)\s*\{.*?\n\}', main, re.S)
assert match, 'owner fault park not found'
body = main[main.index('void app_main(void) {'):]
assert not re.search(r'\breturn\s*;', body), 'app_main can return and IDF can delete the owner'
assert 'ESP_ERROR_CHECK(' not in body, 'startup must park instead of aborting'
assert not re.search(r'\b(?:vTaskDelete|esp_restart|abort)\s*\(', body)
for stage in ('runtime initialization', 'NVS initialization', 'product IO registration',
              'product settings load', 'GAP initialization', 'product IO bootstrap',
              'target codec check', 'Classic security/storage initialization',
              'controller initialization', 'controller callback registration',
              'controller enable', 'runtime start after controller enable'):
    assert f'owner_fault_park("{stage}' in body, stage
assert body.index('owner_fault_park("product IO bootstrap') < body.index('esp_bt_controller_enable(')
assert body.index('owner_fault_park("target codec check') < body.index('esp_bt_controller_enable(')

# Compile the production function itself with a simulated scheduler. Each
# suspension receives a worker callback, then an unexpected resume; the
# second and third suspensions demonstrate that the owner never returns.
prefix = r'''
#include <assert.h>
#include <setjmp.h>
#include <stddef.h>
#include <string.h>
static jmp_buf done;
static unsigned suspends, wakes, logs;
static int owner_alive=1;
static void fake_log(const char *stage){assert(!strcmp(stage,"boot timeout"));logs++;}
static void worker_wake(void){assert(owner_alive);wakes++;}
static void fake_suspend(void *task){assert(task==NULL);suspends++;worker_wake();if(suspends==3)longjmp(done,1);}
#define ESP_LOGE(tag,format,stage) fake_log(stage)
#define vTaskSuspend(task) fake_suspend(task)
'''
suffix = r'''
int main(void){
    if(!setjmp(done))owner_fault_park("boot timeout");
    assert(owner_alive && logs==1 && suspends==3 && wakes==3);
    return 0;
}
'''
out = root / '_build/product'
out.mkdir(parents=True, exist_ok=True)
source = out / 'owner_fault_lifetime.c'
exe = out / ('owner_fault_lifetime.exe' if sys.platform == 'win32' else 'owner_fault_lifetime')
source.write_text(prefix + match.group(0) + suffix, encoding='utf-8')
cc = shutil.which('clang') or (r'C:\Program Files\LLVM\bin\clang.exe' if sys.platform == 'win32' else shutil.which('cc'))
subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror', str(source), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
print('PASS owner fault park, callbacks across resumes, and startup failure exits')
