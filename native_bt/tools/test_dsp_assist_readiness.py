"""Compile actual idle-decoder DSP admission; no SDK, phone or radio access."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parents[1]
out = root / '_build/dsp_assist_readiness'
out.mkdir(parents=True, exist_ok=True)
receipt = out / 'results.json'
receipt.write_text(json.dumps({'passed': False, 'hardware_accessed': False}) + '\n')
source = root / 'esp32_probe/main/probe_audio.c'
source_text = source.read_text()
begin = source_text.index('bool probe_audio_dsp_assist_ready(void){')
end = source_text.index('\nstatic TickType_t decoder_wait_ticks', begin)
actual = source_text[begin:end]
env = os.environ.copy()
if os.name == 'nt':
    vc = Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    values = subprocess.check_output(f'call "{vc}" >nul && set', shell=True, text=True)
    env.update(dict(line.split('=', 1) for line in values.splitlines() if '=' in line and not line.startswith('=')))
cc = shutil.which('clang', path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache = shutil.which('ccache', path=env.get('PATH'))
if not cache:
    matches = sorted((Path.home() / '.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if matches:
        cache = str(matches[-1])
assert cache, 'ccache required'
env.pop('CCACHE_DISABLE', None)
fixture = out / 'readiness.c'
fixture.write_text('''#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stdio.h>
static _Atomic unsigned decoder_wait_kind,pcm_codec;
''' + actual + '''
int main(void){
    const unsigned reasons[]={0,1,2,3,4,5,UINT32_MAX,UINT32_MAX-1};
    unsigned count=0;
    for(unsigned codec=0;codec<256;++codec){
        for(unsigned i=0;i<sizeof reasons/sizeof reasons[0];++i){
            atomic_store(&pcm_codec,codec);
            atomic_store(&decoder_wait_kind,reasons[i]);
            bool expected=false;
#ifdef DBT_PROBE_LHDC_BUILD
            expected=codec==9 && (reasons[i]==0 || reasons[i]==3);
#endif
            if(probe_audio_dsp_assist_ready()!=expected){
                fprintf(stderr,"codec=%u reason=%u\\n",codec,reasons[i]);return 1;
            }
            ++count;
        }
    }
    printf("PASS actual decoder-idle admission: %u cases\\n",count);return 0;
}
''')
resource = Path(subprocess.check_output([cc, '-print-resource-dir'], text=True).strip())
if os.name == 'nt':
    env['PATH'] = str(resource / 'lib/windows') + os.pathsep + env.get('PATH', '')
commands, runs = [], []
for enabled in (False, True):
    exe = out / ('enabled' if enabled else 'disabled')
    if os.name == 'nt':
        exe = exe.with_suffix('.exe')
    command = [cache, cc, '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
               '-fsanitize=address,undefined', '-fno-sanitize-recover=all']
    if os.name == 'nt':
        command += ['-fms-runtime-lib=dll', '-D_DLL', '-Wl,/NODEFAULTLIB:libcmt']
    if enabled:
        command += ['-DDBT_PROBE_LHDC_BUILD=1']
    command += [str(fixture), '-o', str(exe)]
    subprocess.run(command, env=env, check=True)
    result = subprocess.run([str(exe)], env=env, check=True, capture_output=True, text=True, timeout=15)
    print(result.stdout.strip())
    commands.append(command)
    runs.append({'lhdc_enabled': enabled, 'cases': 2048, 'stdout': result.stdout})
assert source.read_text() == source_text, 'source changed during test'
receipt.write_text(json.dumps({'passed': True, 'hardware_accessed': False,
    'source_sha256': hashlib.sha256(source_text.encode()).hexdigest(),
    'sanitizers': ['address', 'undefined'], 'commands': commands, 'runs': runs,
    'scope': 'actual admission function, all codec identifiers and idle/control/loan/handoff/awake reasons; not ESP32 scheduling qualification'}, indent=2) + '\n')
