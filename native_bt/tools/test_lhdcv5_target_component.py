"""Compile the single codec-worker LHDC component with the existing ESP32 toolchain."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/native_codecs/lhdcv5_target_component'
BUILD=ROOT/'_build/esp32'
RECIPE=ROOT/'esp32_probe/components/lhdcv5_decoder/CMakeLists.txt'
LEGACY=ROOT/'codecs/lhdcv5/CMakeLists.txt'
HARNESS=ROOT/'tests/codecs/component_recipe.cmake'
DECODER=ROOT.parent.parent/'LHDC-V5-Decoder-main/decoder'
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]
sys.path.insert(0,str(ROOT/'tools'))
from test_native_codec_target_components import argv_windows,run


def main():
    if os.name!='nt':raise RuntimeError('This check uses the existing Windows ESP32 build')
    OUT.mkdir(parents=True,exist_ok=True)
    cmake=Path(r'C:\Users\ngocv\.espressif\tools\cmake\3.30.2\bin\cmake.exe')
    manifest=OUT/'recipe.txt'
    run([str(cmake),'-DDBT_PROBE_LHDC_BUILD=ON','-DHOST_PYTHON='+sys.executable,
         '-DMANIFEST='+str(manifest),'-DRECIPE='+str(RECIPE),'-P',str(HARNESS)],cwd=OUT)
    fields={}
    for line in manifest.read_text().splitlines():
        key,value=line.split('|',1);fields.setdefault(key,[]).append(value)
    sources=[Path(p) for p in fields['SRCS']]
    assert len(sources)==8 and sources[0].name=='dbt_lhdcv5_codec.c'
    assert not any(p.name=='dbt_lhdcv5.c' for p in sources)
    assert set(fields['REQUIRES'])=={'native_bt','codec_core'}
    assert {'log','esp_common','esp_timer','heap','freertos'}<=set(fields['PRIV_REQUIRES'])
    assert {'-std=c11','-fwrapv','-O3'}<=set(fields['OPTION'])
    port=OUT/'port';receipt=json.loads((port/'lhdc_port.json').read_text())
    assert set(receipt)=={'lhdc_dec.c','lhdc_imdct.c','lhdc_entropy_dec.c','lhdc_sns_synth.c'}
    forbidden=subprocess.run([str(cmake),'-P',str(LEGACY)],capture_output=True,text=True)
    assert forbidden.returncode and 'lhdcv5_decoder as the sole LHDC V5' in forbidden.stderr
    bad=subprocess.run([sys.executable,str(ROOT/'codecs/lhdcv5/prepare_port.py'),'--source',str(port),
                        '--output',str(OUT/'port_rejected')],capture_output=True,text=True)
    assert bad.returncode and 'Unreviewed sibling source' in bad.stderr
    commands=json.loads((BUILD/'compile_commands.json').read_text())
    args=argv_windows(next(x['command'] for x in commands if x['file'].endswith('probe_audio.c')))
    args=args[:args.index('-o')];compiler=Path(args[0]);tools=compiler.parent
    includes=[Path(p) for p in fields['INCLUDE_DIRS']+fields['PRIV_INCLUDE_DIRS']]+[ROOT/'include',ROOT/'codecs/common']
    flags=['-I'+str(p) for p in includes]+[x for x in fields['OPTION'] if x!='PRIVATE']
    inputs=sources+[RECIPE,LEGACY,HARNESS,Path(__file__).resolve(),ROOT/'codecs/lhdcv5/prepare_port.py',
                    ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c']
    inputs+=list((ROOT/'codecs/lhdcv5').glob('*.h'))+list((ROOT/'include').glob('*.h'))+list((ROOT/'codecs/common').glob('*.h'))
    inputs+=list(DECODER.glob('*.h'))+[DECODER/p for p in receipt]
    initial={p.resolve():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    objects=[];warnings=[]
    for index,source in enumerate(sources):
        obj=OUT/(f'{index:02d}_{source.stem}.o')
        result=subprocess.run(args+flags+['-fstack-usage','-o',str(obj),'-c',str(source)],cwd=BUILD,capture_output=True,text=True)
        if result.returncode:raise RuntimeError(f'Failed compiling {source.name}\n{result.stderr[-8000:]}')
        clean=re.sub(r'\x1b\[[0-9;]*[A-Za-z]','',result.stderr)
        if 'warning:' in clean:warnings.append({'source':source.name,'diagnostics':[x.strip() for x in clean.splitlines() if 'warning:' in x]})
        objects.append(obj)
    for source in (ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c'):
        obj=OUT/(source.stem+'.o');run(args+flags+['-fstack-usage','-o',str(obj),'-c',str(source)],cwd=BUILD);objects.append(obj)
    linked=OUT/'lhdcv5_worker_codec_core.esp32.o'
    run([str(compiler),'-nostdlib','-Wl,-r',*[str(p) for p in objects],'-o',str(linked)])
    undefined=run([str(tools/'xtensa-esp32-elf-nm.exe'),'-u',str(linked)])
    for symbol in ('dbt_lhdcv5_codec','dbt_lhdc_mem_alloc','lhdc_dec_init','dbt_lhdc_entropy_ready','dbt_codec_allocate'):
        assert not re.search(r'\b'+symbol+r'\b',undefined),f'unresolved required worker symbol: {symbol}'
    nm=run([str(tools/'xtensa-esp32-elf-nm.exe'),'-S','--size-sort',str(linked)])
    writable=[]
    for line in nm.splitlines():
        m=re.match(r'^[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+([bBdD])\s+(\S+)',line)
        if m:writable.append({'name':m[3],'bytes':int(m[1],16),'kind':m[2]})
    assert not any(x['bytes']>=256 for x in writable),writable
    assert not any(x['name'] in ('s_fac_hist1','s_fac_hist2','s_pow2_mant_recip') for x in writable),writable
    assert sum(bool(re.search(r'\bdbt_lhdc_mem_alloc$',line)) for line in nm.splitlines())==1
    assert not re.search(r'\bdbt_lhdcv5_(?:open|close|decode)$',nm,re.M),'Raw-session adapter linked accidentally'
    table_dump=run([str(tools/'xtensa-esp32-elf-objdump.exe'),'-t',str(linked)])
    read_only={}
    for symbol in ('s_pow2_mant_recip','LHDC_WIN_480'):
        match=re.search(r'^\S+\s+\w+\s+O\s+(\.\S+)\s+\S+\s+'+symbol+r'$',table_dump,re.M)
        assert match and match[1].startswith('.rodata.'),f'{symbol} not in target read-only section'
        read_only[symbol]=match[1]
    stack=[]
    for path in (p.with_suffix('.su') for p in objects):
        for line in path.read_text(errors='replace').splitlines():
            cols=line.split('\t')
            if len(cols)>=3 and cols[1].isdigit():stack.append({'function':cols[0].split(':')[-1],'bytes':int(cols[1]),'classification':cols[2]})
    stack.sort(key=lambda x:x['bytes'],reverse=True)
    imdct=next(x for x in stack if x['function']=='lhdc_imdct_fast_480')
    assert imdct['bytes']<=512,imdct
    size=run([str(tools/'xtensa-esp32-elf-size.exe'),str(linked)]).splitlines()[-1].split()
    assert all(hashlib.sha256(p.read_bytes()).hexdigest()==digest for p,digest in initial.items()),'Source changed during target build'
    report={'passed':True,'scope':'LHDC worker ESP32 compile and relocatable codec-core link only; no firmware link or execution',
            'objects':len(objects),'legacy_component_rejected':True,'source_guard_rejected_modified_input':True,
            'text_bytes':int(size[0]),'data_bytes':int(size[1]),'bss_bytes':int(size[2]),'writable_static_symbols':writable,
            'selected_read_only_tables':read_only,
            'compiler_warnings':warnings,'largest_compiler_stack_frames':stack[:12], 'unresolved_external_symbols':undefined.splitlines(),
            'source_sha256':{str(p):v for p,v in sorted(initial.items())}}
    (OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=='__main__':
    subprocess.run(CHECK,check=True)
    try:main()
    finally:subprocess.run(CHECK,check=True)
