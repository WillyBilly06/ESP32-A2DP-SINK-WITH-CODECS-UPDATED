"""Compile the actual experimental LDAC recipe for ESP32, without firmware."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/native_codecs/ldac_target_component'
BUILD=ROOT/'_build/esp32'
RECIPE=ROOT/'esp32_probe/components/ldac_decoder/CMakeLists.txt'
HARNESS=ROOT/'tests/codecs/component_recipe.cmake'
SDK=ROOT.parent/'esp-idf/components/bt/host/bluedroid/external/libldac-dec'
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]
sys.path.insert(0,str(ROOT/'tools'))
from test_native_codec_target_components import argv_windows,run


def main():
    if os.name!='nt':raise RuntimeError('This check uses the existing Windows ESP32 build')
    OUT.mkdir(parents=True,exist_ok=True)
    cmake=Path(r'C:\Users\ngocv\.espressif\tools\cmake\3.30.2\bin\cmake.exe')
    manifest=OUT/'recipe.txt'
    run([str(cmake),'-DHOST_PYTHON='+sys.executable,'-DMANIFEST='+str(manifest),
         '-DRECIPE='+str(RECIPE),'-P',str(HARNESS)],cwd=OUT)
    fields={}
    for line in manifest.read_text().splitlines():
        key,value=line.split('|',1);fields.setdefault(key,[]).append(value)
    sources=[Path(x) for x in fields['SRCS']]
    assert [p.name for p in sources]==['dbt_ldac.c','dbt_ldac_decoder.c','dbt_ldac_bt.c','dbt_ldac_lib.c']
    assert set(fields['REQUIRES'])=={'native_bt','codec_core'} and {'-std=c11','-fwrapv'}<=set(fields['OPTION'])
    generated=OUT/'port';receipt=json.loads((generated/'ldac_port.json').read_text())
    assert set(receipt)=={'ldac.h','ldacBT_internal.h','ldacBT_internal.o.c','decode_ldac.o.c','unpack_ldac.o.c',
                          'tables_ldac.o.c','tables_sigproc_ldac.o.c'}
    rejected=OUT/'port_rejected';existing=rejected/'ldac.h'
    before=hashlib.sha256(existing.read_bytes()).hexdigest() if existing.exists() else None
    check=subprocess.run([sys.executable,str(ROOT/'codecs/ldac/prepare_port.py'),'--source',str(generated),
                          '--output',str(rejected)],capture_output=True,text=True)
    assert check.returncode and 'Unreviewed SDK LDAC source' in check.stderr
    assert (hashlib.sha256(existing.read_bytes()).hexdigest() if existing.exists() else None)==before
    commands=json.loads((BUILD/'compile_commands.json').read_text())
    args=argv_windows(next(x['command'] for x in commands if x['file'].endswith('probe_audio.c')))
    args=args[:args.index('-o')];compiler=Path(args[0]);tools=compiler.parent
    macros=run([str(compiler),'-dM','-E','-x','c','-'],input='')
    target_char_unsigned='__CHAR_UNSIGNED__' in macros
    includes=[Path(p) for p in fields['INCLUDE_DIRS']+fields['PRIV_INCLUDE_DIRS']]+[ROOT/'include',ROOT/'codecs/common']
    flags=['-I'+str(p) for p in includes]+[x for x in fields['OPTION'] if x!='PRIVATE']
    inputs=sources+[RECIPE,HARNESS,Path(__file__).resolve(),ROOT/'codecs/ldac/prepare_port.py',
                    ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c']
    inputs+=list((ROOT/'codecs/ldac').glob('*.h'))+list((ROOT/'include').glob('*.h'))+list((ROOT/'codecs/common').glob('*.h'))
    inputs+=list((SDK/'src').glob('*.o.c'))+list((SDK/'src').glob('*.h'))+list((SDK/'inc').glob('*.h'))+list(generated.glob('*.h'))+list(generated.glob('*.c'))
    initial={p.resolve():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    objects=[];warnings=[];include_trace=''
    for index,source in enumerate(sources):
        obj=OUT/(f'{index:02d}_{source.stem}.o')
        command=args+flags+['-H','-fstack-usage','-o',str(obj),'-c',str(source)]
        compiled=subprocess.run(command,cwd=BUILD,capture_output=True,text=True)
        if compiled.returncode:raise RuntimeError(f'Failed compiling {source.name}\n{compiled.stderr[-8000:]}')
        clean=re.sub(r'\x1b\[[0-9;]*[A-Za-z]','',compiled.stderr)
        if 'warning:' in clean:warnings.append({'source':source.name,'count':clean.count('warning:'),
            'diagnostics':[line.strip() for line in clean.splitlines() if 'warning:' in line]})
        include_trace+=clean.replace('\\','/').lower();objects.append(obj)
    for source in (ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c'):
        obj=OUT/(source.stem+'.o')
        run(args+flags+['-fstack-usage','-o',str(obj),'-c',str(source)],cwd=BUILD);objects.append(obj)
    for name in receipt:
        expected=(generated/name).as_posix().lower()
        assert expected in include_trace, f'Generated source/header not selected: {name}'
    linked=OUT/'ldac_codec_core.esp32.o'
    run([str(compiler),'-nostdlib','-Wl,-r',*[str(o) for o in objects],'-o',str(linked)])
    undefined=run([str(tools/'xtensa-esp32-elf-nm.exe'),'-u',str(linked)])
    for symbol in ('dbt_ldac_codec','dbt_ldac_mem_alloc','ldacBT_decode','dbt_ldac_bind_pcm','dbt_codec_allocate'):
        assert not re.search(r'\b'+symbol+r'\b',undefined),f'unresolved required codec symbol: {symbol}'
    size=run([str(tools/'xtensa-esp32-elf-size.exe'),str(linked)]).splitlines()[-1].split()
    text_size,data_size,bss_size=map(int,size[:3])
    assert data_size==0 and bss_size<=16,(data_size,bss_size)
    symbols=run([str(tools/'xtensa-esp32-elf-nm.exe'),'-S','--size-sort',str(linked)])
    writable=[]
    for line in symbols.splitlines():
        match=re.match(r'^[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+([bBdD])\s+(\S+)',line)
        if match:writable.append({'name':match[3],'bytes':int(match[1],16),'kind':match[2]})
    if any(x['bytes']>=256 for x in writable):raise RuntimeError('Large mutable static codec state: '+repr(writable))
    table_dump=run([str(tools/'xtensa-esp32-elf-objdump.exe'),'-t',str(linked)])
    read_only={}
    for symbol in ('gaa_nlnn_shift_ldac','ga_hcdec_sf0_ldac','gaa_bwin_ldac','sa_wsin_1fs_ldac'):
        match=re.search(r'^\S+\s+\w+\s+O\s+(\.\S+)\s+\S+\s+'+symbol+r'$',table_dump,re.M)
        assert match and match[1].startswith('.rodata.'),f'{symbol} not in target read-only section'
        read_only[symbol]=match[1]
    stack=[]
    for path in (obj.with_suffix('.su') for obj in objects):
        for line in path.read_text(errors='replace').splitlines():
            cols=line.split('\t')
            if len(cols)>=3 and cols[1].isdigit():stack.append({'function':cols[0].split(':')[-1],'bytes':int(cols[1]),'classification':cols[2]})
    stack.sort(key=lambda x:x['bytes'],reverse=True)
    assert not any('-Wtype-limits' in message for group in warnings for message in group['diagnostics'])
    assert all(hashlib.sha256(p.read_bytes()).hexdigest()==sha for p,sha in initial.items()),'Source changed during test'
    report={'passed':True,'scope':'LDAC component ESP32 compile and relocatable codec-core link only; no firmware link or execution',
            'objects':len(objects),'source_guard_rejected_modified_input':True,'generated_includes_verified':list(receipt),
            'relocatable_text_bytes':text_size,'relocatable_data_bytes':data_size,'relocatable_bss_bytes':bss_size,
            'target_plain_char_unsigned':target_char_unsigned,
            'writable_static_symbols':writable,'selected_read_only_tables':read_only,'compiler_warnings':warnings,
            'largest_compiler_stack_frames':stack[:12],
            'unresolved_external_symbols':undefined.splitlines(),
            'source_sha256':{p.relative_to(ROOT.parent).as_posix():initial[p] for p in sorted(initial)}}
    (OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=='__main__':
    subprocess.run(CHECK,check=True)
    try:main()
    finally:subprocess.run(CHECK,check=True)
