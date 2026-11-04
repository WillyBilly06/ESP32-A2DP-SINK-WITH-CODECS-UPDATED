"""Compile the actual AAC/aptX component recipes with the existing ESP32 toolchain.

This is a relocatable codec-core link, not a firmware build or execution test.
"""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/native_codecs/target_components'
BUILD=ROOT/'_build/esp32'
HARNESS=ROOT/'tests/codecs/component_recipe.cmake'
RECIPES={name:ROOT/'esp32_probe/components'/name/'CMakeLists.txt' for name in ('aac_decoder','aptx_decoder')}
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]


def run(args, **kwargs):
    result=subprocess.run(args,capture_output=True,text=True,**kwargs)
    if result.returncode:
        raise RuntimeError(f"command failed {args[:3]}\n{result.stdout[-4000:]}\n{result.stderr[-6000:]}")
    return result.stdout


def argv_windows(command):
    argc=ctypes.c_int()
    fn=ctypes.windll.shell32.CommandLineToArgvW
    fn.argtypes=[ctypes.c_wchar_p,ctypes.POINTER(ctypes.c_int)]
    fn.restype=ctypes.POINTER(ctypes.c_wchar_p)
    ptr=fn(command,ctypes.byref(argc))
    if not ptr:raise RuntimeError('Cannot parse existing target compiler command')
    try:return list(ptr[:argc.value])
    finally:ctypes.windll.kernel32.LocalFree(ptr)


def main():
    if os.name!='nt':raise RuntimeError('This target-recipe check uses the existing Windows ESP-IDF build')
    OUT.mkdir(parents=True,exist_ok=True)
    cmake=Path(r'C:\Users\ngocv\.espressif\tools\cmake\3.30.2\bin\cmake.exe')
    commands=json.loads((BUILD/'compile_commands.json').read_text())
    base=next(item['command'] for item in commands if item['file'].endswith('probe_audio.c'))
    args=argv_windows(base);args=args[:args.index('-o')]
    compiler=Path(args[0]);tools=compiler.parent
    inputs=[HARNESS,Path(__file__).resolve(),*RECIPES.values(),ROOT/'codecs/aac/prepare_port.py',
            ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c']
    inputs += list((ROOT/'include').glob('*.h'))+list((ROOT/'codecs/common').glob('*.h'))
    initial={}
    def record(paths):
        for path in paths:
            path=path.resolve();initial.setdefault(path,hashlib.sha256(path.read_bytes()).hexdigest())
    record(inputs)
    all_objects=[];recipe_summary={};warnings=[]
    for name,recipe in RECIPES.items():
        folder=OUT/name;folder.mkdir(parents=True,exist_ok=True);manifest=folder/'recipe.txt'
        run([str(cmake),'-DHOST_PYTHON='+sys.executable,'-DMANIFEST='+str(manifest),
             '-DRECIPE='+str(recipe),'-P',str(HARNESS)],cwd=folder)
        fields={};source_options={}
        for line in manifest.read_text().splitlines():
            field,value=line.split('|',1)
            if field=='SOURCE_OPTION':
                source,option=value.rsplit('|',1);source_options.setdefault(Path(source),[]).append(option)
            else:fields.setdefault(field,[]).append(value)
        sources=[Path(x) for x in fields['SRCS']]
        assert set(fields['REQUIRES'])=={'native_bt','codec_core'}
        assert {'-std=c11','-fwrapv'}<=set(fields['OPTION'])
        assert all('btstack' not in str(p).lower() for p in sources)
        if name=='aac_decoder':
            assert len(sources)==19 and sources[2].name=='aacdec_guarded.c'
            assert not any(p.name in ('aacdec.c','aac_decoder_helix.c') for p in sources)
            assert {'ESP32=1','USE_DEFAULT_STDLIB','AAC_MAX_NCHANS=2'}<=set(fields['DEFINITION'])
            guard=json.loads((folder/'port/aac_port.json').read_text())
            assert guard['guard_sites']==2 and guard['source_sha256']=='f3260aeca52c91675761b8dccf2ec83cf42f8cc07ef47da10676ec1aa66ea5d6'
        else:
            assert [p.name for p in sources]==['dbt_aptx.c','dbt_aptx_backend.c']
        includes=[Path(x) for x in fields.get('INCLUDE_DIRS',[])+fields.get('PRIV_INCLUDE_DIRS',[])]
        includes += [ROOT/'include',ROOT/'codecs/common']
        flags=['-I'+str(p) for p in includes]+[v for v in fields['OPTION'] if v!='PRIVATE']
        flags += ['-D'+v for v in fields.get('DEFINITION',[]) if v!='PRIVATE']
        sdk_header_dir=ROOT.parent/'esp-idf/components/bt/host/bluedroid/external'/('libaac-lc/libhelix_aac' if name=='aac_decoder' else 'libfreeaptx/main')
        dependencies=sources+list(sdk_header_dir.rglob('*.h'))+list((ROOT/'codecs'/('aac' if name=='aac_decoder' else 'aptx')).glob('*.h'))
        dependencies.append(sdk_header_dir/('aacdec.c' if name=='aac_decoder' else 'freeaptx.c'))
        record(dependencies)
        if name=='aac_decoder':
            assert initial[(sdk_header_dir/'aacdec.c').resolve()]=='f3260aeca52c91675761b8dccf2ec83cf42f8cc07ef47da10676ec1aa66ea5d6'
            rejection_output=folder/'port_rejected'
            old=rejection_output/'aacdec_guarded.c'
            previous=hashlib.sha256(old.read_bytes()).hexdigest() if old.exists() else None
            rejection=subprocess.run([sys.executable,str(ROOT/'codecs/aac/prepare_port.py'),
                '--source',str(sdk_header_dir/'aactabs.c'),'--output',str(rejection_output)],capture_output=True,text=True)
            assert rejection.returncode and 'Unreviewed Helix AAC source' in rejection.stderr
            assert (hashlib.sha256(old.read_bytes()).hexdigest() if old.exists() else None)==previous
            sbr=subprocess.run(args+flags+['-Wno-error','-DHELIX_FEATURE_AUDIO_CODEC_AAC_SBR',
                '-o',str(folder/'rejected_sbr.o'),'-c',str(sources[2])],cwd=BUILD,capture_output=True,text=True)
            assert sbr.returncode and 'Native AAC-LC requires SBR disabled' in sbr.stderr
        objects=[]
        for index,source in enumerate(sources):
            obj=folder/(f'{index:02d}_{source.stem}.o')
            source_flags=source_options.get(source,[])
            expected=['-Wno-error=misleading-indentation'] if name=='aptx_decoder' and index==0 else []
            assert source_flags==expected,(source,source_flags)
            command=args+flags+source_flags+['-fstack-usage','-o',str(obj),'-c',str(source)]
            compiled=subprocess.run(command,cwd=BUILD,capture_output=True,text=True)
            if compiled.returncode:raise RuntimeError(f'Failed compiling {source.name}\n{compiled.stderr[-6000:]}')
            if 'warning:' in compiled.stderr:
                clean=re.sub(r'\x1b\[[0-9;]*[A-Za-z]','',compiled.stderr)
                warnings.append({'source':source.name,'count':clean.count('warning:'),
                                 'first':next(line.strip() for line in clean.splitlines() if 'warning:' in line)})
            objects.append(obj)
        all_objects+=objects
        recipe_summary[name]={'sources':len(sources),'requires':fields['REQUIRES'],'definitions':fields.get('DEFINITION',[])}
    includes=['-I'+str(ROOT/'include'),'-I'+str(ROOT/'codecs/common')]
    for source in (ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c'):
        obj=OUT/(source.stem+'.o');run(args+includes+['-std=c11','-fwrapv','-fstack-usage','-o',str(obj),'-c',str(source)],cwd=BUILD)
        all_objects.append(obj)
    linked=OUT/'aac_aptx_codec_core.esp32.o'
    run([str(compiler),'-nostdlib','-Wl,-r',*[str(p) for p in all_objects],'-o',str(linked)])
    size_output=run([str(tools/'xtensa-esp32-elf-size.exe'),str(linked)]).splitlines()[-1].split()
    text_size,data_size,bss_size=map(int,size_output[:3])
    assert data_size==0 and bss_size==0,(data_size,bss_size)
    undefined=run([str(tools/'xtensa-esp32-elf-nm.exe'),'-u',str(linked)])
    for symbol in ('dbt_helix_channel_guard','AACInitDecoderPre','aptx_decode32','dbt_codec_allocate','dbt_avdtp_caps_validate'):
        assert not re.search(r'\b'+symbol+r'\b',undefined),f'unresolved required codec symbol: {symbol}'
    symbols=run([str(tools/'xtensa-esp32-elf-nm.exe'),'-S','--size-sort',str(linked)])
    writable=[]
    for line in symbols.splitlines():
        match=re.match(r'^[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+([bBdD])\s+(\S+)',line)
        if match:writable.append({'name':match[3],'bytes':int(match[1],16),'kind':match[2]})
    if any(x['bytes']>=256 for x in writable):raise RuntimeError('Large mutable static codec state: '+repr(writable))
    table_dump=run([str(tools/'xtensa-esp32-elf-objdump.exe'),'-t',str(linked)])
    flash_tables={}
    for symbol in ('raac_cos4sin4tab','raac_sampRateTab','all_tables','quantize_intervals_LF','hd_quantize_intervals_LF'):
        match=re.search(r'^\S+\s+\w+\s+O\s+(\.\S+)\s+\S+\s+'+symbol+r'$',table_dump,re.M)
        assert match and match[1].startswith('.rodata.'),f'{symbol} not in read-only target section'
        flash_tables[symbol]=match[1]
    stack=[]
    for path in (obj.with_suffix('.su') for obj in all_objects):
        for line in path.read_text(errors='replace').splitlines():
            fields=line.split('\t')
            if len(fields)>=3 and fields[1].isdigit():stack.append({'function':fields[0].split(':')[-1],'bytes':int(fields[1]),'classification':fields[2]})
    stack.sort(key=lambda x:x['bytes'],reverse=True)
    assert all(hashlib.sha256(p.read_bytes()).hexdigest()==digest for p,digest in initial.items()),'Source changed during build'
    report={'passed':True,'scope':'ESP32 compile and relocatable link of the actual component recipe plus owned codec core; no firmware link or execution',
            'recipes':recipe_summary,'objects':len(all_objects),'unreviewed_aac_and_sbr_rejected':True,
            'relocatable_text_bytes':text_size,'relocatable_data_bytes':data_size,'relocatable_bss_bytes':bss_size,
            'writable_static_symbols':writable,'selected_read_only_tables':flash_tables,'compiler_warnings':warnings,
            'largest_compiler_stack_frames':stack[:12],
            'unresolved_external_symbols':undefined.splitlines(),
            'source_sha256':{p.relative_to(ROOT.parent).as_posix():initial[p] for p in sorted(initial)}}
    (OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=='__main__':
    subprocess.run(CHECK,check=True)
    try:main()
    finally:subprocess.run(CHECK,check=True)
