"""Compile the opt-in LHDC/FIR performance helper with the existing Xtensa toolchain."""
import json,os,pathlib,re,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;out=root/'_build/avrcp/lhdc-perf';out.mkdir(parents=True,exist_ok=True);port=out/'target_port';dec=ws.parent/'LHDC-V5-Decoder-main/decoder';build=root/'_build/esp32';check=[sys.executable,str(root/'tools/check_workspace.py')]
sys.path.insert(0,str(root/'tools'));from test_native_codec_target_components import argv_windows
subprocess.run(check,check=True)
try:
    if os.name!='nt':raise SystemExit('requires existing Windows Xtensa toolchain')
    subprocess.run([sys.executable,str(root/'codecs/lhdcv5/prepare_port.py'),'--source',str(dec),'--output',str(port)],check=True,stdout=subprocess.DEVNULL)
    data=json.loads((build/'compile_commands.json').read_text());row=next(x for x in data if x['file'].replace('\\','/').endswith('/probe_audio.c'));base=argv_windows(row['command']);base=base[:base.index('-o')];cc=pathlib.Path(base[0]);tools=cc.parent
    incs=[root/'include',root/'codecs/common',root/'codecs/lhdcv5',root/'tests/codecs',dec]
    flags=['-I'+str(p) for p in incs]+['-DDBT_LHDC_STAGE_PROFILE=1','-fstack-usage','-O3']
    objects=[]
    for src in (root/'tests/codecs/lhdcv5_perf_helper.c',port/'lhdc_dec.c',root/'src/dbt_resampler.c'):
        obj=out/(src.stem+'-stage.o');subprocess.run(base+flags+['-o',str(obj),'-c',str(src)],cwd=build,check=True);objects.append(obj)
    linked=out/'lhdc-perf-stage-linked.o';subprocess.run([str(cc),'-nostdlib','-Wl,-r']+[str(x) for x in objects]+['-o',str(linked)],check=True)
    symbols=subprocess.check_output([str(tools/'xtensa-esp32-elf-nm.exe'),'-u',str(linked)],text=True)
    assert not re.search(r'\bdbt_lhdc_stage_record\b',symbols),symbols
    stack=[]
    for obj in objects:
        for line in obj.with_suffix('.su').read_text(errors='replace').splitlines():
            cols=line.split('\t')
            if len(cols)>2 and cols[1].isdigit() and ('dbt_lhdc_perf_' in cols[0] or 'lhdc_dec_decode_channel' in cols[0]):stack.append(line)
    print('PASS Xtensa opt-in stage helper and integer FIR relocatable link')
    print('stack_usage',stack)
finally:subprocess.run(check,check=True)
