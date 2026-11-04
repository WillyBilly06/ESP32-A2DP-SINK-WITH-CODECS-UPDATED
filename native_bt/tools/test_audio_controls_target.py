"""Compile the real ESP32 publication code and inspect non-spinning mask/CAS."""
import hashlib,json,os,pathlib,re,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;build=root/'_build/esp32';out=root/'_build/avrcp';out.mkdir(parents=True,exist_ok=True)
check=[sys.executable,str(root/'tools/check_workspace.py')];subprocess.run(check,check=True)
try:
    if os.name!='nt':raise SystemExit('requires the existing Windows ESP32 toolchain')
    sys.path.insert(0,str(root/'tools'));from test_native_codec_target_components import argv_windows
    db=json.loads((build/'compile_commands.json').read_text());row=next(x for x in db if x['file'].replace('\\','/').endswith('/dbt_audio.c'))
    paths=[root/'src/dbt_audio.c',root/'include/dbt_audio.h',root/'include/dbt_resampler.h',root/'include/dbt_queue.h',ws/'bt_audio_sink/main/dsp/fixed_dsp.h']
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    args=argv_windows(row['command']);args=args[:args.index('-o')];compiler=pathlib.Path(args[0]);tool=compiler.parent/'xtensa-esp32-elf-objdump.exe';obj=out/'control-audio-xtensa.o'
    subprocess.run(args+['-o',str(obj),'-c',str(paths[0])],cwd=build,check=True)
    dis=subprocess.check_output([str(tool),'-dr',str(obj)],text=True)
    table=subprocess.check_output([str(tool),'-t',str(obj)],text=True)
    sections={};sizes={}
    for name in ('controls_enter','controls_exit','dbt_audio_controls','dbt_audio_volume','dbt_audio_eq','update_options','dbt_audio_control_flags','dbt_audio_analysis'):
        m=re.search(r'^\S+\s+\w+\s+F\s+(\S+)\s+([0-9a-fA-F]+)\s+'+re.escape(name)+r'$',table,re.M)
        if not m:raise RuntimeError('missing symbol '+name)
        sections[name]=m.group(1);sizes[name]=int(m.group(2),16)
    for name in ('controls_enter','controls_exit','dbt_audio_controls','dbt_audio_volume','dbt_audio_eq','update_options'):
        assert sections[name].startswith('.iram1.'),(name,sections[name])
    def section(name):
        m=re.search(r'Disassembly of section '+re.escape(sections[name])+r':(.*?)(?=\nDisassembly of section|\Z)',dis,re.S)
        if not m:raise RuntimeError('missing section '+name)
        return m.group(1)
    enter,exit=section('controls_enter'),section('controls_exit')
    assert len(re.findall(r'\brsil\b',enter))==1 and len(re.findall(r'\bs32c1i\b',enter))==1
    assert '_xtos_set_intlevel' in enter and '_xtos_set_intlevel' in exit
    for body in (enter,exit):
        targets=set(re.findall(r'R_XTENSA_ASM_EXPAND\s+(\S+)',body))
        assert targets=={'_xtos_set_intlevel'},targets
    assert re.search(r'\bs32i\b',exit) and exit.index('s32i')<exit.index('_xtos_set_intlevel')
    for name in ('dbt_audio_controls','dbt_audio_volume','dbt_audio_eq','update_options'):
        body=section(name);assert sections['controls_enter'] in body and sections['controls_exit'] in body,name
        targets=set(re.findall(r'R_XTENSA_ASM_EXPAND\s+(\S+)',body))
        assert targets<={sections['controls_enter'],sections['controls_exit']},(name,targets)
    for name in ('dbt_audio_control_flags','dbt_audio_analysis'):
        assert sections['update_options'] in section(name),name
    after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    assert before==after,'source changed during target compilation'
    report={'passed':True,'scope':'Xtensa object instruction inspection, not executed on hardware',
            'masked_entry':'preserves higher prior level, one rsil at level 3, one s32c1i try-CAS',
            'busy_path':'restores prior interrupt level','success_exit':'releases atomic lock before restoring interrupt level','iram_bytes':sum(sizes[n] for n in ('controls_enter','controls_exit','dbt_audio_controls','dbt_audio_volume','dbt_audio_eq','update_options')),
            'source_sha256':before}
    (out/'control-target-inspection.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS Xtensa guarded writers: IRAM, local level mask, one-shot CAS, restore on busy/success; bytes',report['iram_bytes'])
finally:subprocess.run(check,check=True)
