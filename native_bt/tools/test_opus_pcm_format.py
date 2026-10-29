"""Offline unit vectors for the production Opus CIE-to-PCM mapping."""
import ctypes,hashlib,json,os,pathlib,shutil,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1]
workspace=root.parent
out=root/'_build'/'opus_pcm'
out.mkdir(parents=True,exist_ok=True)
inputs=[workspace/'bt_audio_sink/main/bt/a2dp_sink_native.h',workspace/'bt_audio_sink/main/bt/a2dp_sink_native.cpp',root/'tests/codecs/opus_pcm_format.cpp',pathlib.Path(__file__).resolve()]
def hashes():return {p.relative_to(workspace).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
compiler=os.environ.get('CXX') or shutil.which('clang++') or shutil.which('c++')
if not compiler and os.name=='nt':compiler=r'C:\Program Files\LLVM\bin\clang++.exe'
if not compiler:raise SystemExit('C++ compiler required')
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True);before=hashes()
try:
    target=out/('format.dll' if os.name=='nt' else 'format.so')
    command=[compiler,'-std=c++17','-O2','-Wall','-Wextra','-Werror','-I',str(workspace/'bt_audio_sink/main/bt')]
    if os.name=='nt':command+=['-ffreestanding','-fno-builtin','-fno-exceptions','-fno-rtti','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry']
    else:command+=['-shared','-fPIC']
    subprocess.run(command+[str(inputs[2]),'-o',str(target)],check=True)
    dll=ctypes.CDLL(str(target)); names=('android_vectors','opus05_vectors','invalid_is_not_guessed','other_vendor_unchanged','sample_stride','android_exhaustive');rows=[]
    for name in names:
        line=getattr(dll,'opus_pcm_'+name)();rows.append({'name':name,'passed':not line,'failed_line':line or None})
        print(('PASS ' if not line else 'FAIL ')+name+(f' line {line}' if line else ''))
    stable=before==hashes();report={'passed':stable and all(r['passed'] for r in rows),'source_stable':stable,'source_sha256':before,'tests':rows,'scope':'host tests of production CIE-to-PCM helper; not target link, decoder execution, RF or sound'}
    (out/'results.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if not report['passed']:raise SystemExit(1)
finally:subprocess.run(check,check=True)
