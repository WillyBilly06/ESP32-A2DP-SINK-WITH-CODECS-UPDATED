"""Content fingerprint for the combined image's owned and compiled sources."""
import argparse,hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];WS=ROOT.parent

def snapshot():
    paths=set()
    for folder in ('include','src','codecs','esp32_probe','tools'):
        for f in (ROOT/folder).rglob('*'):
            if f.is_file() and '_build' not in f.parts and (f.suffix in ('.c','.cpp','.h','.inc','.py','.ps1') or f.name in ('CMakeLists.txt','sdkconfig','sdkconfig.defaults')):paths.add(f.resolve())
    paths.add((ROOT/'CMakeLists.txt').resolve())
    for f in (WS/'bt_audio_sink/main/dsp').glob('fixed_dsp*'):
        if f.is_file():paths.add(f.resolve())
    commands=ROOT/'_build/esp32/compile_commands.json'
    if commands.exists():
        for entry in json.loads(commands.read_text()):
            f=Path(entry['file']).resolve()
            if f.is_file() and ('_build' not in f.parts or 'port' in f.parts):paths.add(f)
    sibling=WS.parent/'LHDC-V5-Decoder-main/decoder'
    if sibling.exists():paths.update(f.resolve() for f in sibling.glob('*.h'))
    result={}
    for f in sorted(paths):
        try:key=f.relative_to(WS).as_posix()
        except ValueError:key='external/'+f.parent.name+'/'+f.name
        result[key]=hashlib.sha256(f.read_bytes()).hexdigest()
    return result
p=argparse.ArgumentParser();p.add_argument('--save',type=Path);p.add_argument('--verify',type=Path);a=p.parse_args();current=snapshot()
if a.save:a.save.write_text(json.dumps(current,indent=2)+'\n')
if a.verify:
    old=json.loads(a.verify.read_text());changed=sorted(k for k in set(old)|set(current) if old.get(k)!=current.get(k))
    print(json.dumps({'source_content_stable':not changed,'files':len(current),'changed':changed},indent=2))
    if changed:sys.exit(1)
else:print('Fingerprinted',len(current),'source units and owned build inputs')
