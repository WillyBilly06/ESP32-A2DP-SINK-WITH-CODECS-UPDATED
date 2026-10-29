"""Compile the owned wire codec for ESP32 without linking/flashing firmware."""
import hashlib,json,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    rows=json.loads((ROOT/'_build/esp32/compile_commands.json').read_text())
    row=next(r for r in rows if Path(r['file']).name=='dbt_credit.c')
    compiler=Path(re.match(r'^\s*("[^"]+"|\S+)',row['command'])[1].strip('"'))
    assert compiler.name in ['xtensa-esp32-elf-gcc','xtensa-esp32-elf-gcc.exe']
    nm=compiler.with_name(compiler.name.replace('gcc','nm'));out=ROOT/'_build/ertm/wire-target.o'
    files=[ROOT/'src/dbt_ertm.c',ROOT/'include/dbt_ertm.h',ROOT/'include/dbt_queue.h']
    before={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    subprocess.run([str(compiler),'-std=c11','-Os','-Wall','-Wextra','-Werror','-I',str(ROOT/'include'),'-c',str(files[0]),'-o',str(out)],check=True)
    symbols=subprocess.check_output([str(nm),'-S','--format=posix',str(out)],text=True)
    mutable=[line for line in symbols.splitlines() if len(line.split())>1 and line.split()[1] in 'BbDdCcSs'];assert not mutable
    assert before=={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    report={'scope':'ESP32 compiled ERTM wire object only; not linked or active in firmware','compiled':True,'linked_in_probe':False,'mutable_static_symbols':mutable,'object_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'source_sha256':before}
    (ROOT/'_build/ertm/target.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))
if __name__=='__main__':main()
