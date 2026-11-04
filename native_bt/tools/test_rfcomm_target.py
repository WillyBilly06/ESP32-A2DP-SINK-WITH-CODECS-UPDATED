"""Compile RFCOMM and optional probe wrapper into standalone ESP32 objects.
No firmware link, device access, cache cleaning, or reference-source linkage.
Uses the compiler already recorded by the fixed ESP-IDF build tree.
"""
import hashlib, json, re, subprocess, sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
folder=root/'_build/rfcomm';folder.mkdir(parents=True,exist_ok=True)
check=[sys.executable,str(root/'tools/check_workspace.py')]
sources=[root/'src'/f'{name}.c' for name in ('dbt_rfcomm','dbt_rfcomm_mux','dbt_rfcomm_credit','dbt_rfcomm_dlc','dbt_rfcomm_bearer','dbt_rfcomm_spp')]+[root/'esp32_probe/main/probe_rfcomm.c']
tracked=set(sources+[Path(__file__).resolve()]);search=[root/'include',root/'esp32_probe/main']
def dependencies(path):
    for name in re.findall(r'^\s*#\s*include\s*"([^"]+)"',path.read_text(),re.M):
        for directory in search:
            p=directory/name
            if p.is_file() and p not in tracked:
                tracked.add(p);dependencies(p);break
for source in sources:dependencies(source)
def hashes():return {str(p.relative_to(root)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(tracked)}
subprocess.run(check,check=True)
try:
    before=hashes();rows=json.loads((root/'_build/esp32/compile_commands.json').read_text())
    command=next(x['command'] for x in rows if x['file'].endswith('probe_sdp.c'))
    match=re.match(r'("[^"]+"|\S+)',command)
    if not match:raise RuntimeError('Cannot identify pinned build compiler')
    compiler=match.group(1).strip('"')
    version=subprocess.run([compiler,'-dumpfullversion'],check=True,text=True,capture_output=True).stdout.strip()
    objects={}
    for source in sources:
        output=folder/(source.stem+'.o')
        subprocess.run([compiler,'-std=gnu11','-Wall','-Wextra','-Werror','-ffreestanding','-I',str(root/'include'),
                        '-I',str(root/'esp32_probe/main'),'-c',str(source),'-o',str(output)],check=True)
        objects[output.name]=hashlib.sha256(output.read_bytes()).hexdigest();print('PASS',source.name,flush=True)
    if hashes()!=before:raise RuntimeError('Source changed during target-object compilation')
    report={'passed':True,'scope':'standalone ESP32 target objects; no firmware link or hardware','compiler_version':version,
            'source_sha256':before,'object_sha256':objects}
    (folder/'target-objects.json').write_text(json.dumps(report,indent=2)+'\n')
finally:subprocess.run(check,check=True)
