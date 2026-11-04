"""Read deterministic echo failure bytes and bounded connection trace metadata."""
import hashlib,json,pathlib,subprocess
root=pathlib.Path.home()/'.cache/native_bt/stack_tests'
report={}
for name in ['classic-expected.bin','classic-actual.bin']:
    data=(root/name).read_bytes()
    assert len(data)<=4096
    report[name]={'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'last_32':data[-32:].hex()}
path=root/'le-root-trace-filtered.txt'
assert path.stat().st_size<=524288
lines=path.read_text(errors='replace').splitlines();selected=set()
for i,line in enumerate(lines):
    if any(text in line for text in ['Disconnect','Disconnection','Unpaired','Remove Device','Index Removed','Index Added','Hardware Error']):
        selected.update(range(max(0,i-1),min(len(lines),i+7)))
report['termination_trace']=[lines[i] for i in sorted(selected)][-180:]
result=subprocess.run(['systemctl','show','bluetooth.service','-p','MainPID','-p','NRestarts','-p','ActiveState'],capture_output=True,text=True,timeout=5)
report['daemon_status']=result.stdout.splitlines()
print(json.dumps(report,indent=2))
