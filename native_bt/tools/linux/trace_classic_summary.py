"""Inspect only target Classic ACL blocks from the bounded, key-redacted trace."""
import hashlib,json,pathlib,re
path=pathlib.Path.home()/'.cache/native_bt/stack_tests/classic-root-trace-filtered.txt'
data=path.read_bytes();assert len(data)<=524288
blocks=re.split(r'(?=^(?:[^\s:][^\n]*?: )?[<>@=*+] )',data.decode(errors='replace'),flags=re.M)
selected=[]
for block in blocks:
    if not re.match(r'^(?:[^\s:][^\n]*?: )?[<>] ACL Data (TX|RX):',block):continue
    lines=block.splitlines()
    selected.append({'header':lines[0],'first':lines[1:5],'last':lines[-3:]})
print(json.dumps({'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),
                  'acl_blocks':len(selected),'last_acl_blocks':selected[-12:]},indent=2))
