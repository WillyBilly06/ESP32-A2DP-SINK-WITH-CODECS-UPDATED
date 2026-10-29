"""Read only the bounded, previously filtered owned-peer protocol trace."""
import json
from pathlib import Path
import re
path=Path.home()/'.cache/native_bt/stack_tests/classic-root-trace-filtered.txt'
assert path.resolve()==path and path.stat().st_size<=524288
text=path.read_text()
blocks=re.split(r'(?m)(?=^[<>@])',text)
selected=[block for block in blocks if 'AVDTP' in block or 'Connection Request' in block or 'Connection Response' in block]
print(json.dumps({'scope':'owned-peer protocol trace excerpt','protocol':'\n'.join(selected)[-24000:]},indent=2))
