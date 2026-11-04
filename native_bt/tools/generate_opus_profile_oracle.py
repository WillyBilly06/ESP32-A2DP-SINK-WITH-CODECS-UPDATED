"""Add exact architecture-specific PCM hashes without changing encoded vectors.

Run test_opus_char_variants.py first. Refuse stale or mismatched receipts.
"""
import hashlib
import json
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[1]
HEADER=ROOT/'esp32_probe/main/probe_opus_vectors.h'
RECEIPT=ROOT/'_build/native_codecs/opus_char_variants/results.json'
text=HEADER.read_text()
receipt=json.loads(RECEIPT.read_text())
for path,digest in receipt['source_sha256'].items():
    if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:
        raise SystemExit(f'stale arithmetic-profile receipt: {path}')
fast=receipt['vectors']['unsigned_no_harden']
slow=receipt['vectors']['unsigned_slow64']
assert len(fast)==len(slow)==16
assert all(a['actual']==a['expected'] for a in fast)
assert not receipt['different_q31fast']
pattern=re.compile(r'(\{opus_cie_(\d+),\d+,\{[^}]+\},\{[^}]+\},)(?:OPUS_PROFILE_HASH\((\d+)u,(\d+)u\)|(\d+)u)(,\d+,\d+\})')
matches=list(pattern.finditer(text))
assert len(matches)==16
def replace(match):
    index=int(match.group(2));assert index<16
    previous=int(match.group(3) or match.group(5))
    assert previous==fast[index]['actual']
    return f'{match.group(1)}OPUS_PROFILE_HASH({fast[index]["actual"]}u,{slow[index]["actual"]}u){match.group(6)}'
updated=pattern.sub(replace,text)
prefix='''/* Match the SDK celt/arch.h OPUS_FAST_INT64 selection. The component recipe
 * checks the SDK rule at configuration time. Test forcing is host-only. */
#if defined(DBT_OPUS_TEST_FORCE_SLOW64) && defined(ESP_PLATFORM)
#error "Opus oracle test override is forbidden in ESP32 firmware"
#endif
#if defined(DBT_OPUS_TEST_FORCE_SLOW64)
#define OPUS_PROFILE_HASH(fast, slow) (slow)
#elif defined(__x86_64__) || defined(__LP64__) || defined(_WIN64)
#define OPUS_PROFILE_HASH(fast, slow) (fast)
#else
#define OPUS_PROFILE_HASH(fast, slow) (slow)
#endif
'''
marker='static const opus_vector_t opus_vectors[]='
if '/* Match the SDK celt/arch.h OPUS_FAST_INT64 selection.' not in updated:
    assert updated.count(marker)==1
    updated=updated.replace(marker,prefix+marker)
HEADER.write_text(updated)
print('Updated exact fast64 and slow64 hashes for 16 existing packet sets; encoded arrays unchanged')
