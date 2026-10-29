"""Fetch one pinned test-only LHDC V5 encoder without cloning an SDK."""
import hashlib
import io
from pathlib import Path,PurePosixPath
import subprocess
import sys
import tarfile
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/native_codecs/lhdcv5_fixture_encoder'
COMMIT='3f9d1980fa9c57cdfb78792f9455a7dbac50edda'
TREE_SHA='6917725b0598e56af0f7ba20490cfadeb9b0910aefa822c4b71a0af726c0651a'
URL='https://codeload.github.com/WillyBilly06/LHDC-V5-Encoder/tar.gz/'+COMMIT
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]


def main():
    if not OUT.resolve().is_relative_to((ROOT/'_build/native_codecs').resolve()):
        raise RuntimeError('Encoder destination escapes project build tree')
    request=urllib.request.Request(URL,headers={'User-Agent':'native-codec-fixture-test'})
    with urllib.request.urlopen(request,timeout=30) as response:data=response.read(4*1024*1024+1)
    if len(data)>4*1024*1024:raise RuntimeError('Encoder archive exceeds the bound')
    entries={};total=0
    with tarfile.open(fileobj=io.BytesIO(data),mode='r:gz') as archive:
        for member in archive:
            full=PurePosixPath(member.name)
            if full.is_absolute() or '..' in full.parts:raise RuntimeError('Unsafe archive path')
            if not member.isfile() or len(full.parts)<2:continue
            name=PurePosixPath(*full.parts[1:])
            if not (name.parts[0] in ('src','include','examples') or str(name) in ('LICENSE','README.md')):continue
            if name.suffix not in ('.c','.h','.md') and str(name)!='LICENSE':continue
            total+=member.size
            if member.size>1024*1024 or total>8*1024*1024 or len(entries)>=100:
                raise RuntimeError('Encoder source exceeds bounded extraction size')
            if str(name) in entries:raise RuntimeError('Duplicate archive member')
            entries[str(name)]=archive.extractfile(member).read()
    if len(entries)!=17 or not {'src/lhdcv5_api.c','include/lhdcv5_enc.h','examples/encode_stdin.c','LICENSE'}<=set(entries):
        raise RuntimeError('Encoder archive layout changed')
    digest=hashlib.sha256()
    for name in sorted(entries):digest.update(name.encode()+b'\0');digest.update(entries[name])
    if digest.hexdigest()!=TREE_SHA:raise RuntimeError('Encoder source tree hash changed')
    for name,content in entries.items():
        path=OUT.joinpath(*PurePosixPath(name).parts)
        if not path.resolve().is_relative_to(OUT.resolve()):raise RuntimeError('Encoder member escapes destination')
        if path.exists() and path.read_bytes()!=content:raise RuntimeError('Refusing to overwrite differing file: '+str(path))
    for name,content in entries.items():
        path=OUT.joinpath(*PurePosixPath(name).parts);path.parent.mkdir(parents=True,exist_ok=True)
        if not path.exists():
            with path.open('xb') as handle:handle.write(content)
    print(f'Pinned LHDC V5 test encoder ready: {len(entries)} files, {total} source bytes')


if __name__=='__main__':
    subprocess.run(CHECK,check=True)
    try:main()
    finally:subprocess.run(CHECK,check=True)
