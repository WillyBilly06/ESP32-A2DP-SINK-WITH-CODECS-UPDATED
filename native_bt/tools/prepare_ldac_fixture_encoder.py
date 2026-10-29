"""Fetch one pinned AOSP LDAC encoder into the bounded test-only build tree.

This does not modify the SDK, link an encoder into firmware, or remove files.
"""
import hashlib
import io
from pathlib import Path, PurePosixPath
import subprocess
import sys
import tarfile
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/native_codecs/ldac_fixture_encoder'
COMMIT='eeee1a3f5f8df1282e3a6d297085885fd886737b'
TREE_SHA256='9e38fc695ea63c20d60692722006fd65ebf592d6cc04cdf0dc634c86862f2c78'
URL='https://android.googlesource.com/platform/external/libldac/+archive/'+COMMIT+'.tar.gz'
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]


def main():
    if not OUT.resolve().is_relative_to((ROOT/'_build/native_codecs').resolve()):
        raise RuntimeError('Encoder destination resolves outside the project build tree')
    with urllib.request.urlopen(URL,timeout=30) as response:data=response.read(4*1024*1024+1)
    if len(data)>4*1024*1024:raise RuntimeError('AOSP encoder archive is oversized')
    entries={};total=0
    with tarfile.open(fileobj=io.BytesIO(data),mode='r:gz') as archive:
        for member in archive:
            name=PurePosixPath(member.name)
            if name.is_absolute() or '..' in name.parts:raise RuntimeError('Unsafe archive path')
            if not member.isfile():continue
            if not (name.parts[0] in ('src','inc') or str(name) in ('LICENSE','NOTICE')):continue
            if name.suffix not in ('.h','.c') and str(name) not in ('LICENSE','NOTICE'):continue
            total+=member.size
            if member.size>1024*1024 or total>8*1024*1024 or len(entries)>=100:
                raise RuntimeError('Encoder source exceeds bounded extraction size')
            if str(name) in entries:raise RuntimeError('Duplicate archive member')
            entries[str(name)]=archive.extractfile(member).read()
    if len(entries)!=32 or not {'src/ldaclib.c','src/ldacBT.c','inc/ldacBT.h','LICENSE','NOTICE'}<=set(entries):
        raise RuntimeError('Pinned encoder archive layout changed')
    digest=hashlib.sha256()
    for name in sorted(entries):digest.update(name.encode()+b'\0');digest.update(entries[name])
    if digest.hexdigest()!=TREE_SHA256:raise RuntimeError('Pinned encoder content hash changed')
    for name,content in entries.items():
        path=OUT.joinpath(*PurePosixPath(name).parts)
        if not path.resolve().is_relative_to(OUT.resolve()):raise RuntimeError('Encoder member escapes destination')
        if path.exists() and path.read_bytes()!=content:
            raise RuntimeError('Refusing to overwrite differing existing file: '+str(path))
    for name,content in entries.items():
        path=OUT.joinpath(*PurePosixPath(name).parts);path.parent.mkdir(parents=True,exist_ok=True)
        if not path.exists():
            with path.open('xb') as handle:handle.write(content)
    print(f'Pinned AOSP LDAC test encoder ready: {len(entries)} files, {total} source bytes')


if __name__=='__main__':
    subprocess.run(CHECK,check=True)
    try:main()
    finally:subprocess.run(CHECK,check=True)
