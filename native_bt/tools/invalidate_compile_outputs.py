"""Force Ninja to inspect every compiled source through ccache, even after rollback.

Only existing compiler outputs inside this build tree have their timestamps
invalidated. No source, object content, archive, ELF, or hardware is deleted.
"""
import argparse,json,os
from pathlib import Path

def invalidate(build):
    build=Path(build).resolve();outputs=set();databases=[]
    for database in sorted(build.rglob('compile_commands.json')):
        entries=json.loads(database.read_text());databases.append(str(database.relative_to(build)))
        for entry in entries:
            if not entry.get('output'):raise RuntimeError('Compilation database lacks explicit output')
            directory=Path(entry['directory']).resolve()
            path=(directory/entry['output']).resolve();path.relative_to(build)
            outputs.add(path)
    if not databases or not outputs:raise RuntimeError('No compiler outputs found')
    changed=[]
    for path in sorted(outputs):
        if not path.exists():continue
        if not path.is_file():raise RuntimeError('Compiler output is not a file')
        stat=path.stat();os.utime(path,ns=(stat.st_atime_ns,0))
        if path.stat().st_mtime_ns!=0:raise RuntimeError('Output timestamp was not invalidated')
        changed.append(str(path.relative_to(build)))
    return {'passed':True,'scope':'host compiler-output timestamps only; ccache still compiles/hash-checks actual inputs',
            'databases':databases,'outputs':len(outputs),'invalidated':changed,'hardware_accessed':False}

def main():
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--receipt',type=Path,required=True);a=p.parse_args()
    a.receipt.write_text(json.dumps({'passed':False,'state':'invalidating'})+'\n')
    result=invalidate(a.build);a.receipt.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='invalidated'}|{'invalidated_count':len(result['invalidated'])},indent=2))
if __name__=='__main__':main()
