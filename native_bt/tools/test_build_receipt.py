"""Offline negative gates for stale or tampered build input manifests."""
import hashlib,json,tempfile
from pathlib import Path
import source_snapshot as sources

root=Path(__file__).resolve().parents[1]
out=root/'_build/build_receipt';out.mkdir(exist_ok=True)
original=sources.snapshot
try:
    with tempfile.TemporaryDirectory(dir=out) as d:
        path=Path(d)/'sources.json'
        baseline={'source.c':'a'*64,'native_bt/_build/esp32/compile_commands.json':'b'*64}
        path.write_text(json.dumps(baseline))
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        sources.snapshot=lambda:dict(baseline)
        assert sources.verify_manifest(path,digest)==2
        cases=0
        for changed in ({**baseline,'source.c':'c'*64},
                        {**baseline,'native_bt/_build/esp32/compile_commands.json':'d'*64},
                        {'source.c':'a'*64}, {**baseline,'new.c':'e'*64}):
            sources.snapshot=lambda changed=changed:dict(changed)
            try:sources.verify_manifest(path,digest)
            except RuntimeError:cases+=1
            else:raise AssertionError('Stale source set accepted')
        sources.snapshot=lambda:dict(baseline)
        path.write_text(json.dumps({**baseline,'source.c':'f'*64}))
        try:sources.verify_manifest(path,digest)
        except RuntimeError:cases+=1
        else:raise AssertionError('Tampered manifest accepted')
        assert cases==5
    report={'passed':True,'negative_cases':cases,'scope':'offline stale source/define-set and manifest tamper rejection only','hardware_accessed':False}
    (out/'results.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
finally:sources.snapshot=original
