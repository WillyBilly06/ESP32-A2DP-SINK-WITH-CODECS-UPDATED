"""Keep rebuildable/reference budgets strict without discarding RF evidence."""
import contextlib,importlib.util,io,json,pathlib
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('workspace_check',ROOT/'tools/check_workspace.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
cases=[(0,318*1048576,200*1048576,True),(0,512*1048576,900*1048576,True),
       (0,512*1048576+1,0,False),(96*1048576+1,0,0,False)]
for reference,output,evidence,passed in cases:
 values={m.WORKSPACE/'reference':reference,m.ROOT/'_build':output+evidence,m.ROOT/'_build/hardware':evidence}
 stream=io.StringIO()
 with patch.object(m,'tree_bytes',lambda path:values[path]),contextlib.redirect_stdout(stream):code=m.main()
 result=json.loads(stream.getvalue())
 assert result['passed']==passed and (code==0)==passed,result
 assert result['build_mib']==round((output+evidence)/1048576,3)
 assert result['hardware_evidence_mib']==round(evidence/1048576,3)
 assert result['rebuildable_mib']==round(output/1048576,3)
OUT=ROOT/'_build/workspace_budget';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':True,'cases':len(cases),'scope':'actual checker with simulated byte totals; retained evidence reported, 512 MiB rebuildable and 96 MiB reference boundaries reject one-byte excess; actual dependency-path checks still run','hardware_accessed':False},indent=2)+'\n')
print('PASS evidence/rebuildable classification and exact reference/build budget boundaries')
