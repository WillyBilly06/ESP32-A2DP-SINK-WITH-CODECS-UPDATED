"""Pure host tests of bounded pending-trial return capture; no serial access."""
import importlib.util,json,pathlib
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('deploy',ROOT/'tools/deploy_audio_trial.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
BASE=[b'Loaded app from partition at offset 0x110000\n',b'1.0.4-idf6.1\n',b'System ready\n']
class Clock:
    def __init__(self):self.value=0
    def __call__(self):self.value+=0.1;return self.value
class Port:
    def __init__(self):self.chunks=list(BASE);self.timeout=0;self.baudrate=0;self.clears=0
    def reset_input_buffer(self):self.chunks=[];self.clears+=1
    def read(self,n):
        value=self.chunks.pop(0) if self.chunks else b'';assert len(value)<=n;return value
class Controller:
    def __init__(self,prime=False,missing=False,fatal=False):self._port=Port();self.resets=0;self.prime=prime;self.missing=missing;self.fatal=fatal
    def hard_reset(self):
        self.resets+=1
        if self.prime and self.resets==1:self._port.chunks=[b'Loaded app from partition at offset 0x390000\n']
        else:self._port.chunks=([b'Guru Meditation Error\n'] if self.fatal else [])+BASE[:2]+([] if self.missing else BASE[2:])
for rebooted in (True,False):
    c=Controller(prime=not rebooted)
    with patch.object(module.time,'monotonic',Clock()):result,capture=module.return_pending_rf_trial(c,115200,rebooted)
    assert result['baseline_boot_observed'] and not result['baseline_fatal_marker']
    assert c.resets==(1 if rebooted else 2) and c._port.baudrate==115200 and c._port.clears==c.resets
    assert all(x in capture for x in BASE)
c=Controller(missing=True)
with patch.object(module.time,'monotonic',Clock()):result,capture=module.return_pending_rf_trial(c,115200,True)
assert not result['baseline_boot_observed'] and b'System ready' not in capture
c=Controller(fatal=True)
with patch.object(module.time,'monotonic',Clock()):result,capture=module.return_pending_rf_trial(c,115200,True)
assert result['baseline_boot_observed'] and result['baseline_fatal_marker']
out=ROOT/'_build/finite_rf';out.mkdir(exist_ok=True)
(out/'results.json').write_text(json.dumps({'passed':True,'cases':4,'scope':'actual capture/return helper with fake serial, resets and clock; stale capture cleared, markers required, timeout/fatal detected; no hardware','hardware_accessed':False},indent=2)+'\n')
print('PASS finite RF return marker, stale epoch, pending prime, timeout and fatal checks; host only')
