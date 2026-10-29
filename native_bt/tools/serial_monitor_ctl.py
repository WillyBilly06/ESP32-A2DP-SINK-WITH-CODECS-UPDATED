"""Control the local monitor's reader lease; never sends bytes to the ESP32."""
import argparse,json,time,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['release','resume','status','note']);p.add_argument('--state',type=Path,default=ROOT/'_build/serial/monitor-state.json');p.add_argument('--timeout',type=float,default=8);p.add_argument('--text',default='');a=p.parse_args()
    hold=ROOT/'_build/hardware/HARDWARE_HOLD.json'
    if a.action=='resume' and hold.exists() and json.loads(hold.read_text()).get('active',True):
        raise SystemExit('ESP32 reserved by user; automatic serial resume is disabled')
    state=a.state.resolve();state.relative_to((ROOT/'_build').resolve());assert 0<a.timeout<=30
    current=json.loads(state.read_text())
    if a.action=='status':print(json.dumps(current,indent=2));return
    if not current.get('running') or time.time()-current.get('updated_unix',0)>5:raise SystemExit('Monitor is stopped or stale; no command sent')
    cid=uuid.uuid4().hex;command=state.with_suffix('.command.json');temp=command.with_suffix('.tmp')
    temp.write_text(json.dumps({'id':cid,'pid':current['pid'],'action':a.action,'text':a.text[:256]})+'\n');temp.replace(command)
    end=time.monotonic()+a.timeout
    while time.monotonic()<end:
        try:
            value=json.loads(state.read_text())
            if value['pid']!=current['pid']:raise SystemExit('Monitor restarted; command outcome must be rechecked')
            if a.action=='note':done=value.get('note_command_id')==cid
            else:done=value.get('applied_command_id')==cid and value.get('reader_mode')==('released' if a.action=='release' else 'listening') and value.get('connected')==(a.action=='resume')
            if done:print(json.dumps({'action':a.action,'acknowledged':True,'pid':value['pid'],'port':value['port'],'reader_mode':value['reader_mode'],'connected':value['connected']}));return
        except (OSError,ValueError):pass
        time.sleep(.1)
    raise SystemExit('Monitor did not acknowledge actual reader state before timeout; do not assume the port is free')
if __name__=='__main__':main()
