import json,pathlib,re,socket,time
root=pathlib.Path.home()/'.cache/native_bt/stack_tests'
report={}
for name in ['le-root-trace-filtered.txt','le-root-trace.txt']:
    path=root/name
    if not path.exists():continue
    lines=path.read_text(errors='replace').splitlines()
    report[name]={'bytes':path.stat().st_size,'commands':[x for x in lines if 'HCI Command:' in x or 'MGMT Command:' in x][:100]}
    if name.endswith('filtered.txt'):
        selected=set()
        for i,line in enumerate(lines):
            if 'Command:' in line or any(t in line for t in ['Connection','Failed','Status:','Error:']):selected.update(range(max(0,i-1),min(len(lines),i+6)))
        report[name]['details']=[lines[i] for i in sorted(selected)][:130]
    else:
        for i,line in enumerate(lines):
            if '6C:C8:40:42:BF:66' in line.upper():
                report[name]['advertisement_header']=lines[max(0,i-14):i+4];break
try:
    channel=socket.socket(socket.AF_BLUETOOTH,socket.SOCK_SEQPACKET,socket.BTPROTO_L2CAP)
    channel.settimeout(10)
    channel.bind(('00:00:00:00:00:00',0,0,1))
    began=time.monotonic()
    try:
        channel.connect(('6C:C8:40:42:BF:66',0,4,1))
        report['kernel_le_connect_seconds']=round(time.monotonic()-began,3)
        channel.send(bytes([2,5,2]));report['kernel_att_mtu_reply']=channel.recv(1024).hex()
        channel.send(bytes([0x0a,3,0]));report['kernel_att_name_reply']=channel.recv(1024).hex()
    finally:channel.close()
except Exception as error:report['kernel_le_error']=str(error)
print(json.dumps(report,indent=2))
