"""Bounded UART observation without writes or deliberate DTR/RTS resets."""
import argparse
import json
from pathlib import Path
import time
import serial
import sys


def main():
    sys.stdout.reconfigure(encoding="utf-8",errors="backslashreplace")
    parser=argparse.ArgumentParser()
    parser.add_argument("--port",required=True)
    parser.add_argument("--baud",type=int,default=115200)
    parser.add_argument("--seconds",type=float,default=3)
    parser.add_argument("--label",default="observe")
    args=parser.parse_args()
    if args.port.upper() != "COM6" or not 0<args.seconds<=60:
        raise SystemExit("Only the identified COM6 speaker, at most 60 seconds / 32 KiB, is admitted")
    if not args.label or len(args.label)>32 or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_" for c in args.label):
        raise SystemExit("Use a short alphanumeric capture label")
    root=Path(__file__).resolve().parents[1]/"_build/hardware"
    root.mkdir(parents=True,exist_ok=True)
    stream=serial.Serial(port=None,baudrate=args.baud,timeout=0.1)
    stream.dtr=False;stream.rts=False;stream.port=args.port
    capture=bytearray()
    started=time.time();began=time.monotonic()
    try:
        stream.open()
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline and len(capture)<32768:
            capture.extend(stream.read(min(4096,32768-len(capture))))
    except serial.SerialException as error:
        print(json.dumps({"port":args.port,"status":"unavailable","error":str(error)}))
        return 2
    finally:
        stream.close()
    destination=root/(args.label+"-"+args.port+"-"+str(args.baud))
    destination.with_suffix(".bin").write_bytes(capture)
    destination.with_suffix(".txt").write_text(capture.decode("utf-8",errors="backslashreplace"),encoding="utf-8")
    report={"port":args.port,"baud":args.baud,"bytes":len(capture),"start_unix":started,
            "end_unix":time.time(),"elapsed_seconds":round(time.monotonic()-began,3),
            "requested_seconds":args.seconds,"byte_limit_reached":len(capture)>=32768,
            "uart_writes":0,"deliberate_resets":0}
    destination.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report))
    print(capture.decode("utf-8",errors="backslashreplace")[:12000])
    return 0


if __name__=="__main__":raise SystemExit(main())
