"""Run the owned host/runtime tests and C sanitizers on the authorized Linux VM."""
import argparse
import getpass
import hashlib
import json
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[1]
DRIVER = r"""
import importlib.util,json,pathlib,subprocess
root=pathlib.Path.cwd()
spec=importlib.util.spec_from_file_location('host_tests',root/'tools/test_host.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
def run(command):
 p=subprocess.run(command,capture_output=True,text=True,timeout=60)
 if p.returncode: raise RuntimeError(' '.join(command)+'\n'+p.stdout[-4000:]+p.stderr[-8000:])
 return p.stdout
output=run(['python3','tools/test_host.py'])
native=json.loads((root/'_build/host/results.json').read_text())
assert native['passed']
declarations='\n'.join('int test_'+name+'(void);' for name in module.TESTS)
calls='\n'.join('if ((line=test_'+name+'())) { printf("FAIL '+name+' %d\\n",line); return 1; }' for name in module.TESTS)
concurrent=r'''
void stress_begin(void); int stress_produce(unsigned); int stress_consume(void);
static void *produce(void *unused) {
 (void)unused;
 for(unsigned n=0;n<20000;++n) {
  int status; while((status=stress_produce(n))==3) sched_yield();
  if(status) return (void *)1;
 }
 return 0;
}
static void *consume(void *unused) {
 (void)unused;
 for(unsigned n=0;n<20000;++n) {
  int status; while((status=stress_consume())==1) sched_yield();
  if(status) return (void *)1;
 }
 return 0;
}
int audio_stress_begin(void); int audio_stress_end(void); int audio_stress_produce(unsigned);
unsigned audio_stress_render(void); unsigned audio_stress_rendered(void); int audio_stress_consume(void);
int audio_snapshot_begin(void); int audio_snapshot_end(void); int audio_snapshot_write(int); int audio_snapshot_read(void);
static _Atomic int audio_stop,audio_failed,controls_done;
static void *audio_produce(void *unused){
 (void)unused;
 for(unsigned n=0;n<16384 && !atomic_load(&audio_stop);n+=64){
  int result;while((result=audio_stress_produce(n))==5 && !atomic_load(&audio_stop))sched_yield();
  if(result!=0){atomic_store(&audio_failed,1);atomic_store(&audio_stop,1);break;}
 }
 return 0;
}
static void *audio_render(void *unused){
 (void)unused;while(!atomic_load(&audio_stop) && audio_stress_rendered()<16384)if(!audio_stress_render())sched_yield();return 0;
}
static void *audio_consume(void *unused){
 (void)unused;
 for(unsigned blocks=0;blocks<256 && !atomic_load(&audio_stop);){
  int result=audio_stress_consume();if(result==0)++blocks;
  else if(result==1)sched_yield();else{atomic_store(&audio_failed,1);atomic_store(&audio_stop,1);}
 }
 return 0;
}
static void *audio_controls(void *unused){
 (void)unused;
 for(int n=0;n<10000;++n)if(audio_snapshot_write(n%25-12)){atomic_store(&audio_failed,1);break;}
 atomic_store(&controls_done,1);return 0;
}
static int check_audio_owners(void){
 pthread_t producer,renderer,consumer,controls;
 if(audio_stress_begin())return 1;
 atomic_init(&audio_stop,0);atomic_init(&audio_failed,0);atomic_init(&controls_done,0);
 if(pthread_create(&producer,0,audio_produce,0)||pthread_create(&renderer,0,audio_render,0)||pthread_create(&consumer,0,audio_consume,0))return 2;
 pthread_join(producer,0);pthread_join(renderer,0);pthread_join(consumer,0);
 if(atomic_load(&audio_failed)||audio_stress_end())return 3;
 if(audio_snapshot_begin()||pthread_create(&controls,0,audio_controls,0))return 4;
 while(!atomic_load(&controls_done))if(audio_snapshot_read())atomic_store(&audio_failed,1);
 pthread_join(controls,0);
 return atomic_load(&audio_failed)||audio_snapshot_end();
}
'''
main='#include <stdio.h>\n#include <pthread.h>\n#include <sched.h>\n#include <stdatomic.h>\n'+declarations+concurrent+'\nint main(void) { int line;\n'+calls+'''
 pthread_t producer,consumer; void *a,*b;
 stress_begin();
 if(pthread_create(&producer,0,produce,0)||pthread_create(&consumer,0,consume,0))return 2;
 pthread_join(producer,&a);pthread_join(consumer,&b);
 if(a||b)return 3;
 if(check_audio_owners())return 4;
 puts("all host/runtime groups and 20000 concurrent packets passed");return 0;
}
'''
(root/'sanitizer_main.c').write_text(main)
sources=[str(p) for p in sorted((root/'src').glob('*.c'))+sorted((root/'tests').glob('test_*.c'))]
sources+=['codecs/common/dbt_codec.c','codecs/aac/dbt_aac_transport.c']
command=['gcc','-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
 '-fno-omit-frame-pointer','-Iinclude','-Idsp','-Icodecs/common','-Icodecs/aac','-pthread',*sources,'dsp/fixed_dsp.c','sanitizer_main.c','-o','sanitized']
run(command);sanitized=run(['./sanitized'])
thread_command=[arg.replace('-fsanitize=address,undefined','-fsanitize=thread').replace('sanitized','threaded') for arg in command]
run(thread_command);threaded=run(['./threaded'])
report={'passed':True,'scope':'owned host/runtime paired-peer simulation and memory safety; no RF or profiles validation',
 'normal_groups':len(native['tests']),'address_undefined_passed':True,'concurrent_packets':20000,
 'sanitizer_output':sanitized.strip(),'source_sha256':native['source_sha256']}
report['thread_sanitizer_passed']=True
report['audio_three_owner_frames']=16384
(root/'linux_results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT / "_build/python_deps"))
    import paramiko
    password = getpass.getpass("Linux SSH password (session only): ")
    client = paramiko.SSHClient()
    client.load_host_keys(str(ROOT / "_build/linux_known_hosts"))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(args.host, username=args.user, password=password, look_for_keys=False,
                       allow_agent=False, timeout=8, auth_timeout=10)
        password = None
        _, stdout, _ = client.exec_command("python3 -c 'import os; print(os.path.expanduser(\"~/.cache/native_bt/stack_tests\"))'", timeout=8)
        remote = stdout.read().decode().strip()
        if not remote.startswith("/home/") or not remote.endswith("/.cache/native_bt/stack_tests"):
            raise RuntimeError("Unexpected remote workspace")
        _, stdout, stderr = client.exec_command("mkdir -p " + " ".join(shlex.quote(remote + "/" + value)
            for value in ["src", "include", "tests", "tools", "dsp", "codecs/common", "codecs/aac"]), timeout=8)
        if stdout.channel.recv_exit_status():
            raise RuntimeError(stderr.read().decode())
        files = {str(path.relative_to(ROOT)).replace("\\", "/"): path
                 for folder, pattern in [("src", "*.c"), ("include", "*.h"), ("tests", "test_*.c")]
                 for path in (ROOT / folder).glob(pattern)}
        files["tools/test_host.py"] = ROOT / "tools/test_host.py"
        for name in ["codecs/common/dbt_codec.c", "codecs/common/dbt_codec.h", "codecs/aac/dbt_aac_transport.c", "codecs/aac/dbt_aac.h"]:
            files[name] = ROOT / name
        for name in ["fixed_dsp.c", "fixed_dsp.h", "fixed_dsp_tables.inc"]:
            files["dsp/" + name] = ROOT.parent / "bt_audio_sink/main/dsp" / name
        sftp = client.open_sftp()
        hashes = {}
        for name, path in files.items():
            data = path.read_bytes()
            if name == "tools/test_host.py":
                # The small test copy has the DSP beside it, without an SDK copy.
                data = data.replace(b'ROOT.parent / "bt_audio_sink/main/dsp', b'ROOT / "dsp')
            hashes[name] = hashlib.sha256(data).hexdigest()
            with sftp.file(remote + "/" + name, "wb") as handle:
                handle.write(data)
        stdin, stdout, stderr = client.exec_command("cd " + shlex.quote(remote) + " && python3 -", timeout=180)
        stdin.write(DRIVER)
        stdin.flush()
        stdin.channel.shutdown_write()
        output = stdout.read(100000).decode()
        errors = stderr.read(100000).decode()
        if stdout.channel.recv_exit_status():
            raise RuntimeError(errors or output)
        with sftp.file(remote + "/linux_results.json", "r") as handle:
            report = json.loads(handle.read())
        report["uploaded_sha256"] = hashes
        destination = ROOT / "_build/host/linux_results.json"
        destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({key: value for key, value in report.items() if not key.endswith("sha256")}, indent=2))
        sftp.close()
    finally:
        password = None
        client.close()


if __name__ == "__main__":
    main()
