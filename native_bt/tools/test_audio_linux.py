"""Compile actual DSP/storage headers and run sanitizers on the authorized Linux VM."""
import argparse
import getpass
import hashlib
import json
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[1]
STUBS = {
    "sdkconfig.h": '#define CONFIG_I2S_DEFAULT_SAMPLE_RATE 48000\n#define CONFIG_DSP_GOERTZEL_N 512\n#define CONFIG_DSP_CROSSOVER_LP_FREQ 90\n#define CONFIG_DSP_CROSSOVER_HP_FREQ 500\n#define CONFIG_DEFAULT_DEVICE_NAME "speaker"\n',
    "esp_attr.h": '#define IRAM_ATTR\n',
    "esp_log.h": '#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\n#define ESP_LOGE(...) ((void)0)\n#define ESP_LOGD(...) ((void)0)\n',
    "nvs_flash.h": '#include "nvs.h"\n',
    "nvs.h": '''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int nvs_handle_t;
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NVS_NOT_FOUND 1
#define NVS_READWRITE 0
#define NVS_READONLY 1
#ifdef __cplusplus
extern "C" {
#endif
esp_err_t nvs_open(const char *, int, nvs_handle_t *);
void nvs_close(nvs_handle_t);
esp_err_t nvs_commit(nvs_handle_t);
esp_err_t nvs_set_u32(nvs_handle_t,const char *,uint32_t);
esp_err_t nvs_get_u32(nvs_handle_t,const char *,uint32_t *);
esp_err_t nvs_get_i8(nvs_handle_t,const char *,int8_t *);
esp_err_t nvs_get_u8(nvs_handle_t,const char *,uint8_t *);
esp_err_t nvs_get_str(nvs_handle_t,const char *,char *,size_t *);
esp_err_t nvs_set_u8(nvs_handle_t,const char *,uint8_t);
esp_err_t nvs_set_str(nvs_handle_t,const char *,const char *);
const char *esp_err_to_name(esp_err_t);
#ifdef __cplusplus
}
#endif
''',
}
DRIVER = r'''
import json, pathlib, subprocess
root=pathlib.Path.cwd()
def run(command):
 p=subprocess.run(command,capture_output=True,text=True,timeout=50)
 if p.returncode: raise RuntimeError(' '.join(command)+'\n'+p.stderr[-8000:])
 return p
tests=[]
for mode,flags in [('normal',[]),('address_undefined',['-fsanitize=address,undefined','-fno-omit-frame-pointer'])]:
 run(['gcc','-std=c11','-O2','-g',*flags,'-c','app/dsp/fixed_dsp.c','-o',mode+'.o'])
 run(['g++','-std=c++17','-O2','-g',*flags,'-Ihost','-Iapp/dsp','-Iapp/storage','-pthread','test_audio_cpp.cpp',mode+'.o','-o',mode])
 p=run(['./'+mode])
 tests.append({'mode':mode,'passed':True,'output':p.stdout.strip()})
 # Check the scalar C kernel, not display-only float compatibility accessors.
 symbols=run(['nm','-u',mode+'.o']).stdout
 assert not any(v in symbols for v in ['__addsf','__mulsf','__divsf','sinf','cosf','powf','logf'])
try:
 flags=['-fsanitize=thread','-fno-omit-frame-pointer']
 run(['gcc','-std=c11','-O1','-g',*flags,'-c','app/dsp/fixed_dsp.c','-o','thread.o'])
 run(['g++','-std=c++17','-O1','-g',*flags,'-Ihost','-Iapp/dsp','-Iapp/storage','-pthread','test_audio_cpp.cpp','thread.o','-o','thread'])
 p=subprocess.run(['./thread'],capture_output=True,text=True,timeout=50)
 if p.returncode and ('unexpected memory mapping' in p.stderr or 'incompatible memory layout' in p.stderr):
  tests.append({'mode':'thread','status':'runtime_unavailable','error':p.stderr[-2000:]})
 elif p.returncode: raise RuntimeError(p.stderr[-8000:] or 'TSan process failed')
 else: tests.append({'mode':'thread','passed':True,'output':p.stdout.strip()})
except subprocess.TimeoutExpired:
 tests.append({'mode':'thread','status':'runtime_timeout'})
report={'passed':True,'scope':'host thread and numerical memory-safety tests with mocked NVS; no flash/power-loss/acoustic proof','tests':tests}
(root/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
'''


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
        _, stdout, _ = client.exec_command("python3 -c 'import os; print(os.path.expanduser(\"~/.cache/native_bt/audio_tests\"))'", timeout=8)
        remote = stdout.read().decode().strip()
        if not remote.startswith("/home/") or not remote.endswith("/.cache/native_bt/audio_tests"):
            raise RuntimeError("Unexpected remote workspace")
        dirs = [remote + suffix for suffix in ["/app/dsp", "/app/config", "/app/storage", "/host"]]
        _, stdout, stderr = client.exec_command("mkdir -p " + " ".join(shlex.quote(value) for value in dirs), timeout=8)
        if stdout.channel.recv_exit_status(): raise RuntimeError(stderr.read().decode())
        sftp = client.open_sftp()
        hashes = {}
        files = {"app/dsp/" + name: ROOT.parent / "bt_audio_sink/main/dsp" / name
                 for name in ["fixed_dsp.c", "fixed_dsp.h", "fixed_dsp_tables.inc", "dsp_processor.h"]}
        files.update({"app/config/app_config.h": ROOT.parent / "bt_audio_sink/main/config/app_config.h",
                      "app/storage/nvs_settings.h": ROOT.parent / "bt_audio_sink/main/storage/nvs_settings.h",
                      "test_audio_cpp.cpp": ROOT / "tests/test_audio_cpp.cpp"})
        for name, path in files.items():
            data = path.read_bytes(); hashes[name] = hashlib.sha256(data).hexdigest()
            with sftp.file(remote + "/" + name, "wb") as handle: handle.write(data)
        for name, content in STUBS.items():
            with sftp.file(remote + "/host/" + name, "w") as handle: handle.write(content)
        stdin, stdout, stderr = client.exec_command("cd " + shlex.quote(remote) + " && python3 -", timeout=150)
        stdin.write(DRIVER); stdin.flush(); stdin.channel.shutdown_write()
        output = stdout.read(100000).decode(); errors = stderr.read(100000).decode()
        if stdout.channel.recv_exit_status(): raise RuntimeError(errors or output)
        with sftp.file(remote + "/results.json", "r") as handle: report = json.loads(handle.read())
        report["source_sha256"] = hashes
        (ROOT / "_build/dsp/linux_results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(output)
        sftp.close()
    finally:
        password = None
        client.close()


if __name__ == "__main__": main()
