"""Offline RFCOMM tests, with fixed outputs and workspace budget checks."""
import ctypes, hashlib, json, os, shutil, subprocess, sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
files=['include/dbt_rfcomm.h','include/dbt_rfcomm_mux.h','include/dbt_rfcomm_credit.h','include/dbt_rfcomm_bearer.h','include/dbt_rfcomm_dlc.h','include/dbt_rfcomm_spp.h','src/dbt_rfcomm.c','src/dbt_rfcomm_mux.c','src/dbt_rfcomm_credit.c','src/dbt_rfcomm_bearer.c','src/dbt_rfcomm_dlc.c','src/dbt_rfcomm_spp.c','tests/rfcomm/wire.c','tests/rfcomm/mux.c','tests/rfcomm/credit.c','tests/rfcomm/bearer.c','tests/rfcomm/dlc.c','esp32_probe/main/probe_rfcomm.c','esp32_probe/main/probe_rfcomm.h','tools/test_rfcomm.py']
tests=['rfcomm_wire_vectors','rfcomm_wire_lengths_and_credits','rfcomm_wire_transaction_and_corruption','rfcomm_mcc_bounded_and_fuzz','rfcomm_mux_pair_and_close','rfcomm_mux_identity_backpressure','rfcomm_mux_admission_timeout_and_role','rfcomm_mux_crossed_disconnect','rfcomm_pn_and_credit','rfcomm_bearer_runtime_pair','rfcomm_bearer_overflow_and_link_loss','rfcomm_bearer_encrypted_dlc_pair','rfcomm_dlc_mcc_legacy_flow_and_nsc','rfcomm_bearer_bind_oom','rfcomm_bearer_legacy_mode','rfcomm_spp_endpoint_and_sdp','rfcomm_dlc_pn_duplicate_and_deadline','rfcomm_dlc_reverse_direction','rfcomm_probe_echo_restart']
runtime_names=['dbt_queue','dbt_hci','dbt_host','dbt_acl','dbt_l2cap','dbt_channels','dbt_credit','dbt_ertm','dbt_ertm_engine','dbt_runtime','dbt_rx_flow','dbt_sdp']
files += [f'src/{name}.c' for name in runtime_names] + [f'include/{name}.h' for name in runtime_names]
if os.name=='nt':files += ['tests/avrcp/runtime_freestanding.c']
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    before={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in files}
    folder=root/'_build/rfcomm';folder.mkdir(parents=True,exist_ok=True)
    compiler=shutil.which('clang') or shutil.which('gcc') or r'C:\Program Files\LLVM\bin\clang.exe'
    binary=folder/('wire.dll' if os.name=='nt' else 'wire.so')
    args=[compiler,'-std=c11','-O1','-Wall','-Wextra','-Werror','-I',str(root/'include'),'-I',str(root/'esp32_probe/main'),str(root/'esp32_probe/main/probe_rfcomm.c'),str(root/'src/dbt_rfcomm.c'),str(root/'src/dbt_rfcomm_mux.c'),str(root/'src/dbt_rfcomm_credit.c'),str(root/'src/dbt_rfcomm_bearer.c'),str(root/'src/dbt_rfcomm_dlc.c'),str(root/'src/dbt_rfcomm_spp.c'),str(root/'tests/rfcomm/wire.c'),str(root/'tests/rfcomm/mux.c'),str(root/'tests/rfcomm/credit.c'),str(root/'tests/rfcomm/bearer.c'),str(root/'tests/rfcomm/dlc.c')]
    args += [str(root/'src'/f'{name}.c') for name in runtime_names]
    if os.name=='nt':args.append(str(root/'tests/avrcp/runtime_freestanding.c'))
    if os.name=='nt':args+=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry']
    else:args+=['-shared','-fPIC']
    subprocess.run(args+['-o',str(binary)],check=True)
    lib=ctypes.CDLL(str(binary))
    for name in tests:
        result=getattr(lib,name)()
        if result:raise RuntimeError(f'{name} failed at line {result}')
        print('PASS',name)
    sanitized=False
    if os.name!='nt':
        main=folder/'asan_main.c'
        main.write_text(''.join(f'int {name}(void);\n' for name in tests)+'int main(void){\n'+''.join(f'if({name}())return 1;\n' for name in tests)+'return 0;}\n')
        sources=[str(root/'src'/f'{name}.c') for name in runtime_names]
        sources += [str(root/'src'/f'{name}.c') for name in ('dbt_rfcomm','dbt_rfcomm_mux','dbt_rfcomm_credit','dbt_rfcomm_dlc','dbt_rfcomm_bearer','dbt_rfcomm_spp')]
        sources += [str(root/'tests/rfcomm'/f'{name}.c') for name in ('wire','mux','credit','dlc','bearer')]
        sources.append(str(root/'esp32_probe/main/probe_rfcomm.c'))
        exe=folder/'sanitized'
        subprocess.run([compiler,'-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-omit-frame-pointer','-I',str(root/'include'),'-I',str(root/'esp32_probe/main')]+sources+[str(main),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=60);sanitized=True
    after={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in files}
    if before!=after:raise RuntimeError('Sources changed during build')
    (folder/'results.json').write_text(json.dumps({'passed':True,'scope':'offline one-DLC RFCOMM, opt-in SPP endpoint and SDP, simulated runtimes; no target or RF','tests':tests,'address_undefined_sanitizers':sanitized,'source_sha256':before},indent=2)+'\n')
finally:
    subprocess.run(check,check=True)
