import ctypes, os, pathlib, shutil, subprocess
root=pathlib.Path(__file__).resolve().parents[1];out=root/'_build'/'led';out.mkdir(parents=True,exist_ok=True)
cc=os.environ.get('CC') or shutil.which('clang') or shutil.which('cc')
if not cc and os.name=='nt':cc=r'C:\Program Files\LLVM\bin\clang.exe'
lib=out/('input_platform.dll' if os.name=='nt' else 'input_platform.so')
args=[cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-I',str(root/'tests/led/fake'),'-I',str(root/'include')]
if os.name=='nt':args+=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry']
else:args+=['-shared','-fPIC']
args += [str(root/'src/dbt_input.c'),str(root/'src/esp32/dbt_input_esp32.c'),str(root/'tests/led/input_platform_test.c'),'-o',str(lib)]
subprocess.run(args,check=True)
dll=ctypes.CDLL(str(lib))
for name in ('test_input_identity','test_input_platform'):
 r=getattr(dll,name)()
 if r:raise SystemExit(f'FAIL {name} line {r}')
 print('PASS',name)
