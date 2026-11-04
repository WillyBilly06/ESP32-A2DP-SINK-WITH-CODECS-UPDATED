import ctypes, os, pathlib, shutil, subprocess
root=pathlib.Path(__file__).resolve().parents[1];out=root/'_build'/'led';out.mkdir(parents=True,exist_ok=True)
cc=os.environ.get('CC') or shutil.which('clang') or shutil.which('cc')
if not cc and os.name=='nt':cc=r'C:\Program Files\LLVM\bin\clang.exe'
lib=out/('led_platform.dll' if os.name=='nt' else 'led_platform.so')
args=[cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-I',str(root/'tests/led/fake'),'-I',str(root/'include')]
if os.name=='nt':args+=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry']
else:args+=['-shared','-fPIC']
args += [str(root/'src/dbt_led.c'),str(root/'tests/led/platform_test.c'),'-o',str(lib)]
subprocess.run(args,check=True);r=ctypes.CDLL(str(lib)).test_platform()
if r:raise SystemExit(f'FAIL line {r}')
print('PASS fake NVS failures and WS2812 SPI bit vectors')
