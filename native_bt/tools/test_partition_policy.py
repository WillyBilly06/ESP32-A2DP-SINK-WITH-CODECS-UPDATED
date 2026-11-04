"""Offline partition/boot policy; no serial imports or device operations."""
import ctypes,hashlib,json,os,pathlib,shutil,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;out=root/'_build/partition_policy';out.mkdir(exist_ok=True)
cc=shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe'
export=out/'exports.c';export.write_text('#ifdef _WIN32\n__declspec(dllexport)\n#endif\nint policy(unsigned char t,unsigned char s,const char*l,const char*p){extern _Bool dbt_boot_owned_slot(unsigned char,unsigned char,const char*,const char*);return dbt_boot_owned_slot(t,s,l,p);}')
lib=out/('policy.dll' if os.name=='nt' else 'policy.so');flags=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry'] if os.name=='nt' else ['-shared','-fPIC']
subprocess.run([cc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-I',str(root/'include')]+flags+[str(root/'src/dbt_boot_policy.c'),str(export),'-o',str(lib)],check=True)
f=ctypes.CDLL(str(lib)).policy;f.argtypes=[ctypes.c_ubyte,ctypes.c_ubyte,ctypes.c_char_p,ctypes.c_char_p]
for subtype,label in ((16,b'ota_0'),(17,b'ota_1')):assert f(0,subtype,label,b'native_bt_probe')==1
for values in ((0,18,b'recovery',b'native_bt_probe'),(0,16,b'ota_1',b'native_bt_probe'),(0,17,b'ota_1',b'bt_audio_sink'),(0,16,b'ota_0',b'btstack_speaker'),(1,17,b'ota_1',b'native_bt_probe'),(0,17,None,b'native_bt_probe'),(0,17,b'ota_1',None)):
    assert f(*values)==0,values
build=root/'_build/esp32';table=build/'partition_table/partition-table.bin';data=table.read_bytes();backup=(root/'_build/hardware/COM6-original-flash.bin').read_bytes();assert data==backup[0x8000:0x8000+len(data)]
ninja=shutil.which('ninja') or str(pathlib.Path(os.environ['USERPROFILE'])/'.espressif/tools/ninja/1.12.1/ninja.exe')
for target in ('flash','app-flash','bootloader-flash','partition-table-flash'):
    graph=subprocess.check_output([ninja,'-C',str(build),'-t','graph',target],text=True);assert 'label="native_bt_sdk_flash_guard"' in graph,target
rules=(build/'build.ninja').read_text();assert 'partition --type app --subtype ota_1' in rules
# Size-only probe proves capacity is checked against the intended slot, not recovery.
probe=out/'size-probe.bin';size_tool=ws/'esp-idf/components/partition_table/check_sizes.py'
base=[sys.executable,str(size_tool),'--offset','0x8000','partition','--type','app','--subtype']
def size_result(subtype,size):
    with probe.open('wb') as stream:stream.truncate(size)
    return subprocess.run(base+[subtype,str(table),str(probe)],capture_output=True,text=True).returncode
assert size_result('ota_1',0x100010)==0
assert size_result('ota_2',0x100010)!=0
assert size_result('ota_1',0x280001)!=0
with probe.open('wb') as stream:stream.truncate(0)
# Execute only the inert guard itself, never a flash target.
blocked=subprocess.run([sys.executable,str(root/'tools/reject_sdk_flash.py')],capture_output=True,text=True);assert blocked.returncode!=0 and 'No device was accessed' in blocked.stderr
report={'passed':True,'scope':'offline boot/partition metadata and inert flash guard only','owned_slots_verified':2,'recovery_and_legacy_rejected':True,'preserved_table_matches':True,'table_sha256':hashlib.sha256(data).hexdigest(),'sdk_flash_targets_guarded':4,'capacity_boundary_cases':3,'app_size_check_subtype':'ota_1','hardware_accessed':False}
(out/'results.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
