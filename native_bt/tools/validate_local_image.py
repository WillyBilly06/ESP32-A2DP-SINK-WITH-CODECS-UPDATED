"""Validate an ESP32 build artifact offline. This tool never opens a serial port."""
import argparse,hashlib,io,json,re,struct
from pathlib import Path
from esptool.bin_image import ESP32FirmwareImage
from source_snapshot import verify_manifest
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--image',type=Path,default=ROOT/'_build/esp32/native_bt_probe.bin');a=p.parse_args()
build_status=json.loads((a.image.parent/'local-build-status.json').read_text(encoding='utf-8-sig'))
assert build_status['passed'] and build_status['state']=='complete', 'Latest build did not complete; refusing a possibly stale binary'
source_count=verify_manifest(a.image.parent/'local-build-sources.json',build_status['source_manifest_sha256'])
commands=json.loads((a.image.parent/'compile_commands.json').read_text(encoding='utf-8'))
diagnostic=bool(build_status.get('codec_diagnostic_only'))
entry_name='probe_codec_diagnostic.c' if diagnostic else 'probe_main.c'
main_command=next(entry['command'] for entry in commands if Path(entry['file']).name==entry_name)
cache=(a.image.parent/'CMakeCache.txt').read_text()
assert ('DBT_PROBE_CODEC_DIAGNOSTIC_ONLY:BOOL=ON' in cache)==diagnostic
assert ('DBT_PROBE_LHDC_MAX_CHECK:BOOL=ON' in cache)==bool(build_status.get('lhdc_maximum_check'))
assert ('DBT_PROBE_LHDC_STAGE_PROFILE:BOOL=ON' in cache)==bool(build_status.get('lhdc_stage_profile'))
if diagnostic:
    for feature in ('KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','SMP','AVRCP','RFCOMM','AUDIO_TEST','AUDIO_OUTPUT','GATT_TEST','CLASSIC_SECURITY_TEST','INGRESS_TRACE'):
        assert f'DBT_PROBE_{feature}:BOOL=ON' not in cache, feature
product_security=bool(re.search(r'(?:^|\s)-DDBT_PROBE_PRODUCT_SECURITY=1(?:\s|$)',main_command))
rfcomm=bool(re.search(r'(?:^|\s)-DDBT_PROBE_RFCOMM=1(?:\s|$)',main_command))
assert product_security==build_status['product_security'] and rfcomm==build_status['rfcomm']
blob=a.image.read_bytes();assert len(blob)>256 and blob[0]==0xe9
assert hashlib.sha256(blob).hexdigest()==build_status['image_sha256']
image=ESP32FirmwareImage(io.BytesIO(blob));image.verify();assert image.chip_id==0
position=24;checksum=0xef
for _ in range(blob[1]):
    assert position+8<=len(blob)
    address,length=struct.unpack_from('<II',blob,position);position+=8;assert position+length<=len(blob)
    for value in blob[position:position+length]:checksum^=value
    position+=length
checksum_at=position+((15-position)%16);assert blob[checksum_at]==checksum
body_end=checksum_at+1
if image.append_digest:assert hashlib.sha256(blob[:body_end]).digest()==blob[body_end:body_end+32]
assert blob[32:36]==b'\x32\x54\xcd\xab'
project=blob[80:112].split(b'\0',1)[0].decode();assert project=='native_bt_probe'
config_bytes=(a.image.parent/'config/sdkconfig.h').read_bytes()
assert hashlib.sha256(config_bytes).hexdigest()==build_status['config_sha256']
config=config_bytes.decode()
assert '#define CONFIG_BT_CONTROLLER_ONLY 1' in config
assert '#define CONFIG_BTDM_CTRL_MODE_BTDM 1' in config
assert '#define CONFIG_BT_BLUEDROID_ENABLED 1' not in config
assert '#define CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK 1' not in config
receipt=json.loads((ROOT/'_build/hardware/COM6-original-flash.json').read_text())
slot=next(x for x in receipt['partitions'] if x['name']=='ota_1')
assert slot['offset']==0x390000 and slot['size']==0x280000 and len(blob)<=slot['size']
table=(a.image.parent/'partition_table/partition-table.bin').read_bytes()
assert hashlib.sha256(table).hexdigest()==build_status['partition_sha256']
original=(ROOT/'_build/hardware/COM6-original-flash.bin').read_bytes()
assert hashlib.sha256(original).hexdigest()==receipt['sha256']
assert table==original[0x8000:0x8000+len(table)]
assert image.flash_mode==2 and image.flash_size_freq==0x3f
assert image.min_rev_full<=301<=image.max_rev_full
report={'passed':True,'scope':'offline image checksum/digest, ESP32 revision and admitted OTA capacity; no device access or runtime qualification',
 'image_sha256':hashlib.sha256(blob).hexdigest(),'image_bytes':len(blob),'project':project,'chip_id':image.chip_id,'codec_diagnostic_only':diagnostic,
 'revision_range':[image.min_rev_full,image.max_rev_full],'checksum_verified':True,'digest_verified':bool(image.append_digest),
 'latest_local_build_completed':True,'build_sources_match':True,'build_source_units':source_count,'product_io':bool(build_status.get('product_io')),'product_security':product_security,'rfcomm_profile':rfcomm,'controller_only_dual_mode':True,'anti_rollback_fuse_updates':False,'partition_offset':slot['offset'],'partition_bytes':slot['size'],
 'partition_remaining_bytes':slot['size']-len(blob),'compiled_table_matches_preserved_layout':True,'compiled_table_sha256':hashlib.sha256(table).hexdigest(),'flash_mode':'DIO','flash_mib':8,'flash_mhz':80,'hardware_accessed':False,'runtime_qualified':False}
output=ROOT/'_build/host/local-image-validation.json';output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
