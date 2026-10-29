"""Validate an ESP32 build artifact offline. This tool never opens a serial port."""
import argparse,hashlib,io,json,struct
from pathlib import Path
from esptool.bin_image import ESP32FirmwareImage
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--image',type=Path,default=ROOT/'_build/esp32/native_bt_probe.bin');a=p.parse_args()
blob=a.image.read_bytes();assert len(blob)>256 and blob[0]==0xe9
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
config=(a.image.parent/'config/sdkconfig.h').read_text()
assert '#define CONFIG_BT_CONTROLLER_ONLY 1' in config
assert '#define CONFIG_BTDM_CTRL_MODE_BTDM 1' in config
assert '#define CONFIG_BT_BLUEDROID_ENABLED 1' not in config
assert '#define CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK 1' not in config
receipt=json.loads((ROOT/'_build/hardware/COM6-original-flash.json').read_text())
slot=next(x for x in receipt['partitions'] if x['name']=='ota_1')
assert slot['offset']==0x390000 and slot['size']==0x280000 and len(blob)<=slot['size']
assert image.min_rev_full<=301<=image.max_rev_full
report={'passed':True,'scope':'offline image checksum/digest, ESP32 revision and admitted OTA capacity; no device access or runtime qualification',
 'image_sha256':hashlib.sha256(blob).hexdigest(),'image_bytes':len(blob),'project':project,'chip_id':image.chip_id,
 'revision_range':[image.min_rev_full,image.max_rev_full],'checksum_verified':True,'digest_verified':bool(image.append_digest),
 'controller_only_dual_mode':True,'anti_rollback_fuse_updates':False,'partition_offset':slot['offset'],'partition_bytes':slot['size'],
 'partition_remaining_bytes':slot['size']-len(blob),'hardware_accessed':False,'runtime_qualified':False}
output=ROOT/'_build/host/local-image-validation.json';output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
