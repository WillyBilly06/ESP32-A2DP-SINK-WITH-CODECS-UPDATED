"""Identity-checked, read-only full flash backup before live-test deployment."""
import argparse
import binascii
import hashlib
import json
from pathlib import Path
import struct
import sys
import serial.tools.list_ports
import esptool

ROOT=Path(__file__).resolve().parents[1]

def description(image,offset):
    data=image[offset+32:offset+32+256]
    if len(data)<176 or int.from_bytes(data[:4],'little')!=0xabcd5432:return None
    def text(start,size):return data[start:start+size].split(b'\0',1)[0].decode('ascii',errors='replace')
    return {'version':text(16,32),'project_name':text(48,32),'time':text(80,16),'date':text(96,16),
            'idf_version':text(112,32),'elf_sha256':data[144:176].hex()}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--port',required=True);parser.add_argument('--usb-serial',required=True)
    args=parser.parse_args()
    if args.port.upper()!='COM6':raise RuntimeError('Only the correlated speaker port COM6 is admitted')
    matches=[p for p in serial.tools.list_ports.comports() if p.device.upper()==args.port.upper() and p.serial_number==args.usb_serial]
    if len(matches)!=1:raise RuntimeError('USB identity no longer matches the correlated speaker')
    output=ROOT/'_build/hardware';output.mkdir(exist_ok=True,parents=True)
    destination=output/'COM6-original-flash.bin'
    if destination.exists():raise RuntimeError('Original backup already exists; preserve it and inspect its receipt')
    controller=None
    try:
        controller=esptool.detect_chip(port=args.port,baud=115200,connect_mode='default_reset')
        if controller.CHIP_NAME!='ESP32':raise RuntimeError('Unexpected chip family')
        meta={'port':args.port,'usb_serial':args.usb_serial,'usb_location':matches[0].location,
              'chip':controller.get_chip_description(),'mac':':'.join(f'{b:02X}' for b in controller.read_mac()),
              'secure_boot':bool(controller.get_secure_boot_enabled()),
              'flash_encryption':bool(controller.get_flash_encryption_enabled()),'flash_writes':0}
        if meta['secure_boot'] or meta['flash_encryption']:raise RuntimeError('Protected flash requires its signed/encrypted deployment workflow')
        controller=controller.run_stub();controller.change_baud(921600)
        jedec=controller.flash_id();exponent=(jedec>>16)&255
        if not 20<=exponent<=25:raise RuntimeError('Unrecognized or out-of-budget flash capacity')
        size=1<<exponent;meta['flash_id']=f'{jedec:06x}';meta['flash_bytes']=size
        last=[-1]
        def progress(position,total):
            mib=position//1048576
            if mib!=last[0]:print(f'Read {mib} MiB of {total//1048576} MiB',flush=True);last[0]=mib
        image=controller.read_flash(0,size,progress_fn=progress)
        if len(image)!=size:raise RuntimeError('Short flash backup')
        verification=controller.read_flash(0,0x10000)
        if verification!=image[:0x10000]:raise RuntimeError('Readback prefix does not match the full backup')
        destination.write_bytes(image)
        meta['sha256']=hashlib.sha256(image).hexdigest();meta['prefix_readback_matched']=True
        sys.path.insert(0,str(ROOT.parent/'esp-idf/components/partition_table'))
        import gen_esp32part as gen
        table=gen.PartitionTable.from_binary(image[0x8000:0x9000])
        partitions=[]
        for entry in table:
            item={'name':entry.name,'type':entry.type,'subtype':entry.subtype,'offset':entry.offset,'size':entry.size}
            if entry.offset+entry.size>size:raise RuntimeError('Existing partition extends beyond physical flash')
            if entry.type==0:item['app']=description(image,entry.offset)
            if entry.type==1 and entry.subtype==0:
                records=[]
                for index in [0,1]:
                    data=image[entry.offset+index*4096:entry.offset+index*4096+32]
                    sequence=int.from_bytes(data[:4],'little');crc=int.from_bytes(data[28:32],'little')
                    records.append({'sequence':sequence,'state':int.from_bytes(data[24:28],'little'),
                                    'crc_valid':sequence!=0xffffffff and crc==(binascii.crc32(data[:4],0xffffffff)&0xffffffff)})
                item['ota_records']=records
            partitions.append(item)
        meta['partitions']=partitions
        (output/'COM6-original-flash.json').write_text(json.dumps(meta,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(meta,indent=2))
    finally:
        if controller:
            try:controller.hard_reset()
            finally:controller._port.close()

if __name__=='__main__':main()
