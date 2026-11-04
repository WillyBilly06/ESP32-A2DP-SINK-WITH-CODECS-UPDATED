"""Write only the identified speaker's trial OTA slot and one boot-selector sector."""
import binascii
import argparse
import hashlib
import io
import json
from pathlib import Path
import struct
import re
import sys
import time
import serial.tools.list_ports
import esptool
from esptool.bin_image import ESP32FirmwareImage

ROOT=Path(__file__).resolve().parents[1]
HARDWARE=ROOT/'_build/hardware'

def native_bond_entries(data):
    # Read-only SDK parser. Return counts only; never print or persist key data.
    sys.path.insert(0,str(ROOT.parent/'esp-idf/components/nvs_flash/nvs_partition_tool'))
    from nvs_parser import NVS_Partition
    partition=NVS_Partition('nvs',bytearray(data))
    pages=[page for page in partition.pages if page.header['status'] in ('Active','Full')]
    entries=[entry for page in pages for entry in page.entries if entry.state=='Written']
    namespaces=[entry for entry in entries if entry.metadata['namespace']==0 and entry.key=='native_bonds']
    for entry in namespaces:
        if entry.metadata['crc']['original']!=entry.metadata['crc']['computed']:
            raise RuntimeError('Owned bond namespace CRC is invalid')
    ids={entry.data['value'] for entry in namespaces}
    if len(ids)>1:raise RuntimeError('Owned bond namespace is ambiguous')
    owned=[entry for entry in entries if entry.metadata['namespace'] in ids]
    return {'scope':'read-only live flash SDK NVS parse; counts only',
            'namespace_present':bool(ids),'written_entries':len(owned)}

def write_region(controller,offset,data,limit):
    block=1024
    if offset%4096 or offset+((len(data)+4095)//4096)*4096>limit:
        raise RuntimeError('Write/erase extent is outside the verified partition')
    controller.FLASH_WRITE_SIZE=block
    count=controller.flash_begin(len(data),offset)
    for number in range(count):
        chunk=data[number*block:(number+1)*block].ljust(block,b'\xff')
        if offset+(number+1)*block>limit:raise RuntimeError('Flash packet crosses the admitted boundary')
        controller.flash_block(chunk,number)
        if number%256==0:print(f'Writing offset 0x{offset:x}: {number*block}/{len(data)} bytes',flush=True)
    controller.flash_finish(reboot=False)
    actual=controller.read_flash(offset,len(data))
    if actual!=data:raise RuntimeError('Full written-region readback mismatch')
    return {'offset':offset,'bytes':len(data),'readback_sha256':hashlib.sha256(actual).hexdigest(),'readback_matches':True}

def main():
    sys.stdout.reconfigure(encoding='utf-8',errors='backslashreplace')
    parser=argparse.ArgumentParser()
    parser.add_argument('--kind',choices=['audio','probe'],default='probe')
    parser.add_argument('--keep-owned',action='store_true')
    args=parser.parse_args()
    hold=HARDWARE/'HARDWARE_HOLD.json'
    if hold.exists() and json.loads(hold.read_text()).get('active',True):
        raise RuntimeError('ESP32 reserved by user: deployment/reset disabled; perform off-board builds/tests only')
    if args.keep_owned and args.kind!='probe':raise RuntimeError('Persistent owned selection requires the owned probe image')
    original=(HARDWARE/'COM6-original-flash.bin').read_bytes()
    receipt=json.loads((HARDWARE/'COM6-original-flash.json').read_text())
    if hashlib.sha256(original).hexdigest()!=receipt['sha256']:raise RuntimeError('Original backup integrity failed')
    build_name,project_name=('audio_app','bt_audio_sink') if args.kind=='audio' else ('esp32','native_bt_probe')
    image_path=ROOT/'_build'/build_name/(project_name+'.bin');candidate=image_path.read_bytes()
    configuration=(image_path.parent/'config/sdkconfig.h').read_text()
    baud_match=re.search(r'^#define CONFIG_ESP_CONSOLE_UART_BAUDRATE (\d+)$',configuration,re.M)
    if not baud_match:raise RuntimeError('Console baud is missing from the actual build configuration')
    console_baud=int(baud_match.group(1))
    cache=(image_path.parent/'CMakeCache.txt').read_text()
    if args.keep_owned and 'DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' not in cache:raise RuntimeError('Candidate was not built to keep the owned host active')
    if args.keep_owned and '#define CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK 1' in configuration:raise RuntimeError('No anti-rollback fuse changes are admitted')
    destination=HARDWARE/('COM6-owned-active' if args.keep_owned else 'COM6-'+args.kind+'-trial')
    image=ESP32FirmwareImage(io.BytesIO(candidate));image.verify()
    if candidate[32:36]!=b'\x32\x54\xcd\xab' or candidate[80:112].split(b'\0',1)[0]!=project_name.encode():
        raise RuntimeError('Unexpected candidate descriptor')
    partitions=receipt['partitions']
    active=next(p for p in partitions if p['name']=='ota_0')
    trial=next(p for p in partitions if p['name']=='ota_1')
    selector=next(p for p in partitions if p['name']=='otadata')
    if trial['offset']!=0x390000 or trial['size']!=0x280000 or len(candidate)>trial['size']:
        raise RuntimeError('Trial image does not fit the admitted inactive slot')
    usb=[p for p in serial.tools.list_ports.comports() if p.device=='COM6' and p.serial_number==receipt['usb_serial']]
    if len(usb)!=1:raise RuntimeError('Speaker USB identity changed')
    controller=None;rebooted=False
    audio_probe=args.kind=='probe' and 'DBT_PROBE_AUDIO_TEST:BOOL=ON' in (image_path.parent/'CMakeCache.txt').read_text()
    output_probe=audio_probe and 'DBT_PROBE_AUDIO_OUTPUT:BOOL=ON' in (image_path.parent/'CMakeCache.txt').read_text()
    scope='Bluedroid audio application trial' if args.kind=='audio' else (
        'Experimental native AVDTP/SBC/integer-DSP/I2S Audio Sink candidate' if output_probe else
        'Independent AVDTP/SBC decoder diagnostic; PCM counters only, no I2S/product audio' if audio_probe else
        'Independent host probe; optional diagnostic ATT/GATT; no product/audio profiles')
    report={'scope':scope,'console_baud':console_baud,
            'port':'COM6','backup_sha256':receipt['sha256'],'candidate_sha256':hashlib.sha256(candidate).hexdigest(),
            'candidate_elf_sha256':candidate[176:208].hex(),'writes':[]}
    try:
        controller=esptool.detect_chip(port='COM6',baud=115200,connect_mode='default_reset')
        mac=':'.join(f'{v:02X}' for v in controller.read_mac())
        if controller.CHIP_NAME!='ESP32' or mac!=receipt['mac']:raise RuntimeError('Chip/MAC identity mismatch')
        if controller.get_secure_boot_enabled() or controller.get_flash_encryption_enabled():raise RuntimeError('Unexpected protection state')
        controller=controller.run_stub();controller.change_baud(921600)
        if f'{controller.flash_id():06x}'!=receipt['flash_id']:raise RuntimeError('Flash chip changed')
        if controller.read_flash(0,0x9000)!=original[:0x9000]:raise RuntimeError('Bootloader/partition table differs from the preserved original')
        preserved=controller.read_flash(active['offset'],active['size'])
        if preserved!=original[active['offset']:active['offset']+active['size']]:
            raise RuntimeError('Original application slot changed; inspect before deployment')
        current=controller.read_flash(selector['offset'],selector['size'])
        nvs=next(p for p in partitions if p['name']=='nvs')
        report['native_bond_flash_state']=native_bond_entries(controller.read_flash(nvs['offset'],nvs['size']))
        sequences=[]
        for position in [0,4096]:
            row=current[position:position+32];seq=int.from_bytes(row[:4],'little');crc=int.from_bytes(row[28:32],'little')
            if seq!=0xffffffff and crc==(binascii.crc32(row[:4],0xffffffff)&0xffffffff):sequences.append(seq)
        ota_apps=sorted([p for p in partitions if p['type']==0 and 16<=p['subtype']<32],key=lambda p:p['subtype'])
        target_index=next(i for i,p in enumerate(ota_apps) if p['name']=='ota_1')
        sequence=max(sequences,default=0)+1
        while (sequence-1)%len(ota_apps)!=target_index:sequence+=1
        if sequence>=0xffffffff:raise RuntimeError('OTA sequence exhausted')
        report['writes'].append(write_region(controller,trial['offset'],candidate,trial['offset']+trial['size']))
        record=bytearray(b'\xff'*4096)
        struct.pack_into('<I',record,0,sequence);struct.pack_into('<I',record,24,2 if args.keep_owned else 0)
        struct.pack_into('<I',record,28,binascii.crc32(record[:4],0xffffffff)&0xffffffff)
        # Preserve the original valid selector sector, old app, bootloader and user data.
        report['writes'].append(write_region(controller,selector['offset']+4096,record,selector['offset']+selector['size']))
        report['ota_sequence']=sequence;report['trial_offset']=trial['offset']
        report['selector_state_written']='VALID development selection' if args.keep_owned else 'NEW trial'
        report['production_qualification']=False
        report['original_application_preserved']=controller.read_flash(active['offset'],active['size'])==preserved
        if not report['original_application_preserved']:raise RuntimeError('Original application verification failed')
        controller._port.baudrate=console_baud
        controller.hard_reset();rebooted=True
        boot=bytearray();deadline=time.monotonic()+15
        controller._port.timeout=0.2
        while time.monotonic()<deadline and len(boot)<65536:
            boot.extend(controller._port.read(min(4096,65536-len(boot))))
        destination.with_suffix('.bin').write_bytes(boot)
        text=boot.decode('utf-8',errors='backslashreplace')
        destination.with_suffix('.txt').write_text(text,encoding='utf-8')
        report['boot_ready_marker']=('audio_candidate_integer_dsp1 slot=0x390000' in text) if args.kind=='audio' else (
            '"probe":"controller_identified"' in text and '"probe":"gap_state","state":2,' in text)
        if audio_probe:report['boot_ready_marker']=report['boot_ready_marker'] and '"probe":"sbc_probe_ready"' in text
        if output_probe:report['boot_ready_marker']=report['boot_ready_marker'] and '"audio_sink_sdp_advertised":true' in text
        if args.keep_owned:
            records=[]
            for line in text.splitlines():
                try:
                    event=json.loads(line)
                    if event.get('probe')=='boot_selection':records.append(event)
                except (ValueError,AttributeError):pass
            report['owned_boot_confirmation']=next((r for r in records if r.get('kept_active') is True and r.get('ota_state')==2 and r.get('partition')==trial['offset']),None)
            report['boot_ready_marker']=report['boot_ready_marker'] and report['owned_boot_confirmation'] is not None
            report['automatic_bluedroid_restore']=False
        if args.kind=='probe' and 'DBT_PROBE_OPUS_BUILD:BOOL=ON' in cache:
            checks=[]
            for line in text.splitlines():
                try:
                    event=json.loads(line)
                    if event.get('probe')=='target_opus_selftest':checks.append(event)
                except (ValueError,AttributeError):pass
            report['opus_target_check']=next((r for r in checks if r.get('passed') is True and r.get('frames')==48),None)
            report['boot_ready_marker']=report['boot_ready_marker'] and report['opus_target_check'] is not None
        if args.kind=='probe' and 'DBT_PROBE_SBC_BUILD:BOOL=ON' in cache:
            codec_checks=[]
            for line in text.splitlines():
                try:
                    event=json.loads(line)
                    if event.get('probe')=='target_codec_selftest':codec_checks.append(event)
                except (ValueError,AttributeError):pass
            full=all('DBT_PROBE_'+name+':BOOL=ON' in cache for name in ('SBC_BUILD','AAC_BUILD','APTX_BUILD','OPUS_BUILD','LDAC_BUILD','LHDC_BUILD'))
            report['all_codec_target_check']=next((r for r in codec_checks if r.get('passed') is True and r.get('configurations')==r.get('selected') and (not full or r.get('selected')==37)),None)
            report['boot_ready_marker']=report['boot_ready_marker'] and report['all_codec_target_check'] is not None
        fatal_tokens=('***ERROR***','Guru Meditation Error','abort() was called','"probe":"failed"','"probe":"parser_failure"')
        report['boot_fatal_marker']=any(token in text for token in fatal_tokens)
        report['boot_ready_marker']=report['boot_ready_marker'] and not report['boot_fatal_marker']
        report['boot_capture']=destination.with_suffix('.bin').name
        report['boot_tail']=text[-3000:]
        destination.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(report,indent=2))
        if not report['boot_ready_marker']:raise RuntimeError('Trial readiness was not observed; inspect preserved boot capture')
    finally:
        if controller:
            try:
                if not rebooted:controller.hard_reset()
            finally:controller._port.close()

if __name__=='__main__':main()
