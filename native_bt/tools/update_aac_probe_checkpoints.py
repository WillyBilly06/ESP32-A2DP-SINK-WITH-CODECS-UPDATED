"""Regenerate only AAC target-checkpoint rows from verified portable host PCM.

The embedded LATM packets and every non-AAC byte of the header must remain
unchanged. Requires the corrected test_native_codec_aac.py receipt/DLLs and
the same encoded fixtures. This never changes a target error tolerance.
"""
import ctypes as c
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/native_codecs'
HEADER=ROOT/'esp32_probe/main/probe_codec_vectors.h'
RECEIPT=OUT/'aac_real_results.json'
REPORT=ROOT/'_build/aac_math/aac_checkpoints.json'
if os.name!='nt': raise SystemExit('This generator uses the verified Windows AAC host DLLs.')
ffmpeg=shutil.which('ffmpeg')
if not ffmpeg: raise SystemExit('FFmpeg is required for independent PCM verification.')
sys.path.insert(0,str(ROOT/'tools'))
import test_native_codec_aac as aac

def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def points(samples,channels):
    chosen=[]
    for ch in range(channels):
        indices=list(range(ch,len(samples),channels)); peak=max(abs(samples[i]) for i in indices)
        valid=[i for i in indices[len(indices)//4:] if abs(samples[i])>max(64,peak//8)]
        if len(valid)<12: raise RuntimeError('Insufficient non-silent AAC checkpoints')
        chosen.extend((valid[j*(len(valid)-1)//11],samples[valid[j*(len(valid)-1)//11]]) for j in range(12))
    return sorted(chosen)
def fnv(data):
    value=2166136261
    for byte in data: value=((value^byte)*16777619)&0xffffffff
    return value

receipt=json.loads(RECEIPT.read_text())
if not receipt.get('passed'): raise RuntimeError('AAC host verification receipt failed')
source_name='native_bt/codecs/aac/host_target_math.h'
if receipt['source_sha256'].get(source_name)!=digest(ROOT/'codecs/aac/host_target_math.h'):
    raise RuntimeError('AAC host receipt does not include current portable math selection')
for name,expected in receipt['source_sha256'].items():
    source=ROOT.parent/Path(name)
    if not source.is_file() or digest(source)!=expected: raise RuntimeError(f'Stale AAC source receipt: {name}')
native_path=OUT/'aac_real.dll'; reference_path=OUT/'aac_reference.dll'
native=c.CDLL(str(native_path)); reference=c.CDLL(str(reference_path)); u8=c.c_uint8; ptr=c.POINTER(u8)
native.probe_open.argtypes=[c.c_uint,c.c_uint]
native.probe_media.argtypes=[ptr,c.c_uint,c.c_uint,c.c_uint,c.c_uint,c.c_uint,ptr,c.POINTER(c.c_uint)]
reference.baseline_open.argtypes=[c.c_int,c.c_int]
reference.baseline_decode.argtypes=[ptr,c.c_int,c.POINTER(c.c_int16),c.POINTER(c.c_int)]
header_before=HEADER.read_bytes(); text=header_before.decode('utf-8'); original=text; changes=[]; report=[]
for index,(rate,channels) in enumerate(((44100,1),(44100,2),(48000,1),(48000,2)),1):
    fixture=OUT/'vectors'/f'tone_{rate}_{channels}.aac'
    if receipt['fixture_sha256'].get(fixture.name)!=digest(fixture): raise RuntimeError(f'Stale fixture {fixture.name}')
    frames=list(aac.adts(fixture))[:4]
    if len(frames)!=4: raise RuntimeError('AAC fixture needs four complete frames')
    packets=[]
    for number,frame in enumerate(frames):
        m=re.search(r'codec_packet_'+str(index)+'_'+str(number)+r'\[\]=\{([^}]*)\}',text)
        if not m: raise RuntimeError('Missing embedded AAC packet')
        packet=bytes(int(v) for v in m.group(1).split(',') if v.strip())
        if packet!=aac.latm(frame,rate,channels,number>0): raise RuntimeError('Embedded AAC packet differs from receipt fixture')
        packets.append(packet)
    if native.probe_open(rate,channels)!=0 or reference.baseline_open(rate,channels)!=0:
        raise RuntimeError('AAC decoder initialization failed')
    output=bytearray()
    try:
        for number,(packet,frame) in enumerate(zip(packets,frames)):
            payload=(u8*len(packet)).from_buffer_copy(packet); pcm=(u8*8192)(); size=c.c_uint()
            if native.probe_media(payload,len(packet),number*1024,number,1,number,pcm,c.byref(size))!=0:
                raise RuntimeError('Guarded AAC decode failed')
            original_frame=(u8*(len(frame)+4))(); c.memmove(original_frame,frame,len(frame))
            reference_pcm=(c.c_int16*2048)(); frame_count=c.c_int()
            if reference.baseline_decode(original_frame,len(frame),reference_pcm,c.byref(frame_count))!=0 or frame_count.value!=1024:
                raise RuntimeError('Original AAC decoder failed')
            if size.value!=2048*channels or bytes(pcm[:size.value])!=bytes(reference_pcm)[:size.value]:
                raise RuntimeError('Guarded AAC differs from original SDK decoder')
            output.extend(bytes(pcm[:size.value]))
    finally:
        native.probe_close(); reference.baseline_close()
    ffpcm=subprocess.check_output([ffmpeg,'-v','error','-i',str(fixture),'-f','s16le','-acodec','pcm_s16le','-'])
    if len(ffpcm)<len(output): raise RuntimeError('FFmpeg returned too few decoded samples')
    samples=[x[0] for x in struct.iter_unpack('<h',output)]
    independent=[x[0] for x in struct.iter_unpack('<h',ffpcm[:len(output)])]
    errors=[abs(a-b) for a,b in zip(samples,independent)]
    if max(errors)>16 or sum(errors)/len(errors)>=3: raise RuntimeError('AAC differs from independent FFmpeg PCM')
    checkpoint=points(samples,channels); peak=max(abs(value) for value in samples)
    pm=re.search(r'static const probe_pcm_point_t codec_points_'+str(index)+r'\[\]=\{[^;]+\};',text)
    if not pm: raise RuntimeError('Missing AAC checkpoint row')
    replacement='static const probe_pcm_point_t codec_points_'+str(index)+'[]={'+','.join('{%du,%d}'%p for p in checkpoint)+'};'
    changes.append((pm.group(0),replacement)); text=text[:pm.start()]+replacement+text[pm.end():]
    pattern=(r'(\{"aac_'+str(rate)+'_'+str(channels)+r'"[^\n]*,'+str(rate)+r'u,4096u,)'
             r'\d+u,\d+u(,1024,codec_points_'+str(index)+r',\d+\},)')
    vm=re.search(pattern,text)
    if not vm: raise RuntimeError('Missing AAC case row')
    newrow=vm.group(1)+str(fnv(output))+'u,'+str(peak//4)+'u'+vm.group(2)
    changes.append((vm.group(0),newrow)); text=text[:vm.start()]+newrow+text[vm.end():]
    report.append({'rate':rate,'channels':channels,'packet_sha256':[hashlib.sha256(x).hexdigest() for x in packets],
                   'pcm_sha256':hashlib.sha256(output).hexdigest(),'peak':peak,'minimum_peak':peak//4,
                   'ffmpeg_max_error':max(errors),'ffmpeg_mean_error':sum(errors)/len(errors),
                   'points':len(checkpoint),'guarded_matches_original':True})
restored=text
for old,new in reversed(changes):
    if restored.count(new)!=1: raise RuntimeError('AAC replacement not unique')
    restored=restored.replace(new,old,1)
if restored!=original: raise RuntimeError('Unexpected non-AAC header changes')
if HEADER.read_bytes()!=header_before: raise RuntimeError('Header changed concurrently')
REPORT.parent.mkdir(parents=True,exist_ok=True)
changed=text.encode('utf-8')!=header_before
if changed:
    temporary=REPORT.parent/'probe_codec_vectors.tmp'
    temporary.write_bytes(text.encode('utf-8'))
    if HEADER.read_bytes()!=header_before: raise RuntimeError('Header changed concurrently')
    os.replace(temporary,HEADER)
if changed or not REPORT.exists():
    REPORT.write_text(json.dumps({'scope':'AAC host PCM and independent FFmpeg; no target or RF proof',
        'toolchain':'Windows clang portable ESP32 arithmetic, -O2 -fwrapv -funsigned-char',
        'native_dll_sha256':digest(native_path),'reference_dll_sha256':digest(reference_path),
        'before_header_sha256':hashlib.sha256(header_before).hexdigest(),
        'after_header_sha256':digest(HEADER),'cases':report},indent=2)+'\n')
print(json.dumps({'header_sha256':digest(HEADER),'changed':changed,'cases':report},indent=2))
