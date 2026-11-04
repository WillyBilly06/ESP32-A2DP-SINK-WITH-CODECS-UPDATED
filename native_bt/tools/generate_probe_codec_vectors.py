"""Generate bounded flash-resident target self-test vectors from offline fixtures.

Requires the codec host-test DLLs and encoded fixture files already produced by
the codec-specific test scripts. No radio or controller access is performed.
"""
import ctypes
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/native_codecs'
TARGET=ROOT/'esp32_probe/main/probe_codec_vectors.h'
sys.path.insert(0,str(ROOT/'tools'))
import test_native_codec_aac as aac_test
import test_native_codec_aptx as aptx_test
import test_native_codec_ldac_real as ldac_test
import test_native_codec_lhdcv5 as lhdc_test

u8=ctypes.c_uint8;ptr=ctypes.POINTER(u8);u=ctypes.c_uint
def buf(data):return (u8*len(data)).from_buffer_copy(data)
def fnv(data):
    h=2166136261
    for b in data:h=((h^b)*16777619)&0xffffffff
    return h
def points(data,width,channels):
    vals=[x[0] for x in struct.iter_unpack('<h' if width==2 else '<i',data)]
    chosen=[]
    for ch in range(channels):
        indices=list(range(ch,len(vals),channels));peak=max(abs(vals[i]) for i in indices)
        candidates=[i for i in indices[len(indices)//4:] if abs(vals[i])>max(64,peak//8)]
        if len(candidates)<12:raise RuntimeError('Fixture output lacks meaningful sample checkpoints')
        for j in range(12):
            index=candidates[j*(len(candidates)-1)//11];chosen.append((index,vals[index]))
    return sorted(chosen)


def main():
    if sys.platform!='win32':raise RuntimeError('This generator uses the previously built Windows codec fixtures')
    receipt_specs=(('aac_real_results.json','source_sha256'),('aptx_real_results.json','source_sha256'),
                   ('ldac_encoded_results.json','decoder_source_sha256'),('lhdcv5_encoded_results.json','source_sha256'))
    receipts={}
    for name,key in receipt_specs:
        receipt=json.loads((OUT/name).read_text());assert receipt['passed'];receipts[name]=receipt
        for source,expected in receipt[key].items():
            path=Path(source);path=path if path.is_absolute() else ROOT.parent/path
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
                raise RuntimeError(f'Stale fixture receipt {name}: {source}')
    input_paths=[OUT/(name+'.dll') for name in ('aac_real','aptx_native','ldac_native','lhdcv5_native')]
    ldac_cases=((44100,1),(48000,4),(48000,2),(48000,1),(88200,1),(96000,1))
    lhdc_cases=((44100,16),(48000,16),(96000,24),(192000,24))
    input_paths += [OUT/name for name,_ in receipt_specs]+[Path(__file__).resolve(),OUT/'vectors/tone_48000_2.sbc']
    input_paths += [OUT/f'vectors/tone_{rate}_{channels}.aac' for rate in (44100,48000) for channels in (1,2)]
    input_paths += [OUT/f'ldac_fixture_encoder/fixture_{rate}_{mode}_q1.ldac' for rate,mode in ldac_cases]
    input_paths += [OUT/f'lhdcv5_fixture_encoder/tone_{rate}_{depth}.packets' for rate,depth in lhdc_cases]
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in input_paths}
    aac_receipt=receipts['aac_real_results.json']['fixture_sha256']
    for path in input_paths:
        if path.parent.name=='vectors' and aac_receipt.get(path.name)!=before[str(path)]:
            raise RuntimeError('AAC/SBC fixture does not match its host-test receipt: '+str(path))
    for rate,mode in ldac_cases:
        row=next(x for x in receipts['ldac_encoded_results.json']['configurations'] if (x['rate'],x['mode'],x['quality'])==(rate,mode,1))
        path=OUT/f'ldac_fixture_encoder/fixture_{rate}_{mode}_q1.ldac'
        if before[str(path)]!=row['encoded_sha256']:raise RuntimeError('LDAC fixture hash mismatch')
    for rate,depth in lhdc_cases:
        row=next(x for x in receipts['lhdcv5_encoded_results.json']['configurations'] if (x['rate'],x['depth'])==(rate,depth))
        path=OUT/f'lhdcv5_fixture_encoder/tone_{rate}_{depth}.packets'
        if before[str(path)]!=row['encoded_sha256']:raise RuntimeError('LHDC fixture hash mismatch')
    libs={name:ctypes.CDLL(str(OUT/(name+'.dll'))) for name in ('aac_real','aptx_native','ldac_native','lhdcv5_native')}
    aac,aptx,ldac,lhdc=(libs[n] for n in ('aac_real','aptx_native','ldac_native','lhdcv5_native'))
    aac.probe_media.argtypes=[ptr,u,u,u,u,u,ptr,ctypes.POINTER(u)];aac.probe_sbc_open.argtypes=[ptr]
    for lib in (aptx,ldac):
        lib.worker_control.argtypes=[u,ptr,u];lib.worker_packet.argtypes=[ptr,u,u,ptr,ctypes.POINTER(u)]
    aptx.reference_encode.argtypes=[ptr,u,ptr,u,ctypes.POINTER(u)]
    lhdc.worker_control.argtypes=[u,ptr,u];lhdc.worker_packet_pcm.argtypes=[ptr,u,u,u,ctypes.c_uint32,ptr,u,ctypes.POINTER(u)]
    def control(lib,op,cie=None):return lib.worker_control(op,buf(cie) if cie else None,len(cie) if cie else 0)
    cases=[]
    def add(name,feature,ops,cie,packets,outputs,rate,channels,width,frame_samples,exact):
        pcm=b''.join(outputs);assert pcm and len(pcm)%(channels*width)==0
        peak=max(abs(x[0]) for x in struct.iter_unpack('<h' if width==2 else '<i',pcm))
        if peak<256:raise RuntimeError('Fixture PCM has no substantial signal: '+name)
        cases.append(dict(name=name,feature=feature,ops=ops,cie=cie,packets=packets,
                          rate=rate,channels=channels,width=width,frame_samples=frame_samples,
                          sample_frames=len(pcm)//(channels*width),hash=fnv(pcm),minimum_peak=peak//4,exact=exact,
                          checkpoints=[] if exact else points(pcm,width,channels),pcm_sha256=hashlib.sha256(pcm).hexdigest()))
    def direct_packet(data,seq,marker=1,timestamp=0):
        out=(u8*8192)();n=u();r=aac.probe_media(buf(data),len(data),timestamp,seq,marker,seq,out,ctypes.byref(n))
        if r:raise RuntimeError(('direct decoder',seq,r))
        return bytes(out[:n.value])
    raw=(OUT/'vectors/tone_48000_2.sbc').read_bytes();at=0;sbc_frames=[]
    while at<len(raw) and len(sbc_frames)<4:
        h,pool=raw[at+1],raw[at+2];bands=8 if h&1 else 4;blocks=4*(1+((h>>4)&3));mode=(h>>2)&3;ch=2 if mode else 1
        bits=blocks*pool*(ch if mode<2 else 1)+(bands if mode==3 else 0)
        length=4+bands*ch//2+(bits+7)//8;sbc_frames.append(raw[at:at+length]);at+=length
    cie=bytes([0,0,(0x80>>(h>>6))|(8>>mode),(0x80>>((h>>4)&3))|(4 if h&1 else 8)|(2 if h&2 else 1),2,pool])
    assert aac.probe_sbc_open(buf(cie))==0
    packets=[bytes([1])+f for f in sbc_frames];pcm=[direct_packet(p,i,0,i*128) for i,p in enumerate(packets)]
    aac.probe_close();assert aac.probe_live()==0
    add('sbc_48_stereo','SBC','dbt_sbc_codec',cie,packets,pcm,48000,2,2,128,True)
    for rate,channels in ((44100,1),(44100,2),(48000,1),(48000,2)):
        raw_frames=list(aac_test.adts(OUT/f'vectors/tone_{rate}_{channels}.aac'))[:4]
        cie=bytes([0,2,0x40,1 if rate==44100 else 0,(0x80 if rate==48000 else 0)|(8 if channels==1 else 4),0,0,0])
        packets=[aac_test.latm(f,rate,channels,i>0) for i,f in enumerate(raw_frames)]
        assert aac.probe_open(rate,channels)==0
        pcm=[direct_packet(p,i,1,i*1024) for i,p in enumerate(packets)]
        aac.probe_close();assert aac.probe_live()==0
        add(f'aac_{rate}_{channels}','AAC','dbt_aac_codec',cie,packets,pcm,rate,channels,2,1024,False)
    for kind,rate in ((kind,rate) for kind in ('standard','hd','ll') for rate in (44100,48000)):
        cie=aptx_test.cie(kind,rate);assert aptx.reference_open(kind=='hd')==0
        samples=2400;input_pcm=bytearray()
        for i in range(samples):
            for ch in range(2):
                value=round(2000000*math.sin(2*math.pi*(997+ch*300)*i/rate))
                input_pcm.extend((value&0xffffff).to_bytes(3,'little'))
        out=(u8*8192)();n=u();assert aptx.reference_encode(buf(input_pcm),len(input_pcm),out,len(out),ctypes.byref(n))==len(input_pcm)
        data=bytes(out[:n.value]);quantum=6 if kind=='hd' else 4;chunk=(192//quantum)*quantum
        packets=[data[i:i+chunk] for i in range(0,chunk*4,chunk)]
        assert aptx.worker_init()==0 and control(aptx,0,cie)==0 and control(aptx,3)==0
        pcm=[]
        for i,p in enumerate(packets):
            out=(u8*8192)();n=u();drops=aptx.worker_drops();assert aptx.worker_packet(buf(p),len(p),i,out,ctypes.byref(n))==0 and aptx.worker_drops()==drops
            pcm.append(bytes(out[:n.value]))
        assert control(aptx,5)==0 and aptx.probe_live()==0;aptx.reference_close()
        add('aptx_'+kind+'_'+str(rate),'APTX','dbt_aptx_codec',cie,packets,pcm,rate,2,4,0,True)
    for rate,mode in ldac_cases:
        data=(OUT/f'ldac_fixture_encoder/fixture_{rate}_{mode}_q1.ldac').read_bytes();frames=[];at=0
        while at<len(data) and len(frames)<4:
            assert data[at]==0xaa;n=3+((((data[at+1]&7)<<6)|(data[at+2]>>2))+1);frames.append(data[at:at+n]);at+=n
        cie=ldac_test.cie(rate,mode);assert ldac.worker_init()==0 and control(ldac,0,cie)==0 and control(ldac,3)==0
        packets=[bytes([1])+f for f in frames];pcm=[]
        for i,p in enumerate(packets):
            out=(u8*8192)();n=u();drops=ldac.worker_drops();assert ldac.worker_packet(buf(p),len(p),i,out,ctypes.byref(n))==0 and ldac.worker_drops()==drops
            pcm.append(bytes(out[:n.value]))
        assert control(ldac,5)==0 and ldac.probe_live()==0
        add(f'ldac_{rate}_{mode}','LDAC','dbt_ldac_codec',cie,packets,pcm,rate,1 if mode==4 else 2,4,128 if rate<88200 else 256,False)
    for rate,depth in lhdc_cases:
        data=(OUT/f'lhdcv5_fixture_encoder/tone_{rate}_{depth}.packets').read_bytes();at=0;packets=[]
        while at<len(data) and len(packets)<3:
            size=int.from_bytes(data[at:at+2],'little');at+=4;packets.append(bytes([0,0])+data[at:at+size]);at+=size
        cie=lhdc_test.cie(rate,depth);assert lhdc.worker_init()==0 and control(lhdc,0,cie)==0 and control(lhdc,3)==0
        pcm=[];samples=240 if rate==44100 else rate//200
        for i,p in enumerate(packets):
            out=(u8*65536)();n=u();drops=lhdc.worker_drops()
            assert lhdc.worker_packet_pcm(buf(p),len(p),i,i+1,i*samples,out,len(out),ctypes.byref(n))==0 and lhdc.worker_drops()==drops
            pcm.append(bytes(out[:n.value]))
        assert control(lhdc,5)==0 and lhdc.probe_live()==0
        add(f'lhdcv5_{rate}_{depth}','LHDC','dbt_lhdcv5_codec',cie,packets,pcm,rate,2,2 if depth==16 else 4,samples,False)
    lines=['/* Generated from bounded offline codec fixtures; see generator receipt. */',
           '#ifndef PROBE_CODEC_VECTORS_H','#define PROBE_CODEC_VECTORS_H','#include <stdint.h>',
           'typedef struct { uint32_t index; int32_t value; } probe_pcm_point_t;',
           'typedef struct { const char *name; const dbt_codec_ops_t *ops; const uint8_t *cie; uint8_t cie_size;',
           ' const uint8_t *packet[4]; uint16_t bytes[4]; uint8_t packet_count, channels, width, exact, marker;',
           ' uint32_t rate, sample_frames, expected_hash, minimum_peak; uint16_t callback_samples;',
           ' const probe_pcm_point_t *points; uint8_t point_count; } probe_codec_vector_t;']
    def byte_array(name,data):lines.append('static const uint8_t '+name+'[]={'+','.join(str(x) for x in data)+'};')
    for i,c in enumerate(cases):
        lines.append('#ifdef DBT_PROBE_'+c['feature']+'_BUILD');byte_array(f'codec_cie_{i}',c['cie'])
        for j,p in enumerate(c['packets']):byte_array(f'codec_packet_{i}_{j}',p)
        if c['checkpoints']:
            lines.append(f'static const probe_pcm_point_t codec_points_{i}[]={{'+','.join('{%du,%d}'%x for x in c['checkpoints'])+'};')
        lines.append('#endif')
    lines.append('static const probe_codec_vector_t probe_codec_vectors[]={')
    for i,c in enumerate(cases):
        lines.append('#ifdef DBT_PROBE_'+c['feature']+'_BUILD')
        packet_names=','.join(f'codec_packet_{i}_{j}' for j in range(len(c['packets'])))
        packet_sizes=','.join(str(len(p)) for p in c['packets'])
        points_name=f'codec_points_{i}' if c['checkpoints'] else '0'
        lines.append('{"%s",&%s,codec_cie_%d,%d,{%s},{%s},%d,%d,%d,%d,%d,%du,%du,%du,%du,%d,%s,%d},'%
            (c['name'],c['ops'],i,len(c['cie']),packet_names,packet_sizes,len(c['packets']),c['channels'],c['width'],int(c['exact']),int(c['feature']=='AAC'),
             c['rate'],c['sample_frames'],c['hash'],c['minimum_peak'],c['frame_samples'],points_name,len(c['checkpoints'])))
        lines.append('#endif')
    lines += ['{0}','};','#endif']
    text='\n'.join(lines)+'\n'
    if len(text)>150000:raise RuntimeError('Generated vector header exceeds bound')
    if any(hashlib.sha256(p.read_bytes()).hexdigest()!=before[str(p)] for p in input_paths):
        raise RuntimeError('An input changed during vector generation')
    TARGET.write_text(text)
    report={'scope':'offline host-generated target fixtures; no ESP32 execution','cases':[
        {k:(v.hex() if isinstance(v,bytes) else [hashlib.sha256(p).hexdigest() for p in v] if k=='packets' else v)
         for k,v in c.items() if k!='checkpoints'} for c in cases], 'input_sha256':before,
         'header_sha256':hashlib.sha256(TARGET.read_bytes()).hexdigest()}
    (OUT/'probe_codec_vectors.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Generated',len(cases),'codec cases;',len(text),'header bytes')


if __name__=='__main__':main()
