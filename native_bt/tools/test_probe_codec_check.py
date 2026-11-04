"""Run the actual self-test against real host codec implementations, no hardware."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
WS=ROOT.parent
OUT=ROOT/'_build/native_codecs/probe_check_host'
SBC=WS/'esp-idf/components/bt/host/bluedroid/external/sbc/decoder'
AAC=WS/'esp-idf/components/bt/host/bluedroid/external/libaac-lc/libhelix_aac'
APTX=WS/'esp-idf/components/bt/host/bluedroid/external/libfreeaptx/main'
LDAC=WS/'esp-idf/components/bt/host/bluedroid/external/libldac-dec'
LHDC=WS.parent/'LHDC-V5-Decoder-main/decoder'
OPUS=WS/'esp-idf/components/bt/host/bluedroid/external/opus'
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]


def run(args,**kwargs):
    result=subprocess.run(args,capture_output=True,text=True,**kwargs)
    if result.returncode:raise RuntimeError(f'command failed {args[:4]}\n{result.stdout[-5000:]}\n{result.stderr[-5000:]}')
    return result.stdout


def main():
    OUT.mkdir(parents=True,exist_ok=True);env=os.environ.copy()
    if os.name=='nt':
        vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(s.split('=',1) for s in values.splitlines() if '=' in s and not s.startswith('=')))
    for key in ('LHDC_WINFILE','LHDC_PKALL','LHDC_FORCE_REF'):env.pop(key,None)
    compiler=shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe'
    port=OUT/'sbc_port';aacport=OUT/'aac_port';ldacport=OUT/'ldac_port';lhdcport=OUT/'lhdc_port'
    for script,source,destination in (
        ('sbc/prepare_port.py',SBC/'srce',port),('aac/prepare_port.py',AAC/'aacdec.c',aacport),
        ('ldac/prepare_port.py',LDAC/'src',ldacport),('lhdcv5/prepare_port.py',LHDC,lhdcport)):
        run([sys.executable,str(ROOT/'codecs'/script),'--source',str(source),'--output',str(destination)],env=env)
    sbc_names='alloc bitalloc-sbc bitalloc bitstream-decode decoder-oina decoder-private decoder-sbc dequant framing-sbc framing oi_codec_version synthesis-8-generated synthesis-dct8 synthesis-sbc'.split()
    aac_names='aactabs bitstream buffers dct4 decelmnt dequant fft filefmt huffman hufftabs imdct noiseless pns stproc tns trigtabs'.split()
    sources=[ROOT/'tests/codecs/probe_codec_selftest_host.c',
             ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c',ROOT/'src/dbt_sbc_media.c',
             ROOT/'codecs/sbc/dbt_sbc_codec.c',ROOT/'codecs/sbc/dbt_sbc_decoder.c',
             ROOT/'codecs/opus/dbt_opus.c',ROOT/'codecs/opus/scratch/dbt_opus_scratch.c',
             ROOT/'codecs/aac/dbt_aac_decoder.c',ROOT/'codecs/aac/dbt_aac_transport.c',
             ROOT/'codecs/aptx/dbt_aptx.c',ROOT/'codecs/aptx/dbt_aptx_backend.c',
             ROOT/'codecs/ldac/dbt_ldac.c',ROOT/'codecs/ldac/dbt_ldac_decoder.c',
             ROOT/'codecs/ldac/dbt_ldac_bt.c',ROOT/'codecs/ldac/dbt_ldac_lib.c',
             ROOT/'codecs/lhdcv5/dbt_lhdcv5_codec.c']
    sources += [(port if n.startswith('synthesis-') else SBC/'srce')/(n+'.c') for n in sbc_names]
    aac_sources=[aacport/'aacdec_guarded.c']+[AAC/(n+'.c') for n in aac_names]
    sources += aac_sources
    sources += [lhdcport/n for n in ('lhdc_dec.c','lhdc_imdct.c','lhdc_entropy_dec.c','lhdc_sns_synth.c')]
    sources += [LHDC/n for n in ('lhdc_bit_reader.c','lhdc_diag_config.c','lhdc_tables.c')]
    inc=[ROOT/'tests/codecs/probe_audio_stubs',ROOT/'include',ROOT/'esp32_probe/main',ROOT/'codecs/common',
         ROOT/'codecs/sbc',ROOT/'codecs/sbc/port',SBC/'include',ROOT/'codecs/opus',OPUS/'include',
         ROOT/'codecs/aac',AAC,ROOT/'codecs/aptx',APTX,ROOT/'codecs/ldac',ldacport,LDAC/'inc',LDAC/'src',ROOT/'codecs/lhdcv5',LHDC]
    opus_lib=ROOT/'_build/avrcp/probe_opus_guarded/opus.lib'
    if not opus_lib.is_file():raise RuntimeError('Run test_probe_audio_worker.py to build the guarded host Opus library')
    tracked=sources+[opus_lib,ROOT/'esp32_probe/main/probe_codec_check.c',ROOT/'esp32_probe/main/probe_codec_vectors.h',ROOT/'esp32_probe/main/probe_opus_vectors.h',Path(__file__).resolve()]
    # Include textual dependencies used by the SDK amalgamations and all headers.
    tracked += [APTX/'freeaptx.c'] + list((LDAC/'src').glob('*.o.c'))
    tracked += [header for directory in inc for header in directory.glob('*.h')]
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    flags=[compiler,'-std=c11','-O2','-fwrapv','-D_CRT_SECURE_NO_WARNINGS','-DUSE_DEFAULT_STDLIB','-DAAC_MAX_NCHANS=2',
           '-DLHDC_HOST_BUILD','-DDBT_LHDC_NO_HOST_WINFILE','-DDBT_OPUS_GUARDED_API=1']
    flags+=['-DDBT_PROBE_'+x+'_BUILD=1' for x in ('SBC','OPUS','AAC','APTX','LDAC','LHDC')]
    flags+=['-I'+str(p) for p in inc]
    exe=OUT/('probe_codec_selftest.exe' if os.name=='nt' else 'probe_codec_selftest')
    # Select ESP32's portable arithmetic only for SDK AAC translation units;
    # other codecs and their golden values retain their existing flags.
    aac_objects=[]
    for unit in aac_sources:
        obj=OUT/('aac_'+unit.stem+('.obj' if os.name=='nt' else '.o'))
        run(flags+['-funsigned-char','-include',str(ROOT/'codecs/aac/host_target_math.h'),'-c',str(unit),'-o',str(obj)],env=env,cwd=OUT)
        aac_objects.append(obj)
    others=[unit for unit in sources if unit not in aac_sources]
    run(flags+[str(s) for s in others]+[str(o) for o in aac_objects]+[str(opus_lib)]+([] if os.name=='nt' else ['-lm'])+['-o',str(exe)],env=env,cwd=OUT)
    output=run([str(exe)],env=env,cwd=OUT)
    events=[]
    for line in output.splitlines():
        try:events.append(json.loads(line))
        except ValueError:pass
    opus=[e for e in events if e.get('probe')=='target_opus_selftest']
    overall=[e for e in events if e.get('probe')=='target_codec_selftest']
    cases=[e for e in events if e.get('probe')=='target_codec_case']
    assert len(opus)==1 and opus[0]['passed'] and opus[0]['frames']==48
    vector_receipt=json.loads((ROOT/'_build/native_codecs/probe_codec_vectors.json').read_text())
    selected=16+len(vector_receipt['cases'])
    assert len(overall)==1 and overall[0]['passed'] and overall[0]['configurations']==selected and overall[0]['selected']==selected
    assert len(cases)==selected+4 and all(c['passed'] for c in cases[:selected]) and not any(c['passed'] for c in cases[selected:])
    injected=run([str(exe),'allocation-failure'],env=env,cwd=OUT)
    injected_events=[]
    for line in injected.splitlines():
        try:injected_events.append(json.loads(line))
        except ValueError:pass
    assert any(e.get('probe')=='target_opus_selftest' and not e['passed'] for e in injected_events)
    assert any(e.get('probe')=='target_codec_selftest' and not e['passed'] for e in injected_events)
    assert all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha for path,sha in before.items()),'Source changed during host check'
    report={'passed':True,'scope':'actual self-test and codec algorithms on host; timing and FreeRTOS stack metrics are stubs, not target evidence',
            'opus_marker':opus[0],'summary':overall[0],'cases':cases[:selected],
            'negative_checks':['sample_mismatch','exact_hash_mismatch','low_stack','allocation_failure','failed_marker'],
            'source_sha256':before}
    (OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    print(f'PASS host self-test: 48 Opus frames and {selected-16} additional codec cases; stack/time are stubs')


if __name__=='__main__':
    subprocess.run(CHECK,check=True)
    try:main()
    finally:subprocess.run(CHECK,check=True)
