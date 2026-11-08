"""Run the actual self-test against real host codec implementations, no hardware."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import argparse
import re

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
    global LHDC,OUT
    parser=argparse.ArgumentParser()
    parser.add_argument('--lhdc-source',type=Path,help='Sibling LHDC repository or decoder directory')
    parser.add_argument('--deferred',action='store_true',help='Compare real synchronous/deferred PCM with simulated renderer backpressure')
    parser.add_argument('--sanitize',action='store_true',help='AddressSanitizer for deferred adapters and all codec sources including guarded Opus')
    parser.add_argument('--pcm-loans',action='store_true',help='Use asynchronous immutable PCM loans with a delayed provider mock')
    parser.add_argument('--lhdc-esp32-paths',action='store_true',help='Host-execute the LHDC source conditionals selected on classic ESP32; not Xtensa execution')
    parser.add_argument('--lhdc-keep-unused-overlap',action='store_true',help='Retain the unused ESP32 overlap snapshot as an allocation reference')
    parser.add_argument('--lhdc-max-fixture',action='store_true',help='Also run the existing 96-packet maximum LHDC fixture on the host; timing remains stubbed')
    parser.add_argument('--lhdc-fast-mantissa',action='store_true',help='Compare the exact sliding-word mantissa candidate on pinned codec PCM fixtures')
    parser.add_argument('--lhdc-keep-dedicated-pcm',action='store_true',help='Retain the separate LHDC PCM output allocation for paired comparison')
    args=parser.parse_args()
    if args.lhdc_source:
        LHDC=args.lhdc_source.resolve()
        if not (LHDC/'lhdc_dec.c').is_file():LHDC=LHDC/'decoder'
    if args.sanitize and not args.deferred:raise RuntimeError('--sanitize requires --deferred')
    if args.pcm_loans and not args.deferred:raise RuntimeError('--pcm-loans requires --deferred')
    if args.lhdc_keep_unused_overlap and not args.lhdc_esp32_paths:raise RuntimeError('--lhdc-keep-unused-overlap requires --lhdc-esp32-paths')
    if args.deferred:OUT=ROOT/('_build/native_codecs/deferred_asan' if args.sanitize else '_build/native_codecs/deferred_host')
    if args.pcm_loans:OUT=ROOT/('_build/native_codecs/deferred_loans_asan' if args.sanitize else '_build/native_codecs/deferred_loans_host')
    if args.lhdc_esp32_paths:OUT=OUT.with_name(OUT.name+('_esp32_keep_ov' if args.lhdc_keep_unused_overlap else '_esp32'))
    if args.lhdc_keep_dedicated_pcm:OUT=OUT.with_name(OUT.name+'_dedicated_pcm')
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
    subprocess.run(CHECK,check=True)
    env=os.environ.copy()
    if os.name=='nt':
        vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(s.split('=',1) for s in values.splitlines() if '=' in s and not s.startswith('=')))
    for key in list(env):
        if key.startswith('LHDC_'):env.pop(key,None)
    compiler=shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe'
    cache=shutil.which('ccache',path=env.get('PATH'))
    if not cache and os.name=='nt':
        matches=list((Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
        if matches:cache=str(sorted(matches)[-1])
    if not cache:raise RuntimeError('ccache is required for host codec compilation')
    env.pop('CCACHE_DISABLE',None)
    port=OUT/'sbc_port';aacport=OUT/'aac_port';ldacport=OUT/'ldac_port';lhdcport=OUT/'lhdc_port'
    for script,source,destination in (
        ('sbc/prepare_port.py',SBC/'srce',port),('aac/prepare_port.py',AAC/'aacdec.c',aacport),
        ('ldac/prepare_port.py',LDAC/'src',ldacport),('lhdcv5/prepare_port.py',LHDC,lhdcport)):
        run([sys.executable,str(ROOT/'codecs'/script),'--source',str(source),'--output',str(destination)],env=env)
    sbc_names='alloc bitalloc-sbc bitalloc bitstream-decode decoder-oina decoder-private decoder-sbc dequant framing-sbc framing oi_codec_version synthesis-8-generated synthesis-dct8 synthesis-sbc'.split()
    aac_names='aactabs bitstream buffers dct4 decelmnt dequant fft filefmt huffman hufftabs imdct noiseless pns stproc tns trigtabs'.split()
    harness='codec_deferred_real.c' if args.deferred else 'probe_codec_selftest_host.c'
    sources=[ROOT/'tests/codecs'/harness,
             ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c',ROOT/'src/dbt_sbc_media.c',
             ROOT/'codecs/sbc/dbt_sbc_codec.c',ROOT/'codecs/sbc/dbt_sbc_decoder.c',
             ROOT/'codecs/opus/dbt_opus.c',ROOT/'codecs/opus/scratch/dbt_opus_scratch.c',
             ROOT/'codecs/aac/dbt_aac_decoder.c',ROOT/'codecs/aac/dbt_aac_transport.c',
             ROOT/'codecs/aptx/dbt_aptx.c',ROOT/'codecs/aptx/dbt_aptx_backend.c',
             ROOT/'codecs/ldac/dbt_ldac.c',ROOT/'codecs/ldac/dbt_ldac_decoder.c',
             ROOT/'codecs/ldac/dbt_ldac_bt.c',ROOT/'codecs/ldac/dbt_ldac_lib.c',
             ROOT/'codecs/lhdcv5/dbt_lhdcv5_codec.c']
    sources += [(port if n.startswith('synthesis-') else SBC/'srce')/(n+'.c') for n in sbc_names]
    aac_sources=[aacport/'aacdec_guarded.c']+[(aacport/(n+'_guarded.c') if n in ('decelmnt','noiseless','huffman') else AAC/(n+'.c')) for n in aac_names]
    if args.lhdc_max_fixture:sources += [ROOT/'esp32_probe/main/probe_diagnostic_stats.c']
    sources += aac_sources
    sources += [lhdcport/n for n in ('lhdc_dec.c','lhdc_imdct.c','lhdc_entropy_dec.c','lhdc_sns_synth.c')]
    sources += [LHDC/n for n in ('lhdc_bit_reader.c','lhdc_diag_config.c','lhdc_tables.c')]
    inc=[ROOT/'tests/codecs/probe_audio_stubs',ROOT/'include',ROOT/'esp32_probe/main',ROOT/'codecs/common',
         ROOT/'codecs/sbc',ROOT/'codecs/sbc/port',SBC/'include',ROOT/'codecs/opus',OPUS/'include',
         ROOT/'codecs/aac',AAC,ROOT/'codecs/aptx',APTX,ROOT/'codecs/ldac',ldacport,LDAC/'inc',LDAC/'src',ROOT/'codecs/lhdcv5',lhdcport,LHDC]
    opus_lib=ROOT/'_build/avrcp/probe_opus_guarded/opus.lib'
    if args.sanitize:
        opus_build=ROOT/'_build/avrcp/probe_opus_guarded_asan'
        ninja=shutil.which('ninja',path=env.get('PATH')) or str(Path.home()/'.espressif/tools/ninja/1.12.1/ninja.exe')
        override=(ROOT/'codecs/opus/scratch/dbt_opus_scratch_override.h').as_posix()
        cflags=f'-include "{override}" -DDBT_OPUS_GUARDED_API=1 -std=c11 -fwrapv -fsanitize=address -fno-omit-frame-pointer'
        run(['cmake','-S',str(OPUS),'-B',str(opus_build),'-G','Ninja',f'-DCMAKE_MAKE_PROGRAM={ninja}',f'-DCMAKE_C_COMPILER={compiler}',
             '-DCMAKE_BUILD_TYPE=Release',f'-DCMAKE_C_COMPILER_LAUNCHER={cache}',f'-DCMAKE_C_FLAGS={cflags}',
             '-DOPUS_BUILD_SHARED_LIBRARY=OFF','-DOPUS_BUILD_TESTING=OFF','-DOPUS_BUILD_PROGRAMS=OFF','-DOPUS_FIXED_POINT=ON',
             '-DOPUS_ENABLE_FLOAT_API=OFF','-DOPUS_DISABLE_INTRINSICS=ON'],env=env)
        run(['cmake','--build',str(opus_build),'--parallel','4'],env=env)
        matches=list(opus_build.rglob('opus.lib' if os.name=='nt' else 'libopus.a'))
        if len(matches)!=1:raise RuntimeError('Expected one instrumented guarded Opus library')
        opus_lib=matches[0]
    if not opus_lib.is_file():raise RuntimeError('Run test_probe_audio_worker.py to build the guarded host Opus library')
    tracked=sources+[opus_lib,ROOT/'esp32_probe/main/probe_codec_check.c',ROOT/'esp32_probe/main/probe_codec_vectors.h',ROOT/'esp32_probe/main/probe_opus_vectors.h',Path(__file__).resolve()]
    # Include textual dependencies used by the SDK amalgamations and all headers.
    tracked += [APTX/'freeaptx.c'] + list((LDAC/'src').glob('*.o.c'))
    tracked += [header for directory in inc for header in directory.glob('*.h')]
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    flags=[compiler,'-std=c11','-O2','-fwrapv','-D_CRT_SECURE_NO_WARNINGS','-DUSE_DEFAULT_STDLIB','-DAAC_MAX_NCHANS=2',
           '-DLHDC_HOST_BUILD','-DDBT_LHDC_NO_HOST_WINFILE','-DDBT_OPUS_GUARDED_API=1']
    if args.sanitize:
        flags+=['-fsanitize=address','-fno-omit-frame-pointer']
        if os.name=='nt':flags+=['-fms-runtime-lib=dll']
    flags+=['-DDBT_PROBE_'+x+'_BUILD=1' for x in ('SBC','OPUS','AAC','APTX','LDAC','LHDC')]
    if args.lhdc_max_fixture:flags+=['-DDBT_PROBE_LHDC_MAX_CHECK=1']
    if args.pcm_loans:flags+=['-DDBT_TEST_PCM_LOANS=1']
    if args.lhdc_keep_dedicated_pcm:flags+=['-DDBT_LHDC_KEEP_DEDICATED_PCM=1']
    flags+=['-I'+str(p) for p in inc]
    exe=OUT/('probe_codec_selftest.exe' if os.name=='nt' else 'probe_codec_selftest')
    # Cache individual translation units. Select ESP32's portable arithmetic
    # only for AAC SDK units, preserving the other codecs' arithmetic flags.
    objects=[]
    undefined_checked={ROOT/name for name in (
        'tests/codecs/codec_deferred_real.c','codecs/common/dbt_codec.c','src/dbt_sbc_media.c',
        'codecs/sbc/dbt_sbc_codec.c','codecs/sbc/dbt_sbc_decoder.c','codecs/aac/dbt_aac_decoder.c',
        'codecs/aac/dbt_aac_transport.c','codecs/aptx/dbt_aptx.c','codecs/ldac/dbt_ldac_decoder.c',
        'codecs/opus/dbt_opus.c','codecs/lhdcv5/dbt_lhdcv5_codec.c')}
    for index,unit in enumerate(sources):
        obj=OUT/(str(index)+'_'+unit.stem+('.obj' if os.name=='nt' else '.o'));objects.append(obj)
        extra=['-funsigned-char','-include',str(ROOT/'codecs/aac/host_target_math.h')] if unit in aac_sources else []
        if args.lhdc_fast_mantissa and unit == lhdcport/'lhdc_dec.c':extra+=['-DDBT_LHDC_FAST_MANTISSA=1']
        if args.lhdc_esp32_paths and unit.parent in (lhdcport,LHDC,ROOT/'codecs/lhdcv5'):
            extra+=['-DCONFIG_IDF_TARGET_ESP32=1']
            if args.lhdc_keep_unused_overlap:extra+=['-DDBT_LHDC_KEEP_UNUSED_OV_SAVE=1']
        if args.sanitize and unit in undefined_checked:extra+=['-fsanitize=undefined','-fno-sanitize-recover=undefined']
        run([cache]+flags+extra+['-c',str(unit),'-o',str(obj)],env=env,cwd=OUT)
    link_flags=['-fsanitize=address,undefined'] if args.sanitize else []
    if args.sanitize and os.name=='nt':
        # The clang GNU driver selects the ASan DLL thunk from _DLL at the
        # final object-only link; -fms-runtime-lib alone leaves a static thunk.
        link_flags+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
    run([compiler]+link_flags+[str(o) for o in objects]+[str(opus_lib)]+([] if os.name=='nt' else ['-lm'])+['-o',str(exe)],env=env,cwd=OUT)
    if args.sanitize and os.name=='nt':
        resource=Path(run([compiler,'-print-resource-dir']).strip())
        runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
        if not runtime.is_file():runtime=Path(compiler).parent/runtime.name
        if not runtime.is_file():raise RuntimeError('Selected compiler ASan runtime is missing')
        shutil.copy2(runtime,OUT/runtime.name)
    output=run([str(exe)],env=env,cwd=OUT)
    events=[]
    for line in output.splitlines():
        try:events.append(json.loads(line))
        except ValueError:pass
    if args.deferred:
        cases=[e for e in events if e.get('probe')=='deferred_codec_case']
        summaries=[e for e in events if e.get('probe')=='deferred_codec_summary']
        assert len(summaries)==1 and summaries[0]['passed'] and summaries[0]['cases']==len(cases)
        assert summaries[0]['pcm_loans']==args.pcm_loans
        assert len([c for c in cases if c['mode']==0])==38
        assert all(c['passed'] and c['live_bytes']==0 and c['busy_attempts'] for c in cases)
        assert all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha for path,sha in before.items())
        report={'passed':True,'pcm_loans':args.pcm_loans,'lhdc_esp32_paths':args.lhdc_esp32_paths,'lhdc_keep_unused_overlap':args.lhdc_keep_unused_overlap,'lhdc_keep_dedicated_pcm':args.lhdc_keep_dedicated_pcm,'scope':'real host codec algorithms and worker, byte/sample-timestamp comparison with simulated output backpressure and 150 ms pause; optional async PCM-loan mock and classic-ESP32 LHDC conditionals, no target/RF timing',
                'sanitizers':['address for all source-built codecs','undefined for owned adapters/worker/test harness'] if args.sanitize else [],'guarded_opus_instrumented':args.sanitize,
                'summary':summaries[0],'cases':cases,'source_sha256':before}
        (OUT/'results.json').write_text(json.dumps(dict(report,passed=False,functional_passed=True,state='verifying_workspace'),indent=2)+'\n')
        print(f'PASS {len(cases)} real-codec deferred cases: every PCM byte/timestamp identical; fragmentation, batching, all 37 configurations and maximum LHDC fixture; host only')
        return
    max_cases=[e for e in events if e.get('probe')=='lhdc_max_selftest']
    max_hashes=[e for e in events if e.get('probe')=='lhdc_pcm_hash']
    if args.lhdc_max_fixture:
        assert len(max_cases)==3 and all(e['correctness_passed'] and e['sample_frames']==92160 and e['reference_points']==56 for e in max_cases)
        # Host floating-point transforms may differ in low PCM bits from Xtensa.
        # Require repeatable full-output hashes here; compare baseline/candidate
        # receipts separately, while retaining the independent sample points.
        assert len(max_hashes)==3 and len({(e['fnv1a32'],e['prefix_fnv1a32']) for e in max_hashes})==1
    opus=[e for e in events if e.get('probe')=='target_opus_selftest']
    overall=[e for e in events if e.get('probe')=='target_codec_selftest']
    cases=[e for e in events if e.get('probe')=='target_codec_case']
    assert len(opus)==1 and opus[0]['passed'] and opus[0]['frames']==48
    selected=16+len(re.findall(r'^\{"[^"\n]+",&dbt_', (ROOT/'esp32_probe/main/probe_codec_vectors.h').read_text(),re.M))
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
            'lhdc_fast_mantissa':args.lhdc_fast_mantissa,'maximum_lhdc_cases':max_cases,'maximum_lhdc_hashes':max_hashes,
            'opus_marker':opus[0],'summary':overall[0],'cases':cases[:selected],
            'negative_checks':['sample_mismatch','exact_hash_mismatch','low_stack','allocation_failure','failed_marker'],
            'source_sha256':before}
    (OUT/'results.json').write_text(json.dumps(dict(report,passed=False,functional_passed=True,state='verifying_workspace'),indent=2)+'\n')
    print(f'PASS host self-test: 48 Opus frames and {selected-16} additional codec cases; stack/time are stubs')



def release_completed_host_objects(directory):
    """Keep passing executables/results; reclaim only typed host intermediates."""
    base=directory.resolve();items=[]
    for path in sorted(directory.glob('*.obj')):
        if path.is_symlink() or path.resolve().parent!=base:raise RuntimeError('Compiler object escapes completed output folder')
        data=path.read_bytes()
        if data[:2]!=b'\x64\x86':continue  # AMD64 COFF only; never Xtensa/ELF
        items.append({'path':str(path.resolve()),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
    (directory/'completed-object-cleanup.json').write_text(json.dumps({'kind':'completed passing host compiler intermediates','files':items},indent=2)+'\n')
    for item in items:Path(item['path']).unlink()

def archive_completed_symbols(directory):
    items=[]
    for pdb in sorted(directory.glob('*.pdb')):
        raw=pdb.read_bytes();digest=hashlib.sha256(raw).hexdigest()
        dest=pdb.with_name(pdb.name+'.'+digest[:12]+'.gz');dest.write_bytes(gzip.compress(raw))
        if gzip.decompress(dest.read_bytes())!=raw:raise RuntimeError('Symbol archive round-trip mismatch')
        items.append(dict(original=str(pdb),archive=str(dest),sha256=digest,bytes=len(raw),gzip_bytes=dest.stat().st_size,lossless_verified=True))
        (directory/'lossless-symbols.json').write_text(json.dumps(items,indent=2)+chr(10));pdb.unlink()

if __name__=='__main__':
    completed=False
    try:
        main();completed=True
    finally:
        if completed:
            release_completed_host_objects(OUT)
            archive_completed_symbols(OUT)
        subprocess.run(CHECK,check=True)
        if completed:
            final=json.loads((OUT/'results.json').read_text());final.update(passed=True,state='complete')
            (OUT/'results.json').write_text(json.dumps(final,indent=2)+'\n')
