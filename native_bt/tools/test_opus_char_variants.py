"""Compare guarded fixed-point Opus vectors with signed/unsigned host char."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
WS=ROOT.parent
OUT=ROOT/'_build/native_codecs/opus_char_variants'
OPUS=WS/'esp-idf/components/bt/host/bluedroid/external/opus'
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]


def run(command,**kwargs):
    p=subprocess.run(command,capture_output=True,text=True,**kwargs)
    if p.returncode:raise RuntimeError(f'{command[:4]}\n{p.stdout[-3000:]}\n{p.stderr[-3000:]}')
    return p.stdout


def main():
    env=os.environ.copy()
    if os.name=='nt':
        vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    clang=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
    ninja=shutil.which('ninja',path=env.get('PATH')) or str(Path(env['USERPROFILE'])/'.espressif/tools/ninja/1.12.1/ninja.exe')
    override=(ROOT/'codecs/opus/scratch/dbt_opus_scratch_override.h').as_posix()
    tracked=[ROOT/'tests/codecs/opus_char_variants.c',ROOT/'esp32_probe/main/probe_opus_vectors.h',
             ROOT/'tests/codecs/opus_oracle_selftest_host.c',ROOT/'esp32_probe/main/probe_codec_check.c',
             ROOT/'esp32_probe/components/opus_decoder/CMakeLists.txt',ROOT/'codecs/opus/dbt_opus.c',
             ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c',
             ROOT/'codecs/opus/scratch/dbt_opus_scratch.c',ROOT/'codecs/opus/scratch/dbt_opus_scratch.h',
             ROOT/'codecs/opus/scratch/dbt_opus_scratch_override.h',Path(__file__).resolve()]
    tracked += [h for directory in (ROOT/'include',ROOT/'codecs/common',ROOT/'codecs/opus',
                                   ROOT/'tests/codecs/probe_audio_stubs') for h in directory.rglob('*.h')]
    tracked += sorted(p for p in OPUS.rglob('*') if p.is_file() and p.suffix in ('.c','.h'))
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    results={}
    for variant in ('signed','unsigned','unsigned_no_harden','unsigned_slow64','unsigned_slow64_q31fast'):
        signed='unsigned' if variant.startswith('unsigned') else 'signed'
        dest=OUT/variant;dest.mkdir(parents=True,exist_ok=True)
        flags=f'-include "{override}" -DDBT_OPUS_GUARDED_API=1 -std=c11 -fwrapv -f{signed}-char'
        if variant.startswith('unsigned_slow64'):
            original=(OPUS/'celt/arch.h').read_text()
            condition='#if defined(__x86_64__) || defined(__LP64__) || defined(_WIN64)'
            assert original.count(condition)==1
            local=dest/'arch_slow64.h';local.write_text(original.replace(condition,'#if 0'))
            if variant=='unsigned_slow64_q31fast':
                fixed=(OPUS/'celt/fixed_generic.h').read_text()
                slow='#define MULT32_32_Q31(a,b) ADD32(ADD32(SHL(MULT16_16(SHR((a),16),SHR((b),16)),1), SHR(MULT16_16SU(SHR((a),16),((b)&0x0000ffff)),15)), SHR(MULT16_16SU(SHR((b),16),((a)&0x0000ffff)),15))'
                fast='#define MULT32_32_Q31(a,b) ((opus_val32)SHR((opus_int64)(a)*(opus_int64)(b),31))'
                assert fixed.count(slow)==1
                (dest/'fixed_generic.h').write_text(fixed.replace(slow,fast))
            flags+=f' -I"{(OPUS/"include").as_posix()}" -I"{(OPUS/"celt").as_posix()}" -DFIXED_POINT=1 -include "{local.as_posix()}"'
        run(['cmake','-S',str(OPUS),'-B',str(dest),'-G','Ninja',f'-DCMAKE_MAKE_PROGRAM={ninja}',
             f'-DCMAKE_C_COMPILER={clang}','-DCMAKE_BUILD_TYPE=Release',f'-DCMAKE_C_FLAGS={flags}',
             '-DCMAKE_C_FLAGS_RELEASE=-Os -DNDEBUG','-DOPUS_BUILD_SHARED_LIBRARY=OFF',
             '-DOPUS_BUILD_TESTING=OFF','-DOPUS_BUILD_PROGRAMS=OFF','-DOPUS_FIXED_POINT=ON',
             '-DOPUS_ENABLE_FLOAT_API=OFF','-DOPUS_DISABLE_INTRINSICS=ON']+
            (['-DOPUS_HARDENING=OFF'] if variant not in ('signed','unsigned') else []),env=env)
        run(['cmake','--build',str(dest),'--parallel','4'],env=env)
        lib=dest/('opus.lib' if os.name=='nt' else 'libopus.a')
        exe=dest/('opus_char.exe' if os.name=='nt' else 'opus_char')
        includes=[OPUS/'include',ROOT/'codecs/opus/scratch',ROOT/'esp32_probe/main']
        def compile_harness(path,oracle_slow):
            run([clang,'-std=c11','-Os','-fwrapv',f'-f{signed}-char']+
                (['-DDBT_OPUS_TEST_FORCE_SLOW64=1'] if oracle_slow else [])+['-I'+str(x) for x in includes]+
                [str(ROOT/'tests/codecs/opus_char_variants.c'),str(ROOT/'codecs/opus/scratch/dbt_opus_scratch.c'),
                 str(lib)]+([] if os.name=='nt' else ['-lm'])+['-o',str(path)],env=env)
        oracle_slow=variant=='unsigned_slow64'
        compile_harness(exe,oracle_slow)
        lines=run([str(exe)],env=env).splitlines()
        dumped=subprocess.run([str(exe),'pcm'],env=env,capture_output=True,text=True,check=True)
        (dest/'opus2_pcm.txt').write_text(dumped.stderr)
        results[variant]=[{'index':int(i),'actual':int(a),'expected':int(e)} for i,a,e in (x.split() for x in lines)]
        assert len(results[variant])==16
        if variant in ('unsigned_no_harden','unsigned_slow64'):
            run([str(exe),'strict'],env=env)
            wrong=dest/('wrong_oracle.exe' if os.name=='nt' else 'wrong_oracle')
            compile_harness(wrong,not oracle_slow)
            wrong_rows=[tuple(map(int,x.split())) for x in run([str(wrong)],env=env).splitlines()]
            assert [i for i,a,e in wrong_rows if a!=e]==[2,3,4,6,7,8,9,11,12,15]
            assert subprocess.run([str(wrong),'strict'],env=env,capture_output=True).returncode==1
            for selected_slow in (False,True):
                checkexe=dest/('check_slow.exe' if selected_slow else 'check_fast.exe')
                checkinc=[ROOT/'tests/codecs/probe_audio_stubs',ROOT/'include',ROOT/'codecs/common',ROOT/'codecs/opus',OPUS/'include',ROOT/'esp32_probe/main']
                checksrc=[ROOT/'tests/codecs/opus_oracle_selftest_host.c',ROOT/'codecs/common/dbt_codec.c',
                          ROOT/'src/dbt_avdtp_caps.c',ROOT/'codecs/opus/dbt_opus.c',ROOT/'codecs/opus/scratch/dbt_opus_scratch.c']
                run([clang,'-std=c11','-Os','-fwrapv','-DDBT_PROBE_OPUS_BUILD=1','-DDBT_OPUS_GUARDED_API=1']+
                    (['-DDBT_OPUS_TEST_FORCE_SLOW64=1'] if selected_slow else [])+
                    ['-I'+str(x) for x in checkinc]+[str(x) for x in checksrc]+[str(lib)]+
                    ([] if os.name=='nt' else ['-lm'])+['-o',str(checkexe)],env=env)
                result=subprocess.run([str(checkexe)],env=env,capture_output=True,text=True)
                events=[]
                for line in result.stdout.splitlines():
                    try:events.append(json.loads(line))
                    except ValueError:pass
                summaries=[x for x in events if x.get('probe')=='target_codec_selftest']
                assert len(summaries)==1
                matches=selected_slow==oracle_slow
                assert result.returncode==(0 if matches else 1) and summaries[0]['passed']==matches
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
    report={'scope':'host guarded fixed-point only; no Xtensa execution',
            'same_as_fixture':{s:all(x['actual']==x['expected'] for x in r) for s,r in results.items()},
            'different_between_variants':[x['index'] for x,y in zip(results['signed'],results['unsigned']) if x['actual']!=y['actual']],
            'different_hardening':[x['index'] for x,y in zip(results['unsigned'],results['unsigned_no_harden']) if x['actual']!=y['actual']],
            'different_slow64':[x['index'] for x,y in zip(results['unsigned_no_harden'],results['unsigned_slow64']) if x['actual']!=y['actual']],
            'different_q31fast':[x['index'] for x,y in zip(results['unsigned_no_harden'],results['unsigned_slow64_q31fast']) if x['actual']!=y['actual']],
            'vectors':results,'source_sha256':before}
    assert report['same_as_fixture']['signed'] and report['same_as_fixture']['unsigned']
    assert report['same_as_fixture']['unsigned_no_harden'] and report['same_as_fixture']['unsigned_slow64']
    assert report['different_slow64']==[2,3,4,6,7,8,9,11,12,15]
    assert not report['different_q31fast']
    # Check the real Xtensa compiler's architecture selection without executing
    # target code. The Opus component independently validates the SDK rule.
    commands=json.loads((ROOT/'_build/esp32/compile_commands.json').read_text())
    entry=next(x for x in commands if x['file'].replace('\\','/').endswith('/celt/celt_decoder.c'))
    target_cc=entry['command'].split(' ',1)[0].strip('"')
    target_dir=OUT/'target_profile';target_dir.mkdir(exist_ok=True)
    source=target_dir/'check.c'
    source.write_text('#include "probe_opus_vectors.h"\n_Static_assert(OPUS_PROFILE_HASH(111,222)==222,"target oracle must select slow64");\n')
    target_cmd=[target_cc,'-std=c11','-DESP_PLATFORM=1','-I'+str(ROOT/'esp32_probe/main'),'-c',str(source),'-o',str(target_dir/'check.o')]
    run(target_cmd,env=env)
    rejected=subprocess.run(target_cmd+['-DDBT_OPUS_TEST_FORCE_SLOW64=1'],env=env,capture_output=True,text=True)
    assert rejected.returncode!=0 and 'test override is forbidden' in rejected.stderr
    report['xtensa_profile_compile']='slow64 selected; test override rejected; object only'
    (OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k in ('scope','same_as_fixture','different_between_variants','different_hardening','different_slow64','different_q31fast')},indent=2))


if __name__=='__main__':
    subprocess.run(CHECK,check=True)
    try:main()
    finally:subprocess.run(CHECK,check=True)
