"""Compile codec-check feature combinations for ESP32; no target execution."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
BUILD=ROOT/'_build/esp32'
OUT=ROOT/'_build/native_codecs/probe_check_target'
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]
sys.path.insert(0,str(ROOT/'tools'))
from test_native_codec_target_components import argv_windows,run


def main():
    if os.name!='nt':raise RuntimeError('This check uses the existing Windows ESP32 build')
    OUT.mkdir(parents=True,exist_ok=True)
    commands=json.loads((BUILD/'compile_commands.json').read_text())
    args=argv_windows(next(x['command'] for x in commands if x['file'].endswith('probe_audio.c')))
    args=args[:args.index('-o')]
    features=('SBC','OPUS','AAC','APTX','LDAC','LHDC')
    args=[x for x in args if not any(x.startswith('-DDBT_PROBE_'+n+'_BUILD') for n in features)]
    include=['-I'+str(ROOT/'codecs'/n) for n in ('common','sbc','opus','aac','aptx','ldac','lhdcv5')]
    include+=['-I'+str(ROOT/'esp32_probe/main')]
    source=ROOT/'esp32_probe/main/probe_codec_check.c'
    tracked=[source,ROOT/'esp32_probe/main/probe_codec_vectors.h',ROOT/'esp32_probe/main/probe_opus_vectors.h',Path(__file__).resolve()]
    tracked+=list((ROOT/'include').glob('*.h'))+list((ROOT/'codecs/common').glob('*.h'))
    for name in ('sbc','opus','aac','aptx','ldac','lhdcv5'):tracked+=list((ROOT/'codecs'/name).glob('*.h'))
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    variants={'all':features,'opus_only':('OPUS',),'sbc_only':('SBC',),'aac_only':('AAC',),
              'aptx_only':('APTX',),'ldac_only':('LDAC',),'lhdc_only':('LHDC',)}
    tools=Path(args[0]).parent;rows=[]
    for name,enabled in variants.items():
        obj=OUT/(name+'.o');flags=['-DDBT_PROBE_'+n+'_BUILD=1' for n in enabled]
        run(args+include+flags+['-fstack-usage','-o',str(obj),'-c',str(source)],cwd=BUILD)
        undefined=run([str(tools/'xtensa-esp32-elf-nm.exe'),'-u',str(obj)])
        for feature,symbol in (('SBC','dbt_sbc_codec'),('OPUS','dbt_opus_codec'),('AAC','dbt_aac_codec'),
                               ('APTX','dbt_aptx_codec'),('LDAC','dbt_ldac_codec'),('LHDC','dbt_lhdcv5_codec')):
            if (symbol in undefined)!=(feature in enabled):raise RuntimeError(f'{name}: incorrect {feature} dependency')
        values=run([str(tools/'xtensa-esp32-elf-size.exe'),str(obj)]).splitlines()[-1].split()
        rows.append({'variant':name,'features':list(enabled),'text_bytes':int(values[0]),'data_bytes':int(values[1]),'bss_bytes':int(values[2])})
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in before.items()),'Source changed during target compilation'
    report={'passed':True,'scope':'ESP32 object compilation only; no firmware link or execution','variants':rows,'source_sha256':before}
    (OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=='__main__':
    subprocess.run(CHECK,check=True)
    try:main()
    finally:subprocess.run(CHECK,check=True)
