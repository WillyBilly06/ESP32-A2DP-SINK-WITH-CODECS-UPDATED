"""Offline timing decomposition of pinned LHDC fixture packets on the local host.
Stage profiling is opt-in and perturbs timing; numbers are not ESP32 timing."""
import argparse,ctypes,hashlib,json,os,pathlib,shutil,subprocess,sys
parser=argparse.ArgumentParser();parser.add_argument('--packets',type=int,default=120);parser.add_argument('--target-math',action='store_true');parser.add_argument('--no-stage',action='store_true');args=parser.parse_args()
if not 1<=args.packets<=1000:raise SystemExit('packets must be 1..1000')
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;out=root/'_build/avrcp/lhdc-perf';out.mkdir(parents=True,exist_ok=True)
dec=ws.parent/'LHDC-V5-Decoder-main/decoder';port=out/'port';check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True);env=os.environ.copy()
try:
    if os.name=='nt':
        vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    for name in ('LHDC_WINFILE','LHDC_PKALL','LHDC_FORCE_REF','LHDC_CH1_START','LHDC_SNS','LHDC_LDIV','LHDC_SPLIT','LHDC_SLOPE','LHDC_RATEPOW','LHDC_LEVELDB'):env.pop(name,None)
    subprocess.run([sys.executable,str(root/'codecs/lhdcv5/prepare_port.py'),'--source',str(dec),'--output',str(port)],check=True,env=env,stdout=subprocess.DEVNULL)
    cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
    source=[root/'tests/codecs/lhdcv5_perf_fixture.c',root/'tests/codecs/lhdcv5_perf_helper.c',root/'codecs/lhdcv5/dbt_lhdcv5_codec.c',root/'codecs/common/dbt_codec.c',root/'src/dbt_avdtp_caps.c',root/'src/dbt_resampler.c']
    source += [port/n for n in ('lhdc_dec.c','lhdc_imdct.c','lhdc_entropy_dec.c','lhdc_sns_synth.c')]
    source += [dec/n for n in ('lhdc_bit_reader.c','lhdc_diag_config.c','lhdc_tables.c')]
    incs=[root/'include',root/'codecs/common',root/'codecs/lhdcv5',root/'tests/codecs',dec]
    inputs=[root/'codecs/lhdcv5/prepare_port.py',root/'codecs/lhdcv5/dbt_lhdcv5_codec.c',root/'tests/codecs/lhdcv5_perf_fixture.c',root/'tests/codecs/lhdcv5_perf_helper.c',root/'tests/codecs/lhdcv5_perf_helper.h',root/'src/dbt_resampler.c',root/'src/dbt_resampler_coeffs.inc',root/'esp32_probe/main/probe_codec_vectors.h']+source+[dec/n for n in ('lhdc_dec.c','lhdc_imdct.c','lhdc_entropy_dec.c','lhdc_sns_synth.c')]+list(dec.glob('*.h'))
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    path=out/('lhdc-perf.dll' if os.name=='nt' else 'lhdc-perf.so')
    cmd=[cc,'-std=c11','-O3','-fwrapv','-shared','-DLHDC_HOST_BUILD','-DCONFIG_IDF_TARGET_ESP32=1','-DDBT_LHDC_NO_HOST_WINFILE','-D_CRT_SECURE_NO_WARNINGS']
    if not args.no_stage:cmd+=['-DDBT_LHDC_STAGE_PROFILE=1']
    if args.target_math:cmd+=['-DDBT_LHDC_TARGET_MATH=1']
    if os.name!='nt':cmd+=['-fPIC']
    cmd+=sum((['-I',str(x)] for x in incs),[])+[str(x) for x in source]
    if os.name!='nt':cmd+=['-lm']
    subprocess.run(cmd+['-o',str(path)],env=env,check=True)
    class Result(ctypes.Structure):
        _fields_=[('packets',ctypes.c_uint32),('callbacks',ctypes.c_uint32),('frames',ctypes.c_uint32),('checksum',ctypes.c_uint32),('packet_min',ctypes.c_uint32),('packet_max',ctypes.c_uint32),('decode_min',ctypes.c_uint32),('decode_max',ctypes.c_uint32),('packet_sum',ctypes.c_uint64),('callback_sum',ctypes.c_uint64),('decode_sum',ctypes.c_uint64),('stage_sum',ctypes.c_uint64*6),('stage_max',ctypes.c_uint32*6),('stage_calls',ctypes.c_uint32)]
    lib=ctypes.CDLL(str(path));lib.lhdc_perf_fixture.argtypes=[ctypes.c_uint,ctypes.c_uint,ctypes.c_uint,ctypes.POINTER(Result)]
    rows=[]
    for index,rate in enumerate((44100,48000,96000,192000)):
        result=Result();code=lib.lhdc_perf_fixture(index,12,args.packets,ctypes.byref(result))
        if code or result.packets!=args.packets or (not args.no_stage and not result.stage_calls):raise SystemExit(f'FAIL {rate}: code={code} packets={result.packets} stages={result.stage_calls}')
        row={'rate':rate,'packets':result.packets,'callbacks':result.callbacks,'sample_frames':result.frames,'packet_mean_us':result.packet_sum/result.packets,'packet_max_us':result.packet_max,'callback_mean_per_packet_us':result.callback_sum/result.packets,'decoder_estimate_mean_us':result.decode_sum/result.packets,'decoder_estimate_max_us':result.decode_max,'stage_calls':result.stage_calls,'stage_means_us_per_channel':({n:result.stage_sum[i]/result.stage_calls for i,n in enumerate(('pre','entropy','mantissa','dequant_sns','imdct','overlap'))} if result.stage_calls else None)}
        rows.append(row);print(json.dumps(row))
    lib.lhdc_perf_fir.argtypes=[ctypes.c_uint,ctypes.c_uint,ctypes.POINTER(ctypes.c_uint64),ctypes.POINTER(ctypes.c_uint32)]
    fir=[]
    for rate in (44100,48000,96000,192000):
        duration=ctypes.c_uint64();checksum=ctypes.c_uint32();code=lib.lhdc_perf_fir(rate,32768,ctypes.byref(duration),ctypes.byref(checksum))
        if code:raise SystemExit(f'FAIL FIR {rate}: {code}')
        row={'rate':rate,'samples':32768,'elapsed_us':duration.value,'us_per_stereo_frame':duration.value/32768,'checksum':checksum.value};fir.append(row);print(json.dumps(row))
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    if hashes!=before:raise SystemExit('source changed during the host benchmark')
    (out/('host-'+('target-math' if args.target_math else 'baseline')+('-no-stage' if args.no_stage else '-stage')+'.json')).write_text(json.dumps({'scope':'instrumented host; fixture cycling; not target/RF/acoustic evidence','target_math':args.target_math,'stage_instrumented':not args.no_stage,'rows':rows,'fir':fir,'sha256':hashes},indent=2)+'\n')
finally:subprocess.run(check,check=True)
