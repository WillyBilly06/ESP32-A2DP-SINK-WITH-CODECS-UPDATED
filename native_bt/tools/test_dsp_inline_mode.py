"""Actual CMake/CLI guards for the isolated DSP-inline comparison; no hardware."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/dsp_inline_mode';OUT.mkdir(parents=True,exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={'DSP_INLINE_CHECK':'ON','AUDIO_STAGE_PROFILE':'ON','PIPELINE_DIAGNOSTIC':'ON','CODEC_DIAGNOSTIC_ONLY':'ON','LHDC_BUILD':'ON','AUDIO_OUTPUT':'ON','ASYNC_AUDIO':'ON'}
for change in ({},{'AUDIO_STAGE_PROFILE':'OFF'},{'FIR_COMPILER_CHECK':'ON'},{'DSP_STAGE_PROFILE':'ON'},{'PIPELINE_DIAGNOSTIC':'OFF'},{'PRODUCT_IO':'ON'}):
    options=base|change
    result=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in options.items()],'-P',str(guard)],capture_output=True,text=True)
    assert (result.returncode==0)==(not change),result.stdout+result.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
selection=next(node for node in tree.body if isinstance(node,ast.If) and isinstance(node.test,ast.Name) and node.test.id=='fir_check' and any(isinstance(child,ast.For) for child in node.body))
actual_check=compile(ast.Module(body=[selection],type_ignores=[]),'<actual image variant checks>','exec')
def check_variants(fir,profile,inline,files,valid):
    commands=[{'file':name,'command':'compiler -Os '+optimization+' -c input.c'} for name,optimization in files]
    try:exec(actual_check,{'fir_check':fir,'audio_profile':True,'pipeline':True,'dsp_profile':profile,'dsp_inline':inline,'commands':commands,'Path':pathlib.Path,'re':re})
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
dsp_os=('probe_dsp_os.c','-Os');inline_os=('probe_dsp_inline.c','-Os')
check_variants(False,False,True,[dsp_os,inline_os],True)
check_variants(False,False,False,[dsp_os,('probe_dsp_o3.c','-O3'),('probe_dsp_block.c','-Os'),('probe_dsp_word.c','-Os')],True)
check_variants(False,True,False,[dsp_os,('probe_dsp_profile.c','-Os')],True)
fir_files=[('probe_fir_'+name+'.c',opt) for name,opt in (('wide','-O3'),('os','-Os'),('o2','-O2'),('o3','-O3'),('fold','-O3'),('pair','-O3'))]
check_variants(True,False,False,fir_files,True)
check_variants(False,False,True,[dsp_os,('probe_dsp_inline.c','-O3')],False)
check_variants(False,False,True,[dsp_os,('probe_dsp_o3.c','-O3')],False)
check_variants(True,False,False,fir_files[:-1],False)
check_variants(False,False,False,[dsp_os,('probe_dsp_o3.c','-O3')],False)
check_variants(False,False,False,[dsp_os,('probe_dsp_o3.c','-O3'),('probe_dsp_block.c','-Os')],False)
check_variants(False,False,False,[dsp_os,('probe_dsp_o3.c','-O3'),('probe_dsp_block.c','-Os'),('probe_dsp_word.c','-O3')],False)
cli_cases=0
if sys.platform=='win32':
    for arguments in (['-DspInlineCheck'],['-DspInlineCheck','-AudioStageProfile','-DspStageProfile']):
        result=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),*arguments],capture_output=True,text=True)
        assert result.returncode and 'DSP inline comparison requires' in result.stderr
        cli_cases+=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':6,'actual_variant_checks':10,'cli_rejections':cli_cases,'scope':'actual CMake prefix, actual image-validator variant checks and early CLI guards; no build/device execution'},indent=2)+'\n')
print('PASS DSP isolation: six CMake configurations, ten actual image-variant checks and '+str(cli_cases)+' early CLI rejections; no hardware')
