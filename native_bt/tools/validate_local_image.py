"""Validate an ESP32 build artifact offline. This tool never opens a serial port."""
import argparse,hashlib,io,json,re,struct
from pathlib import Path
from esptool.bin_image import ESP32FirmwareImage
from source_snapshot import verify_manifest
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--image',type=Path,default=ROOT/'_build/esp32/native_bt_probe.bin');a=p.parse_args()
build_status=json.loads((a.image.parent/'local-build-status.json').read_text(encoding='utf-8-sig'))
assert build_status['passed'] and build_status['state']=='complete', 'Latest build did not complete; refusing a possibly stale binary'
assert build_status.get('schema',0)>=2 and build_status.get('compiled_inputs_rechecked') is True, 'Compiler input invalidation missing; rebuild through ccache before admission'
source_count=verify_manifest(a.image.parent/'local-build-sources.json',build_status['source_manifest_sha256'])
commands=json.loads((a.image.parent/'compile_commands.json').read_text(encoding='utf-8'))
diagnostic=bool(build_status.get('codec_diagnostic_only'))
pipeline=bool(build_status.get('pipeline_diagnostic'))
entry_name='probe_codec_diagnostic.c' if diagnostic else 'probe_main.c'
main_command=next(entry['command'] for entry in commands if Path(entry['file']).name==entry_name)
cache=(a.image.parent/'CMakeCache.txt').read_text()
stay_native=build_status.get('stay_native',False)
assert type(stay_native) is bool and ('DBT_PROBE_STAY_NATIVE:BOOL=ON' in cache)==stay_native, 'Stay-native receipt/cache mismatch'
assert re.findall(r'(?:^|\s)-DDBT_PROBE_STAY_NATIVE=(\S+)',main_command)==(['1'] if stay_native else [])
assert not re.search(r'(?:^|\s)-UDBT_PROBE_STAY_NATIVE(?:\s|$)',main_command)
if stay_native:
    assert not diagnostic and not pipeline and build_status.get('async_audio') and build_status.get('product_io') and build_status.get('product_security')
    # Passive tick sampling retains the complete product entry and adds no workload.
    assert not build_status.get('rf_protocol_diagnostic')
    assert 'DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' in cache and entry_name=='probe_main.c'

codec_aware_render=build_status.get('codec_aware_render',False)
assert type(codec_aware_render) is bool and ('DBT_PROBE_CODEC_AWARE_RENDER:BOOL=ON' in cache)==codec_aware_render
assert re.findall(r'(?:^|\s)-DDBT_PROBE_CODEC_AWARE_RENDER=(\S+)',main_command)==(['1'] if codec_aware_render else [])
if codec_aware_render:
    assert stay_native and build_status.get('async_audio') and build_status.get('pcm_copied') and not build_status.get('pcm_loans')
scheduler_trace=bool(build_status.get('scheduler_trace'))
assert ('DBT_PROBE_SCHEDULER_TRACE:BOOL=ON' in cache)==scheduler_trace, 'Scheduler sampler receipt/cache mismatch'
assert re.findall(r'(?:^|\s)-DDBT_PROBE_SCHEDULER_TRACE=(\S+)',main_command)==(['1'] if scheduler_trace else [])
sampler_units=[e for e in commands if Path(e['file']).name=='probe_scheduler.c']
assert bool(sampler_units)==scheduler_trace, 'Unexpected or missing sampler unit'
if scheduler_trace:
    assert not diagnostic and not pipeline and ('DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' not in cache or (build_status.get('stay_native') is True and 'DBT_PROBE_STAY_NATIVE:BOOL=ON' in cache))
    assert not re.search(r'(?:^|\s)-UDBT_PROBE_SCHEDULER_TRACE(?:\s|$)',main_command)
    for unit in ('probe_audio.c','probe_scheduler.c'):
        command=next(e['command'] for e in commands if Path(e['file']).name==unit)
        assert re.findall(r'(?:^|\s)-DDBT_PROBE_SCHEDULER_TRACE=(\S+)',command)==['1']
        assert not re.search(r'(?:^|\s)-UDBT_PROBE_SCHEDULER_TRACE(?:\s|$)',command)
console_irq=build_status.get('console_irq',False)
assert type(console_irq) is bool and ('DBT_PROBE_CONSOLE_IRQ:BOOL=ON' in cache)==console_irq
console_units=[e for e in commands if Path(e['file']).name=='probe_console.c']
assert len(console_units)==int(console_irq)
if console_irq:assert stay_native and build_status.get('async_audio') and not diagnostic and not pipeline
for entry in [e for e in commands if Path(e['file']).name in (entry_name,'probe_console.c')]:
    assert re.findall(r'(?:^|\s)-DDBT_PROBE_CONSOLE_IRQ=(\S+)',entry['command'])==(['1'] if console_irq else [])
    assert not re.search(r'(?:^|\s)-UDBT_PROBE_CONSOLE_IRQ(?:\s|$)',entry['command'])
task_runtime_stats=build_status.get('task_runtime_stats',False)
assert type(task_runtime_stats) is bool and ('DBT_PROBE_TASK_RUNTIME_STATS:BOOL=ON' in cache)==task_runtime_stats
if task_runtime_stats:
    assert stay_native and scheduler_trace and build_status.get('async_audio') and not diagnostic and not pipeline
for unit in (entry_name,'probe_scheduler.c','probe_audio.c'):
    found=[e['command'] for e in commands if Path(e['file']).name==unit]
    if task_runtime_stats:assert len(found)==1
    for command in found:
        assert re.findall(r'(?:^|\s)-DDBT_PROBE_TASK_RUNTIME_STATS=(\S+)',command)==(['1'] if task_runtime_stats else [])
        assert not re.search(r'(?:^|\s)-UDBT_PROBE_TASK_RUNTIME_STATS(?:\s|$)',command)
task_cycle_accounting=build_status.get('task_cycle_accounting',False)
assert type(task_cycle_accounting) is bool
assert ('DBT_PROBE_TASK_CYCLE_ACCOUNTING:BOOL=ON' in cache)==task_cycle_accounting
assert not task_cycle_accounting or task_runtime_stats
cycle_units=[e for e in commands if Path(e['file']).name=='probe_task_cycles.c']
assert len(cycle_units)==(1 if task_cycle_accounting else 0)
for unit in (entry_name,'probe_scheduler.c','probe_audio.c','probe_output.c','probe_task_cycles.c'):
    found=[e['command'] for e in commands if Path(e['file']).name==unit]
    if task_cycle_accounting:assert len(found)==1
    for command in found:
        assert re.findall(r'(?:^|\s)-DDBT_PROBE_TASK_CYCLE_ACCOUNTING=(\S+)',command)==(['1'] if task_cycle_accounting else [])
        assert not re.search(r'(?:^|\s)-UDBT_PROBE_TASK_CYCLE_ACCOUNTING(?:\s|$)',command)
trace_header='probe_task_accounting_trace.h'
trace_inclusions=[e for e in commands if trace_header in e['command']]
if task_cycle_accounting:
    kernel=[e for e in commands if Path(e['file']).name=='tasks.c']
    assert len(kernel)==1 and trace_header in kernel[0]['command']
    for e in trace_inclusions:
        assert '/components/freertos/' in str(e['file']).replace('\\','/') and str(e['file']).endswith('.c')
        assert e['command'].count(trace_header)==1
        assert re.search(r'(?:^|\s)-include\s+(?:"[^"]*[/\\]|\S*[/\\])probe_task_accounting_trace\.h(?:["\s]|$)',e['command'])
else:assert not trace_inclusions
volume_ramp=bool(build_status.get('volume_ramp_block'))
volume_compare=bool(build_status.get('volume_ramp_compare'))
assert ('DBT_PROBE_VOLUME_RAMP_BLOCK:BOOL=ON' in cache)==volume_ramp
assert ('DBT_PROBE_VOLUME_RAMP_COMPARE:BOOL=ON' in cache)==volume_compare
assert not volume_compare or volume_ramp
if volume_ramp:
    assert build_status.get('stay_native') and build_status.get('block_dsp') and not diagnostic and not pipeline
for macro,expected in [('DBT_PROBE_VOLUME_RAMP_BLOCK',volume_ramp),('DBT_PROBE_VOLUME_RAMP_COMPARE',volume_compare)]:
    assert re.findall(r'(?:^|\s)-D'+macro+r'=(\S+)',main_command)==(['1'] if expected else [])
    assert not re.search(r'(?:^|\s)-U'+macro+r'(?:\s|$)',main_command)
fixed_volume_unit=next(e for e in commands if Path(e['file']).name=='fixed_dsp.c')
for macro,expected in [('FIXED_DSP_VOLUME_RAMP_BLOCK',volume_ramp),('FIXED_DSP_VOLUME_RAMP_COMPARE',volume_compare)]:
    assert re.findall(r'(?:^|\s)-D'+macro+r'=(\S+)',fixed_volume_unit['command'])==(['1'] if expected else [])
    assert not re.search(r'(?:^|\s)-U'+macro+r'(?:\s|$)',fixed_volume_unit['command'])
early_input_release=build_status.get('early_input_release',False)
assert type(early_input_release) is bool and ('DBT_PROBE_EARLY_INPUT_RELEASE:BOOL=ON' in cache)==early_input_release
for unit in (entry_name,'probe_output.c'):
    entries=[e['command'] for e in commands if Path(e['file']).name==unit]
    if early_input_release:assert len(entries)==1
    for command in entries:
        assert re.findall(r'(?:^|\s)-DDBT_PROBE_EARLY_INPUT_RELEASE=(\S+)',command)==(['1'] if early_input_release else [])
        assert not re.search(r'(?:^|\s)-UDBT_PROBE_EARLY_INPUT_RELEASE(?:\s|$)',command)
if early_input_release:
    assert stay_native and build_status.get('codec_aware_render') and build_status.get('async_audio') and build_status.get('pcm_copied') and build_status.get('unity_pcm_copy') and build_status.get('block_dsp')
    assert not build_status.get('matched_feedback_compare') and not volume_compare and not build_status.get('owner_pacer')
high_rate_pcm_rebalance=build_status.get('high_rate_pcm_rebalance',False)
assert type(high_rate_pcm_rebalance) is bool and ('DBT_PROBE_HIGH_RATE_PCM_REBALANCE:BOOL=ON' in cache)==high_rate_pcm_rebalance
for unit in (entry_name,'probe_output.c'):
    entries=[e['command'] for e in commands if Path(e['file']).name==unit]
    if high_rate_pcm_rebalance:assert len(entries)==1
    for command in entries:
        assert re.findall(r'(?:^|\s)-DDBT_PROBE_HIGH_RATE_PCM_REBALANCE=(\S+)',command)==(['1'] if high_rate_pcm_rebalance else [])
        assert not re.search(r'(?:^|\s)-UDBT_PROBE_HIGH_RATE_PCM_REBALANCE(?:\s|$)',command)
if high_rate_pcm_rebalance:
    assert early_input_release and type(build_status.get('pcm_target_ms')) is int and build_status['pcm_target_ms']==16
    assert 'DBT_PROBE_PCM_TARGET_MS:STRING=16' in cache and 'DBT_PROBE_LHDC_BUILD:BOOL=ON' in cache
    assert not build_status.get('lhdc_core0_render') and 'DBT_PROBE_LHDC_CORE0_RENDER:BOOL=ON' not in cache
matched_feedback=build_status.get('matched_feedback',False)
matched_compare=build_status.get('matched_feedback_compare',False)
for option,enabled in [('MATCHED_FEEDBACK',matched_feedback),('MATCHED_FEEDBACK_COMPARE',matched_compare)]:
    assert type(enabled) is bool and ('DBT_PROBE_'+option+':BOOL=ON' in cache)==enabled
    for command,macro in [(main_command,'DBT_PROBE_'+option),(fixed_volume_unit['command'],'FIXED_DSP_'+option)]:
        assert re.findall(r'(?:^|\s)-D'+macro+r'=(\S+)',command)==(['1'] if enabled else [])
        assert not re.search(r'(?:^|\s)-U'+macro+r'(?:\s|$)',command)
if matched_feedback:
    assert stay_native and build_status.get('block_dsp') and build_status.get('word_dsp') and not diagnostic and not pipeline
if matched_compare:
    assert re.findall(r'-std=(\S+)',fixed_volume_unit['command'])[-1:]==['gnu11'], 'SDK cycle helper requires diagnostic GNU C11'
    assert matched_feedback and not volume_compare and not build_status.get('owner_pacer')
    logger_command=next(e['command'] for e in commands if Path(e['file']).name=='probe_audio.c')
    assert re.findall(r'(?:^|\s)-DDBT_PROBE_MATCHED_FEEDBACK_COMPARE=(\S+)',logger_command)==['1']
    assert not re.search(r'(?:^|\s)-UDBT_PROBE_MATCHED_FEEDBACK_COMPARE(?:\s|$)',logger_command)
owner_pacer=bool(build_status.get('owner_pacer'))
assert ('DBT_PROBE_OWNER_PACER:BOOL=ON' in cache)==owner_pacer, 'Owner budget receipt/cache mismatch'
assert re.findall(r'(?:^|\s)-DDBT_PROBE_OWNER_PACER=(\S+)',main_command)==(['1'] if owner_pacer else [])
pacer_units=[e for e in commands if Path(e['file']).name=='probe_owner_pacer.c']
assert bool(pacer_units)==owner_pacer, 'Unexpected or missing owner budget unit'
if owner_pacer:
    assert build_status.get('stay_native') and build_status.get('priority_order') and build_status.get('async_audio') and build_status.get('pcm_copied') and build_status.get('codec_aware_render')
    assert not diagnostic and not pipeline
    for unit in ('probe_main.c','probe_audio.c','probe_owner_pacer.c'):
        command=next(e['command'] for e in commands if Path(e['file']).name==unit)
        assert re.findall(r'(?:^|\s)-DDBT_PROBE_OWNER_PACER=(\S+)',command)==['1']
        assert not re.search(r'(?:^|\s)-UDBT_PROBE_OWNER_PACER(?:\s|$)',command)
dsp_inline=bool(build_status.get('dsp_inline_check'))
assert ('DBT_PROBE_DSP_INLINE_CHECK:BOOL=ON' in cache)==dsp_inline, 'DSP inline receipt/cache mismatch'
assert bool(re.search(r'(?:^|\s)-DDBT_PROBE_DSP_INLINE_CHECK=1(?:\s|$)',main_command))==dsp_inline
if dsp_inline:
    assert diagnostic and pipeline and build_status.get('audio_stage_profile') and not build_status.get('dsp_stage_profile') and not build_status.get('fir_compiler_check')
    inline_command=next(e['command'] for e in commands if Path(e['file']).name=='probe_dsp_inline.c')
    compare_command=next(e['command'] for e in commands if Path(e['file']).name=='probe_dsp_compare.c')
    for command in (inline_command,compare_command):
        assert re.search(r'(?:^|\s)-DDBT_PROBE_DSP_INLINE_CHECK=1(?:\s|$)',command), 'Inline mode not actually compiled'
    assert re.findall(r'(?:^|\s)-O(?:[0-3sgz]|fast)(?=\s)',inline_command)[-1].strip()=='-Os'
decoder_core=build_status.get('decoder_core',1)
assert type(decoder_core) is int and decoder_core in (0,1)
core_option=re.search(r'^DBT_PROBE_DECODER_CORE:STRING=([01])$',cache,re.M)
if core_option:
    assert int(core_option.group(1))==decoder_core, 'Decoder affinity receipt/cache mismatch'
    for entry in commands:
        if Path(entry['file']).name=='probe_audio.c':
            assert re.search(r'(?:^|\s)-DDBT_PROBE_DECODER_CORE='+str(decoder_core)+r'(?:\s|$)',entry['command']), 'Decoder affinity was not actually compiled'
else:
    assert decoder_core==1, 'Missing explicit decoder affinity'
decoder_priority=build_status.get('decoder_priority',5)
assert type(decoder_priority) is int and decoder_priority in ((21,) if stay_native else (5,6)), 'Invalid decoder priority receipt'
priority_option=re.search(r'^DBT_PROBE_DECODER_PRIORITY:STRING=(\S+)$',cache,re.M)
if priority_option:
    assert priority_option.group(1)==str(decoder_priority), 'Decoder priority receipt/cache mismatch'
    for entry in commands:
        if Path(entry['file']).name=='probe_audio.c':
            assert re.findall(r'(?:^|\s)-DDBT_PROBE_DECODER_PRIORITY=(\S+)',entry['command'])==[str(decoder_priority)], 'Decoder priority not actually compiled'
            assert not re.search(r'(?:^|\s)-UDBT_PROBE_DECODER_PRIORITY(?:\s|$)',entry['command'])
else:
    assert decoder_priority==5, 'Missing explicit decoder priority'
    for entry in commands:
        if Path(entry['file']).name=='probe_audio.c':
            assert re.findall(r'(?:^|\s)-DDBT_PROBE_DECODER_PRIORITY=(\S+)',entry['command']) in ([],['5']), 'Unreceipted decoder priority'
            assert not re.search(r'(?:^|\s)-UDBT_PROBE_DECODER_PRIORITY(?:\s|$)',entry['command'])
if decoder_priority!=5:
    assert priority_option and build_status.get('async_audio') and not diagnostic and not pipeline and not build_status.get('rf_protocol_diagnostic')
    assert ('DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' not in cache or (build_status.get('stay_native') is True and 'DBT_PROBE_STAY_NATIVE:BOOL=ON' in cache))
    assert any(Path(e['file']).name=='probe_audio.c' for e in commands), 'Priority experiment has no decoder task unit'
render_fair_share=build_status.get('render_fair_share',False)
assert type(render_fair_share) is bool and ('DBT_PROBE_RENDER_FAIR_SHARE:BOOL=ON' in cache)==render_fair_share, 'Renderer sharing receipt/cache mismatch'
if render_fair_share:
    assert stay_native and build_status.get('priority_order') and build_status.get('codec_aware_render') and build_status.get('async_audio') and build_status.get('pcm_copied') and not build_status.get('owner_pacer')
    assert type(build_status.get('auxiliary_renderer_priority')) is int and build_status['auxiliary_renderer_priority']==20
for unit in (entry_name,'probe_audio.c','probe_output.c'):
    compiled=[e['command'] for e in commands if Path(e['file']).name==unit]
    if render_fair_share: assert len(compiled)==1, 'Missing fair-sharing unit'
    for command in compiled:
        assert re.findall(r'(?:^|\s)-DDBT_PROBE_RENDER_FAIR_SHARE=(\S+)',command)==(['1'] if render_fair_share else [])
        assert not re.search(r'(?:^|\s)-UDBT_PROBE_RENDER_FAIR_SHARE(?:\s|$)',command)
priority_order=build_status.get('priority_order',False)
assert type(priority_order) is bool and ('DBT_PROBE_PRIORITY_ORDER:BOOL=ON' in cache)==priority_order, 'Priority order receipt/cache mismatch'
host_priority=build_status.get('host_priority',6)
renderer_priority=build_status.get('renderer_priority',6)
assert type(host_priority) is int and host_priority==(22 if stay_native else 7 if priority_order else 6), 'Invalid host priority receipt'
assert type(renderer_priority) is int and renderer_priority==(22 if render_fair_share else 20 if stay_native else 5 if priority_order else 6), 'Invalid renderer priority receipt'
schedule_units=[e for e in commands if Path(e['file']).name in (entry_name,'probe_audio.c','probe_output.c')]
for entry in schedule_units:
    assert re.findall(r'(?:^|\s)-DDBT_PROBE_PRIORITY_ORDER=(\S+)',entry['command'])==(['1'] if priority_order else []), 'Scheduling mode not actually compiled'
    assert not re.search(r'(?:^|\s)-UDBT_PROBE_PRIORITY_ORDER(?:\s|$)',entry['command'])
if priority_order:
    assert decoder_priority==(21 if stay_native else 6) and not diagnostic and not pipeline and build_status.get('async_audio')
    assert not build_status.get('rf_protocol_diagnostic') and ('DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' not in cache or (build_status.get('stay_native') is True and 'DBT_PROBE_STAY_NATIVE:BOOL=ON' in cache))
    assert {Path(e['file']).name for e in schedule_units}=={entry_name,'probe_audio.c','probe_output.c'}, 'Missing ordered task unit'
owner_ready_yield=build_status.get('owner_ready_yield',False)
assert type(owner_ready_yield) is bool and owner_ready_yield==(render_fair_share or decoder_priority==host_priority), 'Owner ready-yield receipt mismatch'
if owner_ready_yield and not render_fair_share:
    assert re.findall(r'(?:^|\s)-DDBT_PROBE_DECODER_PRIORITY=(\S+)',main_command)==['6'], 'Owner yield not actually compiled'
    assert not re.search(r'(?:^|\s)-UDBT_PROBE_DECODER_PRIORITY(?:\s|$)',main_command)
logger_core=build_status.get('logger_core',1-decoder_core)
assert type(logger_core) is int and logger_core in (0,1)
logger_option=re.search(r'^DBT_PROBE_LOGGER_CORE:STRING=([01])$',cache,re.M)
if logger_option:
    assert int(logger_option.group(1))==logger_core, 'Logger affinity receipt/cache mismatch'
    for entry in commands:
        if Path(entry['file']).name=='probe_audio.c':
            assert re.findall(r'(?:^|\s)-DDBT_PROBE_LOGGER_CORE=(\S+)',entry['command'])==[str(logger_core)], 'Logger affinity not actually compiled'
            assert not re.search(r'(?:^|\s)-UDBT_PROBE_LOGGER_CORE(?:\s|$)',entry['command'])
else:
    assert logger_core==1-decoder_core, 'Missing explicit logger affinity'
if stay_native:
    assert decoder_core==0 and logger_core==1, 'Product decoder must be isolated on CPU0'
    assert [build_status.get(k) for k in ('bluetooth_core','renderer_core','background_core')]==[1,1,1], 'Product core-layout receipt mismatch'
    compiled_config=(a.image.parent/'config/sdkconfig.h').read_text()
    for key,value in {'CONFIG_BTDM_CTRL_PINNED_TO_CORE':1,'CONFIG_ESP_MAIN_TASK_AFFINITY':1,
                      'CONFIG_ESP_TIMER_TASK_AFFINITY':0,'CONFIG_FREERTOS_TIMER_SERVICE_TASK_CORE_AFFINITY':1,
                      'CONFIG_LWIP_TCPIP_TASK_AFFINITY':1,'CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1':1}.items():
        match=re.search(r'^#define '+key+r' (0x[0-9a-fA-F]+|[0-9]+)$',compiled_config,re.M)
        assert match and int(match.group(1),0)==value, 'Compiled SDK affinity mismatch: '+key
rf_diagnostic=bool(build_status.get('rf_protocol_diagnostic'))
assert ('DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC:BOOL=ON' in cache)==rf_diagnostic
assert bool(re.search(r'(?:^|\s)-DDBT_PROBE_RF_PROTOCOL_DIAGNOSTIC=1(?:\s|$)',main_command))==rf_diagnostic
if rf_diagnostic:
    probe_command=next(e['command'] for e in commands if Path(e['file']).name=='probe_audio.c')
    assert re.search(r'(?:^|\s)-DDBT_PROBE_RF_PROTOCOL_DIAGNOSTIC=1(?:\s|$)',probe_command)
    assert not diagnostic and not pipeline and not build_status.get('async_audio')
    assert ('DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' not in cache or (build_status.get('stay_native') is True and 'DBT_PROBE_STAY_NATIVE:BOOL=ON' in cache))
    assert build_status.get('product_io') and build_status.get('product_security')
assert ('DBT_PROBE_CODEC_DIAGNOSTIC_ONLY:BOOL=ON' in cache)==diagnostic
assert ('DBT_PROBE_DEFERRED_CODEC_CHECK:BOOL=ON' in cache)==bool(build_status.get('deferred_codec_check'))
assert ('DBT_PROBE_ASYNC_AUDIO:BOOL=ON' in cache)==bool(build_status.get('async_audio'))
assert bool(re.search(r'(?:^|\s)-DDBT_PROBE_ASYNC_AUDIO=1(?:\s|$)',main_command))==bool(build_status.get('async_audio'))
pcm_loans=bool(build_status.get('pcm_loans'))
assert ('DBT_PROBE_PCM_LOANS:BOOL=ON' in cache)==pcm_loans, 'PCM loan receipt/cache mismatch'
assert bool(re.search(r'(?:^|\s)-DDBT_PROBE_PCM_LOANS=1(?:\s|$)',main_command))==pcm_loans
if pcm_loans:
    assert build_status.get('async_audio') and not rf_diagnostic
    assert ('DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' not in cache or (build_status.get('stay_native') is True and 'DBT_PROBE_STAY_NATIVE:BOOL=ON' in cache))
    for loan_unit in ('probe_output.c','probe_pipeline_check.c' if pipeline else 'probe_audio.c'):
        loan_command=next(entry['command'] for entry in commands if Path(entry['file']).name==loan_unit)
        assert re.search(r'(?:^|\s)-DDBT_PROBE_PCM_LOANS=1(?:\s|$)',loan_command), 'PCM loan provider/client not actually compiled'
pcm_copied=bool(build_status.get('pcm_copied'))
assert ('DBT_PROBE_PCM_COPIED:BOOL=ON' in cache)==pcm_copied, 'Compact PCM receipt/cache mismatch'
assert re.findall(r'(?:^|\s)-DDBT_PROBE_PCM_COPIED=(\S+)',main_command)==(['1'] if pcm_copied else [])
for entry in commands:
    if Path(entry['file']).name in ('probe_output.c','probe_audio.c','probe_pipeline_check.c'):
        assert re.findall(r'(?:^|\s)-DDBT_PROBE_PCM_COPIED=(\S+)',entry['command'])==(['1'] if pcm_copied else [])
        assert not re.search(r'(?:^|\s)-UDBT_PROBE_PCM_COPIED(?:\s|$)',entry['command'])
if pcm_copied:
    assert not pcm_loans and build_status.get('async_audio') and not rf_diagnostic and ('DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' not in cache or (build_status.get('stay_native') is True and 'DBT_PROBE_STAY_NATIVE:BOOL=ON' in cache))
    for unit in ('probe_output.c','probe_pipeline_check.c' if pipeline else 'probe_audio.c'):
        assert any(Path(e['file']).name==unit for e in commands), 'Missing compact PCM provider/client'
pcm_target_ms=build_status.get('pcm_target_ms',64)
assert type(pcm_target_ms) is int and pcm_target_ms in (16,32,64), 'Invalid PCM target receipt'
pcm_target_option=re.search(r'^DBT_PROBE_PCM_TARGET_MS:STRING=(16|32|64)$',cache,re.M)
if pcm_target_option:
    assert int(pcm_target_option.group(1))==pcm_target_ms, 'PCM target receipt/cache mismatch'
    for entry in commands:
        if Path(entry['file']).name in (entry_name,'probe_output.c','probe_audio.c','probe_pipeline_check.c'):
            assert re.findall(r'(?:^|\s)-DDBT_PROBE_PCM_TARGET_MS=(\S+)',entry['command'])==[str(pcm_target_ms)], 'PCM target missing, overridden or not actually compiled'
            assert not re.search(r'(?:^|\s)-UDBT_PROBE_PCM_TARGET_MS(?:\s|$)',entry['command']), 'PCM target undefined after selection'
else:
    assert pcm_target_ms==64, 'Missing explicit short PCM target'
    for entry in commands:
        if Path(entry['file']).name in (entry_name,'probe_output.c','probe_audio.c','probe_pipeline_check.c'):
            assert re.findall(r'(?:^|\s)-DDBT_PROBE_PCM_TARGET_MS=(\S+)',entry['command']) in ([],['64']), 'Unreceipted short PCM target'
if pcm_target_ms!=64:
    assert (pcm_loans or pcm_copied) and build_status.get('async_audio') and not rf_diagnostic
    assert ('DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' not in cache or (build_status.get('stay_native') is True and 'DBT_PROBE_STAY_NATIVE:BOOL=ON' in cache)) and pcm_target_option
block_dsp=bool(build_status.get('block_dsp'))
assert ('DBT_PROBE_BLOCK_DSP:BOOL=ON' in cache)==block_dsp, 'Block DSP receipt/cache mismatch'
assert re.findall(r'(?:^|\s)-DDBT_PROBE_BLOCK_DSP=(\S+)',main_command)==(['1'] if block_dsp else []) and not re.search(r'(?:^|\s)-UDBT_PROBE_BLOCK_DSP(?:\s|$)',main_command)
block_core_commands=[e for e in commands if Path(e['file']).name in ('dbt_audio.c','fixed_dsp.c')]
for entry in block_core_commands:
    for definition in ('DBT_AUDIO_BLOCK_DSP','FIXED_DSP_BLOCK_API'):
        assert re.findall(r'(?:^|\s)-D'+definition+r'=(\S+)',entry['command'])==(['1'] if block_dsp else []), 'Block core mode missing, overridden or unreceipted'
        assert not re.search(r'(?:^|\s)-U'+definition+r'(?:\s|$)',entry['command'])
if block_dsp:
    assert (pcm_loans or pcm_copied) and build_status.get('async_audio') and not rf_diagnostic and ('DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' not in cache or (build_status.get('stay_native') is True and 'DBT_PROBE_STAY_NATIVE:BOOL=ON' in cache))
    assert {Path(e['file']).name for e in block_core_commands}=={'dbt_audio.c','fixed_dsp.c'}, 'Missing compiled block core'
assert bool(re.search(r'(?:^|\s)-DDBT_PROBE_DEFERRED_CODEC_CHECK=1(?:\s|$)',main_command))==bool(build_status.get('deferred_codec_check'))
word_dsp=build_status.get('word_dsp',False)
assert type(word_dsp) is bool and ('DBT_PROBE_WORD_DSP:BOOL=ON' in cache)==word_dsp, 'Word DSP receipt/cache mismatch'
assert re.findall(r'(?:^|\s)-DDBT_PROBE_WORD_DSP=(\S+)',main_command)==(['1'] if word_dsp else [])
assert not re.search(r'(?:^|\s)-UDBT_PROBE_WORD_DSP(?:\s|$)',main_command)
for entry in block_core_commands:
    assert re.findall(r'(?:^|\s)-DFIXED_DSP_WORD_QUANT=(\S+)',entry['command'])==(['1'] if word_dsp else []), 'Word kernel not actually compiled'
    assert not re.search(r'(?:^|\s)-UFIXED_DSP_WORD_QUANT(?:\s|$)',entry['command'])
if word_dsp:
    assert block_dsp and build_status.get('async_audio') and not rf_diagnostic
    assert ('DBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON' not in cache or (build_status.get('stay_native') is True and 'DBT_PROBE_STAY_NATIVE:BOOL=ON' in cache))
    assert {Path(e['file']).name for e in block_core_commands}=={'dbt_audio.c','fixed_dsp.c'}
assert ('DBT_PROBE_LHDC_MAX_CHECK:BOOL=ON' in cache)==bool(build_status.get('lhdc_maximum_check'))
assert ('DBT_PROBE_LHDC_STAGE_PROFILE:BOOL=ON' in cache)==bool(build_status.get('lhdc_stage_profile'))
if build_status.get('lhdc_stage_profile'):
    assert re.search(r'(?:^|\s)-DDBT_PROBE_LHDC_STAGE_PROFILE=1(?:\s|$)',main_command)
    for name in ('dbt_lhdcv5_codec.c','lhdc_dec.c'):
        cmd=next(e['command'] for e in commands if Path(e['file']).name==name)
        assert re.search(r'(?:^|\s)-DDBT_LHDC_STAGE_PROFILE=1(?:\s|$)',cmd)
        assert re.search(r'(?:^|\s)-DDBT_LHDC_STAGE_CLOCK_CYCLES=1(?:\s|$)',cmd)
    assert any(Path(e['file']).name=='probe_lhdc_profile.c' for e in commands)
    if not build_status.get('lhdc_maximum_check'):
        assert build_status.get('stay_native') and build_status.get('async_audio') and build_status.get('audio_stage_profile') and not diagnostic and not pipeline

assert ('DBT_PROBE_PIPELINE_DIAGNOSTIC:BOOL=ON' in cache)==pipeline
audio_profile=bool(build_status.get('audio_stage_profile'))
assert ('DBT_PROBE_AUDIO_STAGE_PROFILE:BOOL=ON' in cache)==audio_profile
audio_command=next(entry['command'] for entry in commands if Path(entry['file']).name=='dbt_audio.c')
assert bool(re.search(r'(?:^|\s)-DDBT_AUDIO_STAGE_PROFILE=1(?:\s|$)',audio_command))==audio_profile
assert bool(re.search(r'(?:^|\s)-DDBT_AUDIO_STAGE_PROFILE=1(?:\s|$)',main_command))==audio_profile
dsp_profile=bool(build_status.get('dsp_stage_profile'))
assert ('DBT_PROBE_DSP_STAGE_PROFILE:BOOL=ON' in cache)==dsp_profile
assert bool(re.search(r'(?:^|\s)-DDBT_DSP_STAGE_PROFILE=1(?:\s|$)',main_command))==dsp_profile
if dsp_profile:assert audio_profile
fir_check=bool(build_status.get('fir_compiler_check'))
assert ('DBT_PROBE_FIR_COMPILER_CHECK:BOOL=ON' in cache)==fir_check
assert bool(re.search(r'(?:^|\s)-DDBT_PROBE_FIR_COMPILER_CHECK=1(?:\s|$)',main_command))==fir_check
if fir_check:assert audio_profile and not dsp_profile
if fir_check:
    for variant,optimization in (('wide','-O3'),('os','-Os'),('o2','-O2'),('o3','-O3'),('fold','-O3'),('pair','-O3')):
        variant_command=next(e['command'] for e in commands if Path(e['file']).name==f'probe_fir_{variant}.c')
        assert re.findall(r'(?:^|\s)(-O\S+)',variant_command)[-1]==optimization
elif audio_profile and not stay_native:
    assert pipeline
    candidate=('profile','-Os') if dsp_profile else ('inline','-Os') if dsp_inline else ('o3','-O3')
    for variant,optimization in (('os','-Os'),candidate):
        variant_command=next(entry['command'] for entry in commands if Path(entry['file']).name==f'probe_dsp_{variant}.c')
        assert re.findall(r'(?:^|\s)(-O\S+)',variant_command)[-1]==optimization
    if not dsp_profile and not dsp_inline:
        for block_variant in ('block','word'):
            block_command=next(e['command'] for e in commands if Path(e['file']).name==f'probe_dsp_{block_variant}.c')
            assert re.findall(r'(?:^|\s)(-O\S+)',block_command)[-1]=='-Os'
if audio_profile and stay_native:
    assert not diagnostic and not pipeline and not dsp_profile and not dsp_inline and not fir_check
    assert not any(Path(e['file']).name.startswith(('probe_dsp_', 'probe_fir_', 'probe_pipeline_check')) for e in commands), 'Passive profile must not compile benchmark workloads'
assert bool(re.search(r'(?:^|\s)-DDBT_PROBE_PIPELINE_DIAGNOSTIC=1(?:\s|$)',main_command))==pipeline
if pipeline:
    assert diagnostic and build_status.get('async_audio')
    assert 'DBT_PROBE_AUDIO_OUTPUT:BOOL=ON' in cache and 'DBT_PROBE_LHDC_BUILD:BOOL=ON' in cache
    assert any(Path(entry['file']).name=='probe_pipeline_check.c' for entry in commands)
if diagnostic:
    for feature in ('KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','SMP','AVRCP','RFCOMM','AUDIO_TEST','GATT_TEST','CLASSIC_SECURITY_TEST','INGRESS_TRACE'):
        assert f'DBT_PROBE_{feature}:BOOL=ON' not in cache, feature
    if not pipeline:assert 'DBT_PROBE_AUDIO_OUTPUT:BOOL=ON' not in cache
product_security=bool(re.search(r'(?:^|\s)-DDBT_PROBE_PRODUCT_SECURITY=1(?:\s|$)',main_command))
rfcomm=bool(re.search(r'(?:^|\s)-DDBT_PROBE_RFCOMM=1(?:\s|$)',main_command))
assert product_security==build_status['product_security'] and rfcomm==build_status['rfcomm']
blob=a.image.read_bytes();assert len(blob)>256 and blob[0]==0xe9
assert hashlib.sha256(blob).hexdigest()==build_status['image_sha256']
image=ESP32FirmwareImage(io.BytesIO(blob));image.verify();assert image.chip_id==0
position=24;checksum=0xef
for _ in range(blob[1]):
    assert position+8<=len(blob)
    address,length=struct.unpack_from('<II',blob,position);position+=8;assert position+length<=len(blob)
    for value in blob[position:position+length]:checksum^=value
    position+=length
checksum_at=position+((15-position)%16);assert blob[checksum_at]==checksum
body_end=checksum_at+1
if image.append_digest:assert hashlib.sha256(blob[:body_end]).digest()==blob[body_end:body_end+32]
assert blob[32:36]==b'\x32\x54\xcd\xab'
project=blob[80:112].split(b'\0',1)[0].decode();assert project=='native_bt_probe'
config_bytes=(a.image.parent/'config/sdkconfig.h').read_bytes()
assert hashlib.sha256(config_bytes).hexdigest()==build_status['config_sha256']
config=config_bytes.decode()
for key in ('CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS','CONFIG_FREERTOS_USE_TRACE_FACILITY','CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS','CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID'):
    assert re.findall(r'^#define '+key+r' (\S+)\r?$',config,re.M)==(['1'] if task_runtime_stats else []), 'Kernel accounting mismatch: '+key
if task_runtime_stats:
    for key in ('CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER','CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U32'):
        assert re.findall(r'^#define '+key+r' (\S+)\r?$',config,re.M)==['1']
    for key in ('CONFIG_FREERTOS_RUN_TIME_STATS_USING_CPU_CLK','CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64'):
        assert not re.findall(r'^#define '+key+r' (\S+)\r?$',config,re.M)
if console_irq:
    for key,value in [('CONFIG_ESP_CONSOLE_UART','1'),('CONFIG_ESP_CONSOLE_UART_NUM','0')]:
        assert re.findall(r'^#define '+key+r' (\S+)\r?$',config,re.M)==[value]
    console_baud_values=re.findall(r'^#define CONFIG_ESP_CONSOLE_UART_BAUDRATE (\d+)\r?$',config,re.M)
    assert len(console_baud_values)==1 and int(console_baud_values[0]) in (115200,230400,460800,921600)
    assert build_status.get('console_baud')==int(console_baud_values[0]), 'Console baud receipt/config mismatch'
assert '#define CONFIG_BT_CONTROLLER_ONLY 1' in config
assert '#define CONFIG_BTDM_CTRL_MODE_BTDM 1' in config
assert '#define CONFIG_BT_BLUEDROID_ENABLED 1' not in config
assert '#define CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK 1' not in config
if product_security:
    assert '#define CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0 1' in config
    assert any(Path(e['file']).name=='probe_reset_dispatch.c' for e in commands)
receipt=json.loads((ROOT/'_build/hardware/COM6-original-flash.json').read_text())
slot=next(x for x in receipt['partitions'] if x['name']=='ota_1')
assert slot['offset']==0x390000 and slot['size']==0x280000 and len(blob)<=slot['size']
table=(a.image.parent/'partition_table/partition-table.bin').read_bytes()
assert hashlib.sha256(table).hexdigest()==build_status['partition_sha256']
original=(ROOT/'_build/hardware/COM6-original-flash.bin').read_bytes()
assert hashlib.sha256(original).hexdigest()==receipt['sha256']
assert table==original[0x8000:0x8000+len(table)]
assert image.flash_mode==2 and image.flash_size_freq==0x3f
assert image.min_rev_full<=301<=image.max_rev_full
dsp_assist=build_status.get('dsp_assist',False)
dsp_assist_blocks=build_status.get('dsp_assist_blocks',4)
assert type(dsp_assist) is bool and ('DBT_PROBE_DSP_ASSIST:BOOL=ON' in cache)==dsp_assist, 'DSP helper receipt/cache mismatch'
assert type(dsp_assist_blocks) is int and dsp_assist_blocks in (4,5,6), 'DSP helper batch must be4,5 or6'
assert re.findall(r'^DBT_PROBE_DSP_ASSIST_BLOCKS:STRING=(\S+)\r?$',cache,re.M)==[str(dsp_assist_blocks)], 'DSP helper batch receipt/cache mismatch'
assert dsp_assist or dsp_assist_blocks==4, 'Nondefault batch requires DSP helper'
for unit in (entry_name,'probe_output.c','probe_audio.c'):
    entries=[e['command'] for e in commands if Path(e['file']).name==unit]
    if dsp_assist:assert len(entries)==1
    for command in entries:
        for macro,expected in [('DBT_PROBE_DSP_ASSIST',['1'] if dsp_assist else []),('DBT_PROBE_DSP_ASSIST_BLOCKS',[str(dsp_assist_blocks)] if dsp_assist else [])]:
            assert re.findall(r'(?:^|\s)-D'+macro+r'=(\S+)',command)==expected, 'DSP helper command mismatch: '+unit+' '+macro
            assert not re.search(r'(?:^|\s)-U'+macro+r'(?:\s|$)',command), 'DSP helper command override: '+macro
if dsp_assist:
    assert not diagnostic and not pipeline
    for key in ('stay_native','codec_aware_render','async_audio','pcm_copied','block_dsp','unity_pcm_copy'):
        assert build_status.get(key) is True, 'DSP helper dependency missing: '+key
        assert 'DBT_PROBE_'+key.upper()+':BOOL=ON' in cache, 'DSP helper dependency cache mismatch: '+key
    assert 'DBT_PROBE_LHDC_BUILD:BOOL=ON' in cache
    assert not build_status.get('lhdc_core0_render') and 'DBT_PROBE_LHDC_CORE0_RENDER:BOOL=ON' not in cache
    assert not build_status.get('aptx_snapshot') and 'DBT_PROBE_APTX_SNAPSHOT:BOOL=ON' not in cache
report={'passed':True,'dsp_assist':dsp_assist,'dsp_assist_blocks':dsp_assist_blocks,'scheduler_trace':scheduler_trace,'task_runtime_stats':task_runtime_stats,'task_cycle_accounting':task_cycle_accounting,'console_irq':console_irq,'stay_native':stay_native,'priority_order':priority_order,'host_priority':host_priority,'decoder_priority':decoder_priority,'renderer_priority':renderer_priority,'early_input_release':early_input_release,'high_rate_pcm_rebalance':high_rate_pcm_rebalance,'matched_feedback':matched_feedback,'matched_feedback_compare':matched_compare,'render_fair_share':render_fair_share,'auxiliary_renderer_priority':build_status.get('auxiliary_renderer_priority',renderer_priority),'owner_ready_yield':owner_ready_yield,'scope':'offline image checksum/digest, ESP32 revision and admitted OTA capacity; no device access or runtime qualification',
 'image_sha256':hashlib.sha256(blob).hexdigest(),'image_bytes':len(blob),'project':project,'chip_id':image.chip_id,'codec_diagnostic_only':diagnostic,'pipeline_diagnostic':pipeline,
 'rf_protocol_diagnostic':rf_diagnostic,'pcm_target_ms':pcm_target_ms,'pcm_copied':pcm_copied,'logger_core':logger_core,'block_dsp':block_dsp,'word_dsp':word_dsp,'experimental_short_pcm_target':pcm_target_ms!=64,'revision_range':[image.min_rev_full,image.max_rev_full],'checksum_verified':True,'digest_verified':bool(image.append_digest),
 'latest_local_build_completed':True,'build_sources_match':True,'build_source_units':source_count,'product_io':bool(build_status.get('product_io')),'product_security':product_security,'rfcomm_profile':rfcomm,'controller_only_dual_mode':True,'anti_rollback_fuse_updates':False,'partition_offset':slot['offset'],'partition_bytes':slot['size'],
 'partition_remaining_bytes':slot['size']-len(blob),'compiled_table_matches_preserved_layout':True,'compiled_table_sha256':hashlib.sha256(table).hexdigest(),'flash_mode':'DIO','flash_mib':8,'flash_mhz':80,'hardware_accessed':False,'runtime_qualified':False}
output=ROOT/'_build/host/local-image-validation.json';output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
