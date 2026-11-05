"""Offline ELF placement and direct-call audit; never opens hardware.

Indirect calls and vendor ROM internals are deliberately reported as unproven.
Run using the existing ESP-IDF Python environment (pyelftools).
"""
import argparse,hashlib,json,re,subprocess
from pathlib import Path
from elftools.elf.elffile import ELFFile

ROOT=Path(__file__).resolve().parents[1]
BUILD=ROOT/'_build/esp32'
CRITICAL=('vTaskGenericNotifyGiveFromISR','xQueueGenericSendFromISR',
          'xQueueGiveFromISR','xQueueReceiveFromISR','vTaskSwitchContext','xTaskIncrementTick')
TASK_ONLY=('xQueueReceive','xTaskCreatePinnedToCore','xTaskGenericNotify')

def ram(address):return 0x40080000<=address<0x400a0000
def rom(address):return 0x40000000<=address<0x40080000

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expect-task-flash',action='store_true')
    parser.add_argument('--build-dir',type=Path,default=BUILD)
    parser.add_argument('--output',type=Path,default=ROOT/'_build/host/iram-placement.json')
    args=parser.parse_args();build=args.build_dir;path=build/'native_bt_probe.elf'
    config_path=build/'config/sdkconfig.h'
    if not config_path.is_file():config_path=build/'sdkconfig.h'
    config=config_path.read_text(encoding='utf-8')
    assert '#define CONFIG_I2S_ISR_IRAM_SAFE 1' in config
    assert '#define CONFIG_FREERTOS_PLACE_ISR_FUNCTIONS_INTO_FLASH 1' not in config
    # IDF 6.1 uses a flash default and the opposite FREERTOS_IN_IRAM opt-in.
    # Actual linked addresses below remain the placement proof.
    if args.expect_task_flash:assert '#define CONFIG_FREERTOS_IN_IRAM 1' not in config
    with path.open('rb') as f:
        elf=ELFFile(f);symbols=[]
        for symbol in elf.get_section_by_name('.symtab').iter_symbols():
            if symbol['st_info']['type']=='STT_FUNC' and symbol['st_size']:
                symbols.append({'name':symbol.name,'address':symbol['st_value'],'size':symbol['st_size']})
    by_name={}
    for symbol in symbols:by_name.setdefault(symbol['name'],[]).append(symbol)
    protected=[]
    for name in CRITICAL:
        found=by_name.get(name,[]);assert found and all(ram(x['address']) for x in found),name
        protected.extend(found)
    roots=[s for s in symbols if ram(s['address']) and s['name'] in ('sent','i2s_dma_tx_callback')]
    assert any(s['name']=='sent' for s in roots) and any(s['name']=='i2s_dma_tx_callback' for s in roots)
    cache=(build/'CMakeCache.txt').read_text(encoding='utf-8')
    scheduler_trace='DBT_PROBE_SCHEDULER_TRACE:BOOL=ON' in cache
    if scheduler_trace:
        sampled=[s for s in symbols if s['name'].startswith('sample_tick')]
        assert sampled and all(ram(s['address']) for s in sampled), 'Sampler callbacks must be in IRAM'
        for name in ('xTaskGetCurrentTaskHandle','esp_timer_get_time'):
            getter=by_name.get(name,[])
            assert getter and all(ram(s['address']) for s in getter), 'ISR sampler dependency left in flash: '+name
            protected.extend(getter)
        roots.extend(sampled)
    task_symbols=[]
    for name in TASK_ONLY:
        found=by_name.get(name,[]);assert found,name
        if args.expect_task_flash:assert all(x['address']>=0x400d0000 for x in found),name
        task_symbols.extend(found)
    archiver=re.search(r'^CMAKE_C_COMPILER_AR:FILEPATH=(.+)$',cache,re.M)
    assert archiver, 'Compiler toolchain archiver not recorded'
    compiler=Path(archiver.group(1));objdump=compiler.with_name(compiler.name.replace('gcc-ar','objdump'))
    raw=subprocess.check_output([str(objdump),'-d','--start-address=0x40080000','--stop-address=0x400a0000',str(path)],text=True)
    instructions=[]
    for line in raw.splitlines():
        m=re.match(r'\s*([0-9a-f]+):\s+[0-9a-f]+\s+(.*)',line)
        if m:instructions.append((int(m[1],16),m[2]))
    by_address={s['address']:s for s in symbols};queue=roots[:];seen=set();edges=[];indirect=[];unsafe=[]
    while queue:
        caller=queue.pop()
        if caller['address'] in seen:continue
        seen.add(caller['address'])
        for address,insn in instructions:
            if not caller['address']<=address<caller['address']+caller['size']:continue
            m=re.match(r'call(?:0|4|8|12)\s+([0-9a-f]+)',insn)
            if m:
                target=int(m[1],16);callee=by_address.get(target);edges.append({'caller':caller['name'],'target':hex(target),'callee':callee['name'] if callee else None})
                if not(ram(target) or rom(target)):unsafe.append(edges[-1])
                if ram(target) and callee and (callee['name'].startswith(('dbt_audio_','sample_tick')) or callee['name'] in ('i2s_dma_tx_callback','finish_sent','xTaskGetCurrentTaskHandle')):queue.append(callee)
            if re.match(r'callx(?:0|4|8|12)\s',insn):indirect.append({'caller':caller['name'],'address':hex(address),'instruction':insn})
    report={'passed':not unsafe,'scope':'offline protected-symbol placement and direct-call scan of I2S/owned audio callback functions; SDK calls are terminal nodes; vendor internals, indirect calls, ROM internals, real cache stalls and timing unproven',
        'elf_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'task_functions_in_flash':args.expect_task_flash,'scheduler_trace':scheduler_trace,
        'protected_symbols':protected,'task_only_symbols':task_symbols,'i2s_roots':roots,'direct_functions_walked':len(seen),
        'direct_edges':edges,'unresolved_indirect_calls':indirect,'unsafe_direct_calls':unsafe,'hardware_accessed':False}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'passed':report['passed'],'protected_symbols':len(protected),'direct_functions_walked':len(seen),'indirect_calls_unproven':len(indirect),'unsafe':unsafe},indent=2))
    return int(bool(unsafe))

if __name__=='__main__':raise SystemExit(main())
