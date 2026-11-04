"""Record browser/media progress and native faults as separate observations."""
import hashlib
import json
import argparse
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
HARDWARE=ROOT/'_build/hardware'


def read(name):
    return json.loads((HARDWARE/name).read_text(encoding='utf-8'))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',default='linux-firefox-sbc-fixed.json')
    parser.add_argument('--firmware',default='COM6-probe-firefox.json')
    parser.add_argument('--label',default='firefox-sbc-fixed')
    parser.add_argument('--report',default='firefox-output-validation.json')
    args=parser.parse_args()
    for value in [args.source,args.firmware,args.report,args.label]:
        assert value and Path(value).name==value and '/' not in value and '\\' not in value
    source=read(args.source)
    firmware=read(args.firmware)
    window=read(args.label+'-COM6-115200.json')
    result=source['result']
    assert source['return_code']==0 and result['passed']
    assert source['firmware']['candidate_sha256']==firmware['candidate_sha256']
    assert firmware['boot_ready_marker'] and firmware['original_application_preserved']
    assert all(item['readback_matches'] for item in firmware['writes'])
    assert not window['byte_limit_reached']
    assert window['start_unix']<result['start_unix']<result['end_unix']<window['end_unix']
    assert result['original_default_sink']==result['final_default_sink']
    assert result['firefox_inputs'] and all(row['application']=='Firefox' for row in result['firefox_inputs'])
    assert all(row['sink']==result['sink']['index'] for row in result['firefox_inputs'])
    assert result['att_writes']>=100 and result['att_readbacks']>=10 and result['notifications']>=100
    states=[row for row in result['browser_states'] if result['overlap_start_unix']<=row['received_unix']<=result['overlap_end_unix']]
    assert len(states)>=30 and states[-1]['time']-states[0]['time']>=19
    assert not any(row['paused'] or row['error'] or row['event'] in ['waiting','stalled','ended','error'] for row in states)
    capture=HARDWARE/(args.label+'-COM6-115200.txt')
    rows=[json.loads(line) for line in capture.read_text(encoding='utf-8').splitlines() if line.startswith('{"probe":')]
    pcm=[row for row in rows if row['probe']=='sbc_pcm']
    output=[row for row in rows if row['probe']=='native_i2s']
    faults=[row for row in rows if row['probe']=='failed']
    parsers=[row for row in rows if row['probe']=='parser_failure']
    assert pcm and output and any(row['running'] and row['dma_eofs'] for row in output)
    assert pcm[-1]['frames']>7000 and pcm[-1]['resident']==output[-1]['requested_bytes']==0
    decoder_errors=max(row['drops']+row['resets'] for row in pcm)
    output_errors=max(row['failed']+row['descriptor_errors']+row['underrun_frames'] for row in output)
    passed=not faults and not decoder_errors and not output_errors and not result.get('disconnect_error')
    report={'scope':'finite isolated Firefox/PipeWire SBC + continuous ATT; digital output counters, no acoustic qualification',
        'passed':passed,'browser_workload_passed':True,'firmware':source['firmware'],
        'overlap_seconds':result['overlap_seconds'],'codec_configurations':[row['configuration'] for row in result['transports']],
        'sample_specification':result['sink']['sample_specification'],
        'att_writes':result['att_writes'],'att_readbacks':result['att_readbacks'],'notifications':result['notifications'],
        'sbc_frames':pcm[-1]['frames'],'pcm_bytes':pcm[-1]['bytes'],
        'recorded_decoder_drops':max(row['drops'] for row in pcm),'recorded_sequence_resets':max(row['resets'] for row in pcm),
        'maximum_worker_requested_bytes':max(row['resident'] for row in pcm),
        'maximum_output_requested_bytes':max(row['requested_bytes'] for row in output),
        'recorded_output_underrun_frames':max(row['underrun_frames'] for row in output),
        'recorded_descriptor_errors':max(row['descriptor_errors'] for row in output),
        'final_output':output[-1],'host_faults':faults,'parser_failures':parsers,'disconnect_failed':bool(result.get('disconnect_error')),
        'fault_timing_limit':'UART receipt has no per-row wall timestamps; do not assert an exact fault time relative to the browser overlap',
        'uart_complete_window':True,'uart_capture_sha256':hashlib.sha256(capture.read_bytes()).hexdigest(),
        'physical_i2s_capture':False,'acoustic_test':False,'physical_nvs_stress':False,
        'all_codec_qualification':False,'replacement_ready':False}
    (HARDWARE/args.report).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
    return 0 if passed else 1


if __name__=='__main__':
    raise SystemExit(main())
