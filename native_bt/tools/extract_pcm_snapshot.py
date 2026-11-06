"""Extract an opt-in aptX HD PCM snapshot from archived monitor JSONL. No hardware IO."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def extract(source):
    opener = gzip.open if source.suffix == '.gz' else open
    header = end = None
    blobs = {}
    sources = set()
    with opener(source, 'rt', encoding='utf-8') as stream:
        for line in stream:
            if len(line) > 65536:
                raise ValueError('oversized JSONL record')
            row = json.loads(line)
            record = row.get('fields') or row
            probe = record.get('probe', '')
            if not probe.startswith('aptx_pcm_snapshot'):
                continue
            sources.add(row.get('source_id'))
            if probe == 'aptx_pcm_snapshot':
                if header or end:
                    raise ValueError('multiple or reordered headers')
                header = record
            elif probe == 'aptx_pcm_snapshot_end':
                if not header or end or record['sections'] != header['sections']:
                    raise ValueError('invalid end marker')
                end = record
            elif probe == 'aptx_pcm_snapshot_data':
                if not header or end:
                    raise ValueError('chunk outside snapshot')
                name, text = record['section'], record['hex']
                caps = {'context': 4608, 'encoded': 1800, 'decoded': 1024,
                        'input_ring': 1024, 'dsp_state': 1128, 'dsp_input': 1024,
                        'dsp_output': 1024, 'final_output': 1024}
                if name not in caps or not text or len(text) > 128 or len(text) % 2:
                    raise ValueError('invalid chunk shape')
                data = bytes.fromhex(text)
                target = blobs.setdefault(name, bytearray())
                if record['offset'] != len(target) or len(target) + len(data) > caps[name]:
                    raise ValueError('chunk overlap, gap, or size overflow')
                target.extend(data)
            else:
                raise ValueError('unknown snapshot record')
    if not header or not end or header['schema'] != 1 or header['sections'] != 255 or len(sources) != 1:
        raise ValueError('incomplete, partial, unsupported, or mixed-source snapshot')
    sizes = {'context': header['context_bytes'], 'encoded': header['input_bytes'],
             'decoded': header['pcm_bytes'], 'input_ring': header['pcm_bytes'],
             'dsp_state': header['dsp_bytes'], 'dsp_input': 1024,
             'dsp_output': 1024, 'final_output': 1024}
    if set(blobs) != set(sizes) or any(len(blobs[k]) != size for k, size in sizes.items()):
        raise ValueError('section length mismatch')
    if not 0 < header['processed_bytes'] <= header['input_bytes'] or header['pcm_bytes'] % 8:
        raise ValueError('invalid decoder lengths')
    return header, blobs, sources


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    header, blobs, sources = extract(args.source)
    args.out.mkdir(parents=True, exist_ok=False)
    hashes = {}
    for name, data in blobs.items():
        (args.out / (name + '.bin')).write_bytes(data)
        hashes[name] = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
    replay_args = [str(args.out.resolve()), str(header['context_bytes']),
                   str(header['context_schema']), str(header['dsp_schema']),
                   str(header['input_bytes']), str(header['processed_bytes']),
                   str(header['pcm_bytes']), str(int(header['mix_hook'])), str(header['dsp_bytes'])]
    result = {'header': header, 'hashes': hashes, 'source_ids': list(sources),
              'replay_arguments': replay_args, 'scope': 'one decode call and an independent DSP block; no acoustic verdict'}
    (args.out / 'snapshot.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
