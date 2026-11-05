"""Offline validation of periodic audio snapshots; never accesses a device.

CRC32 detects capture corruption, not authenticity or coherent atomic snapshots.
Legacy mode checks timestamps only and cannot establish checksum protection.
"""
import argparse
import json
from pathlib import Path
import re
import zlib

SNAPSHOTS = set('owner_wait_reasons vhci_timing media_drop_reasons worker_timing '
                'decoder_execution decoder_reset pcm_delivery codec_configuration '
                'media_input sbc_pcm native_profile i2s_allocation_failure '
                'i2s_callback_work render_work i2s_descriptor_faults native_i2s '
                'audio_clock'.split())
TIMES = ('snapshot_begin_us', 'at_us', 'snapshot_span_us')
CHECKSUM = re.compile(rb',"record_crc32":"([0-9a-f]{8})"}$')


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON member')
        result[key] = value
    return result


def audit_capture(raw, allow_legacy=False):
    verified = legacy = candidates = other = 0
    rejected = []
    for number, line in enumerate(raw.splitlines(), 1):
        if b'"probe"' not in line:
            continue
        try:
            record = json.loads(line, object_pairs_hook=unique_object)
            if not isinstance(record, dict):
                raise ValueError('probe record is not an object')
        except (ValueError, UnicodeDecodeError):
            rejected.append({'line': number, 'reason': 'malformed/interleaved JSON'})
            continue
        if not (record.get('probe') in SNAPSHOTS or 'record_crc32' in record
                or any(key in record for key in ('snapshot_begin_us', 'snapshot_span_us'))):
            other += 1
            continue
        candidates += 1
        reason = None
        if not all(type(record.get(key)) is int and 0 <= record[key] <= 0xffffffff
                   for key in TIMES):
            reason = 'missing/invalid snapshot time'
        elif record['snapshot_span_us'] != ((record['at_us'] - record['snapshot_begin_us']) & 0xffffffff):
            reason = 'inconsistent snapshot time'
        elif 'record_crc32' not in record:
            if allow_legacy:
                legacy += 1
            else:
                reason = 'missing CRC32'
        else:
            suffix = CHECKSUM.search(line)
            if not suffix or int(suffix[1], 16) != zlib.crc32(line[:suffix.start()]):
                reason = 'CRC32 mismatch/invalid suffix'
            else:
                verified += 1
        if reason:
            rejected.append({'line': number, 'probe': record.get('probe'), 'reason': reason})
    return {'scope': 'offline record integrity only; no runtime/audio qualification',
            'passed': bool(candidates) and not rejected,
            'snapshot_records': candidates, 'crc32_verified': verified,
            'legacy_timestamp_only': legacy, 'other_records_unchecked': other,
            'all_snapshot_records_checksummed': bool(candidates) and verified == candidates and not rejected,
            'rejected': rejected}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--allow-legacy', action='store_true')
    args = parser.parse_args()
    result = audit_capture(args.capture.read_bytes(), args.allow_legacy)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
