"""Independent zlib vectors and realistic damaged serial-record regressions."""
import unittest
import zlib
from audit_audio_capture import audit_capture


def checked(prefix):
    return prefix + b',"record_crc32":"%08x"}\r\n' % zlib.crc32(prefix)


class CaptureIntegrity(unittest.TestCase):
    prefix = (b'{"probe":"render_work","calls":7,"snapshot_begin_us":4294965296,'
              b'"at_us":1000,"snapshot_span_us":3000')

    def test_valid_wrapped_clock(self):
        result = audit_capture(b'I (1) boot: ready\r\n' + checked(self.prefix))
        self.assertTrue(result['passed'])
        self.assertTrue(result['all_snapshot_records_checksummed'])
        self.assertEqual(result['crc32_verified'], 1)

    def test_one_off_event_time_is_not_a_snapshot(self):
        event = b'{"probe":"media_trace","at_us":3}\n'
        result = audit_capture(event + checked(self.prefix))
        self.assertTrue(result['passed'])
        self.assertEqual(result['other_records_unchecked'], 1)

    def test_counter_corruption_preserves_valid_json(self):
        raw = checked(self.prefix).replace(b'"calls":7', b'"calls":8')
        self.assertFalse(audit_capture(raw)['passed'])
        self.assertIn('CRC32', audit_capture(raw)['rejected'][0]['reason'])

    def test_damaged_probe_key_cannot_hide_a_record(self):
        self.assertFalse(audit_capture(checked(self.prefix).replace(b'"probe"', b'"pobe"'))['passed'])

    def test_missing_character_and_truncation(self):
        for raw in (checked(self.prefix).replace(b'snapshot_span', b'sapshot_span'),
                    checked(self.prefix)[:-12], checked(self.prefix).replace(b'3000', b'300')):
            self.assertFalse(audit_capture(raw)['passed'])

    def test_legacy_is_explicit_and_unprotected(self):
        raw = self.prefix + b'}\n'
        self.assertFalse(audit_capture(raw)['passed'])
        result = audit_capture(raw, allow_legacy=True)
        self.assertTrue(result['passed'])
        self.assertFalse(result['all_snapshot_records_checksummed'])
        self.assertEqual(result['legacy_timestamp_only'], 1)

    def test_duplicate_or_extra_suffix_member(self):
        for prefix in (self.prefix + b',"calls":9', self.prefix + b',"at_us":1001'):
            self.assertFalse(audit_capture(checked(prefix))['passed'])
        extra = checked(self.prefix).replace(b'}\r\n', b',"extra":1}\r\n')
        self.assertFalse(audit_capture(extra)['passed'])

    def test_valid_checksum_does_not_excuse_inconsistent_time(self):
        self.assertFalse(audit_capture(checked(self.prefix.replace(b'3000', b'3001')))['passed'])
        self.assertFalse(audit_capture(b'I (1) boot: ready\n')['passed'])


if __name__ == '__main__':
    unittest.main()
