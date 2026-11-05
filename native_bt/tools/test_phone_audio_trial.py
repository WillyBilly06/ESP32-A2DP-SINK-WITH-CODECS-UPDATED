"""Offline safety checks; these never access ADB or an ESP32."""
import json
import unittest
import xml.etree.ElementTree as ET
import zlib

from run_phone_audio_trial import authorized_details, center, max_config, valid_records


class PhoneTrialTests(unittest.TestCase):
    def test_exact_fixture_identity(self):
        tree = ET.fromstring('<hierarchy><node text="Device\'s Bluetooth address: 6E:C8:40:42:BF:66"/></hierarchy>')
        self.assertFalse(authorized_details(tree))
        tree[0].set('text', "Device's Bluetooth address: 6C:C8:40:42:BF:66")
        self.assertTrue(authorized_details(tree))

    def test_clipped_and_disabled_nodes_rejected(self):
        node = ET.Element('node', enabled='true', bounds='[65,2671][1151,2688]')
        with self.assertRaises(ValueError):
            center(node, (1216, 2688))
        node.set('bounds', '[65,200][1151,400]')
        self.assertEqual(center(node, (1216, 2688)), (608, 300))
        node.set('enabled', 'false')
        with self.assertRaises(ValueError):
            center(node, (1216, 2688))

    def test_fresh_actual_readback_required(self):
        text = 'Current connection\nLHDC V5\n192 kHz\n24-bit\nStereo\nLHDC V5 quality: 1000 kbps target\nSource: Android query\nRead within the last 5 seconds'
        self.assertTrue(max_config(text))
        for old, new in [('192 kHz', '48 kHz'), ('1000 kbps', 'Adaptive'),
                         ('Read within the last 5 seconds', 'Stale'),
                         ('Current connection', 'Preference')]:
            self.assertFalse(max_config(text.replace(old, new)))

    def test_corrupt_fault_record_rejected(self):
        prefix = b'{"probe":"native_i2s","failed":1,"snapshot_begin_us":10,"at_us":20,"snapshot_span_us":10'
        raw = prefix + b',"record_crc32":"' + f'{zlib.crc32(prefix):08x}'.encode() + b'"}'
        self.assertEqual(list(valid_records(raw))[0]['failed'], 1)
        self.assertEqual(list(valid_records(raw.replace(b'"failed":1', b'"failed":0'))), [])
        duplicate = b'{"probe":"native_i2s","failed":0,"failed":1}'
        self.assertEqual(list(valid_records(duplicate)), [])
        self.assertEqual(list(valid_records(b'{"probe":"native_i2s","failed":1}')), [])


if __name__ == '__main__':
    unittest.main()
