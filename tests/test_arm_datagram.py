import copy
import json
from pathlib import Path
import sys
import unittest
import zlib
from _support import FIXTURE_DIR, fixture

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'outputs'))
import g1_arm_datagram as codec


class DatagramTests(unittest.TestCase):
    def setUp(self):
        self.key = bytes(range(32))
        self.row = {
            'type': 'arm_state', 'challenge': '12' * 12, 'state_sequence': 1,
            'q': [.1] * 14, 'motor_q29': [.2] * 29,
            'q_command': [.3] * 14, 'q_target': [.4] * 14,
            'dq_command': [.5] * 14, 'dq': [.6] * 14,
            'tau_measured': [.7] * 14, 'tau_command': [.8] * 14,
            'workspace_proposed_clearance_m': None,
            'state_age_s': .003, 'fsm': 500, 'mode_machine': 6,
            'mode_query_code': -1, 'mode_query_ok': False, 'live': False,
            'fault': False, 'fault_reasons': [], 'status': 'clutch: holding',
            'tracking_limited': [False] * 14,
            'workspace_blocked': [False, True], 'gravity_saturated': [False] * 14,
            'state_source_timestamp_ns': 1792741290538070095,
            'server_monotonic_s': 123456.123456789,
            'initial_q': [.9] * 14, 'profile': 'responsive',
        }

    def encoded(self, seq=1):
        return codec.encode_state(dict(self.row, state_sequence=seq), self.key, seq)

    def test_state_round_trip_and_precision(self):
        metadata, dynamic = codec.split_state(self.row)
        packet = codec.encode_state(dynamic, self.key, 1)
        seq, out = codec.decode_state(packet, self.key, metadata, last_seq=0)
        self.assertEqual(seq, 1)
        self.assertLessEqual(len(packet), 1200)
        for key in ('state_source_timestamp_ns','server_monotonic_s','initial_q'):
            self.assertEqual(out[key], self.row[key])
        self.assertIsNone(out['workspace_proposed_clearance_m'])
        self.assertNotIn('pc_sent_s', out)
        self.assertEqual(out['workspace_blocked'], [False, True])
        for a, b in zip(out['q'], self.row['q']):
            self.assertAlmostEqual(a, b, places=7)

    def test_command_ping_and_target(self):
        for row in ({'type': 'link_ping', 'seq': 3},
                    {'seq': 3, 'follow': False, 'q': [.1] * 14,
                     'challenge': '12' * 12, 'input_age_s': .01}):
            p = codec.encode_command(row, self.key, 3)
            self.assertEqual(codec.decode_command(p, self.key, last_seq=2), (3, row))

    def test_replay_reorder_and_session(self):
        for seq in (8, 9):
            with self.assertRaises(ValueError):
                codec.decode_state(self.encoded(seq), self.key, last_seq=9)
        p = self.encoded(10)
        with self.assertRaises(ValueError):
            codec.decode_state(p, bytes(reversed(self.key)))
        with self.assertRaises(ValueError):
            codec.decode_command(p, self.key)
        with self.assertRaises(ValueError):
            codec.encode_state(self.row, self.key, 2)

    def test_bad_mac_lengths_and_version(self):
        p = self.encoded()
        bad = bytearray(p); bad[-1] ^= 1
        for data in (bytes(bad), p[:-1], b'', b'x' * 1201):
            with self.assertRaises(ValueError):
                codec.decode_state(data, self.key)
        bad = bytearray(p); bad[4] = 99
        with self.assertRaises(ValueError):
            codec.decode_state(bytes(bad), self.key)

    def test_invalid_values_and_payload(self):
        for value in (float('nan'), float('inf'), 1e100):
            row = copy.deepcopy(self.row); row['q'][0] = value
            with self.assertRaises(ValueError):
                codec.encode_state(row, self.key, 1)
        with self.assertRaises(ValueError):
            codec.encode_command({'seq': 1, 'q': [float('nan')]}, self.key, 1)
        with self.assertRaises(ValueError):
            codec.encode_command({'seq': 2}, self.key, 1)
        p = codec._seal(codec._COMMAND, b'{"seq":1,"bad":NaN}', self.key, 1)
        with self.assertRaises(ValueError):
            codec.decode_command(p, self.key)

    def test_bounded_text_decompression(self):
        _, payload = codec._open(self.encoded(), self.key, codec._STATE, None)
        bomb = payload[:codec._FIXED.size] + zlib.compress(b'x' * 100000)
        p = codec._seal(codec._STATE, bomb, self.key, 2)
        with self.assertRaises(ValueError):
            codec.decode_state(p, self.key)
        with self.assertRaises(ValueError):
            codec.encode_state(dict(self.row, status='x' * 5000), self.key, 1)

    @unittest.skipUnless(FIXTURE_DIR is not None, 'Optional recorded fixture: set G1_TEST_FIXTURES')
    def test_recorded_state_size_and_fields(self):
        path = fixture('arm_telemetry_1792741290538070095.jsonl')
        rows = [json.loads(s) for s in path.read_text().splitlines()]
        metadata, _ = codec.split_state(rows[0])
        for row in rows:
            seq = row['state_sequence']
            packet = codec.encode_state(row, self.key, seq)
            self.assertLessEqual(len(packet), 1200)
            _, out = codec.decode_state(packet, self.key, metadata, last_seq=seq-1)
            self.assertNotIn('pc_command', out)
            for name, kind, count, fmt in codec._SPECS:
                if name not in row:
                    self.assertNotIn(name, out)
                elif row[name] is None:
                    self.assertIsNone(out[name])
                elif kind == 'array':
                    for a, b in zip(row[name], out[name]):
                        self.assertAlmostEqual(a, b, delta=2e-6)
                else:
                    self.assertEqual(row[name], out[name])


if __name__ == '__main__':
    unittest.main()
