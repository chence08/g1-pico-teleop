import collections
from pathlib import Path
import socket
import sys
import threading
import time
import types
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'outputs'))
from g1_latest_state import CaptureDelay, LowStateMonitor, PACKET, decode_packet


class LatestStateTests(unittest.TestCase):
    def test_relative_capture_delay_uses_source_clock_not_callback_freshness(self):
        clock = CaptureDelay()
        self.assertEqual(clock.update(100, 900000000000), 0)
        self.assertAlmostEqual(clock.update(101, 900200000000), .8)
        self.assertEqual(clock.update(101.01, 901010000000), 0)

    def test_clock_reset_and_forward_step_fail(self):
        for source in (899999999999, 902000000000):
            clock = CaptureDelay()
            clock.update(100, 900000000000, 1000)
            with self.assertRaises(ValueError):
                clock.update(100.01, source, 1010)

    def test_initial_backlog_clearing_is_not_a_source_clock_step(self):
        clock = CaptureDelay()
        clock.update(100, 900000000000, 1000)
        self.assertEqual(clock.update(100.01, 900210000000, 1210), 0)

    def test_tick_wrap_is_valid_but_reboot_is_not(self):
        clock = CaptureDelay()
        clock.update(100, 900000000000, 2**32 - 5)
        self.assertEqual(clock.update(100.01, 900010000000, 5), 0)
        clock = CaptureDelay()
        clock.update(100, 900000000000, 100000)
        with self.assertRaises(ValueError):
            clock.update(100.01, 900010000000, 0)

    def test_packet_preserves_acquisition_time_and_all_motor_fields(self):
        data = [float(i) for i in range(87)]
        m = decode_packet(PACKET.pack(10, .02, 12345, 99, 6, 3, *data))
        self.assertEqual(m.capture_received_s, 10)
        self.assertEqual(m.motor_state[28].q, 28)
        self.assertEqual(m.motor_state[28].dq, 57)
        self.assertEqual(m.motor_state[28].tau_est, 86)
        self.assertEqual(m.state_ipc_drops, 3)

    def test_reader_discards_superseded_packets_before_callback(self):
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        parent.setblocking(False)
        monitor = LowStateMonitor.__new__(LowStateMonitor)
        monitor.socket = parent
        monitor.ending = threading.Event()
        monitor.errors = collections.deque(maxlen=3)
        monitor.process = types.SimpleNamespace(poll=lambda: None)
        for tick in (1, 2, 3):
            child.send(PACKET.pack(10, 0, tick, tick, 6, 0, *([0.]*87)))
        rows = []
        def receive(m):
            rows.append(m)
            monitor.ending.set()
        started = time.monotonic()
        try:
            monitor._read(receive)
        finally:
            parent.close()
            child.close()
        self.assertLess(time.monotonic() - started, .05)
        self.assertEqual([m.tick for m in rows], [3])
        self.assertEqual(rows[0].capture_received_s, 10)


if __name__ == '__main__':
    unittest.main()
