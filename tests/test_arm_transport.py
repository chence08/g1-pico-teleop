from collections import deque
import io
import json
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'outputs'))
from g1_arm_transport import ClientDatagrams, ServerDatagrams
from g1_arm_datagram import encode_command
from g1_arm_link import ArmLink


class TransportTests(unittest.TestCase):
    def test_local_roundtrip_metadata_and_replayed_packets(self):
        output = io.StringIO()
        ending = threading.Event()
        incoming = deque(maxlen=1)
        received = threading.Event()
        states = []
        row = dict(type='arm_state',state_sequence=1,challenge='ab'*12,
                   q=[.123]*14,initial_q=[.2]*14,profile='responsive',
                   state_age_s=.002,fault=False)
        with patch.dict(os.environ, {'SSH_CONNECTION':'127.0.0.1 1234 127.0.0.1 22'}):
            server = ServerDatagrams(output)
        client = None
        try:
            server.send(row)
            client = ClientDatagrams(json.loads(output.getvalue()))
            def accept(state, stamp):
                states.append(state); received.set()
            threading.Thread(target=server.read,args=(incoming,ending),daemon=True).start()
            threading.Thread(target=client.read,args=(accept,ending),daemon=True).start()
            client.send({'type':'link_ping','seq':1})
            deadline = time.monotonic()+1
            while not incoming and time.monotonic()<deadline: time.sleep(.002)
            self.assertTrue(incoming)
            self.assertEqual(incoming[-1][0]['seq'],1)
            server.send(row)
            self.assertTrue(received.wait(1))
            self.assertEqual(states[-1]['initial_q'],row['initial_q'])
            self.assertLessEqual(states[-1]['transport_packet_bytes'],1200)
            # Authenticated replay and corrupt packets cannot refresh input.
            client.send({'type':'link_ping','seq':1})
            bad=bytearray(encode_command({'seq':2,'type':'link_ping'},client.key,2));bad[-1]^=1
            client.socket.send(bad)
            deadline=time.monotonic()+1
            while server.discarded<2 and time.monotonic()<deadline: time.sleep(.002)
            self.assertGreaterEqual(server.discarded,2)
            self.assertEqual(incoming[-1][0]['seq'],1)
        finally:
            ending.set()
            server.close()
            if client is not None: client.close()

    def test_delayed_feedback_does_not_reset_freshness(self):
        link=ArmLink.__new__(ArmLink)
        link.lock=threading.Lock();link.last_ack_id=None
        link.state=None;link.received=0
        for stamp,received in [(100.,200.),(101.,202.)]:
            link._accept_state(dict(server_monotonic_s=stamp,state_age_s=.003),received)
        self.assertAlmostEqual(link.received,201.)
        self.assertAlmostEqual(link.state['state_age_s'],1.003)
        self.assertAlmostEqual(link.state['robot_state_age_s'],.003)
        link._accept_state(dict(server_monotonic_s=100.5,state_age_s=.003),203.)
        self.assertAlmostEqual(link.received,201.)
        link.process=SimpleNamespace(poll=lambda:None)
        with patch('g1_arm_link.time.monotonic',return_value=202.1):
            state=link.snapshot()
        self.assertAlmostEqual(state['state_age_s'],1.103)


if __name__ == '__main__': unittest.main()
