"""Exercise the live loop with a completely fake SDK; never talks to a robot."""
import argparse
import copy
import io
import json
from pathlib import Path
import queue
import signal
import sys
import threading
import time
import types
import unittest
import numpy as np
from unittest.mock import patch
from _support import URDF

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "outputs"))
import g1_arm_link as link


class RuntimeTest(unittest.TestCase):
    def test_input_loss_releases_and_only_arm_targets_are_written(self):
        self.run_scenario()

    def test_responsive_runtime_velocity_reference_and_loss_release(self):
        self.run_scenario(responsive=True)

    def run_scenario(self, responsive=False):
        writes = []
        stopped = threading.Event()
        incoming = queue.Queue()
        first_state = [None]

        def motor():
            return types.SimpleNamespace(q=.1, dq=0., mode=0, tau=0., kp=0., kd=0.)

        state = types.SimpleNamespace(motor_state=[motor() for _ in range(35)], mode_machine=6)

        class FakeStateMonitor:
            def __init__(self, _interface, callback):
                def produce():
                    while not stopped.wait(.002):
                        state.capture_received_s = time.monotonic()
                        state.capture_delay_s = 0.
                        state.source_timestamp_ns = time.time_ns()
                        state.state_ipc_drops = 0
                        callback(state)
                threading.Thread(target=produce, daemon=True).start()

            def Close(self):
                stopped.set()

        class Subscriber:
            def __init__(self, topic, _type):
                self.topic = topic

            def Init(self, callback, _queue):
                if self.topic == "rt/lowstate":
                    def produce():
                        while not stopped.wait(.002):
                            callback(state)
                    threading.Thread(target=produce, daemon=True).start()

            def Close(self):
                stopped.set()

        class Publisher:
            def __init__(self, topic, _type):
                assert topic == "rt/arm_sdk"

            def Init(self):
                pass

            def Write(self, cmd):
                writes.append((time.monotonic(), copy.deepcopy(cmd)))

            def Close(self):
                pass

        class Loco:
            calls = 0
            def Init(self):
                pass

            def SetTimeout(self, _timeout):
                pass

            def GetFsmId(self):
                # Simulate a slow RPC; it must not block the publishing loop.
                time.sleep(.03)
                self.calls += 1
                if self.calls == 2:
                    return 3104, None
                return 0, 500

        class CRC:
            def Crc(self, _cmd):
                return len(writes) + 1

        class FakeMonitor:
            def __init__(self, _interface):
                self.loco = Loco()
                now = time.monotonic()
                self.result = ((0, 500), now, now)
                self.ending = threading.Event()
                def poll():
                    while not self.ending.wait(.15):
                        result = self.loco.GetFsmId()
                        now = time.monotonic()
                        self.result = (result, now, now if result[0] == 0 else self.result[2])
                threading.Thread(target=poll, daemon=True).start()

            def snapshot(self, timeout=0):
                return self.result

            def close(self):
                self.ending.set()

        def command():
            cmd = types.SimpleNamespace(motor_cmd=[motor() for _ in range(35)], crc=0)
            for m in cmd.motor_cmd:
                m.q = 0.
            return cmd

        class Input:
            def readline(self, _limit):
                try:
                    return incoming.get(timeout=8)
                except queue.Empty:
                    return ''

        class FakeWorkspace:
            def __init__(self, _path): pass
            def clearance(self, _q29): return np.ones(2)
            def interpolate(self, _q, _v, proposed, velocity, _q29):
                return proposed, velocity, [False,False], [1.,1.]

        class Output(io.StringIO):
            def write(self, line):
                row = json.loads(line)
                now = time.monotonic()
                if first_state[0] is None:
                    first_state[0] = now
                elapsed = now - first_state[0]
                if elapsed < 1.0 or 1.4 <= elapsed < 1.9:
                    incoming.put(json.dumps({"seq": int(elapsed * 1e9) + 1,
                        "challenge": row["challenge"], "input_age_s": .01,
                        "q": [v + .1 for v in row["initial_q"]],
                        "follow": elapsed > .1 and not .7 < elapsed < .9}) + "\n")
                else:
                    # A healthy network without fresh tracking must not keep
                    # motors under user control or defeat the input watchdog.
                    incoming.put(json.dumps({"type": "link_ping", "seq": int(elapsed * 1e9) + 1}) + "\n")
                return super().write(line)

        leaves = {
            "unitree_sdk2py.core.channel": dict(ChannelFactoryInitialize=lambda *_: None,
                ChannelSubscriber=Subscriber, ChannelPublisher=Publisher),
            "unitree_sdk2py.idl.unitree_hg.msg.dds_": dict(LowState_=object, LowCmd_=object),
            "unitree_sdk2py.idl.default": dict(unitree_hg_msg_dds__LowCmd_=command),
            "unitree_sdk2py.g1.loco.g1_loco_client": dict(LocoClient=Loco),
            "unitree_sdk2py.utils.crc": dict(CRC=CRC)}
        modules = {}
        for name, values in leaves.items():
            parts = name.split('.')
            for idx in range(1, len(parts) + 1):
                prefix = '.'.join(parts[:idx])
                if prefix not in modules:
                    modules[prefix] = types.ModuleType(prefix)
                    modules[prefix].__path__ = []
            vars(modules[name]).update(values)
        handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
        output = Output()
        try:
            with patch.dict(sys.modules, modules), patch.object(sys, 'stdin', Input()), \
                    patch.object(sys, 'stdout', output), patch.object(link, 'walking_client_connected', return_value=False), \
                    patch.object(link, 'FsmMonitor', FakeMonitor), \
                    patch.object(link, 'LowStateMonitor', FakeStateMonitor), \
                    patch('g1_arm_workspace.ArmWorkspace', FakeWorkspace), \
                    patch.object(link, 'REMOTE_GRAVITY_URDF', URDF):
                link.server(argparse.Namespace(interface='fake', live=True, seconds=6,
                                                profile='responsive' if responsive else 'commissioning'))
        finally:
            stopped.set()
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
        self.assertGreater(len(writes), 50)
        self.assertGreater(max(cmd.motor_cmd[29].q for _, cmd in writes), .1)
        self.assertEqual(writes[-1][1].motor_cmd[29].q, 0.)
        for _, cmd in writes:
            self.assertTrue(all(m.mode == 0 and m.q == 0 for idx, m in enumerate(cmd.motor_cmd)
                                if idx not in link.ARM_IDS and idx != 29))
            self.assertTrue(all(abs(cmd.motor_cmd[i].q - .1) <= .15 for i in link.ARM_IDS))
        for (_, before), (_, after) in zip(writes, writes[1:]):
            self.assertLessEqual(abs(after.motor_cmd[15].q - before.motor_cmd[15].q),
                                 .040001 if responsive else .002001)
        if responsive:
            self.assertTrue(any(abs(cmd.motor_cmd[15].dq) > .01 for _,cmd in writes))
            self.assertTrue(all(abs(cmd.motor_cmd[15].dq) <= 2.00001 for _,cmd in writes))
        self.assertLess(writes[-1][0] - first_state[0], 5.5)
        states = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertTrue(any(row['status'] == 'following' and
                            row['server_monotonic_s'] > first_state[0] + 1.5 for row in states),
                        'held-grip fresh input must recover after the short outage')
        self.assertTrue(any(not row['mode_query_ok'] for row in states))
        self.assertTrue(all(not row['fault'] for row in states))


if __name__ == '__main__':
    unittest.main()
