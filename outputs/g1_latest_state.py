"""Independent, latest-sample G1 feedback acquisition (no motor publisher).

The installed SDK listener takes one queued DDS sample per notification. In the
servo process this delivered old samples with deceptively fresh callback times.
Use KeepLast(1), a separate Python process and bounded local datagrams instead.
"""
from collections import deque
import math
import os
import select
import socket
import struct
import subprocess
import sys
import threading
import time
from types import SimpleNamespace


# Receipt, estimated delivery delay, DDS source stamp, hardware tick, mode,
# producer drops, followed by q/dq/tau for the 29 body motors.
PACKET = struct.Struct('!ddQQQQ87d')


class CaptureDelay:
    """Relative delivery delay; source/host clocks need no shared epoch.

    This lower-envelope estimate cannot observe fixed pre-existing network
    latency. A source clock reset is an error rather than fresh feedback.
    """
    def __init__(self):
        self.minimum_offset = None
        self.last_source_ns = None
        self.last_tick = None

    def update(self, received_s, source_ns, tick=None):
        if source_ns <= 0 or (self.last_source_ns is not None and source_ns < self.last_source_ns):
            raise ValueError('invalid or regressed DDS source clock')
        if tick is not None and self.last_tick is not None:
            delta_tick = tick - self.last_tick
            if delta_tick < 0 and self.last_tick > 2**32 - 10000 and tick < 10000:
                delta_tick += 2**32
            if delta_tick < 0:
                raise ValueError('robot tick reset')
            # Verified on this G1: tick advances in milliseconds. Compare two
            # source clocks, so clearing an initial queue is not a clock reset.
            if abs((source_ns - self.last_source_ns)*1e-9 - delta_tick*.001) > .1:
                raise ValueError('DDS source clock discontinuity relative to robot tick')
        offset = received_s - source_ns * 1e-9
        if self.minimum_offset is None:
            self.minimum_offset = offset
        self.minimum_offset = min(self.minimum_offset, offset)
        self.last_source_ns = source_ns
        self.last_tick = tick
        return max(0., offset - self.minimum_offset)


def decode_packet(packet):
    values = PACKET.unpack(packet)
    received, delay, source_ns, tick, mode, drops = values[:6]
    data = values[6:]
    if not all(math.isfinite(x) for x in (received, delay, *data)) or delay < 0:
        raise ValueError('nonfinite state packet')
    motors = [SimpleNamespace(q=data[i], dq=data[29+i], tau_est=data[58+i]) for i in range(29)]
    return SimpleNamespace(motor_state=motors, mode_machine=mode, tick=tick,
                           capture_received_s=received, capture_delay_s=delay,
                           source_timestamp_ns=source_ns, state_ipc_drops=drops)


def capture_server(interface, fd):
    """Worker entry point. Reads only rt/lowstate, exits on parent stdin EOF."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from cyclonedds.sub import DataReader
    from cyclonedds.qos import Qos, Policy
    from cyclonedds.internal import InvalidSample
    ChannelFactoryInitialize(0, interface)
    participant = DomainParticipant(0)
    topic = Topic(participant, 'rt/lowstate', LowState_)
    reader = DataReader(participant, topic,
                        Qos(Policy.History.KeepLast(1), Policy.Reliability.BestEffort))
    transport = socket.socket(fileno=fd)
    transport.setblocking(False)
    ending = threading.Event()
    def watch_parent():
        sys.stdin.read(1)
        ending.set()
    threading.Thread(target=watch_parent, daemon=True).start()
    clock = CaptureDelay()
    drops = 0
    last_source = None
    try:
        while not ending.is_set():
            samples = reader.take(1)
            received = time.monotonic()
            if samples:
                msg = samples[-1]
                if isinstance(msg, InvalidSample) or not msg.sample_info.valid_data:
                    continue
                source = int(msg.sample_info.source_timestamp)
                if source != last_source:
                    delay = clock.update(received, source, int(msg.tick))
                    data = [float(getattr(msg.motor_state[i], attr))
                            for attr in ('q', 'dq', 'tau_est') for i in range(29)]
                    packet = PACKET.pack(received, delay, source, int(msg.tick),
                                         int(msg.mode_machine), drops, *data)
                    try:
                        transport.send(packet)
                    except BlockingIOError:
                        drops += 1  # Explicit; bounded IPC never blocks capture.
                    last_source = source
            ending.wait(.001)
    finally:
        transport.close()


class LowStateMonitor:
    def __init__(self, interface, callback):
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        parent.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        child.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
        parent.setblocking(False)
        self.socket = parent
        self.ending = threading.Event()
        self.errors = deque(maxlen=3)
        try:
            self.process = subprocess.Popen(
                [sys.executable, '-u', os.path.abspath(__file__), interface, str(child.fileno())],
                stdin=subprocess.PIPE, stdout=sys.stderr, pass_fds=(child.fileno(),))
        except Exception:
            parent.close()
            child.close()
            raise
        child.close()
        self.reader = threading.Thread(target=self._read, args=(callback,), daemon=True)
        self.reader.start()

    def _read(self, callback):
        while not self.ending.is_set() and self.process.poll() is None:
            try:
                if not select.select([self.socket], [], [], .1)[0]:
                    continue
                packet = self.socket.recv(PACKET.size + 1)
                # Discard superseded packets before exposing feedback. Timestamp
                # remains the acquisition-process receipt, never this delivery.
                for _ in range(64):
                    try:
                        packet = self.socket.recv(PACKET.size + 1, socket.MSG_DONTWAIT)
                    except BlockingIOError:
                        break
                callback(decode_packet(packet))
            except BlockingIOError:
                continue
            except (OSError, ValueError, struct.error) as exc:
                self.errors.append(str(exc))
                return

    def Close(self):
        self.ending.set()
        self.process.stdin.close()
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=2)
        self.reader.join(timeout=.3)
        self.socket.close()


if __name__ == '__main__':
    capture_server(sys.argv[1], int(sys.argv[2]))
