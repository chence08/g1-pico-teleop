"""Arm link with compact latest-only UDP and an SSH commissioning fallback.

Only --live constructs a motor publisher, exclusively on rt/arm_sdk. Never
releases the motion service, changes FSM, or writes rt/lowcmd/legs/waist/hands.
This is commissioning software, not a collision or balance guarantee.
"""
import argparse
import gc
from collections import deque
from contextlib import ExitStack
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from g1_latest_state import LowStateMonitor
from g1_config import REMOTE_TOOLS, REMOTE_PYTHON, DDS_INTERFACE, ssh


ARM_IDS = list(range(15, 29))
# Same G1 29-DOF URDF used by the preview. Keep receiver limits independent
# of incoming targets; they are checked in addition to the initial-pose range.
ARM_LOWER = [-3.0892, -1.5882, -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558,
             -3.0892, -2.2515, -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558]
ARM_UPPER = [2.6704, 2.2515, 2.618, 2.0944, 1.972222054, 1.614429558, 1.614429558,
             2.6704, 1.5882, 2.618, 2.0944, 1.972222054, 1.614429558, 1.614429558]
REMOTE_SCRIPT = REMOTE_TOOLS + "/g1_arm_link.py"
REMOTE_GRAVITY_URDF = REMOTE_TOOLS + "/g1_body29_hand14.urdf"


def operating_joint_limits():
    """Absolute operating endpoints, including observed firmware soft stops."""
    import numpy as np
    lower, upper = np.array(ARM_LOWER)+.015, np.array(ARM_UPPER)-.015
    # Both rolls stopped at +/-2.147 during the full-range hardware test.
    upper[1], lower[8] = 2.10, -2.10
    # G1 elbow zero is a bent, forward forearm, not anatomical extension.
    # Preserve flexion through negative q; stop short of the straight-arm pole.
    lower[3], lower[10] = -1.0, -1.0
    upper[3], upper[10] = 1.50, 1.50
    return lower, upper


def bounded_joint_step(q, velocity, target, dt, lower, upper,
                       speed=.1, acceleration=.3, time_constant=.08, braking=False):
    """Fresh-target interpolation; protective bounds override acceleration."""
    import numpy as np
    dt = min(max(float(dt), 0), .05)
    target = np.clip(target, lower, upper)
    wanted = np.clip((target - q) / time_constant, -speed, speed)
    if braking:
        error = target - q
        braking_speed = np.sqrt((acceleration * dt)**2 + 2 * acceleration * np.abs(error)) - acceleration * dt
        wanted = np.sign(error) * np.minimum(np.abs(wanted), braking_speed)
    velocity = velocity + np.clip(wanted - velocity, -acceleration * dt, acceleration * dt)
    proposed = q + velocity * dt
    command = np.clip(proposed, lower, upper)
    velocity[proposed != command] = 0.
    return command, velocity


def tracking_joint_step(q, velocity, target, measured, dt, lower, upper,
                        speed=2., acceleration=10., lead=.10):
    """One trajectory generator with measured-position anti-windup.

    An obstructed joint cannot accumulate ever-larger PD position error while
    other joints continue following. Fresh feedback is required by the caller.
    """
    import numpy as np
    command, velocity = bounded_joint_step(q, velocity, target, dt, lower, upper,
                                           speed=speed, acceleration=acceleration,
                                           time_constant=.015, braking=True)
    protected = np.clip(command, np.maximum(lower, measured-lead),
                        np.minimum(upper, measured+lead))
    velocity[protected != command] = 0.
    return protected, velocity, (np.abs(protected-command) > 1e-7)


def fsm_monitor_mode(code, fsm, last_success_age):
    if last_success_age > 2:
        return "fault"
    if code == 0:
        return "valid" if fsm == 500 else "fault"
    # Installed Unitree SDK: RPC_ERR_CLIENT_API_TIMEOUT. Unknown mode is a
    # hold condition, never permission to keep following on the previous mode.
    return "wait" if code == 3104 else "fault"


def fsm_monitor_server(interface):
    """Read-only RPC worker with its own DDS participant and Python process.

    On this Orin, publishing at 250 Hz in the RPC process reproduces API
    timeouts even on an unused topic. Keep that contention out of this worker.
    """
    output = sys.stdout
    sys.stdout = sys.stderr
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    ChannelFactoryInitialize(0, interface)
    loco = LocoClient()
    loco.SetTimeout(1)
    loco.Init()
    ending = threading.Event()
    def watch_parent():
        sys.stdin.read(1)  # EOF when the arm-link parent exits or closes us.
        ending.set()
    threading.Thread(target=watch_parent, daemon=True).start()
    last_success = 0.
    start = time.monotonic()
    while not ending.is_set() and time.monotonic() - start < 930:
        code, fsm = loco.GetFsmId()
        received = time.monotonic()
        if code == 0:
            last_success = received
        output.write(json.dumps([code, fsm, received, last_success]) + "\n")
        output.flush()
        ending.wait(.05 if code == 3104 else .5)


class FsmMonitor:
    def __init__(self, interface):
        self.result = None
        self.process = subprocess.Popen(
            [sys.executable, "-u", os.path.abspath(__file__), "--fsm-monitor", "--interface", interface],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            try:
                code, fsm, received, success = json.loads(line)
                if (type(code) is int and (fsm is None or type(fsm) is int)
                        and all(type(t) in (int, float) and math.isfinite(t) for t in (received, success))):
                    self.result = ((code, fsm), received, success)
            except (ValueError, TypeError):
                pass  # SDK diagnostic prints are never a mode confirmation.

    def snapshot(self, timeout=0):
        deadline = time.monotonic() + timeout
        while self.result is None and self.process.poll() is None and time.monotonic() < deadline:
            time.sleep(.01)
        if self.result is None or self.process.poll() is not None:
            return ((-1, None), 0., 0.)  # Fail closed on worker exit or no result.
        return self.result

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.terminate()  # Read-only worker; no motor release here.
            self.process.wait(timeout=2)


def walking_client_connected():
    # Existing independent walking/gesture server; never stop or arm it here.
    try:
        for table in ("/proc/net/tcp", "/proc/net/tcp6"):
            with open(table) as file:
                for line in list(file)[1:]:
                    columns = line.split()
                    if int(columns[1].split(":")[1], 16) == 5602 and columns[3] == "01":
                        return True
    except (OSError, ValueError, IndexError):
        return True  # Fail closed if ownership cannot be checked.
    return False


def validate_target(row, initial_q, challenges, now, last_seq, joint_range=0.15):
    """Robot clock challenges bound network queue age without clock sync."""
    if not math.isfinite(joint_range) or not 0 <= joint_range <= 1.5:
        raise ValueError("joint range must be in [0, 1.5]; zero uses URDF limits")
    if not isinstance(row, dict):
        raise ValueError("target must be an object")
    seq = row.get("seq")
    if type(seq) is not int or seq <= last_seq:
        raise ValueError("repeated or invalid target sequence")
    nonce = row.get("challenge")
    challenge_ages = [now-issued for token, issued in challenges if nonce == token and 0 <= now-issued <= .2]
    if not challenge_ages:
        raise ValueError("expired challenge: queued or stale target")
    age = row.get("input_age_s")
    if not isinstance(age, (int, float)) or not math.isfinite(age) or not 0 <= age <= 0.15:
        raise ValueError("stale PICO input")
    # The challenge round trip upper-bounds command transit without clock
    # synchronization. Add it conservatively instead of pretending transit was
    # zero when the robot received the PC's input-age duration.
    if age + min(challenge_ages) > .15:
        raise ValueError('PICO input age plus challenge round trip exceeds 150 ms')
    q = row.get("q")
    if not isinstance(q, list) or len(q) != 14:
        raise ValueError("target must contain 14 joints")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in q):
        raise ValueError("nonfinite joint target")
    if joint_range and any(abs(v - initial) > joint_range for v, initial in zip(q, initial_q)):
        raise ValueError(f"target outside initial +/-{joint_range:g} rad commissioning range")
    if any(v < lo or v > hi for v, lo, hi in zip(q, ARM_LOWER, ARM_UPPER)):
        raise ValueError("target outside URDF joint limits")
    if type(row.get("follow")) is not bool:
        raise ValueError("follow must be boolean")
    for key in ("pc_sampled_s", "pc_ready_s", "pc_sent_s", "ik_ms"):
        value = row.get(key)
        if value is not None and (type(value) not in (int, float) or not math.isfinite(value)):
            raise ValueError("invalid timing metadata")
    return seq, list(q), row["follow"]


class ArmLink:
    """PC client with latest-only datagrams or the commissioning SSH fallback."""
    def __init__(self, live=False, joint_range=0.15, profile="commissioning", gravity_comp="off", velocity_reference=None, transport=None, remote_script=None):
        if not math.isfinite(joint_range) or not 0 <= joint_range <= 1.5:
            raise ValueError("joint range must be in [0, 1.5]; zero uses URDF limits")
        if profile not in ("commissioning", "responsive"):
            raise ValueError("unknown control profile")
        if gravity_comp not in ("off", "calibrated"):
            raise ValueError("unknown gravity compensation mode")
        self.transport = transport or ('udp' if profile == 'responsive' else 'ssh')
        if self.transport not in ('udp', 'ssh'):
            raise ValueError('unknown arm transport')
        self.profile = profile
        velocity_reference = profile == 'responsive' if velocity_reference is None else bool(velocity_reference)
        script=REMOTE_SCRIPT if remote_script is None else remote_script
        if script not in (REMOTE_SCRIPT,REMOTE_TOOLS+'/g1_suspended_body.py'):
            raise ValueError('Unknown robot controller script')
        remote = [REMOTE_PYTHON, '-u', script, '--seconds', '900', '--joint-range', str(float(joint_range)),
                  '--profile', profile, '--gravity-comp', gravity_comp, '--transport', self.transport,
                  '--interface', DDS_INTERFACE, '--velocity-reference', 'on' if velocity_reference else 'off']
        if live: remote.append('--live')
        self.process = subprocess.Popen(ssh(remote),
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.lock = threading.Lock()
        self.state, self.received, self.target = None, 0., None
        self.stop = threading.Event()
        self.errors = deque(maxlen=5)
        self.last_ack_id = None
        self.datagrams = None
        self.leg_target = None
        self.minimum_feedback_offset = None
        self.last_server_stamp = None
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.writer = threading.Thread(target=self._write, daemon=True)
        self.reader.start()
        self.writer.start()

    def _read(self):
        for line in self.process.stdout:
            try:
                row = json.loads(line)
                if row.get('type') == 'arm_udp_bootstrap':
                    if self.transport != 'udp' or self.datagrams is not None:
                        raise ValueError('unexpected UDP bootstrap')
                    from g1_arm_transport import ClientDatagrams
                    datagrams = ClientDatagrams(row)
                    self.datagrams = datagrams
                    threading.Thread(target=datagrams.read,
                        args=(self._accept_state, self.stop), daemon=True).start()
                elif row.get("type") == "arm_state" and getattr(self, 'transport', 'ssh') == 'ssh':
                    self._accept_state(row, time.monotonic())
                elif row.get("type") == "arm_error":
                    self.errors.append(line.strip())
                    with self.lock:
                        if self.state is not None:
                            self.state = dict(self.state, fault=True,
                                              fault_reasons=row.get("fault_reasons", []),
                                              status="Orin exited with a latched fault")
                            self.received = time.monotonic()
                else:
                    self.errors.append(line.strip())
            except (ValueError, AttributeError, TypeError, KeyError):
                # Never copy a failed private bootstrap (contains session key)
                # into diagnostics. Normal SDK messages retain their context.
                self.errors.append('Invalid arm bootstrap/state' if 'arm_udp_bootstrap' in line else line.strip())

    def _accept_state(self, row, received):
        with self.lock:
            extra_delay = 0.
            stamp = row.get('server_monotonic_s')
            if stamp is not None:
                previous = getattr(self, 'last_server_stamp', None)
                if previous is not None and stamp <= previous:
                    return
                self.last_server_stamp = stamp
                offset = received - stamp
                minimum = getattr(self, 'minimum_feedback_offset', None)
                self.minimum_feedback_offset = offset if minimum is None else min(minimum, offset)
                extra_delay = max(0., offset - self.minimum_feedback_offset)
                row['feedback_network_delay_s'] = extra_delay
                row['robot_state_age_s'] = row['state_age_s']
                row['state_age_s'] += extra_delay
            # Delayed delivery cannot reset freshness to zero. Fixed path delay
            # remains unobservable here; the robot challenge independently
            # rejects commands whose full feedback/command round trip is stale.
            self.state, self.received = row, received - extra_delay
            if row.get("pc_sent_s") is not None and row.get("command_id") != self.last_ack_id:
                self.state["pc_send_to_ack_ms"] = 1000 * (received - row["pc_sent_s"])
                self.last_ack_id = row.get("command_id")

    def snapshot(self, timeout=0):
        deadline = time.monotonic() + timeout
        while True:
            with self.lock:
                row = None if self.state is None else dict(self.state)
                age = time.monotonic() - self.received
            if row is not None:
                row["link_age_s"] = age
                if 'robot_state_age_s' in row:
                    row['state_age_s'] = row['robot_state_age_s'] + age
                return row
            if time.monotonic() >= deadline or self.process.poll() is not None:
                raise RuntimeError("No Orin state: " + " | ".join(self.errors))
            time.sleep(0.02)

    def submit(self, q, sample_time, follow, source_sequence=None, ik_ms=None):
        with self.lock:
            self.target = (list(q), float(sample_time), bool(follow), time.monotonic(), source_sequence, ik_ms)

    def submit_legs(self,q,sampled,sequence,motion_verified):
        q=list(q)
        if len(q)!=12 or any(type(v) not in (int,float) or not math.isfinite(v) for v in q):
            raise ValueError('Expected twelve finite leg targets')
        if type(sampled) not in (int,float) or not math.isfinite(sampled) or sampled>time.monotonic():
            raise ValueError('Invalid original body observation time')
        if type(sequence) is not int or sequence<2 or type(motion_verified) is not bool:
            raise ValueError('Invalid leg source sequence/verification')
        with self.lock:
            self.leg_target=(q,float(sampled),sequence,motion_verified)

    def _write(self):
        seq = 0
        while not self.stop.wait(0.01 if self.profile == "responsive" else 0.02):
            with self.lock:
                state = self.state
                target = self.target
                received = self.received
                legs=getattr(self,'leg_target',None)
            if not state and self.datagrams is None:
                continue
            seq += 1
            row = {"type": "link_ping", "seq": seq}
            if state and target and time.monotonic() - received <= 0.15:
                q, sampled, follow, ready, source_sequence, ik_ms = target
                age = max(0, time.monotonic() - sampled)
                if age <= 0.15:
                    row = {"seq": seq, "challenge": state["challenge"], "q": q,
                           "input_age_s": age, "follow": follow,
                           "pc_sampled_s": sampled, "pc_ready_s": ready,
                           "pc_sent_s": time.monotonic(), "source_sequence": source_sequence,
                           "ik_ms": ik_ms}
                    if legs is not None:
                        leg_q,leg_sampled,body_sequence,verified=legs
                        row.update(leg_q=leg_q,body_input_age_s=max(0.,time.monotonic()-leg_sampled),
                                   body_sequence=body_sequence,body_motion_verified=verified)
            try:
                if self.datagrams is not None:
                    self.datagrams.send(row)
                elif self.transport == 'ssh':
                    line = json.dumps(row, allow_nan=False)
                    self.process.stdin.write(line + "\n")
                    self.process.stdin.flush()
            except (OSError, ValueError):
                return

    def close(self):
        self.stop.set()
        # Closing input invokes the Orin's bounded release. Never kill its
        # publisher mid-release; watchdog also handles blocked/lost SSH.
        self.writer.join(timeout=0.1)
        if not self.writer.is_alive():
            try:
                self.process.stdin.close()
            except OSError:
                pass
        # If a pipe writer is stuck, stop refreshing its target. The robot's
        # watchdog ends the session independently; do not block on its I/O lock.
        try:
            self.process.wait(timeout=6)
        except subprocess.TimeoutExpired:
            self.process.terminate()
        if self.datagrams is not None:
            self.datagrams.close()


def server(args):
    # Include startup failures in cleanup: both workers are read-only but must
    # never survive a rejected session or failed calibration.
    with ExitStack() as resources:
        return _server(args, resources)


def _server(args, resources):
    joint_range = getattr(args, "joint_range", 0.15)
    profile = getattr(args, "profile", "commissioning")
    responsive = profile == "responsive"
    gravity_mode = getattr(args, "gravity_comp", "off")
    velocity_reference = getattr(args, 'velocity_reference', 'on' if responsive else 'off') == 'on'
    if not math.isfinite(joint_range) or not 0 <= joint_range <= 1.5:
        raise ValueError("joint range must be in [0, 1.5]; zero uses URDF limits")
    import numpy as np
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC

    # SDK prints are stderr; stdout is exclusively the SSH JSON protocol.
    output = sys.stdout
    sys.stdout = sys.stderr
    datagrams = None
    if getattr(args, 'transport', 'ssh') == 'udp':
        from g1_arm_transport import ServerDatagrams
        datagrams = ServerDatagrams(output)
        resources.callback(datagrams.close)
    ChannelFactoryInitialize(0, args.interface)
    lock = threading.Lock()
    latest = [None, 0.]
    state_history = deque(maxlen=1200)
    seen_arm = [0., False]
    our_crcs = deque(maxlen=1000)

    def on_state(msg):
        with lock:
            latest[:] = [msg, msg.capture_received_s]
            state_history.append(msg)

    def on_arm(msg):
        if msg.crc not in our_crcs:
            seen_arm[:] = [time.monotonic(), True]

    subscriber = LowStateMonitor(args.interface, on_state)
    resources.callback(subscriber.Close)
    monitor = ChannelSubscriber("rt/arm_sdk", LowCmd_)
    monitor.Init(on_arm, 1)
    resources.callback(monitor.Close)
    start = time.monotonic()
    while time.monotonic() - start < 3 and latest[0] is None:
        time.sleep(0.02)
    with lock:
        msg, received = latest
    if msg is None or time.monotonic() - received > 0.1:
        raise RuntimeError("No fresh robot state")
    time.sleep(0.5)
    fsm_monitor = FsmMonitor(args.interface)
    resources.callback(fsm_monitor.close)
    (code, fsm), _, _ = fsm_monitor.snapshot(timeout=4)
    if code != 0 or fsm != 500:
        raise RuntimeError(f"First-test mode must remain FSM 500; observed {code}, {fsm}. No mode change performed.")
    if args.live and (walking_client_connected() or seen_arm[1]):
        raise RuntimeError("Competing walking/arm control detected; refusing to create motor publisher")
    with lock:
        msg, received = latest
    initial = [float(msg.motor_state[i].q) for i in ARM_IDS]
    if not np.isfinite(initial).all():
        raise RuntimeError("Invalid measured arm pose")
    if any(v < lo or v > hi for v, lo, hi in zip(initial, ARM_LOWER, ARM_UPPER)):
        raise RuntimeError("Measured arm positions outside commissioning URDF limits")
    command = np.array(initial)
    command_velocity = np.zeros(14)
    command_lower = np.maximum(ARM_LOWER, np.array(initial) - joint_range) if joint_range else np.array(ARM_LOWER)
    command_upper = np.minimum(ARM_UPPER, np.array(initial) + joint_range) if joint_range else np.array(ARM_UPPER)
    if responsive:
        operating_lower, operating_upper = operating_joint_limits()
        command_lower = np.maximum(command_lower, operating_lower)
        command_upper = np.minimum(command_upper, operating_upper)
        if np.any(command < command_lower) or np.any(command > command_upper):
            raise RuntimeError('Starting pose outside responsive operating endpoints')
    desired = command.copy()
    gravity = None
    workspace = None
    if responsive:
        from g1_arm_workspace import ArmWorkspace
        workspace = ArmWorkspace(REMOTE_GRAVITY_URDF)
        startup_clearance = workspace.clearance([msg.motor_state[i].q for i in range(29)])
        if np.any(startup_clearance < 0):
            raise RuntimeError(f'Starting arms intersect the torso approximation: {startup_clearance.tolist()}')
    gravity_calibration = None
    if gravity_mode == "calibrated":
        from g1_gravity_comp import CalibratedGravity
        with lock:
            history = list(state_history)[-500:]
        if len(history) < 50:
            raise RuntimeError('Not enough passive state for holding-torque calibration')
        motor_q = np.array([[m.motor_state[i].q for i in range(29)] for m in history])
        arm_dq = np.array([[m.motor_state[i].dq for i in ARM_IDS] for m in history])
        arm_tau = np.array([[m.motor_state[i].tau_est for i in ARM_IDS] for m in history])
        if (not np.isfinite(motor_q).all() or not np.isfinite(arm_tau).all() or not np.isfinite(arm_dq).all()
                or np.percentile(np.abs(arm_dq),95) > .1
                or np.max(np.ptp(motor_q[:,15:29],axis=0)) > .02):
            raise RuntimeError('Hold arms still for passive gravity calibration, then restart')
        gravity = CalibratedGravity(REMOTE_GRAVITY_URDF, np.median(motor_q,axis=0), np.median(arm_tau,axis=0))
        if np.any(gravity.calibration_clipped):
            raise RuntimeError('Measured holding torque exceeds configured feedforward limits')
        gravity_calibration = dict(reference_q=gravity.reference_motor_q29.tolist(),
                                   baseline_tau=gravity.baseline.tolist(), limits=gravity.limits.tolist())
    torque_command = np.zeros(14)
    gravity_saturated = [False] * 14
    tracking_limited = [False] * 14
    workspace_blocked = [False, False]
    workspace_clearance = None
    weight = 0.
    ever_followed = False
    released = False
    # No motor publisher is created in shadow mode.
    publisher = None
    crc = CRC()
    if args.live:
        publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        publisher.Init()
        resources.callback(publisher.Close)
    challenges = deque(maxlen=30)
    last_seq, accepted, rejected = 0, 0, 0
    link_pings = 0
    command_meta = {}
    last_apply_time = None
    last_target, last_follow = 0., False
    reason = "release both grips before following"
    ending = threading.Event()
    incoming = deque(maxlen=1)
    protocol_error = threading.Event()
    outgoing = deque(maxlen=1)
    ownership_result = [walking_client_connected(), time.monotonic()]

    def poll_ownership():
        while not ending.wait(0.1):
            ownership_result[:] = [walking_client_connected(), time.monotonic()]

    def write_output():
        while not ending.wait(0.01):
            if outgoing:
                try:
                    row = outgoing.pop()
                    if datagrams is not None:
                        datagrams.send(row)
                    else:
                        output.write(json.dumps(row, allow_nan=False) + '\n')
                        output.flush()
                except (OSError, ValueError, TypeError):
                    ending.set()
                    return

    def read_input():
        if datagrams is not None:
            # SSH is the private bootstrap/lifetime channel, never a second
            # command source. EOF triggers the same bounded robot release.
            if sys.stdin.read(1):
                protocol_error.set()
            ending.set()
            return
        while not ending.is_set():
            line = sys.stdin.readline(4097)
            if not line:
                ending.set()
                return
            if len(line) > 4096 or not line.endswith("\n"):
                protocol_error.set()
                ending.set()
                return
            try:
                incoming.append((json.loads(line), time.monotonic()))
            except ValueError:
                protocol_error.set()
                ending.set()
                return

    threading.Thread(target=read_input, daemon=True).start()
    if datagrams is not None:
        threading.Thread(target=datagrams.read, args=(incoming, ending), daemon=True).start()
    threading.Thread(target=poll_ownership, daemon=True).start()
    threading.Thread(target=write_output, daemon=True).start()
    signal.signal(signal.SIGINT, lambda *_: ending.set())
    signal.signal(signal.SIGTERM, lambda *_: ending.set())
    start = last_tick = time.monotonic()
    next_state = start
    shutting_down = None
    fault = False
    fault_reasons = []
    loop_max_gap = 0.
    state_sequence = 0
    mode_machine = int(msg.mode_machine)
    gc_was_enabled = gc.isenabled()
    if responsive:
        gc.disable()  # Reference counting remains; avoid cyclic scans in servo.
    try:
        while True:
            now = time.monotonic()
            loop_max_gap = max(loop_max_gap, now - last_tick)
            dt = min(max(now - last_tick, 0), 0.02)
            last_tick = now
            with lock:
                msg, received = latest
            measured = np.array([msg.motor_state[i].q for i in ARM_IDS])
            state_age = max(0., now - received) + msg.capture_delay_s
            (code, fsm), mode_received, mode_success_received = fsm_monitor.snapshot()
            mode_query = fsm_monitor_mode(code, fsm, now - mode_success_received)
            owner_connected, owner_received = ownership_result
            competing = owner_connected or now - owner_received > 0.5
            conditions = [(state_age > 0.5, "joint feedback older than 500 ms"),
                          (not np.isfinite(measured).all(), "nonfinite measured joints"),
                          (mode_query == "fault", f"FSM query failed or stale: code={code} fsm={fsm}"),
                          (msg.mode_machine != mode_machine, "mode_machine changed"),
                          (seen_arm[1], "unrecognized arm_sdk publisher traffic"),
                          (competing, "external walking control connection"),
                          (protocol_error.is_set(), "malformed input")]
            if any(bad for bad, _ in conditions):
                fault = True
                for bad, detail in conditions:
                    if bad and detail not in fault_reasons:
                        fault_reasons.append(detail)
                reason = "fault: stale state, changed mode, competing control, or malformed input"
            if incoming:
                row, input_received_s = incoming.pop()
                try:
                    if isinstance(row, dict) and row.get("type") == "link_ping":
                        # Transport health only. Never refresh motor-command age,
                        # grip release, target sequence, desired pose, or weight.
                        link_pings += 1
                    else:
                        accepted_time = time.monotonic()
                        seq, target, follow = validate_target(row, initial, challenges, accepted_time, last_seq, joint_range)
                        if responsive and (np.any(np.array(target) < command_lower) or np.any(np.array(target) > command_upper)):
                            raise ValueError('Target outside responsive operating endpoints')
                        last_seq, accepted = seq, accepted + 1
                        command_meta = {key: row[key] for key in
                                        ("pc_sampled_s", "pc_ready_s", "pc_sent_s", "source_sequence", "ik_ms")
                                        if key in row}
                        command_meta.update(command_id=seq, robot_input_received_s=input_received_s,
                                            robot_accepted_s=accepted_time)
                        challenge_age = min(accepted_time-issued for token, issued in challenges
                                            if token == row['challenge'])
                        input_age_upper = row['input_age_s'] + challenge_age
                        command_meta.update(robot_challenge_age_s=challenge_age,
                                            robot_input_age_upper_s=input_age_upper)
                        if all(row.get(key) is not None for key in ("pc_sampled_s", "pc_ready_s", "pc_sent_s")):
                            command_meta["pc_input_to_send_ms"] = 1000 * (row["pc_sent_s"] - row["pc_sampled_s"])
                            command_meta["pc_ready_to_send_ms"] = 1000 * (row["pc_sent_s"] - row["pc_ready_s"])
                        if not follow:
                            released = True
                            desired = command.copy()
                        if follow and released and not fault and mode_query == "valid":
                            if not ever_followed:
                                command = measured.copy()
                            desired = np.array(target)
                            ever_followed = True
                        last_follow = follow and released and mode_query == "valid"
                        # Repeated packets for one cached input may not renew
                        # that input's lifetime. Age is a duration, so this does
                        # not subtract timestamps from independent clocks.
                        last_target = accepted_time - input_age_upper
                        reason = "following" if last_follow else "clutch: holding"
                except (ValueError, TypeError, KeyError):
                    rejected += 1
            if now - last_target > 0.2:
                last_follow = False
                desired = command.copy()
                # A short outage freezes commands. Fresh held-grip input may
                # resume after the PC reanchors, without discarding initial
                # release authorization. >1s still ends an active session.
                reason = "input stale: hold; awaiting fresh input"
            if mode_query == "wait":
                last_follow, released = False, False
                desired = command.copy()
                reason = "FSM query timeout: holding; waiting for confirmed mode, then release BOTH grips"
            if state_age > .1:
                last_follow = False
                desired = command.copy()
                reason = 'joint feedback pause: holding; awaiting fresh state'
            if gravity is not None:
                try:
                    torque_command, gravity_info = gravity.evaluate([msg.motor_state[i].q for i in range(29)], return_info=True)
                    gravity_saturated = gravity_info['saturated'].tolist()
                except Exception:
                    fault = True
                    if "gravity evaluation failed" not in fault_reasons:
                        fault_reasons.append("gravity evaluation failed")
                    gravity = None  # Release with the last finite bounded torque.
            release_due = (shutting_down is not None or ending.is_set() or now-start >= args.seconds
                           or (ever_followed and now-last_target > 1))
            if not release_due and weight > 0.8 and np.max(np.abs(command - measured)) > 0.2:
                fault = True
                if "joint tracking error over 0.2 rad" not in fault_reasons:
                    fault_reasons.append("joint tracking error over 0.2 rad")
                reason = "fault: arm tracking error exceeds 0.2 rad"
            if fault:
                reason = "FAULT LATCHED: state/mode/ownership/target tracking invalid; restart after resolving"
            if (fault and args.live) or ending.is_set() or now - start >= args.seconds or (ever_followed and now - last_target > 1):
                shutting_down = now if shutting_down is None else shutting_down
            if shutting_down is not None:
                desired = command.copy()
                command_velocity[:] = 0.
                weight = max(0., weight - dt / 2.)
            elif ever_followed and mode_query == "valid":
                weight = min(1., weight + dt / (.5 if responsive else 3.))
                if responsive and weight < 1:
                    command = measured.copy()
                    command_velocity[:] = 0.
                elif last_follow:
                    if responsive:
                        proposed, proposed_velocity, limited = tracking_joint_step(
                            command, command_velocity, desired, measured, dt, command_lower, command_upper)
                        try:
                            command, command_velocity, workspace_blocked, workspace_clearance = workspace.interpolate(
                                command, command_velocity, proposed, proposed_velocity,
                                [msg.motor_state[i].q for i in range(29)])
                        except Exception:
                            fault = True
                            command_velocity[:] = 0.
                            if 'workspace evaluation failed' not in fault_reasons:
                                fault_reasons.append('workspace evaluation failed')
                        tracking_limited = limited.tolist()
                    else:
                        command, command_velocity = bounded_joint_step(
                            command, command_velocity, desired, dt, command_lower, command_upper)
                else:
                    command_velocity[:] = 0.
            else:
                command_velocity[:] = 0.
            if publisher and (ever_followed or shutting_down is not None):
                cmd = unitree_hg_msg_dds__LowCmd_()
                cmd.mode_pr, cmd.mode_machine = 0, mode_machine
                for idx, motor_id in enumerate(ARM_IDS):
                    motor = cmd.motor_cmd[motor_id]
                    motor.mode, motor.q = 1, float(command[idx])
                    motor.dq = float(command_velocity[idx]) if velocity_reference and last_follow else 0.
                    motor.tau = float(torque_command[idx])
                    motor.kp = 40. if idx % 7 >= 4 else 60.
                    motor.kd = 1.5
                cmd.motor_cmd[29].q = weight
                cmd.crc = crc.Crc(cmd)
                our_crcs.append(cmd.crc)
                publisher.Write(cmd)
                last_apply_time = time.monotonic()
                if command_meta and "robot_first_published_s" not in command_meta:
                    command_meta["robot_first_published_s"] = last_apply_time
            if now >= next_state:
                state_sequence += 1
                token = os.urandom(12).hex()
                challenges.append((token, now))
                row = {"type": "arm_state", "challenge": token,
                       "q": [float(v) if np.isfinite(v) else None for v in measured],
                       "motor_q29": [float(msg.motor_state[i].q) for i in range(29)],
                       "q_command": command.tolist(),
                       "q_target": desired.tolist(),
                       "dq_command": command_velocity.tolist(),
                       "initial_q": initial, "dq": [float(msg.motor_state[i].dq) if np.isfinite(msg.motor_state[i].dq) else None for i in ARM_IDS],
                       "state_age_s": state_age, "fsm": fsm, "mode_machine": mode_machine,
                       "mode_query_ok": mode_query == "valid", "mode_query_code": code,
                       "mode_last_success_age_s": now - mode_success_received,
                       "joint_range_rad": joint_range,
                       "operating_lower_rad": command_lower.tolist(),
                       "operating_upper_rad": command_upper.tolist(),
                       "profile": profile,
                       "speed_limit_rad_s": 2. if responsive else .1,
                       "acceleration_limit_rad_s2": 10. if responsive else .3,
                       "tracking_lead_limit_rad": .10 if responsive else None,
                       "tracking_limited": tracking_limited,
                       "workspace_blocked": workspace_blocked,
                       "workspace_proposed_clearance_m": workspace_clearance,
                       "velocity_reference": velocity_reference,
                       "takeover_active": responsive and ever_followed and weight < 1,
                       "tau_measured": [float(msg.motor_state[i].tau_est) for i in ARM_IDS] if hasattr(msg.motor_state[15], 'tau_est') else None,
                       "gravity_comp": gravity_mode, "tau_command": torque_command.tolist(),
                       "gravity_calibration": gravity_calibration, "gravity_saturated": gravity_saturated,
                       "live": args.live, "weight": weight, "status": reason,
                       "accepted": accepted, "rejected": rejected, "fault": fault,
                       "link_pings": link_pings,
                       "fault_reasons": fault_reasons.copy(), "server_monotonic_s": now,
                       "state_sequence": state_sequence, "loop_max_gap_s": loop_max_gap,
                       "robot_tick": int(getattr(msg, "tick", 0)),
                       "state_capture_received_s": received,
                       "state_delivery_delay_s": msg.capture_delay_s,
                       "state_source_timestamp_ns": msg.source_timestamp_ns,
                       "state_ipc_drops": msg.state_ipc_drops,
                       "walking_client_connected": competing}
                row.update(command_meta)
                if "robot_first_published_s" in command_meta:
                    row["robot_receive_to_publish_ms"] = 1000 * (command_meta["robot_first_published_s"] - command_meta["robot_input_received_s"])
                row['arm_transport'] = 'udp' if datagrams is not None else 'ssh'
                outgoing.append(row)
                next_state = now + (1 / 60 if responsive else 1 / 30)
            if shutting_down is not None and weight == 0:
                break
            time.sleep(max(0., 0.004 - (time.monotonic() - now)))
    finally:
        ending.set()
        if responsive and gc_was_enabled:
            gc.enable()
        if fault:
            # Deliver the final reason after motor output has stopped. Otherwise
            # a zero-weight startup fault can exit before the output thread runs.
            try:
                output.write(json.dumps({"type": "arm_error", "fault_reasons": fault_reasons}) + "\n")
                output.flush()
            except OSError:
                pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--fsm-monitor", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--interface", default=DDS_INTERFACE)
    parser.add_argument("--seconds", type=float, default=900)
    parser.add_argument("--joint-range", type=float, default=0.15)
    parser.add_argument("--profile", choices=("commissioning", "responsive"), default="commissioning")
    parser.add_argument('--transport', choices=('ssh', 'udp'), default='ssh')
    parser.add_argument("--gravity-comp", choices=("off", "calibrated"), default="off")
    parser.add_argument("--velocity-reference", choices=("on", "off"))
    args = parser.parse_args()
    if args.velocity_reference is None:
        args.velocity_reference = 'on' if args.profile == 'responsive' else 'off'
    if not 1 <= args.seconds <= 900:
        parser.error("Use 1 <= seconds <= 900")
    if args.fsm_monitor:
        fsm_monitor_server(args.interface)
    else:
        server(args)
