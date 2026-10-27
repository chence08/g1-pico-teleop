"""Record native PICO controller/body/object-tracker SDK snapshots over Wi-Fi.

Run with the existing xrobo Python 3.11 environment. The existing tv environment
consumes the loopback preview stream. All logs are diagnostic, not synchronized
robot demonstrations. SDK getters are cached and not an atomic capture API.
"""
import argparse
import json
from pathlib import Path
import socket
import queue
import threading
import time
import uuid

import numpy as np
from pico_native_input import BODY_NAMES


class AsyncDiagnosticLog:
    """Bounded diagnostic writer: disk delays cannot block controller delivery.

    Overflow is counted, never hidden. This remains a diagnostic log, not an
    episode recorder with a guarantee of completeness.
    """
    def __init__(self, path):
        self.file = path.open('x', encoding='utf-8')
        self.queue = queue.Queue(maxsize=128)
        self.ending = threading.Event()
        self.dropped = 0
        self.error = None
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def __enter__(self):
        return self

    def write(self, line):
        if self.error:
            raise RuntimeError('Diagnostic writer failed') from self.error
        try:
            self.queue.put_nowait(line)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        flushed = time.monotonic()
        try:
            while not self.ending.is_set() or not self.queue.empty():
                try:
                    self.file.write(self.queue.get(timeout=.1))
                except queue.Empty:
                    pass
                if time.monotonic() - flushed > 2:
                    self.file.flush()
                    flushed = time.monotonic()
        except Exception as error:
            self.error = error
        finally:
            try:
                self.file.close()
            except Exception as error:
                self.error = self.error or error

    def __exit__(self, exc_type, *_):
        self.ending.set()
        self.worker.join(timeout=3)
        print(f'Diagnostic log dropped snapshots={self.dropped}', flush=True)
        if exc_type is None and (self.worker.is_alive() or self.error):
            raise RuntimeError('Diagnostic writer did not finish cleanly') from self.error


def control_snapshot(row):
    """Only fields consumed by the receiver; retain original acquisition times."""
    keys = ('schema', 'session_id', 'host_poll_monotonic_s', 'source_timestamp_ns',
            'source_timestamp_after_ns', 'source_timestamp_unchanged_during_poll',
            'head_pose', 'left_controller_pose', 'right_controller_pose',
            'left_grip', 'right_grip', 'left_trigger', 'right_trigger',
            'bridge_poll_duration_s', 'host_loop_gap_s')
    result = {key: row[key] for key in keys if key in row}
    body = row.get('body')
    result['body'] = ({'source_timestamp_ns': body['source_timestamp_ns'], 'poses': body['poses']}
                      if body else None)
    return result


def effective_body_timestamp(sdk_timestamp, joint_timestamps):
    """v1.1.1 can omit Body.timeStampNs while supplying joint timestamps."""
    if int(sdk_timestamp) > 0:
        return int(sdk_timestamp), "sdk_body_timestamp"
    positive = [int(t) for t in joint_timestamps if int(t) > 0]
    if positive:
        return max(positive), "max_body_joint_timestamp"
    # Never substitute controller or host time for absent body acquisition time.
    return 0, "unavailable"


def read_sdk(sdk):
    before = int(sdk.get_time_stamp_ns())
    row = {"source_timestamp_ns": before,
           "head_pose": list(sdk.get_headset_pose())}
    for side in ("left", "right"):
        row[f"{side}_controller_pose"] = list(getattr(sdk, f"get_{side}_controller_pose")())
        for key in ("grip", "trigger", "axis"):
            value = getattr(sdk, f"get_{side}_{key}")()
            row[f"{side}_{key}"] = list(value) if key == "axis" else float(value)
    row["buttons"] = {name: bool(getattr(sdk, f"get_{name}_button")()) for name in ("A", "B", "X", "Y")}
    row["body"] = None
    if sdk.is_body_data_available():
        stamp = int(sdk.get_body_timestamp_ns())
        joint_timestamps = list(sdk.get_body_joints_timestamp())
        effective, timestamp_source = effective_body_timestamp(stamp, joint_timestamps)
        row["body"] = {"kind": "estimated_body_joints", "source_timestamp_ns": effective,
                       "sdk_body_timestamp_ns": stamp, "timestamp_source": timestamp_source,
                       "joint_names": BODY_NAMES, "poses": sdk.get_body_joints_pose(),
                       "velocities": sdk.get_body_joints_velocity(),
                       "accelerations": sdk.get_body_joints_acceleration(),
                       "joint_timestamps": joint_timestamps,
                       "sdk_body_timestamp_after_ns": int(sdk.get_body_timestamp_ns())}
    row["motion"] = None
    count = int(sdk.num_motion_data_available())
    if count:
        count = max(0, min(count, 3))
        row["motion"] = {"kind": "independent_tracker_poses", "source_timestamp_ns": int(sdk.get_motion_timestamp_ns()),
                         "serial_numbers": list(sdk.get_motion_tracker_serial_numbers())[:count],
                         "poses": list(sdk.get_motion_tracker_pose())[:count],
                         "velocities": list(sdk.get_motion_tracker_velocity())[:count],
                         "accelerations": list(sdk.get_motion_tracker_acceleration())[:count],
                         "timestamp_after_ns": int(sdk.get_motion_timestamp_ns())}
    row["source_timestamp_after_ns"] = int(sdk.get_time_stamp_ns())
    row["source_timestamp_unchanged_during_poll"] = before == row["source_timestamp_after_ns"]
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=3600)
    parser.add_argument("--hz", type=float, default=60)
    parser.add_argument("--port", type=int, default=8013)
    parser.add_argument("--log", type=Path)
    args = parser.parse_args()
    if args.seconds <= 0 or not 1 <= args.hz <= 120 or not 1 <= args.port <= 65535:
        parser.error("Invalid duration, polling rate, or port")
    import xrobotoolkit_sdk as sdk
    project = Path(__file__).resolve().parent.parent
    log_path = args.log or project / "work/pico_native/logs" / (time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6] + ".jsonl")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    session = uuid.uuid4().hex
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Diagnostic log: {log_path}", flush=True)
    print("Connect native XRoboToolkit app over Wi-Fi; enable Head, Controller, Full Body, Send.", flush=True)
    print("Ankles are estimated body joints 7/8. No robot commands are sent.", flush=True)
    initialized = False
    from pico_sdk_lease import NativeSdkLease
    lease=NativeSdkLease('terminal A bridge')
    try:
        lease.acquire()
        with AsyncDiagnosticLog(log_path) as log:
            sdk.init()
            initialized = True
            start = time.monotonic()
            next_print, stats_start = start + 2, start
            counts = [0, 0, 0]
            last = (0, 0, 0)
            last_changes = [None, None, None]
            last_poll = start
            while time.monotonic() - start < args.seconds:
                tick = time.monotonic()
                row = read_sdk(sdk)
                row.update(bridge_poll_duration_s=time.monotonic() - tick,
                           host_loop_gap_s=tick - last_poll,
                           diagnostic_dropped_snapshots=log.dropped)
                last_poll = tick
                row.update(schema="pico-native-sdk-snapshot-v1", session_id=session,
                           host_poll_monotonic_s=tick, host_wall_time_ns=time.time_ns(),
                           snapshot_kind="SDK_cached_getters_not_atomic",
                           sdk_coordinate_convention="xyz + xyzw; SDK output preserved")
                stamps = (row["source_timestamp_ns"],
                          row["body"]["source_timestamp_ns"] if row["body"] else 0,
                          row["motion"]["source_timestamp_ns"] if row["motion"] else 0)
                changed = False
                for i, stamp in enumerate(stamps):
                    if stamp > 0 and stamp != last[i]:
                        if last[i] and stamp < last[i]:
                            session = uuid.uuid4().hex
                            row["session_id"] = session
                        counts[i] += 1
                        last_changes[i] = tick
                        changed = True
                if changed:
                    encode = lambda value: np.asarray(value).tolist()
                    payload = json.dumps(control_snapshot(row), allow_nan=False, default=encode).encode()
                    if len(payload) < 65000:
                        sock.sendto(payload, ("127.0.0.1", args.port))
                    else:
                        raise ValueError("Snapshot exceeds local preview datagram size")
                    log.write(json.dumps(row, allow_nan=False, default=encode) + "\n")
                last = stamps
                if tick >= next_print:
                    elapsed = tick - stats_start
                    ages = [None if t is None else round(1000*(tick-t)) for t in last_changes]
                    print(f"NATIVE source_hz={counts[0]/elapsed:.1f} body_hz={counts[1]/elapsed:.1f} "
                          f"object_hz={counts[2]/elapsed:.1f} source/body/object_age_ms={ages} "
                          f"body={'yes' if row['body'] else 'no'} object_trackers="
                          f"{len(row['motion']['poses']) if row['motion'] else 0} "
                          f"poll_ms={row['bridge_poll_duration_s']*1000:.2f} "
                          f"loop_gap_ms={row['host_loop_gap_s']*1000:.2f} log_dropped={log.dropped}", flush=True)
                    if row["body"] and ages[1] is not None and ages[1] < 500:
                        p = row["body"]["poses"]
                        print(f"ANKLE ESTIMATES SDK xyz: left={np.round(p[7][:3],3)} right={np.round(p[8][:3],3)}", flush=True)
                    elif not row["body"]:
                        print("BODY MISSING: SDK has received no body joints. Check Full Body mode, tracker pairing, and calibration. Stop B and A before using pico_native.sh probe; PC Service allows one SDK reader.", flush=True)
                    else:
                        print("BODY STALE: body timestamps stopped advancing; ankle markers remain hidden until fresh body data resumes.", flush=True)
                    counts = [0, 0, 0]
                    stats_start, next_print = tick, tick + 2
                time.sleep(max(0, 1/args.hz - (time.monotonic()-tick)))
    except KeyboardInterrupt:
        print("Native recording stopped.", flush=True)
    finally:
        try:
            if initialized:sdk.close()
        finally:
            sock.close();lease.release()


if __name__ == "__main__":
    main()
