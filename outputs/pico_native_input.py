"""Local native-PICO preview adapter. No robot SDKs or hardware output.

The SDK supplies cached getters rather than an atomic frame/per-controller
validity API. Fresh source timestamps detect a stopped stream, but do not
establish independently fresh controller poses. Preview use only.
"""
import json
import socket
import threading
import time
from types import SimpleNamespace

import numpy as np

BODY_NAMES = ["pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee",
              "spine2", "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot",
              "neck", "left_collar", "right_collar", "head", "left_shoulder", "right_shoulder",
              "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hand", "right_hand"]
BASIS = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])


def pose7_matrix(pose):
    p = np.asarray(pose, dtype=float)
    if p.shape != (7,) or not np.isfinite(p).all():
        raise ValueError("Expected finite xyz + xyzw pose")
    norm = np.linalg.norm(p[3:])
    if abs(norm - 1) > 0.03:
        raise ValueError("Invalid quaternion")
    x, y, z, w = p[3:] / norm
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = p[:3]
    return t


def sdk_controller_event(row):
    event = {}
    for side in ("left", "right"):
        event[side] = pose7_matrix(row[f"{side}_controller_pose"]).flatten(order="F").tolist()
        grip = float(row[f"{side}_grip"])
        if not np.isfinite(grip) or not 0 <= grip <= 1:
            raise ValueError("Invalid grip")
        event[f"{side}State"] = {"squeeze": grip > 0.8, "squeezeValue": grip}
        if f'{side}_trigger' in row:
            trigger=float(row[f'{side}_trigger'])
            if not np.isfinite(trigger) or not 0<=trigger<=1:
                raise ValueError('Invalid controller trigger')
            event[f'{side}State']['triggerValue']=trigger
    return event


class NativeReceiver:
    def __init__(self, on_event, on_reset, port=8013):
        self.on_event, self.on_reset = on_event, on_reset
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(0.2)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.head = np.eye(4)
        self.body = None
        self.body_seen = 0.
        self.body_stamp = 0
        self.body_sequence = 0
        self.origin = None
        self.session = None
        self.last_stamp = None
        self.need_release = True
        self.bad_packets = 0
        self.received_packets = 0
        self.last_packet_seen = 0.
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def consume(self, row, now):
        if row.get("schema") != "pico-native-sdk-snapshot-v1":
            raise ValueError("Unknown bridge schema")
        sampled = float(row["host_poll_monotonic_s"])
        if not np.isfinite(sampled) or sampled > now + 0.05 or now - sampled > 1:
            raise ValueError("Stale or invalid local bridge timestamp")
        self.received_packets = getattr(self,'received_packets',0)+1
        session = str(row["session_id"])
        if session != self.session:
            self.session, self.last_stamp, self.need_release = session, None, True
            self.origin = None
            with self.lock:
                self.body, self.body_stamp, self.body_seen = None, 0, 0.
                self.body_sequence = 0
            self.on_reset()
        with self.lock:
            body = row.get("body")
            if body and body["source_timestamp_ns"] > self.body_stamp:
                poses = np.asarray(body["poses"], dtype=float)
                if (poses.shape == (24, 7) and np.isfinite(poses).all()
                        and np.all(np.abs(np.linalg.norm(poses[:,3:],axis=1)-1.) <= .03)):
                    self.body = poses.copy()
                    self.body_stamp = body["source_timestamp_ns"]
                    self.body_seen = sampled
                    self.body_sequence = getattr(self,'body_sequence',0)+1
        stamp = int(row["source_timestamp_ns"])
        if stamp <= 0 or (self.last_stamp is not None and stamp <= self.last_stamp):
            return
        self.last_stamp = stamp
        event = sdk_controller_event(row)
        head = pose7_matrix(row["head_pose"])
        self.last_packet_seen = now
        with self.lock:
            self.head[:3, :3] = BASIS @ head[:3, :3] @ BASIS.T
            self.head[:3, 3] = BASIS @ head[:3, 3]
        if self.need_release:
            if event["leftState"]["squeeze"] or event["rightState"]["squeeze"]:
                return
            self.need_release = False
        self.on_event(event, sampled)

    def _run(self):
        while not self.stop.is_set():
            try:
                payload, _ = self.sock.recvfrom(65535)
                self.consume(json.loads(payload), time.monotonic())
            except socket.timeout:
                continue
            except OSError:
                break
            except (ValueError, KeyError, TypeError, OverflowError):
                self.bad_packets += 1

    def get_tele_data(self):
        with self.lock:
            return SimpleNamespace(head_pose=self.head.copy())

    def get_body_arm_points(self, max_age=.2):
        """Estimated shoulder/elbow/wrist positions in the robot-axis basis."""
        with self.lock:
            age=time.monotonic()-self.body_seen
            if self.body is None or age>max_age:
                return None,age
            poses=self.body.copy()
        points=np.array([poses[[16,18,20],:3],poses[[17,19,21],:3]])
        return (BASIS@points.reshape(-1,3).T).T.reshape(2,3,3),age

    def get_body_leg_data(self, max_age=.15):
        """Estimated bundle transforms, observed age, sequence; not per-joint capture age."""
        with self.lock:
            age=max(0.,time.monotonic()-self.body_seen) if self.body is not None else float('inf')
            sequence=getattr(self,'body_sequence',0)
            if self.body is None or age>max_age or sequence<2:
                return None,age,sequence
            poses=self.body.copy()
        try:
            transforms=np.stack([pose7_matrix(pose) for pose in poses])
        except (TypeError,ValueError):
            return None,age,sequence
        transforms[:,:3,3]=(BASIS@transforms[:,:3,3].T).T
        transforms[:,:3,:3]=BASIS@transforms[:,:3,:3]@BASIS.T
        return transforms,age,sequence

    def update_visualizer(self, viewer):
        import meshcat.geometry as mg
        with self.lock:
            body = None if self.body is None else self.body.copy()
            age = time.monotonic() - self.body_seen
        node = viewer["human_body_estimate"]
        if body is None or age > 0.5:
            node.set_property("visible", False)
            return
        points = (BASIS @ body[:, :3].T).T
        if self.origin is None:
            self.origin = points[0].copy()
        points += np.array([0., -1.2, 0.]) - self.origin
        node.set_property("visible", True)
        node["joints"].set_object(mg.Points(mg.PointsGeometry(points.T.astype(np.float32)),
                                           mg.PointsMaterial(color=0xffaa33, size=0.025)))
        for idx, name, color in ((7, "left_ankle_estimate", 0x22cc66),
                                  (8, "right_ankle_estimate", 0xff4466)):
            node[name].set_object(mg.Sphere(0.045), mg.MeshLambertMaterial(color=color))
            transform = np.eye(4)
            transform[:3, 3] = points[idx]
            node[name].set_transform(transform)

    def close(self):
        self.stop.set()
        self.sock.close()
        self.thread.join(timeout=1)
