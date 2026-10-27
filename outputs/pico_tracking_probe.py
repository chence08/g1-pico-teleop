"""Read native PC Service packet structure without sending device/robot commands.

Uses the ABI from installed SDK/include/PXREARobotSDK.h. Prints field names,
joint counts, and acquisition status; does not print poses or device serials.
"""
import os
import argparse
import collections
import ctypes
import json
import threading
import time


class DeviceState(ctypes.Structure):
    _fields_ = [("devID", ctypes.c_char * 32), ("stateJson", ctypes.c_char * 16352)]


def as_json(value):
    return json.loads(value) if isinstance(value, str) else value


def motion_poses(text):
    """Read only the current packet, respecting len (the client can retain old joints)."""
    from pico_native_input import pose7_matrix
    value = as_json(json.loads(text).get("value", {}))
    block = as_json(value.get("Motion")) if isinstance(value, dict) else None
    if not isinstance(block, dict):
        return []
    joints = block.get("joints", [])
    count = max(0, min(int(block.get("len", 0)), len(joints), 3))
    result = []
    for joint in joints[:count]:
        pose = joint["p"]
        pose = [float(v) for v in pose.split(",")] if isinstance(pose, str) else pose
        result.append((str(joint["sn"]), pose7_matrix(pose)))
    return result


class ObjectViewer:
    """Raw tracker positions; packet receipt freshness, not acquisition freshness."""
    def __init__(self):
        import meshcat
        import meshcat.geometry as mg
        self.viewer = meshcat.Visualizer()
        self.geometry = mg
        self.identities = {}
        self.origin = None
        print(f"Open raw tracker viewer on this PC: {self.viewer.url()}", flush=True)
        print("Markers are physical trackers, not anatomical ankles. Colors have no left/right assignment.", flush=True)
        print("Hidden after 500ms without Motion packets. Tracker acquisition timestamps are unavailable.", flush=True)

    def update(self, poses, received, now):
        import numpy as np
        from pico_native_input import BASIS
        root = self.viewer["raw_trackers"]
        root.set_property("visible", bool(poses) and now - received <= 0.5)
        if not poses or now - received > 0.5:
            return
        if self.origin is None:
            self.origin = np.mean([BASIS @ t[:3, 3] for _, t in poses], axis=0)
        active = set()
        for identity, pose in poses:
            if identity not in self.identities:
                self.identities[identity] = len(self.identities)
            idx = self.identities[identity]
            active.add(idx)
            node = root[f"tracker_{idx}"]
            node.set_property("visible", True)
            node.set_object(self.geometry.Sphere(0.045), self.geometry.MeshLambertMaterial(
                color=(0x22cc66, 0xff4466, 0x5599ff)[idx % 3]))
            transform = np.eye(4)
            transform[:3, :3] = BASIS @ pose[:3, :3] @ BASIS.T
            transform[:3, 3] = BASIS @ pose[:3, 3] - self.origin
            node.set_transform(transform)
        for idx in self.identities.values():
            if idx not in active:
                root[f"tracker_{idx}"].set_property("visible", False)


def summarize_packet(text):
    outer = json.loads(text)
    value = as_json(outer.get("value", {}))
    if not isinstance(value, dict):
        raise ValueError("Tracking payload is not an object")
    summary = {"function": outer.get("functionName"), "fields": sorted(value)}
    for name in ("Body", "Motion"):
        block = as_json(value.get(name))
        if isinstance(block, dict):
            summary[name] = {"fields": sorted(block),
                             "joint_count": len(block.get("joints", [])),
                             "has_timestamp": "timeStampNs" in block}
        elif block is not None:
            summary[name] = {"type": type(block).__name__}
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--viewer", action="store_true", help="Show raw Object Tracking markers (requires meshcat)")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("seconds must be positive")
    library = os.environ.get("PICO_SDK_LIBRARY")
    if not library: raise RuntimeError("Set PICO_SDK_LIBRARY to the SDK library built for this architecture")
    lib = ctypes.CDLL(library)
    callback_type = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p)
    lock = threading.Lock()
    counts = collections.Counter()
    last_print = [0.]
    latest_motion = [[], 0.]
    viewer = ObjectViewer() if args.viewer else None

    @callback_type
    def callback(context, kind, status, pointer):
        if kind != 1 << 25 or not pointer:
            return
        try:
            state = ctypes.cast(pointer, ctypes.POINTER(DeviceState)).contents
            packet_text = bytes(state.stateJson).decode("utf-8")
            summary = summarize_packet(packet_text)
            poses = motion_poses(packet_text) if viewer else []
            with lock:
                if viewer:
                    latest_motion[:] = [poses, time.monotonic()]
                counts["packets"] += 1
                counts["Body_present"] += "Body" in summary
                counts["Motion_present"] += "Motion" in summary
                if time.monotonic() - last_print[0] >= 1:
                    print("RAW STRUCTURE " + json.dumps(summary), flush=True)
                    last_print[0] = time.monotonic()
        except Exception as error:
            with lock:
                counts["decode_errors"] += 1
                if counts["decode_errors"] == 1:
                    print(f"Packet decode error: {type(error).__name__}: {error}", flush=True)

    lib.PXREAInit.argtypes = [ctypes.c_void_p, callback_type, ctypes.c_uint]
    lib.PXREAInit.restype = ctypes.c_int
    lib.PXREADeinit.argtypes = []
    lib.PXREADeinit.restype = ctypes.c_int
    from pico_sdk_lease import NativeSdkLease
    lease=NativeSdkLease('raw tracking probe').acquire()
    result = lib.PXREAInit(None, callback, 0xffffffff)
    if result != 0:
        lease.release()
        raise RuntimeError(f"SDK initialization failed ({result}); start PC Service first")
    print("Read-only packet probe: keep XRoboToolkit connected with Send enabled.", flush=True)
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            if viewer:
                with lock:
                    poses, received = latest_motion
                viewer.update(poses, received, time.monotonic())
            time.sleep(0.03 if viewer else 0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:lib.PXREADeinit()
        finally:lease.release()
    with lock:
        print("SUMMARY " + json.dumps(dict(counts)), flush=True)
    if not counts["packets"]:
        print("No state packets observed; verify PC Service, native-app connection, and Send.")
    elif not counts["Body_present"]:
        print("Tracking packets received, but no Body field. The native-client body-output path is not producing Body; this probe cannot identify which headset-side condition failed.")
        if counts["Motion_present"]:
            print("Motion output observed: raw tracker output is available. Full Body and Object Tracking are separate modes.")
        else:
            print("If Full Body was selected, test Object Tracking separately and rerun this probe; compare Motion_present and joint_count.")
    else:
        print("Body field observed. Compare its structure with the Python SDK parser.")


if __name__ == "__main__":
    main()
