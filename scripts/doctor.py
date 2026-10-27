#!/usr/bin/env python3
"""Offline workstation checks: never initialize XR SDK, connect, or actuate."""
import argparse
import ast
import importlib
import importlib.metadata
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import re
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
X86_64_ELF_MACHINE = 62
SDK_FUNCTIONS = (
    "init", "close", "get_time_stamp_ns", "get_headset_pose",
    "get_left_controller_pose", "get_right_controller_pose",
    "get_left_grip", "get_right_grip", "get_left_trigger", "get_right_trigger",
    "get_left_axis", "get_right_axis", "get_A_button", "get_B_button",
    "get_X_button", "get_Y_button", "is_body_data_available",
    "get_body_timestamp_ns", "get_body_joints_timestamp", "get_body_joints_pose",
    "get_body_joints_velocity", "get_body_joints_acceleration",
    "num_motion_data_available", "get_motion_timestamp_ns",
    "get_motion_tracker_serial_numbers", "get_motion_tracker_pose",
    "get_motion_tracker_velocity", "get_motion_tracker_acceleration",
)


def architecture():
    value = platform.machine().lower()
    return {"amd64": "x86_64"}.get(value, value)


def elf_machine(path):
    """Read ELF machine without executing a foreign binary or invoking ldd."""
    with Path(path).open("rb") as source:
        header = source.read(20)
    if len(header) < 20 or header[:4] != b"\x7fELF":
        raise ValueError("Not an ELF binary: " + str(path))
    if header[4] != 2 or header[5] not in (1, 2):
        raise ValueError("Expected a 64-bit ELF binary: " + str(path))
    return struct.unpack(("<" if header[5] == 1 else ">") + "H", header[18:20])[0]


def native_elf(path):
    path = Path(path).expanduser().resolve(strict=True)
    observed = elf_machine(path)
    if architecture() != "x86_64" or observed != X86_64_ELF_MACHINE:
        raise ValueError(f"ELF CPU mismatch: {path}; machine={observed}, host={architecture()}")
    return {"path": str(path), "elf_machine": observed, "host": architecture()}


class Checks:
    def __init__(self):
        self.rows = []

    def run(self, name, function):
        try:
            details = function()
            self.rows.append({"name": name, "ok": True, "details": details})
            return details
        except Exception as exc:
            self.rows.append({"name": name, "ok": False,
                              "error": f"{type(exc).__name__}: {exc}"})
            return None


def core_check():
    if platform.system() != "Linux" or architecture() != "x86_64":
        raise RuntimeError("This workstation requires Linux x86_64")
    lock = json.loads((ROOT / "dependencies.lock.json").read_text())
    for name, item in lock["repositories"].items():
        if not re.fullmatch(r"[0-9a-f]{40}", item["revision"]):
            raise ValueError("Dependency is not pinned to a full revision: " + name)
    count = 0
    for source in (ROOT / "outputs").glob("*.py"):
        ast.parse(source.read_text(), filename=str(source))
        count += 1
    return {"architecture": architecture(), "python": platform.python_version(),
            "source_files_parsed": count, "source_lock": lock["schema"],
            "scope": "offline source checks only; component checks are explicit flags"}


def checkout_check(repo):
    def git(*args):
        return subprocess.check_output(["git", "-C", str(repo), *args],
                                       text=True, timeout=15, stderr=subprocess.PIPE).strip()
    if not repo.is_dir():
        raise FileNotFoundError("Run bootstrap or set XR_TELEOPERATE_REPO: " + str(repo))
    lock = json.loads((ROOT / "dependencies.lock.json").read_text())["repositories"]["xr_teleoperate"]
    observed = git("rev-parse", "HEAD")
    if observed != lock["revision"]:
        raise ValueError("xr_teleoperate revision differs from dependencies.lock.json: " + observed)
    if git("status", "--porcelain", "--untracked-files=no"):
        raise ValueError("xr_teleoperate or its submodules contain tracked edits")
    return {"path": str(repo), "revision": observed}


def ik_check(repo):
    # Avoid vendor visualizers and their servers. This checks the actual G1 model,
    # symbolic Pinocchio FK, IPOPT, and nominal mesh collision backend locally.
    import numpy as np
    import casadi as ca
    import pinocchio as pin
    from pinocchio import casadi as cpin
    import meshcat  # noqa: F401 -- upstream imports it even without a viewer
    import logging_mp  # noqa: F401
    cache = ROOT / ".build/doctor/matplotlib"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache))
    import matplotlib  # noqa: F401
    try:
        import hppfcl as fcl
    except ImportError:
        import coal as fcl
    if pin.__version__ != "3.1.0" or ca.__version__ != "3.6.7" or np.__version__ != "1.26.4":
        raise RuntimeError("IK stack differs from tested pins: " +
                           f"pinocchio={pin.__version__}, casadi={ca.__version__}, numpy={np.__version__}")
    urdf = repo / "assets/g1/g1_body29_hand14.urdf"
    if not urdf.is_file():
        raise FileNotFoundError("Pinned G1 URDF missing: " + str(urdf))
    sys.path.insert(0, str(ROOT / "outputs"))
    from g1_arm_workspace import ArmWorkspace
    from g1_gravity_comp import MOTOR_NAMES_29
    model = pin.buildModelFromUrdf(str(urdf))
    q0 = pin.neutral(model)
    indices = [model.joints[model.getJointId(name)].idx_q for name in MOTOR_NAMES_29]
    if len(set(indices)) != 29:
        raise ValueError("G1 motor-name mapping is incomplete")
    for side in ("left", "right"):
        for name, value in (("hip_pitch", -.15), ("knee", .35), ("ankle_pitch", -.20), ("elbow", 1.0)):
            q0[model.joints[model.getJointId(f"{side}_{name}_joint")].idx_q] = value
    q0[model.joints[model.getJointId("left_shoulder_roll_joint")].idx_q] = .2
    q0[model.joints[model.getJointId("right_shoulder_roll_joint")].idx_q] = -.2
    symbolic_model = cpin.Model(model)
    symbolic_data = symbolic_model.createData()
    symbolic_q = ca.SX.sym("doctor_model_q", model.nq)
    cpin.forwardKinematics(symbolic_model, symbolic_data, symbolic_q)
    ankle_id = model.getJointId("left_ankle_roll_joint")
    endpoint = ca.Function("doctor_ankle_fk", [symbolic_q], [symbolic_data.oMi[ankle_id].translation])
    varying = [model.joints[model.getJointId(name)].idx_q
               for name in ("left_hip_pitch_joint", "left_knee_joint")]
    goal = q0.copy()
    goal[varying] += np.array([-.02, .03])
    target = endpoint(goal)
    opti = ca.Opti()
    joints = opti.variable(2)
    candidate = ca.MX(q0)
    for i, index in enumerate(varying):
        candidate[index] = joints[i]
    opti.minimize(1000 * ca.sumsqr(endpoint(candidate)-target)
                  + 1e-7 * ca.sumsqr(joints-ca.DM(q0[varying])))
    opti.subject_to(opti.bounded(ca.DM(q0[varying]-.1), joints, ca.DM(q0[varying]+.1)))
    opti.set_initial(joints, q0[varying])
    opti.solver("ipopt", {"print_time": False},
                {"print_level": 0, "sb": "yes", "max_iter": 60, "tol": 1e-9})
    solution = opti.solve()
    solved_q = q0.copy()
    solved_q[varying] = np.asarray(solution.value(joints)).ravel()
    error = float(np.linalg.norm(np.asarray(endpoint(solved_q)-target)))
    if not math.isfinite(error) or error > 1e-4:
        raise RuntimeError(f"Offline G1 IK residual too large: {error} m")
    workspace = ArmWorkspace(urdf)
    clearance = workspace.clearance(q0[indices])
    if not np.isfinite(clearance).all():
        raise RuntimeError("Nominal G1 mesh query returned nonfinite clearance")
    transform = getattr(fcl, "Transform3f", None) or fcl.Transform3s
    sphere = fcl.Sphere(.1)
    if not fcl.collide(sphere, transform(), sphere, transform(),
                       fcl.CollisionRequest(), fcl.CollisionResult()):
        raise RuntimeError("Collision backend failed an overlapping-spheres check")
    far = transform(np.eye(3), np.array([1., 0., 0.]))
    if fcl.collide(sphere, transform(), sphere, far,
                   fcl.CollisionRequest(), fcl.CollisionResult()):
        raise RuntimeError("Collision backend failed a separated-spheres check")
    return {"pinocchio": pin.__version__, "casadi": ca.__version__,
            "numpy": np.__version__, "fcl": getattr(fcl, "__version__", "unknown"),
            "urdf": str(urdf), "model_nq": model.nq, "motor_count": len(indices),
            "offline_ik_position_error_m": error, "nominal_mesh_clearance_m": clearance.tolist(),
            "scope": "software/model checks; no live IK, viewer, SDK, or motor connection"}


def sdk_library_default():
    return ROOT / "third_party/XRoboToolkit-PC-Service-Pybind/lib/libPXREARobotSDK.so"


def pico_check():
    import numpy as np
    spec = importlib.util.find_spec("xrobotoolkit_sdk")
    if spec is None or not spec.origin:
        raise ImportError("Build xrobotoolkit_sdk in g1-pico first")
    extension = native_elf(spec.origin)
    library = native_elf(os.environ.get("PICO_SDK_LIBRARY") or sdk_library_default())
    sdk = importlib.import_module("xrobotoolkit_sdk")
    missing = [name for name in SDK_FUNCTIONS if not callable(getattr(sdk, name, None))]
    if missing:
        raise RuntimeError("Native binding lacks required APIs: " + ", ".join(missing))
    version = importlib.metadata.version("xrobotoolkit_sdk")
    if version != "1.0.2":
        raise RuntimeError("Unexpected native binding version: " + version)
    return {"extension": extension, "raw_probe_library": library,
            "binding_version": version, "numpy": np.__version__,
            "required_api_count": len(SDK_FUNCTIONS), "sdk_initialized": False,
            "scope": "import, ELF architecture and API inventory only; no service connection"}


def service_check(explicit_binary):
    script = os.environ.get("PICO_SERVICE_SCRIPT")
    if not script:
        generated = ROOT / ".build" / architecture() / "launch_service.sh"
        if generated.is_file():
            script = str(generated)
        else:
            raise ValueError("Set PICO_SERVICE_SCRIPT or build the service explicitly")
    launcher = Path(script).expanduser().resolve(strict=True)
    if not launcher.is_file():
        raise ValueError("PICO_SERVICE_SCRIPT is not a file")
    candidates = []
    if explicit_binary:
        candidates.append(Path(explicit_binary))
    else:
        candidates.extend((launcher.parent / "RoboticsServiceProcess",
                           ROOT / "third_party/XRoboToolkit-PC-Service/RoboticsService/bin/RoboticsServiceProcess"))
    binary = next((path for path in candidates if path.is_file()), None)
    if binary is None:
        raise ValueError("Cannot locate service executable; pass --service-binary PATH")
    details = native_elf(binary)
    details.update(launcher=str(launcher), service_started=False,
                   scope="ELF architecture only; runtime libraries and headset receipt still need commissioning")
    return details


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core", action="store_true", help="Standard-library checks (always included)")
    parser.add_argument("--ik", action="store_true", help="Run in g1-ik: model FK/IK and mesh collision checks")
    parser.add_argument("--pico", action="store_true", help="Run in g1-pico: binding import/API/CPU checks")
    parser.add_argument("--service", action="store_true", help="Check configured service ELF without starting it")
    parser.add_argument("--service-binary", help="Executable corresponding to PICO_SERVICE_SCRIPT")
    parser.add_argument("--repo", type=Path,
                        default=Path(os.environ.get("XR_TELEOPERATE_REPO", ROOT / "third_party/xr_teleoperate")))
    parser.add_argument("--json", action="store_true", help="Print a machine-readable report")
    args = parser.parse_args()
    checks = Checks()
    checks.run("core", core_check)
    if args.ik:
        checkout = checks.run("xr_teleoperate_checkout", lambda: checkout_check(args.repo.expanduser().resolve()))
        if checkout is not None:
            checks.run("g1_ik_and_collision", lambda: ik_check(args.repo.expanduser().resolve()))
    if args.pico:
        checks.run("pico_native_binding", pico_check)
    if args.service:
        checks.run("pico_service_binary", lambda: service_check(args.service_binary))
    report = {"ok": all(row["ok"] for row in checks.rows), "checks": checks.rows,
              "robot_connected": False, "sdk_initialized": False,
              "note": "Offline software checks only; verify live tracking and latency when commissioning the workstation."}
    if args.json:
        print(json.dumps(report, indent=2, allow_nan=False))
    else:
        for row in checks.rows:
            print(("PASS " if row["ok"] else "FAIL ") + row["name"])
            print("  " + (json.dumps(row["details"], allow_nan=False) if row["ok"] else row["error"]))
        print(report["note"])
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
