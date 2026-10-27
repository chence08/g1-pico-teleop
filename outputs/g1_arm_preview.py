"""Native PICO-to-G1 preview with an optional, explicit bounded physical arm link.

The default is virtual only. --arm-link live enables robot arm commands.

This uses upstream G1 arm IK and its stock Dex3 visual model. It does not
validate Inspire geometry, contacts, balance, collisions, or physical safety.
Hold BOTH side grips to move; release either to clutch and hold the preview.
The event timeout detects transport loss, not individual XR tracking validity.
"""

import argparse
import json
from multiprocessing import Array
import os
from pathlib import Path
import sys
import time

import numpy as np


def forward(ik, q, pin):
    pin.framesForwardKinematics(ik.reduced_robot.model, ik.reduced_robot.data, q)
    return [ik.reduced_robot.data.oMf[idx].homogeneous.copy()
            for idx in (ik.L_hand_id, ik.R_hand_id)]


def use_measured_posture_reference(ik, q, joint_range=None, responsive=False):
    """Avoid pulling a physical robot toward upstream IK's zero-pose prior."""
    import casadi
    posture_cost = casadi.sumsqr(ik.var_q - casadi.DM(q))
    if responsive:
        wrist_cost = casadi.sumsqr(ik.var_q[[5,6,12,13]])
        elbows = casadi.vertcat(*[ik.cdata.oMi[ik.cmodel.getJointId(f'{side}_elbow_joint')].translation
                                  for side in ('left','right')])
        ik.elbow_function = casadi.Function('arm_elbow_positions', [ik.cq], [elbows])
        ik.elbow_target_parameter = ik.opti.parameter(6)
        ik.elbow_hint_enable = ik.opti.parameter()
        ik.opti.set_value(ik.elbow_target_parameter, np.array(ik.elbow_function(q)).ravel())
        ik.opti.set_value(ik.elbow_hint_enable, 0.)
        elbow_cost = casadi.sumsqr(ik.elbow_function(ik.var_q)-ik.elbow_target_parameter)
        ik.opti.minimize(500 * ik.translational_cost + 10 * ik.rotation_cost
                         + .02 * posture_cost + .08 * ik.smooth_cost + .03 * wrist_cost
                         + 8 * ik.elbow_hint_enable * elbow_cost)
        from g1_arm_workspace import add_ik_clearance
        add_ik_clearance(ik)
        # A local solve follows the measured configuration through the global
        # workspace. It cannot jump to a distant IK branch for a small TCP move.
        ik.measured_q_parameter = ik.opti.parameter(14)
        ik.opti.set_value(ik.measured_q_parameter, q)
        ik.opti.subject_to(ik.opti.bounded(-.25, ik.var_q-ik.measured_q_parameter, .25))
        from teleop.utils.weighted_moving_filter import WeightedMovingFilter
        ik.smooth_filter = WeightedMovingFilter(np.array([1.]), 14)
    else:
        ik.opti.minimize(50 * ik.translational_cost + ik.rotation_cost
                         + 0.02 * posture_cost + 0.1 * ik.smooth_cost)
    if joint_range is not None:
        # Solve for a reachable pose inside the commissioned box rather than
        # solving outside it and dropping the operator's clutch every time.
        margin = min(.02, joint_range / 4)
        ik.session_lower = (np.maximum(ik.reduced_robot.model.lowerPositionLimit, q - joint_range + margin)
                            if joint_range else ik.reduced_robot.model.lowerPositionLimit.copy())
        ik.session_upper = (np.minimum(ik.reduced_robot.model.upperPositionLimit, q + joint_range - margin)
                            if joint_range else ik.reduced_robot.model.upperPositionLimit.copy())
        if responsive:
            from g1_arm_link import operating_joint_limits
            lower, upper = operating_joint_limits()
            ik.session_lower = np.maximum(ik.session_lower, lower)
            ik.session_upper = np.minimum(ik.session_upper, upper)
        ik.opti.subject_to(ik.opti.bounded(casadi.DM(ik.session_lower), ik.var_q,
                                         casadi.DM(ik.session_upper)))


class JointTrajectory:
    """Velocity/acceleration bounded commands; holds reset velocity immediately."""
    def __init__(self, q, lower, upper, speed=.08, acceleration=.3):
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        self.speed, self.acceleration = speed, acceleration
        self.reset(q)

    def reset(self, q):
        self.q = np.asarray(q, dtype=float).copy()
        self.velocity = np.zeros_like(self.q)

    def step(self, target, dt):
        from g1_arm_link import bounded_joint_step
        self.q, self.velocity = bounded_joint_step(
            self.q, self.velocity, np.asarray(target), dt, self.lower, self.upper,
            speed=self.speed, acceleration=self.acceleration, time_constant=.18)
        return self.q.copy()


def arm_feedback_mode(state, process_exit=None):
    """Brief link gaps hold; robot faults and prolonged loss end the session."""
    if state["fault"] or process_exit not in (None,0):
        return "fault"
    if (state.get('control_scope')=='suspended_body' and
            state.get('status')=='damping suspended body'):
        return 'shutdown'
    if process_exit is not None or state["state_age_s"] > 0.5:
        return "fault"
    if state["link_age_s"] > 1:
        return "lost"
    if state["link_age_s"] > 0.2:
        return "gap"
    if state['state_age_s'] > .1:
        return 'state_wait'
    if not state.get("mode_query_ok", True):
        return "mode_wait"
    return "fresh"


def relative_targets(controller, anchor_controller, anchor_robot, scale):
    targets = []
    for pose, origin, robot in zip(controller, anchor_controller, anchor_robot):
        target = robot.copy()
        target[:3, 3] += scale * (pose[:3, 3] - origin[:3, 3])
        target[:3, :3] = pose[:3, :3] @ origin[:3, :3].T @ robot[:3, :3]
        targets.append(target)
    return targets


def engagement_has_moved(targets, anchors, orientation):
    for target, anchor in zip(targets, anchors):
        if np.linalg.norm(target[:3,3] - anchor[:3,3]) > .0005:
            return True
        if orientation == 'controller':
            angle = np.arccos(np.clip((np.trace(target[:3,:3] @ anchor[:3,:3].T)-1)/2,-1,1))
            if angle > .002:
                return True
    return False


def prepare_receiving_pose(link, live):
    """Deliberate arm-only setup motion, separate from operator demonstrations."""
    target = np.array([0.,.25,0.,-.40,0.,0.,0., 0.,-.25,0.,-.40,0.,0.,0.])
    if not live:
        return np.array(link.snapshot()['q'], dtype=float)
    print('SETUP: moving arms to the forward receiving pose; keep grips released.', flush=True)
    start = time.monotonic()
    settled_since = None
    while time.monotonic()-start < 7:
        now = time.monotonic()
        state = link.snapshot()
        if arm_feedback_mode(state, link.process.poll()) != 'fresh':
            raise RuntimeError('Receiving-pose setup lost confirmed fresh robot feedback')
        measured = np.array(state['q'], dtype=float)
        follow = now-start >= .4
        link.submit(target if follow else measured, now, follow)
        settled = (follow and state.get('weight',0)>.99 and
                   np.max(np.abs(measured-target)) < .035 and
                   np.max(np.abs(state['dq'])) < .06)
        settled_since = (now if settled_since is None else settled_since) if settled else None
        if settled_since is not None and now-settled_since > .3:
            print('SETUP complete: receiving pose measured and settled.', flush=True)
            return measured
        time.sleep(.02)
    raise RuntimeError('Receiving-pose setup did not settle; arm control will release')


def decode_controller_packet(value):
    """Decode one complete browser event; never replace absent inputs with zeros."""
    poses, grips = [], []
    for side in ("left", "right"):
        pose = np.asarray(value.get(side), dtype=float)
        state = value.get(f"{side}State")
        if pose.size != 16 or not isinstance(state, dict):
            raise ValueError("Incomplete controller event")
        pose = pose.reshape(4, 4, order="F")
        if (not np.isfinite(pose).all() or
                not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-4) or
                not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=0.03) or
                np.linalg.det(pose[:3, :3]) < 0.95):
            raise ValueError("Invalid controller transform")
        if "squeeze" not in state and "squeezeValue" not in state:
            raise ValueError("Missing grip state")
        grips.append(bool(state.get("squeeze", False)) or float(state.get("squeezeValue", 0)) > 0.8)
        poses.append(pose)
    return poses, grips


def controller_reference(poses, reference_rotation):
    """Convert OpenXR world poses to a fixed robot-axis reference."""
    basis = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
    result = []
    for pose in poses:
        target = np.eye(4)
        target[:3, 3] = reference_rotation @ basis @ pose[:3, 3]
        target[:3, :3] = reference_rotation @ basis @ pose[:3, :3] @ basis.T
        result.append(target)
    return result


def input_mode(age, grips, released, hold_timeout=0.25, rearm_timeout=1.0):
    if age > rearm_timeout:
        return "lost", False
    if age > hold_timeout:
        return "gap", released
    if not all(grips):
        return "clutch", released or not any(grips)
    return ("follow" if released else "release"), released


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    from g1_config import XR_REPO, REMOTE_TOOLS
    parser.add_argument("--repo", type=Path, default=XR_REPO)
    parser.add_argument("--self-test", action="store_true", help="Check model and IK without starting network services")
    parser.add_argument("--viewer-only", action="store_true", help="Show the model without starting XR tracking")
    parser.add_argument("--scale", type=float)
    parser.add_argument("--seconds", type=float, default=900)
    parser.add_argument("--native-port", type=int, default=8013)
    parser.add_argument("--arm-link", choices=("shadow", "live"), help="Optional bounded Orin arm connection; native input only")
    parser.add_argument("--joint-range", type=float, help="Session-start envelope up to 1.5 rad; 0 uses absolute URDF limits")
    parser.add_argument("--profile", choices=("commissioning", "responsive"), default="commissioning")
    parser.add_argument('--arm-transport', choices=('udp','ssh'), help='Responsive default: compact latest-only UDP, bootstrapped over SSH')
    parser.add_argument('--hands', choices=('none','inspire'), default='none',
                        help='Optional Inspire DFX trigger control with separate serial worker')
    parser.add_argument('--legs', choices=('none','suspended'), default='none',
                        help='Explicit suspended full-body owner; no standing/walking controller')
    parser.add_argument("--orientation", choices=("fixed", "controller"), help="Wrist target orientation")
    parser.add_argument("--gravity-comp", choices=("off", "calibrated"))
    parser.add_argument('--ready-pose', choices=('receive','current'), default='current',
                        help='Start at measured pose (default), or deliberately move to receiving preset')
    parser.add_argument('--elbow-hint', choices=('body','posture'), default='body',
                        help='Soft upper-arm guidance from fresh estimated PICO body joints')
    parser.add_argument("--no-viewer", action="store_true", help="Remove Meshcat/body rendering from the control loop")
    args = parser.parse_args()
    responsive = args.profile == "responsive"
    args.scale = args.scale if args.scale is not None else (1. if responsive else .5)
    args.joint_range = args.joint_range if args.joint_range is not None else (0. if responsive else .15)
    args.orientation = args.orientation or ("controller" if responsive else "fixed")
    args.gravity_comp = args.gravity_comp or ("calibrated" if responsive else "off")
    if not 0 < args.scale <= 1 or args.seconds <= 0:
        parser.error("Use 0 < scale <= 1 and seconds > 0")
    if not np.isfinite(args.joint_range) or not 0 <= args.joint_range <= 1.5:
        parser.error("Use 0 <= joint-range <= 1.5 radians; 0 uses URDF limits")
    if args.arm_link and (args.self_test or args.viewer_only):
        parser.error("--arm-link requires native input and a normal tracking preview")
    if args.hands=='inspire' and not args.arm_link:
        parser.error('--hands inspire requires native input and --arm-link live or shadow')
    if args.legs=='suspended' and (not args.arm_link or not responsive or
            args.arm_transport=='ssh' or args.joint_range!=0 or args.ready_pose!='current' or args.gravity_comp!='calibrated'):
        parser.error('--legs suspended requires native, --arm-link, responsive/UDP, joint-range 0, current pose and calibrated gravity')
    repo = args.repo.resolve()
    scratch = Path(__file__).resolve().parent.parent / "work" / "g1_arm_preview"
    (scratch / "teleop").mkdir(parents=True, exist_ok=True)
    if not (scratch / "assets").exists():
        (scratch / "assets").symlink_to(repo / "assets", target_is_directory=True)
    project = Path(__file__).resolve().parent.parent
    os.environ.setdefault("MPLCONFIGDIR", str(scratch / "matplotlib"))
    # Upstream IK resolves its assets and model cache relative to the CWD.
    os.chdir(scratch / "teleop")
    sys.path.insert(0, str(repo))
    import pinocchio as pin
    from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

    # Upstream cache contains model objects but omits their visual geometry.
    # Rebuild in this script's scratch directory to keep the viewer populated.
    Path("g1_29_model_cache.pkl").unlink(missing_ok=True)
    ik = G1_29_ArmIK(Unit_Test=False, Visualization=False)
    model = ik.reduced_robot.model
    # Upstream adds end-effector frames after constructing the reduced robot.
    ik.reduced_robot.data = model.createData()
    if model.nq != 14:
        raise RuntimeError(f"Expected 14 arm joints, got {model.nq}")
    q = pin.neutral(model)
    for name, value in (("left_elbow_joint", 1.0), ("right_elbow_joint", 1.0),
                        ("left_shoulder_roll_joint", 0.2), ("right_shoulder_roll_joint", -0.2)):
        joint = model.joints[model.getJointId(name)]
        q[joint.idx_q] = value
    q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)

    if args.self_test:
        test_q = q.copy()
        test_q[model.joints[model.getJointId("left_elbow_joint")].idx_q] += 0.05
        targets = forward(ik, test_q, pin)
        for _ in range(8):
            q, _ = ik.solve_ik(*targets, current_lr_arm_motor_q=q)
        if q is None or not np.isfinite(q).all():
            raise RuntimeError("IK returned invalid joint positions")
        actual = forward(ik, q, pin)
        errors = [np.linalg.norm(a[:3, 3] - t[:3, 3]) for a, t in zip(actual, targets)]
        if max(errors) > 0.02:
            raise RuntimeError(f"IK position errors exceed 2 cm: {errors}")
        print(f"PASS: 14 arm joints; IK position errors (m): {errors}")
        print("No robot communication or network services started.")
        return

    arm_link = None
    arm_initial = None
    leg_ik = None
    leg_gate = None
    if args.arm_link:
        from g1_arm_link import ArmLink
        arm_link = ArmLink(live=args.arm_link == "live", joint_range=args.joint_range,
                           profile=args.profile, gravity_comp=args.gravity_comp, transport=args.arm_transport,
                           remote_script=REMOTE_TOOLS+'/g1_suspended_body.py' if args.legs=='suspended' else None)
        try:
            robot_state = arm_link.snapshot(timeout=12)
            if robot_state.get("joint_range_rad") != args.joint_range:
                raise RuntimeError("Workstation/Orin joint ranges do not match; deploy the updated arm link")
            if robot_state.get('profile') != args.profile:
                raise RuntimeError('Workstation/Orin control profiles do not match')
            q = np.array(robot_state["q"], dtype=float)
            arm_initial = np.array(robot_state["initial_q"], dtype=float)
            if (q.shape != (14,) or not np.isfinite(q).all() or
                    np.any(q < model.lowerPositionLimit) or np.any(q > model.upperPositionLimit)):
                raise RuntimeError("Measured arm positions do not match model limits")
            if not responsive:
                use_measured_posture_reference(ik, q, args.joint_range, responsive=False)
            print(f"ORIN {args.arm_link.upper()}: profile={args.profile}, range={args.joint_range:g} rad (0=URDF), "
                  f"robot cap={2. if responsive else .1:g} rad/s, orientation={args.orientation}, transport={arm_link.transport}.", flush=True)
            if args.legs=='suspended':
                if robot_state.get('control_scope')!='suspended_body':raise RuntimeError('Wrong full-body server')
                from g1_leg_ik import SuspendedLegIK
                from g1_leg_input import LegMotionGate
                initial29=np.array(robot_state['motor_q29'],dtype=float)
                initial29[:12]=robot_state['leg_initial_q']
                leg_ik=SuspendedLegIK(initial29,repo/'assets/g1/g1_body29_hand14.urdf')
                leg_ik.session_lower=np.maximum(leg_ik.session_lower,robot_state['leg_lower_rad'])
                leg_ik.session_upper=np.minimum(leg_ik.session_upper,robot_state['leg_upper_rad'])
                leg_gate=LegMotionGate()
                print('Suspended full-body mode: fixed pelvis; both grips clutch arms and legs. Live exit leaves damping.',flush=True)
            else:print("Arm-only body connection; optional hands use a separate worker.", flush=True)
        except Exception:
            arm_link.close()
            raise

    vis = None
    if not args.no_viewer:
        from pinocchio.visualize import MeshcatVisualizer
        vis = MeshcatVisualizer(model, ik.reduced_robot.collision_model, ik.reduced_robot.visual_model)
        vis.initViewer(open=False)
        vis.loadViewerModel("g1_arm_preview")
        vis.display(q)
        print(f"Open this on the PC: {vis.viewer.url()}", flush=True)
    else:
        print("Headless control: no Meshcat or body rendering in the control loop.", flush=True)
    trajectory = None
    if arm_link and not responsive:
        lower = np.maximum(model.lowerPositionLimit, arm_initial - args.joint_range) if args.joint_range else model.lowerPositionLimit
        upper = np.minimum(model.upperPositionLimit, arm_initial + args.joint_range) if args.joint_range else model.upperPositionLimit
        trajectory = JointTrajectory(q, lower, upper)
    if vis is not None:print("The visual hand model is Dex3; physical Inspire fingers use their separate worker.", flush=True)
    if args.viewer_only:
        try:
            time.sleep(args.seconds)
        except KeyboardInterrupt:
            pass
        return

    # Atomic snapshot: valid timestamp, sequence, grips, two matrices, then
    # received timestamp/count, rejected count, maximum received-event gap.
    snapshot = Array("d", 40)
    hand_input = None

    def store_event(value, sampled=None):
        nonlocal hand_input
        received = time.monotonic()
        with snapshot.get_lock():
            if snapshot[36] > 0:
                snapshot[39] = max(snapshot[39], received - snapshot[36])
            snapshot[36] = received
            snapshot[37] += 1
        try:
            poses, grips = decode_controller_packet(value)
        except (TypeError, ValueError):
            with snapshot.get_lock():
                snapshot[38] += 1
            return
        with snapshot.get_lock():
            snapshot[0] = received if sampled is None else sampled
            snapshot[1] += 1
            snapshot[2:4] = grips
            snapshot[4:20] = poses[0].flatten().tolist()
            snapshot[20:36] = poses[1].flatten().tolist()
        states=[value[s+'State'] for s in ('left','right')]
        hand_input=([float(s['triggerValue']) for s in states],
                    received if sampled is None else sampled) if all('triggerValue' in s for s in states) else None

    tv = None
    telemetry = None
    hand_link = None
    last_logged_state = None
    try:
        if arm_link:
            from pico_native_bridge import AsyncDiagnosticLog
            telemetry_path = project / 'work' / f'arm_telemetry_{time.time_ns()}.jsonl'
            telemetry = AsyncDiagnosticLog(telemetry_path)
            print(f'Arm telemetry: {telemetry_path}', flush=True)
        if args.hands=='inspire':
            from g1_inspire_teleop import InspireHandLink
            hand_link=InspireHandLink(live=args.arm_link=='live')
            print('Inspire: release BOTH index triggers once, then squeeze to curl; release to open. Side grips remain arm clutch.',flush=True)
        from pico_native_input import NativeReceiver

        def reset_input():
            nonlocal hand_input
            hand_input=None
            with snapshot.get_lock():
                snapshot[0] = 0

        tv = NativeReceiver(store_event, reset_input, args.native_port)
        print(f"Native preview listening on local port {args.native_port}; start Terminal A (pico_native.sh bridge).", flush=True)
        print("Orange points = estimated human joints; green/red = left/right ankle estimates beside G1.", flush=True)
        if leg_ik is None:print("Human estimate is display-offset, not registered to the G1 or used for leg commands.", flush=True)
        else:print('With grips RELEASED, lift/lower EACH foot once (>=3 cm) to verify the estimated body stream before engaging.',flush=True)
        if arm_link and responsive:
            if args.ready_pose == 'receive' and args.arm_link == 'live':
                print('Waiting for fresh PICO input with BOTH grips released before setup.', flush=True)
                deadline = time.monotonic()+60
                while True:
                    with snapshot.get_lock():
                        received, left_grip, right_grip = snapshot[0], snapshot[2], snapshot[3]
                    if received and time.monotonic()-received < .15 and not left_grip and not right_grip:
                        break
                    if time.monotonic() > deadline:
                        raise RuntimeError('No fresh released-grip PICO input; no setup motion was started')
                    state = arm_link.snapshot()
                    if arm_feedback_mode(state, arm_link.process.poll()) in ('fault','lost'):
                        raise RuntimeError('Robot feedback failed while waiting for PICO')
                    time.sleep(.02)
                q = prepare_receiving_pose(arm_link, True)
            use_measured_posture_reference(ik, q, args.joint_range, responsive=True)
        print("Release both grips, then hold BOTH to move.", flush=True)
        print("Release either grip to hold and reposition. Ctrl+C exits.", flush=True)
        enabled = False
        anchored = False
        released = False
        status_last = None
        start = time.monotonic()
        next_stats = start + 2
        stats_start, last_received, last_valid = start, 0, 0
        last_tick = start
        solve_times = []
        reference_rotation = np.eye(3)
        feedback_interrupted = False
        feedback_reset_required = False
        last_ik_ms = 0.
        raw_tcp_targets = None
        ik_position_error = None
        ik_orientation_error = None
        elbow_reference = None
        elbow_hint_active = False
        elbow_hint_targets = None
        elbow_hint_age = None
        leg_session=None
        leg_last_sequence=0
        leg_info=None
        leg_q=None
        body_age=float('inf')
        body_sequence=0
        while time.monotonic() - start < args.seconds:
            tick = time.monotonic()
            if hand_link is not None:
                current_hands=hand_input
                if current_hands is not None:hand_link.submit(*current_hands)
                hand_state=hand_link.snapshot()
                if hand_state and (hand_state['link_age_s']>1 or hand_link.process.poll() is not None):
                    raise RuntimeError('Inspire feedback lost; fingers retain last grasp')
            if arm_link:
                robot_state = arm_link.snapshot()
                feedback_mode = arm_feedback_mode(robot_state, arm_link.process.poll())
                if feedback_mode=='shutdown':
                    if telemetry:
                        telemetry.write(json.dumps(dict(robot_state,pc_monotonic_s=time.monotonic(),
                            pc_status='TERMINAL: robot shutdown'))+'\n')
                    print('Suspended robot entered bounded shutdown; closing B and waiting for damping. After B exits, use bash outputs/pico_native.sh restore for normal AI ownership.',flush=True)
                    break
                if feedback_mode in ("fault", "lost"):
                    if telemetry:
                        telemetry.write(json.dumps(dict(robot_state,
                            pc_monotonic_s=time.monotonic(), input_age_s=None,
                            pc_status='TERMINAL: '+feedback_mode, pc_command=q.tolist(),
                            ik_position_error_m=ik_position_error,
                            ik_orientation_error_rad=ik_orientation_error,
                            ik_duration_ms=last_ik_ms))+'\n')
                    raise RuntimeError(
                        f"Orin feedback invalid: link_age_ms={1000*robot_state['link_age_s']:.1f}, "
                        f"joint_state_age_ms={1000*robot_state['state_age_s']:.1f}, "
                        f"fault={robot_state['fault']}, reasons={robot_state.get('fault_reasons', [])}, "
                        f"ssh_exit={arm_link.process.poll()}, status={robot_state['status']}")
                if feedback_mode in ("gap", "mode_wait", "state_wait"):
                    enabled = False
                    if feedback_mode != 'state_wait':
                        anchored = False
                        released = False
                        feedback_reset_required = True
                    feedback_interrupted = True
                    status = ('ORIN JOINT FEEDBACK PAUSE: holding; awaiting fresh state' if feedback_mode == 'state_wait' else
                              "ORIN FEEDBACK GAP >200ms: frozen; waiting for feedback, then release BOTH grips"
                              if feedback_mode == "gap" else
                              "ORIN FSM QUERY TIMEOUT: frozen; waiting for confirmed mode, then release BOTH grips")
                    if status != status_last:
                        print(status, flush=True)
                        status_last = status
                    # ArmLink's writer stops at 150 ms feedback age. The Orin's
                    # independent target expiry still applies; no stale targets
                    # are submitted while this loop waits for fresh feedback.
                    last_tick = tick
                    time.sleep(0.01)
                    continue
                if feedback_interrupted:
                    q = np.array(robot_state["q"], dtype=float)
                    if trajectory:
                        trajectory.reset(q)
                    enabled = False
                    if feedback_reset_required:
                        anchored = False
                        released = False
                    ik.smooth_filter._data_queue.clear()
                    feedback_interrupted = False
                    print("ORIN FEEDBACK RESTORED: measured pose reloaded" +
                          ("; release BOTH grips to re-arm" if feedback_reset_required else "; engagement reference preserved"), flush=True)
                    feedback_reset_required = False
                if args.arm_link == "live" and vis is not None:
                    vis.display(np.array(robot_state["q"]))
            dt = min(max(tick - last_tick, 1 / 120), 0.05)
            last_tick = tick
            data = tv.get_tele_data()
            if vis is not None:
                tv.update_visualizer(vis.viewer)
            with snapshot.get_lock():
                sample = np.array(snapshot[:])
            age = max(0, tick - sample[0]) if sample[0] else float("inf")
            grips = [bool(v) for v in sample[2:4]]
            poses = [sample[4:20].reshape(4, 4), sample[20:36].reshape(4, 4)]
            mode, released = input_mode(age, grips, released, hold_timeout=.15 if arm_link else .25)
            body=None
            if leg_ik is not None:
                if leg_session!=tv.session:
                    if leg_session is not None:
                        raise RuntimeError('Body source session changed: restart B after restarting terminal A')
                    leg_session=tv.session;leg_gate.reset();leg_ik.calibrated=False
                    leg_last_sequence=0;anchored=False
                body_sample_time=time.monotonic()
                body,body_age,body_sequence=tv.get_body_leg_data()
                body_sample_time-=body_age  # Preserve original observation time, including across repeated solves.
                if body is not None and not any(grips):leg_gate.observe(body,int(body_sequence))
                if mode=='follow' and (body is None or not leg_gate.verified):
                    mode='body_wait';enabled=False
                    if body_age>1:anchored=False;released=False
                if mode in ('clutch','lost','release'):
                    leg_ik.calibrated=False;leg_last_sequence=0
                if mode=='follow' and not anchored:
                    try:
                        leg_ik.calibrate(body,np.array(robot_state['motor_q29'],dtype=float))
                        leg_last_sequence=0
                    except (ValueError,RuntimeError) as exc:
                        mode='leg_invalid';enabled=False;anchored=False;released=False
                        leg_error=str(exc)
            if mode == "lost":
                enabled = False
                anchored = False
                gated = (tv.need_release and
                         tick-getattr(tv,'last_packet_seen',0) < .2)
                status = ('PICO PACKETS ARRIVING: release BOTH grips before the first engagement' if gated else
                          "INPUT LOST >1s: frozen; release both grips to re-arm")
            elif mode == "gap":
                # Freeze while input is stale, preserving the fixed engagement
                # frame. Fresh absolute poses pass through the robot limiter;
                # packet loss must not silently recalibrate operator mapping.
                enabled = False
                status = f"INPUT GAP >{150 if arm_link else 250}ms: frozen; awaiting a complete event"
            elif mode == "clutch":
                enabled = False
                anchored = False
                status = "HOLD: release both, reposition, then hold both grips"
            elif mode == 'body_wait':
                status=('BODY WAIT: fresh full-body estimates required' if body is None else
                        'BODY CHECK: with grips released, lift/lower each foot once; observed excursion cm='+str(np.round(leg_gate.distance*100,1).tolist()))
            elif mode == 'leg_invalid':
                status='LEG CALIBRATION HOLD: '+leg_error+'; release both grips to re-arm'
            elif mode == "follow":
                if not anchored:
                    if responsive and arm_link:
                        # Re-engage from measured position, not an old target
                        # that the physical arm may not yet have reached.
                        q = np.array(robot_state['q'], dtype=float)
                    # Freeze headset yaw at engagement; subsequent head turns
                    # no longer move the controller reference frame.
                    head = data.head_pose[:3, :3]
                    yaw = np.arctan2(head[1, 0], head[0, 0])
                    c, s = np.cos(yaw), np.sin(yaw)
                    reference_rotation = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
                    controller = controller_reference(poses, reference_rotation)
                    anchor_controller = controller
                    anchor_robot = forward(ik, q, pin)
                    ik.smooth_filter._data_queue.clear()
                    engagement_moved = False
                    elbow_reference = None
                    anchored = True
                enabled = True
                controller = controller_reference(poses, reference_rotation)
                targets = relative_targets(controller, anchor_controller, anchor_robot, args.scale)
                if arm_link and args.orientation == "fixed":
                    for target, anchor in zip(targets, anchor_robot):
                        target[:3, :3] = anchor[:3, :3]
                solve_start = time.monotonic()
                if responsive and arm_link:
                    elbow_hint_active = False
                    if args.elbow_hint == 'body' and hasattr(tv,'get_body_arm_points'):
                        body_points,elbow_hint_age = tv.get_body_arm_points()
                        if body_points is not None:
                            try:
                                from g1_elbow_reference import ElbowReference
                                body_points=(reference_rotation@body_points.reshape(-1,3).T).T.reshape(2,3,3)
                                if elbow_reference is None:
                                    measured_q=np.array(robot_state['q'],dtype=float)
                                    forward(ik,measured_q,pin)
                                    shoulders=np.array([ik.reduced_robot.data.oMi[model.getJointId(f'{side}_shoulder_pitch_joint')].translation
                                                        for side in ('left','right')])
                                    elbows=np.array([ik.reduced_robot.data.oMi[model.getJointId(f'{side}_elbow_joint')].translation
                                                     for side in ('left','right')])
                                    elbow_reference=ElbowReference(body_points,shoulders,elbows)
                                elbow_hint_targets=elbow_reference.targets(body_points)
                                ik.opti.set_value(ik.elbow_target_parameter,elbow_hint_targets.ravel())
                                elbow_hint_active=True
                            except ValueError:
                                pass  # Invalid estimates cannot steer the arm.
                    ik.opti.set_value(ik.elbow_hint_enable,float(elbow_hint_active))
                if arm_link and not engagement_moved:
                    engagement_moved = engagement_has_moved(targets, anchor_robot, args.orientation)
                    if responsive and elbow_hint_active:
                        engagement_moved |= np.max(np.linalg.norm(
                            np.array(ik.elbow_function(q)).reshape(2,3)-elbow_hint_targets,axis=1))>.005
                if arm_link and not engagement_moved:
                    # Avoid IK null-space regularization moving the arms merely
                    # because the operator engaged an otherwise unmoved grip.
                    candidate = q.copy()
                else:
                    seed = q.copy()
                    if responsive and arm_link:
                        measured_q = np.array(robot_state['q'], dtype=float)
                        ik.opti.set_value(ik.measured_q_parameter, measured_q)
                        seed = np.clip(seed, measured_q-.25, measured_q+.25)
                        seed = np.clip(seed, ik.session_lower, ik.session_upper)
                    candidate, _ = ik.solve_ik(*targets, current_lr_arm_motor_q=seed)
                solve_times.append(time.monotonic() - solve_start)
                last_ik_ms = (time.monotonic() - solve_start) * 1000
                raw_tcp_targets = [target.tolist() for target in targets]
                if time.monotonic() - sample[0] > (.15 if arm_link else .25):
                    enabled = False
                    status = "INPUT GAP during solve: frozen; awaiting a complete event"
                elif (arm_link and args.joint_range and candidate is not None and np.isfinite(candidate).all()
                      and np.any(np.abs(candidate - arm_initial) > args.joint_range)):
                    enabled, released = False, False
                    anchored = False
                    status = (f"COMMISSIONING RANGE: candidate delta={np.max(np.abs(candidate - arm_initial)):.3f} rad "
                              f"exceeds {args.joint_range:.3f}; holding, release both grips and reposition")
                elif candidate is not None and np.isfinite(candidate).all():
                    if arm_link and responsive:
                        # One trajectory generator on the Orin. No PC-side
                        # low-pass or commissioning speed limit in series.
                        q = candidate.copy()
                    elif arm_link:
                        q = trajectory.step(candidate, dt)
                    else:
                        delta = (1 - np.exp(-dt / 0.08)) * (candidate - q)
                        q += np.clip(delta, -.6 * dt, .6 * dt)
                    q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)
                    if responsive and arm_link:
                        solved_poses = forward(ik, q, pin)
                        ik_position_error = [float(np.linalg.norm(actual[:3,3] - target[:3,3]))
                                             for actual, target in zip(solved_poses, targets)]
                        ik_orientation_error = [float(np.arccos(np.clip((np.trace(actual[:3,:3] @ target[:3,:3].T) - 1)/2,-1,1)))
                                                for actual, target in zip(solved_poses, targets)]
                    if args.arm_link != "live" and vis is not None:
                        vis.display(q)
                    limited = (arm_link and np.any(np.minimum(candidate - ik.session_lower,
                                                              ik.session_upper - candidate) < .005))
                    status = (f"FOLLOWING: scale={args.scale:g}" +
                              ("; workspace limited" if limited else "") + "; release either grip to hold")
                else:
                    enabled, released = False, False
                    anchored = False
                    status = "IK INVALID: frozen; release both grips to re-arm"
            else:
                anchored = False
                status = "RELEASE BOTH GRIPS before engaging"
            if arm_link:
                if leg_ik is not None and enabled and mode=='follow' and body_sequence!=leg_last_sequence:
                    try:
                        leg_q,leg_info=leg_ik.solve(body,np.array(robot_state['motor_q29'],dtype=float))
                    except (ValueError,RuntimeError) as exc:
                        leg_info=dict(solve_ok=False,failure=str(exc),solve_ms=None)
                    if leg_info['solve_ok']:
                        arm_link.submit_legs(leg_q.tolist(),body_sample_time,int(body_sequence),leg_gate.verified)
                        leg_last_sequence=body_sequence
                    else:
                        enabled=False;anchored=False;released=False;leg_ik.calibrated=False
                        status='LEG IK HOLD: '+str(leg_info['failure'])+'; release both grips to re-arm'
                if trajectory and (not enabled or mode != "follow"):
                    trajectory.reset(q)
                arm_link.submit(q, sample[0], enabled and mode == "follow",
                                source_sequence=int(sample[1]), ik_ms=last_ik_ms)
            if status != status_last:
                print(status, flush=True)
                status_last = status
            if telemetry and robot_state['state_sequence'] != last_logged_state:
                telemetry.write(json.dumps(dict(robot_state,
                    pc_monotonic_s=time.monotonic(), input_age_s=age if np.isfinite(age) else None,
                    input_sequence=int(sample[1]), grips=grips, pc_status=status,
                    pc_command=q.tolist(), raw_tcp_targets=raw_tcp_targets,
                    ik_position_error_m=ik_position_error, ik_orientation_error_rad=ik_orientation_error,
                    elbow_hint_active=elbow_hint_active, estimated_elbow_targets=elbow_hint_targets.tolist() if elbow_hint_targets is not None else None,
                    elbow_hint_age_s=elbow_hint_age if elbow_hint_age is not None and np.isfinite(elbow_hint_age) else None,
                    pc_body_age_s=body_age if np.isfinite(body_age) else None,pc_body_sequence=body_sequence,
                    pc_leg_motion_verified=leg_gate.verified if leg_gate else False,
                    pc_leg_command=leg_q.tolist() if leg_q is not None else None,pc_leg_ik=leg_info,
                    ik_duration_ms=last_ik_ms)) + '\n')
                last_logged_state = robot_state['state_sequence']
            if tick >= next_stats:
                elapsed = tick - stats_start
                ik_ms = 1000 * np.percentile(solve_times, 95) if solve_times else 0
                print(f"TIMING receive_hz={(sample[37]-last_received)/elapsed:.1f} "
                      f"valid_hz={(sample[1]-last_valid)/elapsed:.1f} "
                      f"age_ms={age*1000:.0f} max_gap_ms={sample[39]*1000:.0f} "
                      f"rejected={int(sample[38])} ik_p95_ms={ik_ms:.1f}", flush=True)
                if arm_link:
                    measured_delta = np.max(np.abs(np.array(robot_state['q']) - arm_initial))
                    tracking_error = np.max(np.abs(np.array(robot_state['q_command']) - np.array(robot_state['q'])))
                    print(f"ORIN live={robot_state['live']} weight={robot_state['weight']:.2f} "
                          f"accepted={robot_state['accepted']} rejected={robot_state['rejected']} "
                          f"measured_delta_rad={measured_delta:.3f} tracking_error_rad={tracking_error:.3f} "
                          f"status={robot_state['status']}", flush=True)
                    print("ORIN_JOINTS " + json.dumps({
                        "ids": list(range(15, 29)),
                        "initial_rad": np.round(arm_initial, 5).tolist(),
                        "requested_rad": np.round(robot_state['q_command'], 5).tolist(),
                        "measured_rad": np.round(robot_state['q'], 5).tolist(),
                        "mode_query_code": robot_state.get('mode_query_code'),
                        "mode_last_success_age_s": robot_state.get('mode_last_success_age_s'),
                    }), flush=True)
                    if leg_ik is not None:
                        print('LEGS '+json.dumps(dict(body_sequence=body_sequence,body_age_ms=round(body_age*1000) if np.isfinite(body_age) else None,
                              motion_verified=leg_gate.verified,ankle_excursion_cm=np.round(leg_gate.distance*100,1).tolist(),
                              solve_ms=leg_info['solve_ms'] if leg_info else None,
                              solve_ok=leg_info['solve_ok'] if leg_info else None)),flush=True)
                stats_start, last_received, last_valid = tick, sample[37], sample[1]
                solve_times.clear()
                next_stats = tick + 2
            time.sleep(max(0.0, 1 / (60 if responsive else 30) - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print("Preview stopped.", flush=True)
    finally:
        if hand_link is not None:
            hand_link.close()
        if arm_link is not None:
            arm_link.close()
        if telemetry is not None:
            telemetry.__exit__(None)
        if tv is not None:
            tv.close()


if __name__ == "__main__":
    main()
