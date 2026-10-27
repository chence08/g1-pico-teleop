"""Fast torso clearance approximation shared by IK and robot interpolation.

Torso box comes from the stock G1 torso mesh in torso_link coordinates. Spheres
sample arm centerlines. This does not model Inspire fingers, harness or scenery.
"""
import numpy as np
from pathlib import Path

TORSO_MIN = np.array([-.066643, -.107715, -.008909])
TORSO_MAX = np.array([.083628, .107715, .311620])
TORSO_CENTER = (TORSO_MIN + TORSO_MAX) / 2
TORSO_HALF = (TORSO_MAX - TORSO_MIN) / 2
SAMPLE_RADII = np.array([.045]*5 + [.040] + [.043, .056, .050])
LINK_NAMES = ('shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
LINK_OFFSETS = ([0,0,0], [0,0,0], [.028,0,0], [.023,0,0], [0,0,0])


def centerline_points(shoulder, elbow, wrist_roll, wrist_pitch, wrist_yaw):
    return [shoulder + fraction*(elbow-shoulder) for fraction in (0,.25,.5,.75,1)] + [
        (elbow+wrist_roll)/2, wrist_roll, wrist_pitch, wrist_yaw]


def add_ik_clearance(ik):
    import casadi as ca
    torso = ik.cdata.oMf[ik.cmodel.getFrameId('torso_link')]
    expressions = []
    for side in ('left', 'right'):
        placements = [ik.cdata.oMi[ik.cmodel.getJointId(f'{side}_{name}_joint')] for name in LINK_NAMES]
        points = [p.translation + p.rotation @ ca.DM(offset) for p,offset in zip(placements,LINK_OFFSETS)]
        for point, radius in zip(centerline_points(*points), SAMPLE_RADII):
            local = torso.rotation.T @ (point - torso.translation)
            delta = ca.fmax(ca.fabs(local - ca.DM(TORSO_CENTER)) - ca.DM(TORSO_HALF), 0)
            # Leave interpolation/tracking headroom instead of solving exactly
            # on the robot-side geometric boundary, where paths can get stuck.
            expressions.append(ca.sumsqr(delta) - (radius+.010)**2)
    function = ca.Function('arm_torso_clearance_sq', [ik.cq], [ca.vertcat(*expressions)])
    ik.opti.subject_to(function(ik.var_q) >= 0)
    ik.workspace_clearance_function = function


class ArmWorkspace:
    def __init__(self, urdf_path):
        import pinocchio as pin
        try:
            import hppfcl
        except ImportError:
            import coal as hppfcl
        from g1_gravity_comp import MOTOR_NAMES_29
        self.pin = pin
        self.fcl = hppfcl
        self.transform_type = getattr(hppfcl, 'Transform3f', None) or hppfcl.Transform3s
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.indices = [self.model.joints[self.model.getJointId(n)].idx_q for n in MOTOR_NAMES_29]
        self.torso_id = self.model.getFrameId('torso_link')
        self.arm_ids = [[self.model.getJointId(f'{side}_{n}_joint')
                         for n in LINK_NAMES]
                        for side in ('left', 'right')]
        meshes = Path(urdf_path).resolve().parent / 'meshes'
        loader = hppfcl.MeshLoader()
        self.torso_mesh = loader.load(str(meshes / 'torso_link_rev_1_0.STL'))
        self.arm_meshes = [[loader.load(str(meshes / f'{side}_{name}_link.STL')) for name in LINK_NAMES]
                           for side in ('left','right')]

    def clearance(self, motor_q29):
        value = np.asarray(motor_q29, dtype=float)
        if value.shape != (29,) or not np.isfinite(value).all():
            raise ValueError('workspace requires finite motor q29')
        q = self.pin.neutral(self.model)
        q[self.indices] = value
        self.pin.framesForwardKinematics(self.model, self.data, q)
        torso = self.data.oMf[self.torso_id]
        results = []
        for ids, meshes in zip(self.arm_ids, self.arm_meshes):
            placements = [self.data.oMi[j] for j in ids]
            points = [p.translation + p.rotation @ np.array(offset) for p,offset in zip(placements,LINK_OFFSETS)]
            distance = []
            for point, radius in zip(centerline_points(*points), SAMPLE_RADII):
                local = torso.rotation.T @ (point - torso.translation)
                delta = np.maximum(np.abs(local - TORSO_CENTER) - TORSO_HALF, 0)
                distance.append(np.linalg.norm(delta) - radius)
            # Exact nominal torso/arm meshes backstop the analytic samples.
            torso_tf = self.transform_type(torso.rotation, torso.translation)
            for placement, mesh in zip(placements, meshes):
                transform = self.transform_type(placement.rotation, placement.translation)
                result = self.fcl.CollisionResult()
                request = self.fcl.CollisionRequest()
                request.security_margin = .002
                if self.fcl.collide(self.torso_mesh, torso_tf, mesh, transform, request, result):
                    distance.append(-.002)
            results.append(min(distance))
        return np.asarray(results)

    def interpolate(self, command, velocity, proposed, proposed_velocity, measured_q29):
        """Block only an arm whose next command enters the torso approximation.

        Improving clearance is allowed if starting inside the approximation.
        No extra time-domain filter is added.
        """
        before = np.asarray(measured_q29, dtype=float).copy()
        after = before.copy()
        before[15:29] = command
        after[15:29] = proposed
        old_clearance, new_clearance = self.clearance(before), self.clearance(after)
        blocked = (new_clearance < 0) & ((old_clearance >= 0) | (new_clearance <= old_clearance))
        proposed, proposed_velocity = proposed.copy(), proposed_velocity.copy()
        for arm in np.flatnonzero(blocked):
            sl = slice(7*arm, 7*arm+7)
            proposed[sl] = command[sl]
            proposed_velocity[sl] = 0
        return proposed, proposed_velocity, blocked.tolist(), new_clearance.tolist()
