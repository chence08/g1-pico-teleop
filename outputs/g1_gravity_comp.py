"""Locally calibrated arm gravity. No robot communication.

Fixed-base, upright-pelvis approximation. Unmeasured hand joints stay neutral.
The measured holding torque corrects one pose, not unknown Inspire inertia.
"""
from pathlib import Path
import numpy as np
import pinocchio as pin

MOTOR_NAMES_29 = (
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
    'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint')


def _finite_vector(value, length, name):
    raw = np.asarray(value)
    if raw.shape != (length,) or raw.dtype.kind not in 'iuf':
        raise ValueError(f'{name} must be a real vector of shape ({length},)')
    out = raw.astype(float, copy=True)
    if not np.isfinite(out).all():
        raise ValueError(f'{name} must contain finite values')
    return out


class CalibratedGravity:
    """One servo-thread instance; evaluate(q29) returns bounded 14-arm torques."""
    def __init__(self, urdf_path, reference_motor_q29, measured_hold_tau14):
        self.urdf_path = str(Path(urdf_path).resolve())
        self.reference_motor_q29 = _finite_vector(reference_motor_q29, 29, 'reference_motor_q29')
        self.baseline = _finite_vector(measured_hold_tau14, 14, 'measured_hold_tau14')
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        qidx, vidx = [], []
        for name in MOTOR_NAMES_29:
            if not self.model.existJointName(name):
                raise ValueError(f'URDF missing G1 motor joint {name}')
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(f'G1 joint {name} needs one q and v coordinate')
            qidx.append(joint.idx_q)
            vidx.append(joint.idx_v)
        if len(set(qidx)) != 29 or len(set(vidx)) != 29:
            raise ValueError('G1 motor mapping is not one-to-one')
        self.motor_q_indices = np.array(qidx, dtype=int)
        self.motor_v_indices = np.array(vidx, dtype=int)
        self.arm_v_indices = self.motor_v_indices[15:29]
        effort = np.asarray(self.model.effortLimit)[self.arm_v_indices]
        if not np.isfinite(effort).all() or np.any(effort <= 0):
            raise ValueError('Arm effort limits must be finite and positive')
        self.limits = np.minimum(8., .2 * effort)
        self.reference_model_gravity = self._model_gravity(self.reference_motor_q29)
        self.bias = self.baseline - self.reference_model_gravity
        self.calibration_clipped = np.abs(self.baseline) > self.limits

    def model_configuration(self, motor_q29):
        q = pin.neutral(self.model)
        q[self.motor_q_indices] = _finite_vector(motor_q29, 29, 'motor_q29')
        return q

    def _model_gravity(self, motor_q29):
        q = self.model_configuration(motor_q29)
        gravity = np.asarray(pin.computeGeneralizedGravity(self.model, self.data, q))[self.arm_v_indices].copy()
        if gravity.shape != (14,) or not np.isfinite(gravity).all():
            raise ValueError('Invalid model arm gravity')
        return gravity

    def evaluate(self, current_motor_q29, return_info=False):
        nominal = self._model_gravity(current_motor_q29)
        unclipped = self.baseline + (nominal - self.reference_model_gravity)
        clipped = np.clip(unclipped, -self.limits, self.limits)
        if not np.isfinite(clipped).all():
            raise ValueError('Invalid calibrated gravity')
        if return_info:
            return clipped.copy(), dict(nominal_model=nominal.copy(), unclipped=unclipped.copy(),
                                        saturated=np.abs(unclipped) > self.limits)
        return clipped.copy()
