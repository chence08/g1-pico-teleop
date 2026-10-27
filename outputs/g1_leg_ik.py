"""Fixed-pelvis suspended G1 leg IK. No balance/contact control or communication."""
import time
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
import casadi as ca
from g1_gravity_comp import MOTOR_NAMES_29
from g1_elbow_reference import align_directions

LEG_NAMES=MOTOR_NAMES_29[:12]
BODY_HIPS,BODY_KNEES,BODY_ANKLES=(1,2),(4,5),(7,8)
ENGAGEMENT_RANGE=np.tile([.6,.35,.35,.6,.25,.15],2)
CALIBRATION_TRACKING_TOLERANCE=.05


def finite_vector(value,length,name):
    out=np.asarray(value,dtype=float)
    if out.shape!=(length,) or not np.isfinite(out).all():raise ValueError(f'{name} must be finite length {length}')
    return out.copy()


def body_transforms(value):
    value=np.asarray(value,dtype=float)
    if not np.isfinite(value).all():raise ValueError('Nonfinite body poses')
    if value.shape==(24,7):
        result=np.repeat(np.eye(4)[None],24,axis=0);result[:,:3,3]=value[:,:3]
        for i,q in enumerate(value[:,3:]):
            norm=np.linalg.norm(q)
            if abs(norm-1)>.03:raise ValueError('Body quaternion not normalized')
            result[i,:3,:3]=pin.Quaternion(q/norm).matrix()
    elif value.shape==(24,4,4):result=value.copy()
    else:raise ValueError('Expected body shape (24,7) or (24,4,4)')
    for i in (0,*BODY_HIPS,*BODY_KNEES,*BODY_ANKLES):
        t=result[i];r=t[:3,:3]
        if (not np.allclose(t[3],[0,0,0,1],atol=1e-6) or
                not np.allclose(r.T@r,np.eye(3),atol=.01) or np.linalg.det(r)<.99):
            raise ValueError('Invalid body transform')
    return result


def pelvis_relative_segments(body):
    t=body_transforms(body);pelvis=t[0];r=pelvis[:3,:3].T
    points=(r@(t[:,:3,3]-pelvis[:3,3]).T).T
    upper=points[list(BODY_KNEES)]-points[list(BODY_HIPS)]
    lower=points[list(BODY_ANKLES)]-points[list(BODY_KNEES)]
    lu=np.linalg.norm(upper,axis=1);ll=np.linalg.norm(lower,axis=1)
    if np.any(np.r_[lu,ll]<.08) or np.any(np.r_[lu,ll]>.9):raise ValueError('Invalid estimated human leg lengths')
    rotations=np.array([r@t[i,:3,:3] for i in BODY_ANKLES])
    return upper/lu[:,None],lower/ll[:,None],rotations


class SuspendedLegIK:
    def __init__(self,q29,urdf):
        q29=finite_vector(q29,29,'q29')
        full=pin.buildModelFromUrdf(str(urdf));full_q=pin.neutral(full)
        for name,value in zip(MOTOR_NAMES_29,q29):
            if not full.existJointName(name):raise ValueError(f'Missing {name}')
            joint=full.joints[full.getJointId(name)]
            if joint.nq!=1:raise ValueError('Expected scalar G1 joints')
            full_q[joint.idx_q]=value
        lock=[i for i in range(1,full.njoints) if full.names[i] not in LEG_NAMES]
        self.model=pin.buildReducedModel(full,lock,full_q);self.data=self.model.createData()
        self.indices=np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in LEG_NAMES])
        if self.model.nq!=12:raise ValueError('Expected twelve leg coordinates')
        self.hip_ids=[self.model.getJointId(f'{s}_hip_pitch_joint') for s in ('left','right')]
        self.knee_ids=[self.model.getJointId(f'{s}_knee_joint') for s in ('left','right')]
        self.ankle_ids=[self.model.getJointId(f'{s}_ankle_pitch_joint') for s in ('left','right')]
        self.foot_ids=[self.model.getJointId(f'{s}_ankle_roll_joint') for s in ('left','right')]
        self.urdf_lower=self.model.lowerPositionLimit[self.indices].copy()
        self.urdf_upper=self.model.upperPositionLimit[self.indices].copy()
        self.session_lower=np.maximum(self.urdf_lower,q29[:12]-ENGAGEMENT_RANGE)
        self.session_upper=np.minimum(self.urdf_upper,q29[:12]+ENGAGEMENT_RANGE)
        self.cmodel=cpin.Model(self.model);self.cdata=self.cmodel.createData()
        cq=ca.SX.sym('leg_q',12);model_q=ca.SX.zeros(12)
        for i,j in enumerate(self.indices):model_q[int(j)]=cq[i]
        cpin.forwardKinematics(self.cmodel,self.cdata,model_q)
        positions=ca.vertcat(*[self.cdata.oMi[i].translation for i in self.knee_ids+self.ankle_ids])
        rotations=[self.cdata.oMi[i].rotation for i in self.foot_ids]
        self.position_function=ca.Function('leg_positions',[cq],[positions])
        self.rotation_functions=[ca.Function(f'leg_rotation_{i}',[cq],[r]) for i,r in enumerate(rotations)]
        desired_rotations=[ca.SX.sym(f'desired_leg_rotation_{i}',3,3) for i in range(2)]
        # Smooth chordal SO(3) error avoids log3's derivative singularity at an
        # exactly unchanged opposite foot. Numeric log3 below is diagnostic only.
        rotation_error=ca.vertcat(*[ca.reshape(r-target,9,1)/np.sqrt(2.) for r,target in zip(rotations,desired_rotations)])
        self.rotation_error_function=ca.Function('leg_rotation_error',[cq,*desired_rotations],[rotation_error])
        self.opti=ca.Opti();self.var_q=self.opti.variable(12)
        self.measured_parameter=self.opti.parameter(12);self.last_parameter=self.opti.parameter(12)
        self.reference_parameter=self.opti.parameter(12)
        self.lower_parameter=self.opti.parameter(12);self.upper_parameter=self.opti.parameter(12)
        self.position_parameter=self.opti.parameter(12)
        self.rotation_parameters=[self.opti.parameter(3,3) for _ in range(2)]
        error=self.position_function(self.var_q)-self.position_parameter
        re=self.rotation_error_function(self.var_q,*self.rotation_parameters)
        self.opti.minimize(250*ca.sumsqr(error[:6])+1000*ca.sumsqr(error[6:])+5*ca.sumsqr(re)
                           +.05*ca.sumsqr(self.var_q-self.reference_parameter)+.1*ca.sumsqr(self.var_q-self.last_parameter))
        self.opti.subject_to(self.opti.bounded(self.lower_parameter,self.var_q,self.upper_parameter))
        self.opti.subject_to(self.opti.bounded(-.25,self.var_q-self.measured_parameter,.25))
        # Keep nominal knees/ankles apart during this suspended commissioning stage.
        pos=self.position_function(self.var_q)
        self.opti.subject_to(pos[1]>=.06);self.opti.subject_to(pos[4]<=-.06)
        self.opti.subject_to(pos[7]>=.06);self.opti.subject_to(pos[10]<=-.06)
        self.opti.solver('ipopt',{'expand':True,'print_time':False,'ipopt.sb':'yes','ipopt.print_level':0,
                                'ipopt.max_iter':35,'ipopt.tol':1e-5,'ipopt.acceptable_tol':1e-4})
        self.calibrated=False

    def geometry(self,legs):
        model_q=np.zeros(12);model_q[self.indices]=finite_vector(legs,12,'legs')
        pin.forwardKinematics(self.model,self.data,model_q)
        positions=[np.array([self.data.oMi[i].translation.copy() for i in ids]) for ids in (self.hip_ids,self.knee_ids,self.ankle_ids)]
        rotations=np.array([self.data.oMi[i].rotation.copy() for i in self.foot_ids])
        return (*positions,rotations)

    def calibrate(self,body,q29):
        self.calibrated=False
        measured=finite_vector(q29,29,'q29')[:12]
        if np.any(measured<self.urdf_lower) or np.any(measured>self.urdf_upper):raise ValueError('Measured legs outside URDF limits')
        upper,lower,human_rotations=pelvis_relative_segments(body)
        hips,knees,ankles,rotations=self.geometry(measured);ru,rl=knees-hips,ankles-knees
        if (np.any(knees[:,1]*[1,-1]<.06-1e-6) or
                np.any(ankles[:,1]*[1,-1]<.06-1e-6)):
            raise ValueError('Measured knee/ankle inside suspended medial plane')
        self.hips=hips;self.length_upper=np.linalg.norm(ru,axis=1);self.length_lower=np.linalg.norm(rl,axis=1)
        self.upper_alignment=[align_directions(a,b) for a,b in zip(upper,ru)]
        self.lower_alignment=[align_directions(a,b) for a,b in zip(lower,rl)]
        self.human_rotation_anchor=human_rotations;self.robot_rotation_anchor=rotations
        self.reference=measured.copy();self.last=measured.copy()
        # PD tracking can settle slightly beyond a command bound. Re-engage
        # from that measured geometry without moving the immutable command
        # envelope; the solver projects the request back inside it.
        outside=np.maximum(self.session_lower-measured,measured-self.session_upper)
        if np.any(outside>CALIBRATION_TRACKING_TOLERANCE):
            raise ValueError('Measured legs outside session envelope')
        anchor=np.clip(measured,self.session_lower,self.session_upper)
        self.lower=np.maximum(self.session_lower,anchor-ENGAGEMENT_RANGE)
        self.upper=np.minimum(self.session_upper,anchor+ENGAGEMENT_RANGE)
        for parameter,value in ((self.reference_parameter,measured),(self.lower_parameter,self.lower),(self.upper_parameter,self.upper)):
            self.opti.set_value(parameter,value)
        self.calibrated=True

    def targets(self,body):
        if not self.calibrated:raise RuntimeError('Calibrate leg IK before solving')
        upper,lower,human_rotations=pelvis_relative_segments(body)
        knees=self.hips+np.array([length*(r@direction) for length,r,direction in zip(self.length_upper,self.upper_alignment,upper)])
        ankles=knees+np.array([length*(r@direction) for length,r,direction in zip(self.length_lower,self.lower_alignment,lower)])
        rotations=np.array([now@anchor.T@robot for now,anchor,robot in zip(human_rotations,self.human_rotation_anchor,self.robot_rotation_anchor)])
        return knees,ankles,rotations

    def solve(self,body,q29):
        measured=finite_vector(q29,29,'q29')[:12];knees,ankles,rotations=self.targets(body)
        self.opti.set_value(self.position_parameter,np.r_[knees.ravel(),ankles.ravel()])
        self.opti.set_value(self.measured_parameter,measured);self.opti.set_value(self.last_parameter,self.last)
        for parameter,value in zip(self.rotation_parameters,rotations):self.opti.set_value(parameter,value)
        seed=np.clip(self.last,np.maximum(self.lower,measured-.25),np.minimum(self.upper,measured+.25))
        self.opti.set_initial(self.var_q,seed);started=time.perf_counter();success=True;failure=None
        try:
            if np.any(np.maximum(self.lower,measured-.25)>np.minimum(self.upper,measured+.25)):
                raise ValueError('Measured feedback outside feasible session/trust intersection')
            result=finite_vector(self.opti.solve().value(self.var_q),12,'IK solution');result=np.clip(result,self.lower,self.upper)
        except (RuntimeError,ValueError) as exc:
            result=measured.copy();success=False;failure=str(exc).splitlines()[-1][:200]
        solve_ms=(time.perf_counter()-started)*1000;self.last=result.copy()
        _,ak,aa,ar=self.geometry(result)
        info=dict(solve_ok=success,solve_ms=solve_ms,failure=failure,knee_targets_m=knees.tolist(),
                  ankle_targets_m=ankles.tolist(),ankle_rotation_targets=rotations.tolist(),
                  knee_error_m=np.linalg.norm(ak-knees,axis=1).tolist(),ankle_error_m=np.linalg.norm(aa-ankles,axis=1).tolist(),
                  ankle_orientation_error_rad=[float(np.linalg.norm(pin.log3(a@b.T))) for a,b in zip(ar,rotations)],
                  engagement_lower=self.lower.tolist(),engagement_upper=self.upper.tolist(),
                  at_envelope=bool(np.any(np.minimum(result-self.lower,self.upper-result)<.003)))
        return result,info
