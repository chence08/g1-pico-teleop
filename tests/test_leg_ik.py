import sys,unittest
from pathlib import Path
import numpy as np
import pinocchio as pin
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'outputs'))
from g1_leg_ik import SuspendedLegIK,body_transforms
from _support import URDF

class LegIKTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.q=np.zeros(29);cls.q[:12]=[-.1,.03,0,.25,-.1,0,-.1,-.03,0,.25,-.1,0]
        cls.ik=SuspendedLegIK(cls.q,URDF)
    def body(self,q):
        hips,knees,ankles,rotations=self.ik.geometry(q[:12]);t=np.repeat(np.eye(4)[None],24,axis=0)
        for s,(h,k,a) in enumerate(zip((1,2),(4,5),(7,8))):
            t[h,:3,3]=hips[s];t[k,:3,3]=hips[s]+1.4*(knees[s]-hips[s])
            t[a,:3,3]=t[k,:3,3]+1.4*(ankles[s]-knees[s]);t[a,:3,:3]=rotations[s]
        return t
    def test_initial_pose_preserved_and_body_scale_ignored(self):
        b=self.body(self.q);self.ik.calibrate(b,self.q);q,info=self.ik.solve(b,self.q)
        self.assertTrue(info['solve_ok'],info);np.testing.assert_allclose(q,self.q[:12],atol=2e-4)
        self.assertLess(max(info['ankle_error_m']),1e-5)
    def test_pelvis_rigid_motion_does_not_move_legs(self):
        b=self.body(self.q);self.ik.calibrate(b,self.q)
        rigid=np.eye(4);rigid[:3,:3]=pin.exp3(np.array([.15,-.1,.4]));rigid[:3,3]=[2,-3,1]
        q,info=self.ik.solve(rigid[None]@b,self.q)
        self.assertTrue(info['solve_ok'],info);np.testing.assert_allclose(q,self.q[:12],atol=2e-4)
    def test_knee_flexion_recovered_with_real_model_endpoints(self):
        self.ik.calibrate(self.body(self.q),self.q);target=self.q.copy()
        target[[0,6]]-=.06;target[[3,9]]+=.15;target[[4,10]]-=.09
        q,info=self.ik.solve(self.body(target),self.q)
        self.assertTrue(info['solve_ok'],info);self.assertTrue(np.all(q[[3,9]]>self.q[[3,9]]+.08))
        self.assertLess(max(info['ankle_error_m']),.004);self.assertLess(max(info['knee_error_m']),.004)
        np.testing.assert_allclose(q,target[:12],atol=.025)
    def test_limits_and_measured_local_trust(self):
        self.ik.calibrate(self.body(self.q),self.q);target=self.q.copy()
        target[[0,6]]-=.5;target[[3,9]]+=1.2;target[[4,10]]-=.3
        q,info=self.ik.solve(self.body(target),self.q)
        self.assertTrue(info['solve_ok'],info);self.assertLessEqual(max(abs(q-self.q[:12])),.250001)
        self.assertTrue(np.all(q>=self.ik.lower-1e-8));self.assertTrue(np.all(q<=self.ik.upper+1e-8))
    def test_invalid_body_rejected(self):
        with self.assertRaises(ValueError):body_transforms(np.zeros((24,7)))
        b=self.body(self.q);b[4,:3,3]=b[1,:3,3]
        with self.assertRaises(ValueError):self.ik.calibrate(b,self.q)

    def test_recorded_asymmetric_pose_left_flexion_and_recovery(self):
        baseline=self.q.copy()
        baseline[:12]=[-.3109866083,-.0126468195,.020502992,.4447862506,-.1538804919,.005068065,
                       -.2552361488,.0246118698,-.0106197735,.4518510699,-.1766255498,-.0734446794]
        ik=SuspendedLegIK(baseline,URDF);ik.calibrate(self.body(baseline),baseline);measured=baseline.copy();peak=0
        for step in range(80):
            phase=np.sin(np.pi*step/79)**2;target=baseline.copy()
            target[0]-=.12*phase;target[3]+=.3*phase;target[4]-=.18*phase
            q,info=ik.solve(self.body(target),measured)
            self.assertTrue(info['solve_ok'],info);np.testing.assert_allclose(q[6:],baseline[6:12],atol=1e-5)
            self.assertLess(max(info['ankle_error_m']),.002);peak=max(peak,q[3]-baseline[3]);measured[:12]=q
        self.assertGreater(peak,.29);np.testing.assert_allclose(q,baseline[:12],atol=2e-4)

    def test_reclutch_cannot_ratchet_session_bounds(self):
        ik=SuspendedLegIK(self.q,URDF);moved=self.q.copy();moved[0]+=.5
        ik.calibrate(self.body(moved),moved)
        self.assertLessEqual(ik.upper[0],self.q[0]+.6+1e-9)
        outside=moved.copy();outside[0]+=.2
        with self.assertRaises(ValueError):ik.calibrate(self.body(outside),outside)
        self.assertFalse(ik.calibrated)

    def test_medial_pose_rejected_and_clears_calibration(self):
        ik=SuspendedLegIK(self.q,URDF);ik.calibrate(self.body(self.q),self.q)
        bad=self.q.copy();bad[1]=-.27
        with self.assertRaises(ValueError):ik.calibrate(self.body(bad),bad)
        self.assertFalse(ik.calibrated)

    def test_reclutch_recovers_small_tracking_offset_without_expanding_limits(self):
        ik=SuspendedLegIK(self.q,URDF)
        lower=ik.session_lower.copy();upper=ik.session_upper.copy()
        measured=self.q.copy();measured[10]=lower[10]-.025
        ik.calibrate(self.body(measured),measured)
        q,info=ik.solve(self.body(measured),measured)
        self.assertTrue(info['solve_ok'],info)
        self.assertTrue(np.all(q>=lower-1e-9));self.assertTrue(np.all(q<=upper+1e-9))
        np.testing.assert_array_equal(ik.session_lower,lower)
        np.testing.assert_array_equal(ik.session_upper,upper)
        for _ in range(20):
            measured[:12]=q;q,info=ik.solve(self.body(self.q),measured)
            self.assertTrue(info['solve_ok'],info)
        np.testing.assert_allclose(q,self.q[:12],atol=.005)
        self.assertLess(max(info['ankle_error_m']),.002)
        measured[10]=lower[10]-.075
        with self.assertRaises(ValueError):ik.calibrate(self.body(measured),measured)
        self.assertFalse(ik.calibrated)

if __name__=='__main__':unittest.main()
