import json
from pathlib import Path
import sys
import unittest
import numpy as np
from _support import FIXTURE_DIR, MODELS_ENABLED, URDF, fixture

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'outputs'))
from g1_arm_link import tracking_joint_step, operating_joint_limits, ARM_LOWER, ARM_UPPER
from g1_arm_workspace import ArmWorkspace


class TrackingWorkspaceTests(unittest.TestCase):
    def test_operating_endpoints_respect_observed_roll_stops_and_urdf(self):
        lo,hi=operating_joint_limits()
        self.assertEqual(hi[1],2.10)
        self.assertEqual(lo[8],-2.10)
        self.assertEqual(lo[3],-1.)
        self.assertEqual(lo[10],-1.)
        self.assertEqual(hi[3],1.5)
        self.assertTrue(np.all(lo>ARM_LOWER))
        self.assertTrue(np.all(hi<ARM_UPPER))
        self.assertLess(lo[0],-3.)

    def test_stalled_joint_does_not_wind_up_other_joints_continue(self):
        q=np.zeros(14);v=q.copy();measured=q.copy();lo=np.full(14,-3.);hi=-lo
        for _ in range(500):
            q,v,limited=tracking_joint_step(q,v,np.full(14,.8),measured,.004,lo,hi)
            self.assertLessEqual(abs(q[0]),.10000001)
            measured[1:]+=.2*(q[1:]-measured[1:])
        self.assertTrue(limited[0])
        self.assertGreater(q[1],.79)
        self.assertLess(abs(v[0]),.001)
        for _ in range(500):
            q,v,_=tracking_joint_step(q,v,np.full(14,-.3),measured,.004,lo,hi)
            measured+=.2*(q-measured)
        np.testing.assert_allclose(q,-.3,atol=.001)

    def test_fast_follow_uses_one_generator_without_overshoot(self):
        q=np.zeros(14);v=q.copy();measured=q.copy();lo=np.full(14,-3.);hi=-lo
        peak_speed=0
        for _ in range(400):
            before=v.copy()
            q,v,_=tracking_joint_step(q,v,np.full(14,.8),measured,.004,lo,hi)
            measured+=.3*(q-measured)
            peak_speed=max(peak_speed,float(max(v)))
            self.assertLessEqual(max(q),.80000001)
            self.assertLessEqual(max(abs(v-before)),.0400001)
        self.assertGreater(peak_speed,1.5)
        np.testing.assert_allclose(q,.8,atol=.001)

    @unittest.skipUnless(FIXTURE_DIR is not None and MODELS_ENABLED,
                         'Optional recorded model test: use model mode and G1_TEST_FIXTURES')
    def test_recorded_stall_is_excluded_and_interpolation_blocks_that_arm(self):
        path=fixture('arm_telemetry_1792738170648760689.jsonl')
        with path.open() as f: rows=[json.loads(x) for x in f]
        guard=ArmWorkspace(URDF)
        initial=np.array(rows[0]['motor_q29']);bad=np.array(rows[-1]['motor_q29'])
        self.assertTrue(np.all(guard.clearance(initial)>0))
        self.assertLess(guard.clearance(bad)[0],0)
        q0=initial[15:29].copy();candidate=bad[15:29].copy()
        q,v,blocked,_=guard.interpolate(q0,np.zeros(14),candidate,np.ones(14),initial)
        self.assertTrue(blocked[0])
        np.testing.assert_array_equal(q[:7],q0[:7])
        np.testing.assert_array_equal(v[:7],np.zeros(7))
        self.assertFalse(blocked[1])


if __name__=='__main__': unittest.main()
