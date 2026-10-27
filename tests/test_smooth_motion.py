from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'outputs'))
from g1_arm_link import bounded_joint_step
from g1_arm_preview import JointTrajectory,input_mode,engagement_has_moved


class SmoothMotionTests(unittest.TestCase):
    def test_velocity_and_acceleration_limits_on_step_and_reversal(self):
        q=np.zeros(14);v=q.copy();lo=np.full(14,-.5);hi=-lo
        for i in range(600):
            goal=np.full(14,.4 if i<200 else -.4)
            before=v.copy();oldq=q.copy()
            q,v=bounded_joint_step(q,v,goal,.01,lo,hi)
            self.assertLessEqual(np.max(np.abs(v)),.100001)
            self.assertLessEqual(np.max(np.abs(v-before)),.003001)
            self.assertLessEqual(np.max(np.abs(q-oldq)),.001001)
        self.assertTrue(np.all(q < -.1))

    def test_bounds_always_win_and_clutch_reset_has_no_residual_motion(self):
        motion=JointTrajectory(np.zeros(14),np.full(14,-.5),np.full(14,.5))
        for _ in range(1000):
            q=motion.step(np.full(14,100),.01)
            self.assertTrue(np.all(q<=.5))
        held=q.copy();motion.reset(q)
        for _ in range(20):
            np.testing.assert_array_equal(motion.step(held,.01),held)

    def test_short_input_gap_keeps_release_authorization_but_long_loss_disarms(self):
        self.assertEqual(input_mode(.16,[True,True],True,hold_timeout=.15),('gap',True))
        self.assertEqual(input_mode(.02,[True,True],True,hold_timeout=.15),('follow',True))
        self.assertEqual(input_mode(1.01,[True,True],True,hold_timeout=.15),('lost',False))

    def test_responsive_braking_has_no_step_overshoot_and_bounds_acceleration(self):
        for target in (.05,.1,.3,.5):
            q=np.zeros(14);v=q.copy();lo=np.full(14,-1.5);hi=-lo
            for _ in range(400):
                before=v.copy()
                q,v=bounded_joint_step(q,v,np.full(14,target),.004,lo,hi,
                                      speed=1.,acceleration=6.,time_constant=.015,braking=True)
                self.assertLessEqual(np.max(np.abs(v)),1.000001)
                self.assertLessEqual(np.max(np.abs(v-before)),.024001)
                self.assertLessEqual(np.max(q),target+1e-8)
            np.testing.assert_allclose(q,target,atol=1e-4)

    def test_stationary_wrist_rotation_starts_controller_orientation_ik(self):
        anchors=[np.eye(4),np.eye(4)]
        targets=[a.copy() for a in anchors]
        angle=.2;c,s=np.cos(angle),np.sin(angle)
        targets[0][:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]]
        self.assertTrue(engagement_has_moved(targets,anchors,'controller'))
        self.assertFalse(engagement_has_moved(targets,anchors,'fixed'))
        self.assertFalse(engagement_has_moved(anchors,anchors,'controller'))
