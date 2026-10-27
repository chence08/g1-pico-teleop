from pathlib import Path
import sys,unittest
import numpy as np
import pinocchio as pin
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from g1_gravity_comp import CalibratedGravity,MOTOR_NAMES_29
from _support import URDF

class GravityTests(unittest.TestCase):
 def setUp(self):
  self.q0=np.zeros(29)
  self.q0[15:29]=[.292,.202,-.039,.98,.093,.048,-.05,.296,-.206,.059,.978,-.087,.03,.033]
  self.baseline=np.array([.8,2.5,.3,-.7,0.,-.2,.1,.8,-2.5,-.3,-.7,0.,-.2,-.1])
 def make(self,q=None,baseline=None):
  return CalibratedGravity(URDF,self.q0 if q is None else q,self.baseline if baseline is None else baseline)
 def test_reference_exactly_recovers_measured_holding_torque(self):
  g=self.make();np.testing.assert_array_equal(g.evaluate(self.q0),self.baseline)
  np.testing.assert_allclose(g.bias+g.reference_model_gravity,self.baseline,atol=1e-15)
  self.assertFalse(g.calibration_clipped.any())
 def test_named_mapping_handles_hand_coordinates_and_current_waist(self):
  g=self.make();self.assertGreater(g.model.nq,29);self.assertNotEqual(g.motor_q_indices[22],22)
  q=self.q0.copy();q[12:15]=[.2,-.15,.3];q[22:29]+=[.1,-.1,.2,-.15,.05,.1,-.05]
  mapped=g.model_configuration(q)
  for i,name in enumerate(MOTOR_NAMES_29):
   j=g.model.joints[g.model.getJointId(name)];self.assertEqual(mapped[j.idx_q],q[i])
  rest=np.ones(g.model.nq,dtype=bool);rest[g.motor_q_indices]=False
  np.testing.assert_array_equal(mapped[rest],pin.neutral(g.model)[rest])
 def test_arm_and_waist_motion_change_finite_gravity_delta(self):
  g=self.make();q=self.q0.copy();q[15]-=.35;q[18]+=.4
  torque,info=g.evaluate(q,return_info=True);self.assertTrue(np.isfinite(torque).all())
  self.assertGreater(np.max(np.abs(info['unclipped']-self.baseline)),.05)
  np.testing.assert_allclose(info['unclipped'],self.baseline+info['nominal_model']-g.reference_model_gravity,atol=1e-14)
  q=self.q0.copy();q[13:15]=[.2,.3]
  self.assertGreater(np.max(np.abs(g.evaluate(q)-self.baseline)),.05)
 def test_effort_fraction_and_absolute_caps(self):
  g=self.make(baseline=np.full(14,100.))
  np.testing.assert_array_equal(g.limits,np.minimum(8.,.2*np.asarray(g.model.effortLimit)[g.arm_v_indices]))
  torque,info=g.evaluate(self.q0,return_info=True)
  np.testing.assert_array_equal(torque,g.limits);np.testing.assert_array_equal(info['unclipped'],np.full(14,100.))
  self.assertTrue(info['saturated'].all());self.assertTrue(g.calibration_clipped.all())
 def test_invalid_shapes_and_nonfinite_values(self):
  for q in (np.zeros(28),np.full(29,np.nan),np.ones((29,1)),np.ones(29,dtype=bool)):
   with self.assertRaises(ValueError):self.make(q=q)
  for tau in (np.zeros(13),np.full(14,np.inf)):
   with self.assertRaises(ValueError):self.make(baseline=tau)
  g=self.make()
  for q in (np.zeros(30),np.full(29,np.inf)):
   with self.assertRaises(ValueError):g.evaluate(q)
