from pathlib import Path
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from g1_elbow_reference import ElbowReference,align_directions


class ElbowReferenceTests(unittest.TestCase):
    def setUp(self):
        self.human=np.array([[[0,.2,1.4],[0,.2,1.1],[.2,.2,1.1]],
                             [[0,-.2,1.4],[0,-.2,1.1],[.2,-.2,1.1]]])
        self.shoulders=np.array([[0,.1,.3],[0,-.1,.3]])
        self.elbows=np.array([[.03,.2,.12],[.03,-.2,.12]])

    def test_engagement_preserves_robot_elbow_without_matching_body_proportions(self):
        ref=ElbowReference(self.human,self.shoulders,self.elbows)
        np.testing.assert_allclose(ref.targets(self.human),self.elbows,atol=1e-12)
        np.testing.assert_allclose(ref.targets(self.human+np.array([3,2,1])),self.elbows,atol=1e-12)

    def test_estimated_upper_arm_motion_changes_hint_but_not_robot_arm_length(self):
        ref=ElbowReference(self.human,self.shoulders,self.elbows)
        moved=self.human.copy();moved[0,1,1]+=.1
        target=ref.targets(moved)
        self.assertGreater(np.linalg.norm(target[0]-self.elbows[0]),.02)
        np.testing.assert_allclose(np.linalg.norm(target-self.shoulders,axis=1),ref.lengths)
        np.testing.assert_allclose(target[1],self.elbows[1],atol=1e-12)

    def test_parallel_antiparallel_and_invalid_directions(self):
        for a,b in [([0,0,1],[0,0,-1]),([0,1,0],[0,1,0]),([1,0,0],[0,1,0])]:
            r=align_directions(a,b);np.testing.assert_allclose(r@a,b,atol=1e-12)
            np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(r),1.)
        with self.assertRaises(ValueError):align_directions([0,0,0],[1,0,0])
        bad=self.human.copy();bad[0,1]=bad[0,0]
        with self.assertRaises(ValueError):ElbowReference(bad,self.shoulders,self.elbows)


if __name__=='__main__': unittest.main()
