"""Estimated-body motion check. No network, SDK or motor access."""
import numpy as np


class LegMotionGate:
    """Require deliberate movement of both estimated ankles within a session.

    This checks changes in the estimated skeleton. It cannot establish the
    independent capture time or validity of either physical ankle tracker.
    """
    def __init__(self,threshold=.03):
        self.threshold=threshold;self.reset()

    def reset(self):
        self.anchor=None;self.last_sequence=0;self.distance=np.zeros(2)

    @property
    def verified(self):
        return bool(np.all(self.distance>=self.threshold))

    def observe(self,body,sequence):
        if type(sequence) is not int or sequence<2:raise ValueError('Two advancing body bundles required')
        if sequence<self.last_sequence:
            self.reset()
        if sequence==self.last_sequence:return self.verified
        body=np.asarray(body,dtype=float)
        if body.shape!=(24,4,4) or not np.isfinite(body).all():raise ValueError('Invalid estimated body')
        pelvis=body[0];relative=(pelvis[:3,:3].T@(body[[7,8],:3,3]-pelvis[:3,3]).T).T
        if self.anchor is None:self.anchor=relative.copy()
        self.distance=np.maximum(self.distance,np.linalg.norm(relative-self.anchor,axis=1))
        self.last_sequence=sequence
        return self.verified
