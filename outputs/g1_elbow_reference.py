"""Relative upper-arm guidance from estimated PICO body joints.

Calibrate at each deliberate engagement, preserving the robot's starting pose.
This is a soft redundancy preference, never a direct joint-angle command.
"""
import numpy as np


def align_directions(source, target):
    a=np.asarray(source,dtype=float);b=np.asarray(target,dtype=float)
    if a.shape!=(3,) or b.shape!=(3,) or not np.isfinite([a,b]).all():
        raise ValueError('Invalid arm direction')
    if min(np.linalg.norm(a),np.linalg.norm(b))<1e-6:
        raise ValueError('Zero arm direction')
    a=a/np.linalg.norm(a);b=b/np.linalg.norm(b)
    c=float(np.clip(a@b,-1,1));v=np.cross(a,b)
    if c>1-1e-10:return np.eye(3)
    if c<-1+1e-10:
        axis=np.cross(a,np.eye(3)[np.argmin(abs(a))]);axis/=np.linalg.norm(axis)
        return 2*np.outer(axis,axis)-np.eye(3)
    x,y,z=v;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+k+k@k/(1+c)


class ElbowReference:
    def __init__(self, human, robot_shoulders, robot_elbows):
        directions=self.directions(human)
        self.shoulders=np.asarray(robot_shoulders,dtype=float).copy()
        elbows=np.asarray(robot_elbows,dtype=float)
        if self.shoulders.shape!=(2,3) or elbows.shape!=(2,3) or not np.isfinite([self.shoulders,elbows]).all():
            raise ValueError('Invalid robot upper-arm anchors')
        vectors=elbows-self.shoulders;self.lengths=np.linalg.norm(vectors,axis=1)
        if np.any(self.lengths<.05) or np.any(self.lengths>.4):raise ValueError('Invalid robot upper-arm length')
        self.rotations=[align_directions(a,b) for a,b in zip(directions,vectors)]

    @staticmethod
    def directions(human):
        body=np.asarray(human,dtype=float)
        if body.shape!=(2,3,3) or not np.isfinite(body).all():raise ValueError('Invalid human arm points')
        vectors=body[:,1]-body[:,0];lengths=np.linalg.norm(vectors,axis=1)
        if np.any(lengths<.1) or np.any(lengths>.6):raise ValueError('Invalid human upper-arm estimate')
        return vectors/lengths[:,None]

    def targets(self, human):
        directions=self.directions(human)
        return np.array([s+length*(rotation@direction) for s,length,rotation,direction in
                         zip(self.shoulders,self.lengths,self.rotations,directions)])
