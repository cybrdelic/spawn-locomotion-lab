"""Native observations preserving the complete pretrained locomotion contract.

No reference skeleton is simulated. These features come from the one actual rig.
Five additional inputs describe measured support and requested body height.
"""
from pathlib import Path
import sys,json,os
import numpy as np
import mujoco,torch
from scipy.spatial.transform import Rotation
mimickit_root=os.environ.get('MIMICKIT_ROOT')
if not mimickit_root:raise RuntimeError('Set MIMICKIT_ROOT to a licensed MimicKit checkout')
sys.path.insert(0,str(Path(mimickit_root)/'mimickit'))
from envs.char_env import compute_char_obs
from envs.task_steering_env import compute_steering_observations
from motor.rotation import matrix_jacobian
from scipy.optimize import least_squares
ROOT=Path(__file__).resolve().parents[1]
SIZE=150

class Coordinates:
    def __init__(self,rig):
        self.rig=rig;m=rig.model
        source=json.loads((ROOT/'policy/native-humanoid-contract.json').read_text(encoding='utf-8'))
        self.groups=[];self.keys=[rig.names.index(n) for n in ['head','right_hand','left_hand','right_foot','left_foot']]
        joint_bodies=[i for i in rig.ids[1:] if m.body_jntnum[i]>0]
        assert len(joint_bodies)==len(source['joints'])
        for body,source_joint in zip(joint_bodies,source['joints']):
            ids=np.flatnonzero(m.jnt_bodyid==body)
            assert len(ids)==source_joint['n']
            self.groups.append((int(body),ids,m.jnt_qposadr[ids],m.jnt_dofadr[ids],source_joint['v']-6))
        self.action_mean=None;self.action_std=None

    def observe(self,goal):
        r=self.rig;m,d=r.model,r.data
        rotations=np.zeros((len(r.names)-1,4),dtype=np.float32);rotations[:,3]=1
        velocity=np.zeros(28,dtype=np.float32)
        for body,ids,q,v,slot in self.groups:
            angles=d.qpos[q]
            if len(ids)==3:
                rotations[r.names.index(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,body))-1]=Rotation.from_euler('XZY',angles).as_quat()
                a,b,c=angles;cb,sb,cc,sc=np.cos(b),np.sin(b),np.cos(c),np.sin(c)
                A=np.array([[cb*cc,-sc,0],[-sb,0,1],[cb*sc,cc,0]])
                velocity[slot:slot+3]=A@d.qvel[v]
            else:
                rotations[r.names.index(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,body))-1]=Rotation.from_rotvec(m.jnt_axis[ids[0]]*angles[0]).as_quat()
                velocity[slot]=d.qvel[v[0]]
        motion=np.empty(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,1,motion,0)
        t=lambda x:torch.tensor(np.asarray(x),dtype=torch.float32).unsqueeze(0)
        root_quat=np.roll(d.qpos[3:7],-1)
        char=compute_char_obs(t(d.qpos[:3]),t(root_quat),t(motion[3:]),t(motion[:3]),t(rotations),t(velocity),t(d.xpos[r.ids][self.keys]),False,True)
        speed=float(np.linalg.norm(goal[:2]));direction=goal[:2]/speed if speed>1e-7 else goal[4:6]
        task=compute_steering_observations(t(root_quat),t(direction),torch.tensor([speed],dtype=torch.float32),t(goal[4:6]))
        clearances=[d.geom_xpos[g,2]-abs(d.geom_xmat[g].reshape(3,3)[2])@m.geom_size[g]-r.floor for g in r.foot_geoms]
        extra=torch.tensor(np.r_[r.loads/r.mass_force,np.asarray(clearances)/.2,(goal[3]-.88)/.2],dtype=torch.float32).unsqueeze(0)
        obs=torch.cat([char,task,extra],-1)
        assert obs.shape==(1,SIZE) and torch.isfinite(obs).all()
        return obs[0].numpy()

    def project(self,desired):
        r=self.rig;m,d=r.model,r.data;q=d.qpos.copy()
        for body,ids,positions,velocities,slot in self.groups:
            ranges=m.jnt_range[ids]
            if len(ids)==3:
                rotation=Rotation.from_rotvec(desired[slot:slot+3]);raw=rotation.as_euler('XZY');value=np.clip(raw,ranges[:,0],ranges[:,1])
                if np.max(abs(raw-value))>1e-8:
                    matrix=rotation.as_matrix();current=d.qpos[positions]
                    def error(angles):return np.r_[(Rotation.from_euler('XZY',angles).as_matrix()-matrix).ravel(),.02*(angles-current)]
                    def jac(angles):return np.r_[matrix_jacobian(angles),.02*np.eye(3)]
                    value=least_squares(error,value,jac=jac,bounds=(ranges[:,0],ranges[:,1]),max_nfev=12,ftol=1e-6,xtol=1e-6,gtol=1e-6).x
                q[positions]=value
            else:q[positions]=np.clip(desired[slot],ranges[0,0],ranges[0,1])
        return q[r.qadr]

    def encode_target(self,native_target):
        r=self.rig;q=r.data.qpos.copy();q[r.qadr]=native_target;result=np.zeros(28)
        for body,ids,positions,velocities,slot in self.groups:
            result[slot:slot+len(ids)]=Rotation.from_euler('XZY',q[positions]).as_rotvec() if len(ids)==3 else q[positions]
        return result
