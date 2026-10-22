"""Actual bounded rig and a compact, contact-aware physical observation.

One control input: normalized bounded native joint targets. Positions are written
only in reset (spawn). Runtime motion always comes from native joint muscles.
"""
import numpy as np
import mujoco
from motor.body import Body
from contact_policy.kinematics import standing_pose


OBSERVATION_SIZE=94

class Rig(Body):
    hz=30

    def __init__(self,xml_source=None):
        super().__init__(False,xml_source=xml_source)
        self.substeps=8;self.contract['controlHz']=self.hz
        self.original=self.data.qpos.copy()
        self.joints=self.model.actuator_trnid[:,0]
        self.qadr=self.model.jnt_qposadr[self.joints]
        self.vadr=self.model.jnt_dofadr[self.joints]
        self.mid=self.model.jnt_range[self.joints].mean(axis=1)
        self.half=np.diff(self.model.jnt_range[self.joints],axis=1).ravel()*.5
        self.feet=[int(self.ids[self.names.index(n)]) for n in ['right_foot','left_foot']]
        self.mass_force=float(self.model.body_subtreemass[1])*9.81
        self.ground_forces=np.zeros((2,3));self.loads=np.zeros(2);self.other_load=0.;self.slip=0.
        self.foot_geoms=[int(np.flatnonzero(self.model.geom_bodyid==f)[0]) for f in self.feet]
        self.floor=float(self.model.geom_pos[np.flatnonzero((self.model.geom_bodyid==0)&(self.model.geom_type==mujoco.mjtGeom.mjGEOM_PLANE))[0],2])
        self.reset()

    def reset(self,height=.85,xy=(0.,0.),yaw=0.,velocity=(0.,0.)):
        m,d=self.model,self.data
        mujoco.mj_resetData(m,d);d.qpos[:]=self.original
        mujoco.mj_forward(m,d)
        d.qpos[:]=standing_pose(self,height)
        d.qpos[:2]=xy;d.qpos[3:7]=[np.cos(yaw/2),0,0,np.sin(yaw/2)]
        d.qvel[:]=0.;d.qvel[:2]=velocity
        mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d)
        d.ctrl[:]=d.actuator_length+self.kd*d.actuator_velocity/self.kp
        mujoco.mj_forward(m,d)
        self.contacts()

    def contacts(self):
        m,d=self.model,self.data;self.loads[:]=0.;self.ground_forces[:]=0.;self.other_load=0.;self.slip=0.
        force=np.empty(6);jac=np.empty((3,m.nv))
        for i,c in enumerate(d.contact[:d.ncon]):
            a,b=[int(m.geom_bodyid[g]) for g in [c.geom1,c.geom2]]
            if a and b:continue
            body=a or b
            mujoco.mj_contactForce(m,d,i,force)
            world=c.frame.reshape(3,3).T@force[:3]*(1 if a==0 else -1)
            normal=max(0.,world[2])
            if body in self.feet:
                self.loads[self.feet.index(body)]+=normal
                self.ground_forces[self.feet.index(body)]+=world
                if normal>.1*self.mass_force:
                    mujoco.mj_jac(m,d,jac,None,c.pos,body)
                    self.slip+=normal/self.mass_force*float(np.linalg.norm((jac@d.qvel)[:2]))
            else:self.other_load+=normal

    def act(self,action):
        action=np.clip(action,-1.,1.)
        self.apply_pose(self.mid+self.half*action)
        self.contacts()

    def observe(self,goal):
        # goal = [world vx,world vy,yaw rate,root height,world face x,face y]
        m,d=self.model,self.data;rotation=d.xmat[1].reshape(3,3)
        motion=np.empty(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,1,motion,0)
        velocity=d.subtree_linvel[1]@rotation;angular=motion[:3]@rotation
        foot=(d.xpos[self.feet]-d.qpos[:3])@rotation
        weights=self.loads/max(1.,self.loads.sum())
        support=np.sum(d.xpos[self.feet,:2]*weights[:,None],axis=0) if self.loads.sum()>1 else d.xpos[self.feet,:2].mean(axis=0)
        capture=d.subtree_com[1,:2]+d.subtree_linvel[1,:2]/np.sqrt(9.81/max(.2,d.subtree_com[1,2]))
        capture_local=np.r_[capture-support,0.]@rotation
        command=np.r_[goal[:2],0.]@rotation;face=np.r_[goal[4:6],0.]@rotation
        clearance=np.array([d.geom_xpos[g,2]-abs(d.geom_xmat[g].reshape(3,3)[2])@m.geom_size[g]-self.floor for g in self.foot_geoms])
        normals=np.array([np.array([0.,0.,1.])@d.xmat[f].reshape(3,3) for f in self.feet])
        observation=np.r_[np.array([0.,0.,-1.])@rotation,velocity/4.,angular/5.,
            (d.qpos[self.qadr]-self.mid)/self.half,d.qvel[self.vadr]/10.,
            foot.ravel(),(self.ground_forces@rotation).ravel()/self.mass_force,capture_local[:2],
            (d.qpos[2]-self.floor-.8)/.2,clearance/.2,normals.ravel(),command[:2]/4.,
            goal[2]/2.,(goal[3]-.8)/.2,face[:2]].astype(np.float32)
        assert observation.shape==(OBSERVATION_SIZE,) and np.isfinite(observation).all()
        return np.clip(observation,-10,10)
