"""One physical muscle implementation shared by training and native runtime."""
from pathlib import Path
import json
import numpy as np,mujoco
from muscle.control import control
class Drive:
 def __init__(self,body,coordinates):
  self.body=body;self.coordinates=coordinates
  spec=json.loads((Path(__file__).parent/'contract.json').read_text())
  self.source_kp=np.array(spec['kp']);self.source_kd=np.array(spec['kd']);self.source_caps=np.array(spec['caps'])
  groups=self.coordinates.groups
  self.ball=[g for g in groups if len(g[1])==3];self.hinges=[g for g in groups if len(g[1])==1]
  self.ball_q=np.array([g[2] for g in self.ball]);self.ball_v=np.array([g[3] for g in self.ball]);self.ball_slots=np.array([g[4]+np.arange(3) for g in self.ball])
  # Source effort limits live in the orthogonal child-frame torque components.
  # A bounded hinge coordinate is a different axis: copying the same scalar cap
  # after A.T clips an already limited physical wrench a second time.
  for body,ids,q,v,slot in self.ball:
   self.body.caps[v-6]=np.linalg.norm(self.source_caps[slot:slot+3])
  self.body.model.actuator_forcerange[:]=np.column_stack([-self.body.caps,self.body.caps])
  self.hq=np.array([g[2][0] for g in self.hinges]);self.hv=np.array([g[3][0] for g in self.hinges]);self.hs=np.array([g[4] for g in self.hinges])
  assert np.array_equal(self.body.vadr,np.arange(6,34))

 def set_control(self,target):
     r=self.body;d=r.data
     control(d.qpos,d.qvel,d.ctrl,target,self.ball_q,self.ball_v,self.ball_slots,self.hq,self.hv,self.hs,self.source_kp,self.source_kd,self.source_caps,r.qadr,r.vadr,r.kp,r.kd,r.caps)
 def apply(self,target):
     r=self.body;d=r.data
     for _ in range(r.substeps):
         self.set_control(target)
         assert not d.xfrc_applied.any() and not d.qfrc_applied.any()
         mujoco.mj_step(r.model,d)
     mujoco.mj_forward(r.model,d);mujoco.mj_subtreeVel(r.model,d);r.contacts()
