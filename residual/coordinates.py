"""Add twice actual whole-body kinetic energy per unit mass to native observation."""
import numpy as np,mujoco
from foundation.coordinates import Coordinates as SourceCoordinates
SIZE=151
class Coordinates(SourceCoordinates):
    observation_size=151
    def __init__(self,rig):super().__init__(rig);self.momentum=np.empty(rig.model.nv)
    def kinetic_speed_squared(self):
        r=self.rig;mujoco.mj_mulM(r.model,r.data,self.momentum,r.data.qvel)
        return max(0.,float(r.data.qvel@self.momentum/r.model.body_subtreemass[1]))
    def observe(self,goal):return np.r_[super().observe(goal),self.kinetic_speed_squared()].astype(np.float32)
