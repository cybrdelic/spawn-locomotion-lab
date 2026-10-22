"""One observed state: body, terrain, measured effector loads and contact intent."""
import numpy as np
from terrain.body import SceneCoordinates
EFFECTORS=['right_foot','left_foot','right_hand','left_hand']
GRID=np.array([[x,y] for x in [-.3,.3,.9,1.5,2.1] for y in [-.7,-.35,0,.35,.7]])
def heading(rig):
    forward=rig.data.xmat[1].reshape(3,3)[:2,0].copy();length=float(np.linalg.norm(forward))
    return forward/length if length>1e-8 else np.array([1.,0.])

class Coordinates(SceneCoordinates):
    observation_size=216
    def extra(self):
        r=self.rig;d=r.data;forward=heading(r)
        H=np.array([[forward[0],-forward[1],0.],[forward[1],forward[0],0.],[0.,0.,1.]])
        xy=d.qpos[:2]+GRID@H[:2,:2].T;terrain=r.world.height_many(xy)-d.qpos[2]
        ids=[int(r.ids[r.names.index(n)]) for n in EFFECTORS];forces=r.support_forces[ids]@H/r.mass_force
        targets=np.asarray(getattr(r,'contact_goals',np.zeros((4,7)))).copy()
        enabled=targets[:,6:7]
        targets[:,:3]=((targets[:,:3]-d.qpos[:3])@H)*.5*enabled;targets[:,3:6]=(targets[:,3:6]@H)*enabled
        return np.r_[terrain,forces.ravel(),targets.ravel()].astype(np.float32)
    def observe(self,goal):
        body=super().observe(goal);body[0]-=self.rig.reference_height()
        return np.r_[body,self.extra()].astype(np.float32)
class DeferredCoordinates(Coordinates):
    deferred=False
    def observe(self,goal):return None if self.deferred else super().observe(goal)
