"""The same articulated body, with general surface contacts and own-body energy."""
import numpy as np,mujoco
from contact_policy.core import Rig
from residual.coordinates import Coordinates
from terrain.world import World
class SceneRig(Rig):
    def __init__(self,course=True):
        self.world=World(course);super().__init__(xml_source=self.world.xml);self.world.bind(self)
    def reset(self,*args,**kwargs):
        self.world.events.clear();self.support_height=0.;self.contact_goals=np.zeros((4,7));super().reset(*args,**kwargs)
    def reference_height(self):
        if self.loads.sum()>.1*self.mass_force:return self.support_height
        return float(np.min(self.world.height_many(self.data.geom_xpos[self.foot_geoms,:2])))
    def contacts(self):
        m,d=self.model,self.data
        self.loads[:]=0.;self.ground_forces[:]=0.;self.other_load=0.;self.slip=0.
        hands=[int(self.ids[self.names.index(n)]) for n in ['right_hand','left_hand']]
        support_z=0.;support_weight=0.
        self.hand_loads=np.zeros(2);self.projectile_force=np.zeros(3);self.unsafe_load=0.;self.support_forces=np.zeros((m.nbody,3));self.effector_points=np.zeros((4,3));self.effector_normals=np.zeros((4,3));self.effector_weights=np.zeros(4);self.effector_geoms=[set() for _ in range(4)];self.surface_up_loads=[{} for _ in range(4)]
        source=set(self.ids);force=np.empty(6);jac=np.empty((3,m.nv))
        for i,c in enumerate(d.contact[:d.ncon]):
            a,b=[int(m.geom_bodyid[g]) for g in [c.geom1,c.geom2]]
            if (a in source)==(b in source):continue
            body=a if a in source else b;mujoco.mj_contactForce(m,d,i,force)
            world=c.frame.reshape(3,3).T@force[:3]*(1 if b in source else -1);normal=max(0.,world[2]);self.support_forces[body]+=world
            if (b if body==a else a)!=0:self.projectile_force+=world
            if body in self.feet+hands:
                effector=(self.feet+hands).index(body);outward=c.frame[:3]*(1 if body==b else -1);weight=max(0.,float(world@outward));point=c.pos+(.5*c.dist*c.frame[:3])*(1 if body==a else -1)
                self.effector_points[effector]+=weight*point;self.effector_normals[effector]+=weight*outward;self.effector_weights[effector]+=weight;self.effector_geoms[effector].add(int(c.geom2 if body==a else c.geom1))
                if outward[2]>.6:
                    other=int(c.geom2 if body==a else c.geom1)
                    self.surface_up_loads[effector][other]=self.surface_up_loads[effector].get(other,0.)+normal
            if body in self.feet:
                index=self.feet.index(body);self.loads[index]+=normal;self.ground_forces[index]+=world
                other_point=c.pos+(.5*c.dist*c.frame[:3])*(1 if body==a else -1)
                support_z+=normal*other_point[2];support_weight+=normal
                if normal>.1*self.mass_force:
                    mujoco.mj_jac(m,d,jac,None,c.pos,body);self.slip+=normal/self.mass_force*float(np.linalg.norm((jac@d.qvel)[:2]))
            elif body in hands:self.hand_loads[hands.index(body)]+=normal
            else:self.unsafe_load+=normal
        for i,w in enumerate(self.effector_weights):
            if w>1e-8:self.effector_points[i]/=w;self.effector_normals[i]/=max(1e-8,np.linalg.norm(self.effector_normals[i]))
        self.other_load=float(self.hand_loads.sum()+self.unsafe_load)
        if support_weight>.1*self.mass_force:
            height=support_z/support_weight;self.support_height=0. if abs(height)<1e-8 else float(height)
        elif not hasattr(self,'support_height'):self.support_height=0.
class SceneCoordinates(Coordinates):
    def kinetic_speed_squared(self):
        r=self.rig;velocity=r.data.qvel.copy();velocity[34:]=0.
        mujoco.mj_mulM(r.model,r.data,self.momentum,velocity)
        return max(0.,float(velocity@self.momentum/r.model.body_subtreemass[1]))
    def observe(self,goal):
        obs=super().observe(goal);r=self.rig;m,d=r.model,r.data
        for i,g in enumerate(r.foot_geoms):
            pos=d.geom_xpos[g];bottom=pos[2]-abs(d.geom_xmat[g].reshape(3,3)[2])@m.geom_size[g]
            obs[147+i]=(bottom-r.world.height(pos[0],pos[1],max(4.,float(pos[2]+.2))))/.2
        return obs
