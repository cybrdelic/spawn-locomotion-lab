"""One runtime movement owner; no legacy-controller inheritance or clip fallback."""
from pathlib import Path
import os, hashlib
import numpy as np
import torch
from natural.actor import Actor
from natural.coordinates import Coordinates, GRID
from natural.goals import Intent
from terrain.body import SceneRig
from muscle.drive import Drive

ROOT = Path(__file__).resolve().parents[1]
FILES = ['unified_runtime/controller.py','natural/actor.py','natural/coordinates.py',
         'natural/goals.py','terrain/body.py','terrain/world.py','terrain/course.json',
         'muscle/drive.py','muscle/control.py','muscle/contract.json',
         'contact_policy/core.py','motor/body.py','support/bounded-humanoid.xml']
HASH = hashlib.sha256(b''.join((ROOT/p).read_bytes() for p in FILES)).hexdigest()[:12]


class Monitor:
    def __init__(self):
        self.reason = None
        self.values = {}
        self.unsafe_seconds = 0.

    def update(self, rig):
        m, d = rig.model, rig.data
        q = d.qpos[rig.qadr]
        ranges = m.jnt_range[rig.joints]
        limit = float(np.max(np.maximum(ranges[:,0]-q,q-ranges[:,1])))
        effort = float(np.max(abs(d.actuator_force)/rig.caps))
        self.unsafe_seconds = self.unsafe_seconds+1/30 if rig.unsafe_load>.1*rig.mass_force else 0.
        reason = None
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all(): reason='Nonfinite physical state'
        elif d.xfrc_applied.any() or d.qfrc_applied.any() or np.max(abs(d.qfrc_actuator[:6]))>1e-8: reason='Forbidden external/root actuation'
        elif effort>1.00001: reason='Finite muscle cap exceeded'
        elif limit>.1: reason='Joint-stop overshoot'
        self.reason = reason
        self.values = dict(rootUp=float(d.xmat[1,8]),maxEffortFraction=effort,
            jointStopOvershoot=max(0.,limit),unsafeSupportSeconds=self.unsafe_seconds,
            handLoads=rig.hand_loads.tolist(),
            fallen=bool(self.unsafe_seconds>.065 or (d.qpos[2]<rig.reference_height()+.35 and d.xmat[1,8]<.7)),
            recoveryAllowed=True)
        return reason


class Controller:
    def __init__(self, speed=0., circle=True, radius=6., checkpoint=None):
        path = checkpoint or os.environ.get('FOUNDATION_CHECKPOINT')
        if not path: raise ValueError('An explicit reviewed216-input checkpoint is required')
        self.path = Path(path)
        torch.set_num_threads(1)
        self.actor = Actor(initialize_base=False)
        self.actor.load_state_dict(torch.load(self.path,map_location='cpu',weights_only=True))
        self.actor.eval()
        self.allow_circle = os.environ.get('LOCOMOTION_COURSE','1')=='0'
        self.circle, self.radius = bool(circle and self.allow_circle), radius
        self.circle_center = np.zeros(2)
        self.speed, self.height, self.standing_height = float(speed), .88, .88
        self.body = SceneRig(course=not self.allow_circle)
        self.model, self.data = self.body.model, self.body.data
        self.names, self.contract = self.body.names, self.body.contract
        self.body.reset(height=.88,xy=(radius,0.) if self.circle else (0.,0.),yaw=np.pi/2 if self.circle else 0.)
        self.coordinates = Coordinates(self.body)
        self.drive = Drive(self.body,self.coordinates)
        self.intent = Intent(speed,self.height,self.circle,radius)
        self.monitor = Monitor()
        self.last_angle = float(np.arctan2(self.data.qpos[1],self.data.qpos[0]))
        self.turns = 0.
        self.checkpoint = ('Unified contact feedback / '+hashlib.sha256(self.path.read_bytes()).hexdigest()[:12]
                           +' / runtime:'+HASH+' / unified:'+HASH)

    def reset(self): self.__init__(circle=self.circle,radius=self.radius,checkpoint=str(self.path))
    def close(self): pass
    def set_contacts(self, contacts): self.intent.set_contacts(self.body,contacts)
    def request_jump(self): self.intent.request_jump(self.body)
    def request_traversal(self):
        self.intent.request_traversal(self.body)
        self.speed = self.intent.speed
    def disturb(self, side, speed=3.): return self.body.world.launch(side,speed)
    def goal(self):
        self.intent.speed, self.intent.height = self.speed, self.height
        self.intent.circle, self.intent.center = self.circle, self.circle_center
        return self.intent.observe(self.body)

    @torch.no_grad()
    def step(self):
        observation = self.coordinates.observe(self.goal())
        target = self.actor.target(torch.from_numpy(observation)).numpy()
        self.drive.apply(target)
        self.monitor.update(self.body)
        angle = float(np.arctan2(self.data.qpos[1],self.data.qpos[0]))
        self.turns += float(np.arctan2(np.sin(angle-self.last_angle),np.cos(angle-self.last_angle))/(2*np.pi))
        self.last_angle = angle
        return self.snapshot()

    def snapshot(self):
        r, d = self.body, self.data
        result = dict(r.snapshot(),turns=self.turns,physicalFailure=self.monitor.reason,**self.monitor.values,
                      supportDiagnostics=dict(rootActuation=0,runtimeClips=0,policyOwners=1),
                      scene=r.world.snapshot())
        R = d.xmat[1].reshape(3,3)
        xy = d.qpos[:2]+GRID@R[:2,:2].T
        result['terrain'] = dict(heightPatch=(r.world.height_many(xy)-d.qpos[2]).tolist(),shape=[5,5],
            handLoads=(r.hand_loads/r.mass_force).tolist(),footLoads=(r.loads/r.mass_force).tolist(),
            clearances=self.coordinates.observe(self.goal())[147:149].tolist())
        result.update(forces=r.support_forces[r.ids].tolist(),traversalTrained=False,
                      projectileContactForce=r.projectile_force.tolist(),contactGoals=r.contact_goals.tolist(),
                      goalEvents=[e.copy() for e in self.intent.events[-12:]],
                      traversalGoal=self.intent.traversal.copy() if self.intent.traversal is not None else None,
                      jumpGoal=self.intent.jump.copy() if self.intent.jump is not None else None)
        return result
