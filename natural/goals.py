"""Shared continuous movement intent. Geometry/contact goals never move the body."""
import numpy as np
from natural.coordinates import heading
class Intent:
    def __init__(self,speed=0.,height=.88,circle=False,radius=6.,center=(0.,0.)):
        self.speed=float(speed);self.height=height;self.circle=circle;self.radius=radius;self.center=np.array(center,dtype=float)
        self.stop_direction=None;self.contacts=np.zeros((4,7));self.traversal=None;self.jump=None;self.events=[];self.completed=set()
    def set_contacts(self,r,contacts):
        contacts=np.asarray(contacts,dtype=float)
        if contacts.shape!=(4,7) or not np.isfinite(contacts).all() or not np.isin(contacts[:,6],[0,1]).all():raise ValueError('Expected four finite contact positions/normals/masks')
        enabled=contacts[:,6]>0
        if enabled.any() and (np.max(np.linalg.norm(contacts[enabled,:3]-r.data.qpos[:3],axis=1))>3 or np.max(abs(np.linalg.norm(contacts[enabled,3:6],axis=1)-1))>.001):raise ValueError('Contact intent must have unit normals and be within3m')
        self.contacts=contacts.copy()
    def request_jump(self,r,rise=.55):
        rise=float(rise)
        if not np.isfinite(rise) or not 0<rise<=2:raise ValueError('Expected a finite positive jump-rise goal up to2m')
        com=float(r.data.subtree_com[1,2]);supported=self.feet_support(r)
        self.jump=dict(start=float(r.data.time),armed=supported,airborne=False,
                       com_origin=com,com_apex=com+rise,peak_com=com,requestedRise=rise,
                       flight_start=None,maximum_flight=0.,landing_start=None,spawnedAirborne=False)
    @staticmethod
    def support_force(r):return float(np.linalg.norm(r.support_forces[r.ids],axis=1).sum())
    @staticmethod
    def feet_support(r):return bool(r.loads.sum()>.1*r.mass_force and r.data.xmat[1,8]>.7 and r.unsafe_load<.1*r.mass_force)
    def jump_height(self,r,origin):
        j=self.jump;d=r.data;now=float(d.time);com=float(d.subtree_com[1,2]);vz=float(d.subtree_linvel[1,2])
        force=self.support_force(r);supported=force>.1*r.mass_force
        if not j['armed'] and self.feet_support(r):
            j.update(armed=True,com_origin=com,com_apex=com+j['requestedRise'],peak_com=com)
        j['peak_com']=max(j['peak_com'],com)
        if j['armed'] and force<.05*r.mass_force:
            if j['flight_start'] is None and (vz>.2 or j['airborne']):
                j['flight_start']=now;j['airborne']=True
            if j['flight_start'] is not None:j['maximum_flight']=max(j['maximum_flight'],now-j['flight_start'])
            j['landing_start']=None
        elif supported:
            if j['flight_start'] is not None:
                j['maximum_flight']=max(j['maximum_flight'],now-j['flight_start']);j['flight_start']=None
            nonfeet=force-float(np.linalg.norm(r.support_forces[r.feet],axis=1).sum())
            feet=self.feet_support(r) and r.loads.sum()>.8*r.mass_force and nonfeet<.1*r.mass_force
            if j['airborne'] and feet:
                if j['landing_start'] is None:j['landing_start']=now
                elif now-j['landing_start']>=.06:
                    landing=j['spawnedAirborne']
                    reached=j['peak_com']>=j['com_apex']-min(.10,.2*j['requestedRise'])
                    flight=j['maximum_flight']>=(0. if landing else .10)
                    kind=('landingCompleted' if landing else 'jumpCompleted') if reached and flight else 'jumpFailed'
                    self.events.append(dict(kind=kind,time=now,peakRise=j['peak_com']-j['com_origin'],requestedRise=j['requestedRise'],flightSeconds=j['maximum_flight']))
                    self.jump=None;return self.height
            else:j['landing_start']=None
        else:
            if j['flight_start'] is not None:
                j['maximum_flight']=max(j['maximum_flight'],now-j['flight_start']);j['flight_start']=None
            j['landing_start']=None
        if (not j['armed'] and now-j['start']>.25) or now-j['start']>3:
            self.events.append(dict(kind='jumpFailed',reason='noSupport' if not j['armed'] else 'timeout',time=now,peakRise=j['peak_com']-j['com_origin']))
            self.jump=None;return self.height
        # Descending uses the same standing-height/contact feedback for landing.
        # During ascent, root-height error is exactly COM-apex error; moving the
        # pelvis relative to COM cannot fabricate a successful jump.
        if j['airborne'] and vz<=0:return self.height
        j['apex']=j['com_apex']+float(d.qpos[2])-com
        return j['apex']-origin
    def request_traversal(self,r,fallback_jump=True):
        forward=heading(r);candidates=[]
        for row,g in zip(r.world.spec['obstacles'],r.world.static_boxes):
            if row['name'] in self.completed:continue
            delta=r.model.geom_pos[g,:2]-r.data.qpos[:2];ahead=float(delta@forward);side=abs(forward[0]*delta[1]-forward[1]*delta[0])
            if .2<ahead<2.5 and side<1.5:candidates.append((ahead,row,g))
        if not candidates:
            if fallback_jump:self.request_jump(r)
            return
        _,row,g=min(candidates,key=lambda x:x[0]);top=r.model.geom_pos[g,2]+r.model.geom_size[g,2];rise=top-r.reference_height()
        kind='step' if rise<=.35 else 'vault' if rise<=.95 else 'climb'
        self.traversal=dict(name=row['name'],geom=g,kind=kind,start=float(r.data.time),touched=False);self.speed=max(1.,self.speed)
    def observe(self,r):
        d=r.data;origin=r.reference_height()
        if self.circle:
            position=d.qpos[:2]-self.center;radius=max(.5,float(np.linalg.norm(position)));radial=position/radius
            direction=np.array([-radial[1],radial[0]])-radial*(radius-self.radius)/.83;direction/=max(1e-8,np.linalg.norm(direction))
        else:direction=np.array([1.,0.])
        if self.speed<.05:
            if self.stop_direction is None:
                self.stop_direction=heading(r)
            direction=self.stop_direction
        else:self.stop_direction=None
        height=self.height;contacts=self.contacts.copy()
        if self.jump is not None:
            height=self.jump_height(r,origin)
        if self.traversal is not None:
            t=self.traversal;gid=t['geom'];p=r.model.geom_pos[gid];half=r.model.geom_size[gid];top=p[2]+half[2];front=p[0]-half[0];far=p[0]+half[0];kind=t['kind']
            # Only pressure on THIS surface proves contact. A foot touching
            # its side while carrying weight on the floor cannot satisfy it.
            surface=np.array([loads.get(gid,0.) for loads in r.surface_up_loads])
            if (surface[:2] if kind=='step' else surface[2:]).sum()>.05*r.mass_force:t['touched']=True
            feet_top=float(surface[:2].sum())
            transfer=feet_top>.6*r.mass_force if kind!='vault' else d.qpos[0]>far
            for i,side in enumerate([-1,1]):
                if kind!='step' and not transfer:contacts[2+i]=[front+.08,p[1]+side*.22,top,0,0,1,1]
                landing=np.array([far+.4 if kind=='vault' else front+.35,p[1]+side*.12]);contacts[i]=[*landing,float(r.world.height_many(landing)),0,0,1,1]
            height=self.height+max(0.,top-origin)
            if kind=='vault' and transfer:height=self.height
            nonfeet=self.support_force(r)-float(np.linalg.norm(r.support_forces[r.feet],axis=1).sum())
            supported=(r.loads.sum()>.8*r.mass_force and nonfeet<.1*r.mass_force and d.xmat[1,8]>.9)
            target_floor=origin if kind=='vault' else top
            standing=abs(float(d.qpos[2])-target_floor-self.height)<.1
            cleared=(d.qpos[0]>far+.3) if kind=='vault' else (d.qpos[0]>front+.25 and feet_top>.8*r.mass_force)
            if cleared and t['touched'] and supported and standing:
                self.completed.add(t['name'])
                self.events.append(dict(kind='obstacleCompleted',movement=kind,surface=t['name'],time=float(d.time)));self.traversal=None;contacts[:]=0;height=self.height
            elif d.time-t['start']>10:
                self.events.append(dict(kind='traversalGoalTimeout',surface=t['name'],time=float(d.time)));self.traversal=None;contacts[:]=0;height=self.height
        r.contact_goals=contacts
        return np.r_[direction*self.speed,self.speed/self.radius if self.circle else 0.,height,direction]
