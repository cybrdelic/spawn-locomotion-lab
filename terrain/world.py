"""Actual static collision geometry and a physically thrown disturbance object."""
from pathlib import Path
import json,weakref,xml.etree.ElementTree as ET
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1]
class World:
    def __init__(self,course=True):
        self.spec=json.loads((ROOT/'terrain/course.json').read_text()) if course else {'name':'Disturbance floor','obstacles':[]}
        xml=ET.parse(ROOT/'support/bounded-humanoid.xml').getroot();world=xml.find('worldbody')
        for row in self.spec['obstacles']:
            assert len(row['position'])==len(row['halfSize'])==3 and all(0<x<5 for x in row['halfSize'])
            ET.SubElement(world,'geom',dict(name='course_'+row['name'],type='box',pos=' '.join(map(str,row['position'])),size=' '.join(map(str,row['halfSize'])),friction='1 .01 .001',condim='3',group='0',rgba='.24 .46 .46 1'))
        prop=ET.SubElement(world,'body',dict(name='disturbance',pos='40 40 .2'))
        ET.SubElement(prop,'freejoint',dict(name='disturbance_free'))
        ET.SubElement(prop,'geom',dict(name='disturbance_ball',type='sphere',size='.18',mass='4',friction='.7 .01 .001',condim='3',group='2',rgba='.9 .35 .12 1'))
        self.xml=ET.tostring(xml,encoding='unicode');self.events=[]
    def bind(self,rig):
        self.rig=weakref.proxy(rig);m=rig.model
        self.prop=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'disturbance');j=int(m.body_jntadr[self.prop]);self.q=int(m.jnt_qposadr[j]);self.v=int(m.jnt_dofadr[j])
        assert self.q==35 and self.v==34 and not (m.actuator_trnid[:,0]==j).any()
        m.geom_group[np.isin(m.geom_bodyid,rig.ids)]=1
        self.static_boxes=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'course_'+v['name']) for v in self.spec['obstacles']]
        self.groups=np.array([1,0,0,0,0,0],dtype=np.uint8)
    def height(self,x,y,ceiling=4.):
        hit=np.zeros(1,np.int32);distance=mujoco.mj_ray(self.rig.model,self.rig.data,np.array([x,y,ceiling]),np.array([0.,0.,-1.]),self.groups,1,-1,hit)
        return ceiling-float(distance) if distance>=0 else 0.
    def height_many(self,xy):
        xy=np.asarray(xy);height=np.zeros(xy.shape[:-1]);m=self.rig.model
        for g in self.static_boxes:
            p=m.geom_pos[g];half=m.geom_size[g];inside=np.all(abs(xy-p[:2])<=half[:2],axis=-1)
            height=np.where(inside,np.maximum(height,p[2]+half[2]),height)
        return height
    def launch(self,side='left',speed=3.):
        local={'front':[-1,0],'back':[1,0],'left':[0,-1],'right':[0,1]}
        if side not in local or not np.isfinite(speed) or not 1.5<=speed<=5:raise ValueError('Expected front/back/left/right and speed1.5-5m/s')
        r=self.rig;d=r.data;before=d.qpos[:35].copy();R=d.xmat[1].reshape(3,3);direction=R[:2,:2]@np.array(local[side]);direction/=np.linalg.norm(direction)
        p=d.subtree_com[1].copy();distance=max(.4,.25*speed);p[:2]-=direction*distance;p[2]=d.xpos[1,2]+.1;vertical=.5*9.81*distance/speed
        d.qpos[self.q:self.q+3]=p;d.qpos[self.q+3:self.q+7]=[1,0,0,0];d.qvel[self.v:self.v+6]=np.r_[direction*speed,vertical,0.,0.,0.]
        mujoco.mj_forward(r.model,d);mujoco.mj_subtreeVel(r.model,d);r.contacts()
        assert np.array_equal(before,d.qpos[:35]) and not d.qfrc_applied.any() and not d.xfrc_applied.any()
        event=dict(time=float(d.time),side=side,objectMass=4.,objectHorizontalSpeed=speed,objectSpeed=float(np.hypot(speed,vertical)),injectedMomentum=float(4*np.hypot(speed,vertical)),injectedEnergy=float(2*(speed**2+vertical**2)),characterPoseChange=0,method='physical projectile collision')
        self.events.append(event);return event
    def snapshot(self):
        d=self.rig.data;rows=[dict(name=r['name'],kind='box',position=r['position'],rotation=[0,0,0,1],halfSize=r['halfSize'],role=r['role']) for r in self.spec['obstacles']]
        rows.append(dict(name='disturbance',kind='sphere',position=d.xpos[self.prop].tolist(),rotation=np.roll(d.xquat[self.prop],-1).tolist(),radius=.18,role='disturbance',active=bool(self.events)))
        return dict(name=self.spec['name'],entities=rows,disturbances=self.events[-12:])

