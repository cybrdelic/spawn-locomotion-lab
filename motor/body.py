"""Source-preserving bounded body. Only descendant impedance motors act."""
from pathlib import Path
import json,numpy as np,mujoco
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
class Body:
 def __init__(self,circle=True,xml_source=None):
  self.contract=json.loads((ROOT/'support/bounded-humanoid-contract.json').read_text());self.names=self.contract['names'];self.ids=np.array(self.contract['bodies']);xml=ET.parse(ROOT/'support/bounded-humanoid.xml').getroot() if xml_source is None else ET.fromstring(xml_source);sensors=ET.SubElement(xml,'sensor')
  for kind,name,body in [('subtreecom','com','pelvis'),('subtreelinvel','com_velocity','pelvis'),('framequat','root_quat','pelvis'),('frameangvel','root_omega','pelvis'),('framequat','torso_quat','torso'),('frameangvel','torso_omega','torso'),('framelinvel','root_velocity','pelvis')]:
   attributes={'name':name,'body':body} if kind.startswith('subtree') else {'name':name,'objtype':'xbody','objname':body};ET.SubElement(sensors,kind,attributes)
  for name in ['right_foot','left_foot']:
   foot=xml.find(".//body[@name='"+name+"']");geom=foot.find('geom');attributes={k:v for k,v in geom.attrib.items() if k in ['type','size','pos','quat','fromto']};attributes['name']=name+'_support_site';ET.SubElement(foot,'site',attributes)
   ET.SubElement(sensors,'touch',{'name':name+'_load','site':attributes['name']})
   ET.SubElement(sensors,'framepos',{'name':name+'_position','objtype':'xbody','objname':name})
   ET.SubElement(sensors,'framequat',{'name':name+'_quat','objtype':'xbody','objname':name})
  self.model=m=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));self.data=d=mujoco.MjData(m);m.opt.timestep=1/self.contract['physicsHz'];self.substeps=self.contract['physicsHz']//self.contract['controlHz']
  self.kp=np.array([g['kp'] for g in self.contract['postureGains']]);self.kd=np.array([g['kd'] for g in self.contract['postureGains']]);self.caps=m.actuator_forcerange[:,1].copy();assert np.all(self.caps>0) and np.all(m.actuator_trnid[:,0]>0)
  m.actuator_gainprm[:]=0;m.actuator_biasprm[:]=0;m.actuator_gainprm[:,0]=self.kp;m.actuator_biasprm[:,1]=-self.kp;m.actuator_biasprm[:,2]=-self.kd;m.actuator_biastype[:]=mujoco.mjtBias.mjBIAS_AFFINE
  m.actuator_ctrllimited[:]=False
  d.qpos[:3]=self.contract['init'][:3];d.qpos[3:7]=[1,0,0,0];d.qpos[7:7+len(self.contract['init'])-6]=self.contract['init'][6:]
  if circle:d.qpos[0]=6.;d.qpos[3:7]=[np.sqrt(.5),0,0,np.sqrt(.5)]
  mujoco.mj_forward(m,d);d.ctrl[:]=d.actuator_length;mujoco.mj_forward(m,d)
 def apply(self,torque):
  m,d=self.model,self.data
  for _ in range(self.substeps):
   mujoco.mj_forward(m,d);command=torque() if callable(torque) else torque;command=np.clip(command,-self.caps,self.caps)
   d.ctrl[:]=d.actuator_length+(command+self.kd*d.actuator_velocity)/self.kp
   assert not d.xfrc_applied.any() and not d.qfrc_applied.any()
   mujoco.mj_step(m,d)
  self.last_prediction_sensor=d.sensordata.copy();mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d)
 def snapshot(self):
  m,d=self.model,self.data;forces=np.zeros((m.nbody,3));local=np.empty(6)
  for i,c in enumerate(d.contact[:d.ncon]):
   a,b=[int(m.geom_bodyid[g]) for g in [c.geom1,c.geom2]]
   if a and b:continue
   mujoco.mj_contactForce(m,d,i,local);world=c.frame.reshape(3,3).T@local[:3]
   if a:forces[a]-=world
   if b:forces[b]+=world
  velocity=np.empty(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,1,velocity,0)
  return {'time':float(d.time),'root':d.qpos[:3].tolist(),'rotation':np.roll(d.qpos[3:7],-1).tolist(),'velocity':velocity[3:].tolist(),'comVelocity':d.subtree_linvel[1].tolist(),'com':d.subtree_com[1].tolist(),'positions':d.xpos[self.ids].tolist(),'rotations':np.roll(d.xquat[self.ids],-1,axis=1).tolist(),'forces':forces[self.ids].tolist(),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist()}
 def apply_pose(self,target):
  m,d=self.model,self.data;assert np.isfinite(target).all();d.ctrl[:]=target
  assert not d.xfrc_applied.any() and not d.qfrc_applied.any()
  mujoco.mj_step(m,d,nstep=self.substeps)
  self.last_prediction_sensor=d.sensordata.copy();mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d)
