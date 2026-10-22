"""Joint-limited redundant leg geometry; reference only, never applied poses."""
import numpy as np
from scipy.spatial.transform import Rotation

def bounded_leg_ik(vector,root_rotation,foot_rotation,preferred,upper,lower,limits):
 local=root_rotation.T@vector;D=np.clip(np.linalg.norm(local),.05,upper+lower-.00001)
 knee=np.arccos(np.clip((D*D-upper*upper-lower*lower)/(2*upper*lower),-1,1));maximum=max(0.,np.arccos(np.clip(abs(local[0])/D,0,1))-.00001)
 lo=max(limits[1,0],-maximum);hi=min(limits[1,1],maximum)
 yaw=np.unique(np.r_[np.linspace(lo,hi,129),np.clip(preferred,lo,hi)])
 effective=np.arcsin(np.clip(-local[0]/(D*np.cos(yaw)),-1,1));y=-D*np.sin(effective)*np.sin(yaw);z=-D*np.cos(effective)
 roll=np.arctan2(local[1],-local[2])-np.arctan2(y,-z);hip=effective-np.arctan2(lower*np.sin(knee),upper+lower*np.cos(knee))
 hip_rotation=Rotation.from_euler('XZY',np.stack([roll,yaw,hip],axis=-1)).as_matrix();chain=root_rotation@hip_rotation@Rotation.from_euler('Y',knee).as_matrix()
 ankle=Rotation.from_matrix(chain.transpose(0,2,1)@foot_rotation).as_euler('XZY')
 angles=np.column_stack([roll,yaw,hip,np.full(len(yaw),knee),ankle]);bounded=np.clip(angles,limits[:,0],limits[:,1]);violation=np.sum((angles-bounded)**2,axis=-1)
 score=1e9*np.sum((angles[:,:4]-bounded[:,:4])**2,axis=-1)+1e4*np.sum((angles[:,4:]-bounded[:,4:])**2,axis=-1)+.1*(yaw-preferred)**2+np.sum(angles[:,4:]**2,axis=-1)
 best=int(np.argmin(score));return bounded[best],float(violation[best])



import mujoco

def standing_pose(body,height):
    m,d=body.model,body.data
    pose=d.qpos.copy();pose[2]=height
    names={mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_ACTUATOR,i):i for i in range(m.nu)}
    root_rotation=d.xmat[1].reshape(3,3)
    for side in ['right','left']:
        hip=int(body.ids[body.names.index(side+'_thigh')]);shin=int(body.ids[body.names.index(side+'_shin')]);foot=int(body.ids[body.names.index(side+'_foot')])
        indices=[names[side+'_'+axis] for axis in ['hip_x','hip_z','hip_y','knee','ankle_x','ankle_z','ankle_y']]
        joints=m.actuator_trnid[indices,0]
        vector=d.xpos[foot]-d.xpos[hip]+np.array([0.,0.,d.qpos[2]-height])
        angles,_=bounded_leg_ik(vector,root_rotation,d.xmat[foot].reshape(3,3),0.,np.linalg.norm(m.body_pos[shin]),np.linalg.norm(m.body_pos[foot]),m.jnt_range[joints])
        pose[m.jnt_qposadr[joints]]=angles
    return pose
