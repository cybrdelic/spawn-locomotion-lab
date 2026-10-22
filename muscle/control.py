"""Compiled implementation candidate of the SAME audited physical drive."""
import math,time
import numpy as np,mujoco
from numba import njit
@njit(cache=True,nogil=True)
def control(q,velocity,ctrl,target,bq,bv,slots,hq,hv,hs,kp,kd,caps,nq,nv,nkp,nkd,ncaps):
    torque=np.empty(28)
    for group in range(len(bq)):
        a,b,c=q[bq[group,0]],q[bq[group,1]],q[bq[group,2]]
        sa,ca=math.sin(a/2),math.cos(a/2);sb,cb=math.sin(b/2),math.cos(b/2);sc,cc=math.sin(c/2),math.cos(c/2)
        x=sa*cb*cc-ca*sb*sc;y=-sa*sb*cc+ca*cb*sc;z=ca*sb*cc+sa*cb*sc;w=ca*cb*cc+sa*sb*sc
        if w<0:x=-x;y=-y;z=-z;w=-w
        norm=math.sqrt(x*x+y*y+z*z);scale=2*math.atan2(norm,w)/norm if norm>1e-15 else 2.
        cb,sb,cc,sc=math.cos(b),math.sin(b),math.cos(c),math.sin(c)
        vx,vz,vy=velocity[bv[group,0]],velocity[bv[group,1]],velocity[bv[group,2]]
        omega=(cb*cc*vx-sc*vz,-sb*vx+vy,cb*sc*vx+cc*vz);length=(x*scale,y*scale,z*scale)
        t=np.empty(3)
        for j in range(3):
            s=slots[group,j];f=kp[s]*(target[s]-length[j])-kd[s]*omega[j];t[j]=min(caps[s],max(-caps[s],f))
        torque[bv[group,0]-6]=cb*cc*t[0]-sb*t[1]+cb*sc*t[2]
        torque[bv[group,1]-6]=-sc*t[0]+cc*t[2]
        torque[bv[group,2]-6]=t[1]
    for j in range(len(hq)):
        s=hs[j];f=kp[s]*(target[s]-q[hq[j]])-kd[s]*velocity[hv[j]];torque[hv[j]-6]=min(caps[s],max(-caps[s],f))
    for i in range(28):
        t=min(ncaps[i],max(-ncaps[i],torque[i]));ctrl[i]=q[nq[i]]+(t+nkd[i]*velocity[nv[i]])/nkp[i]
