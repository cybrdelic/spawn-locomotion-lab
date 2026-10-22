import {Box3,Matrix4,Quaternion,Vector3} from 'three';
const V=()=>new Vector3(),Q=()=>new Quaternion();
const legNames=new Set(['LeftUpLeg','LeftLeg','LeftFoot','RightUpLeg','RightLeg','RightFoot']);
function frame(down,hinge){
 const y=down.clone().normalize(),z=hinge.clone().addScaledVector(y,-hinge.dot(y)).normalize();
 const x=y.clone().cross(z).normalize();z.crossVectors(x,y).normalize();
 return Q().setFromRotationMatrix(new Matrix4().makeBasis(x,y,z));
}
function setWorld(bone,q){bone.parent.updateWorldMatrix(true,false);bone.quaternion.copy(bone.parent.getWorldQuaternion(Q()).invert().multiply(q));bone.updateMatrixWorld(true);}
function soleCloud(asset,foot){
 const names=new Set();foot.traverse(b=>{if(b.isBone)names.add(b.name);});const points=[];const inverse=foot.getWorldQuaternion(Q()).invert(),origin=foot.getWorldPosition(V());
 asset.model.traverse(mesh=>{
  if(!mesh.isSkinnedMesh)return;mesh.updateWorldMatrix(true,false);mesh.skeleton.update();
  const index=mesh.geometry.attributes.skinIndex,weight=mesh.geometry.attributes.skinWeight,position=mesh.geometry.attributes.position;
  if(!index||!weight)return;
  const selected=mesh.skeleton.bones.map(b=>names.has(b.name));
  for(let i=0;i<position.count;i++){
   const ids=[index.getX(i),index.getY(i),index.getZ(i),index.getW(i)],ws=[weight.getX(i),weight.getY(i),weight.getZ(i),weight.getW(i)];
   if(ids.reduce((sum,id,j)=>sum+(selected[id]?ws[j]:0),0)<.95)continue;
   const p=mesh.getVertexPosition(i,V()).applyMatrix4(mesh.matrixWorld);points.push(p);
  }
 });
 if(points.length<8)throw Error('No rigid sole geometry for '+foot.name);
 const box=new Box3().setFromPoints(points),marker=box.getCenter(V());marker.y=box.min.y;
 return {marker:marker.sub(origin).applyQuaternion(inverse),points:new Float64Array(points.flatMap(p=>p.sub(origin).applyQuaternion(inverse).toArray())),count:points.length};
}
function lowest(points,q){const e=new Matrix4().makeRotationFromQuaternion(q).elements;let result=Infinity;for(let i=0;i<points.length;i+=3)result=Math.min(result,e[1]*points[i]+e[5]*points[i+1]+e[9]*points[i+2]);return result;}
export class ContactRetarget{
 constructor(asset,bindings,correspondence,conversion,geometry){
  this.asset=asset;this.bones=asset.bones;this.bindings=bindings;this.map=correspondence;this.conversion=conversion;this.inverse=conversion.clone().invert();this.geometry=geometry;this.arms=new Map();this.legs=[];this.last=null;
  const depth=b=>{let n=0;while(b.parent){n++;b=b.parent;}return n;};this.ordered=Object.keys(correspondence).sort((a,b)=>depth(this.bones[a])-depth(this.bones[b]));
  const s=geometry.reference;
  for(const [parent,child] of Object.entries({LeftArm:'LeftForeArm',LeftForeArm:'LeftHand',RightArm:'RightForeArm',RightForeArm:'RightHand'})){
   const bind=bindings.get(child).w.clone().sub(bindings.get(parent).w).normalize();
   const local=this.position(s,child).sub(this.position(s,parent)).applyQuaternion(this.rotation(s,parent).invert()).normalize();
   this.arms.set(parent,Q().setFromUnitVectors(bind,local).multiply(bindings.get(parent).q));
  }
  for(const side of ['Left','Right']){
   const upper=side+'UpLeg',lower=side+'Leg',foot=side+'Foot',up=bindings.get(lower).w.clone().sub(bindings.get(upper).w),down=bindings.get(foot).w.clone().sub(bindings.get(lower).w);
   const forward=bindings.get(side+'ToeBase').w.clone().sub(bindings.get(foot).w);forward.y=0;forward.normalize();
   let hinge=up.clone().cross(down);if(hinge.lengthSq()<1e-8)hinge=forward.clone().cross(up);
   hinge.normalize();if(up.clone().cross(hinge).dot(forward)<0)hinge.negate();
   const cloud=soleCloud(asset,this.bones[foot]),physical=geometry.feet[side.toLowerCase()+'_foot'];
   const pf=new Vector3().fromArray(physical.forwardLocal).applyQuaternion(conversion).normalize(),pu=new Vector3().fromArray(physical.upLocal).applyQuaternion(conversion).normalize();
   const physicalFrame=Q().setFromRotationMatrix(new Matrix4().makeBasis(pf,pu,pf.clone().cross(pu).normalize()));
   const avatarFrame=Q().setFromRotationMatrix(new Matrix4().makeBasis(forward,new Vector3(0,1,0),forward.clone().cross(new Vector3(0,1,0)).normalize()));
   this.legs.push({side,upper,lower,foot,L1:up.length(),L2:down.length(),upperOffset:frame(up,hinge).invert().multiply(bindings.get(upper).q),lowerOffset:frame(down,hinge).invert().multiply(bindings.get(lower).q),footOffset:physicalFrame.multiply(avatarFrame.invert()).multiply(bindings.get(foot).q),cloud,physical,knee:geometry.knees[side.toLowerCase()+'_shin']});
  }
  const physicalMid=this.position(s,'LeftUpLeg').add(this.position(s,'RightUpLeg')).multiplyScalar(.5);
  const avatarMid=bindings.get('LeftUpLeg').w.clone().add(bindings.get('RightUpLeg').w).multiplyScalar(.5);
  this.rootOffset=physicalMid.sub(V().fromArray(s.root).applyQuaternion(conversion)).sub(avatarMid.sub(bindings.get('Hips').w)).applyQuaternion(this.rotation(s,'Hips').invert());this.basePose(s);
  const offsets=this.legs.map(leg=>{
   const hip=this.bones[leg.upper].getWorldPosition(V()),target=this.footTarget(s,leg).position;
   const u=this.position(s,leg.lower).sub(this.position(s,leg.upper)),v=this.position(s,leg.foot).sub(this.position(s,leg.lower));
   const flex=u.angleTo(v),reach=Math.sqrt(leg.L1**2+leg.L2**2+2*leg.L1*leg.L2*Math.cos(flex));
   return target.y+Math.sqrt(Math.max(0,reach**2-(target.x-hip.x)**2-(target.z-hip.z)**2))-hip.y;
  });
  // One fixed anatomy calibration. The physical root is never moved by rendering.
  this.rootOffset.y+=Math.min(...offsets);this.update(s);
 }
 position(s,name){return V().fromArray(s.positions[s.names.indexOf(this.map[name])]).applyQuaternion(this.conversion);}
 rotation(s,name){return this.conversion.clone().multiply(Q().fromArray(s.rotations[s.names.indexOf(this.map[name])])).multiply(this.inverse);}
 basePose(s){
  const root=V().fromArray(s.root).applyQuaternion(this.conversion);
  for(const name of this.ordered){
   if(legNames.has(name))continue;
   const bone=this.bones[name],q=this.rotation(s,name).multiply(this.arms.get(name)||this.bindings.get(name).q);setWorld(bone,q);
   if(name==='Hips'){bone.position.copy(bone.parent.worldToLocal(root.clone().add(this.rootOffset.clone().applyQuaternion(this.rotation(s,'Hips')))));bone.updateMatrixWorld(true);}
  }
 }
 footTarget(s,leg){
  const q=this.rotation(s,leg.foot),origin=this.position(s,leg.foot),local=V().fromArray(leg.physical.soleLocal).applyQuaternion(this.conversion),sole=local.applyQuaternion(q).add(origin);
  const orientation=q.clone().multiply(leg.footOffset),position=sole.clone().sub(leg.cloud.marker.clone().applyQuaternion(orientation));
  const up=V().fromArray(leg.physical.upLocal).applyQuaternion(this.conversion).applyQuaternion(q),forward=V().fromArray(leg.physical.forwardLocal).applyQuaternion(this.conversion).applyQuaternion(q),right=forward.clone().cross(up);
  const center=V().fromArray(leg.physical.soleLocal).addScaledVector(V().fromArray(leg.physical.upLocal),leg.physical.halfSize[2]).applyQuaternion(this.conversion).applyQuaternion(q).add(origin);
  const minY=center.y-Math.abs(forward.y)*leg.physical.halfSize[0]-Math.abs(right.y)*leg.physical.halfSize[1]-Math.abs(up.y)*leg.physical.halfSize[2];
  position.y=minY-lowest(leg.cloud.points,orientation);return {position,orientation,sole,minY};
 }
 update(s){
  this.basePose(s);const diagnostics=[];
  for(const leg of this.legs){
   const hip=this.bones[leg.upper].getWorldPosition(V()),target=this.footTarget(s,leg),direction=target.position.clone().sub(hip),distance=direction.length();direction.normalize();
   const physicalDown=this.position(s,leg.lower).sub(this.position(s,leg.upper)).normalize(),kneeAxis=V().fromArray(leg.knee.axisLocal).applyQuaternion(this.conversion).applyQuaternion(this.rotation(s,leg.lower));
   const pole=physicalDown.clone().cross(kneeAxis);pole.addScaledVector(direction,-pole.dot(direction)).normalize();
   const minimum=Math.sqrt(leg.L1**2+leg.L2**2+2*leg.L1*leg.L2*Math.cos(leg.knee.maxFlexion));
   const reach=Math.max(minimum,Math.min(leg.L1+leg.L2-1e-6,distance)),along=(leg.L1**2-leg.L2**2+reach**2)/(2*reach),bend=Math.sqrt(Math.max(0,leg.L1**2-along**2));
   const knee=hip.clone().addScaledVector(direction,along).addScaledVector(pole,bend),ankle=hip.clone().addScaledVector(direction,reach),hinge=pole.clone().cross(direction).normalize();
   setWorld(this.bones[leg.upper],frame(knee.clone().sub(hip),hinge).multiply(leg.upperOffset));
   const actualKnee=this.bones[leg.lower].getWorldPosition(V());setWorld(this.bones[leg.lower],frame(ankle.clone().sub(actualKnee),hinge).multiply(leg.lowerOffset));setWorld(this.bones[leg.foot],target.orientation);
   const actual=this.bones[leg.foot].getWorldPosition(V()),q=this.bones[leg.foot].getWorldQuaternion(Q()),sole=leg.cloud.marker.clone().applyQuaternion(q).add(actual);
   diagnostics.push({side:leg.side,clearanceError:actual.y+lowest(leg.cloud.points,q)-target.minY,soleCenterError:Math.hypot(sole.x-target.sole.x,sole.z-target.sole.z),reachError:Math.abs(distance-reach),kneeFlexion:Math.acos(Math.max(-1,Math.min(1,(reach**2-leg.L1**2-leg.L2**2)/(2*leg.L1*leg.L2)))),maxFlexion:leg.knee.maxFlexion});
  }
  this.asset.model.updateMatrixWorld(true);this.last={contactRetargeting:true,time:s.time,rootOffsetLocal:this.rootOffset.toArray(),feet:diagnostics};return this.last;
 }
}
