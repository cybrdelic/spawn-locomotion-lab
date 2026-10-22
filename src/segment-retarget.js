import {Quaternion,Vector3} from 'three';

export const segmentChildren={LeftUpLeg:'LeftLeg',LeftLeg:'LeftFoot',RightUpLeg:'RightLeg',RightLeg:'RightFoot',LeftArm:'LeftForeArm',LeftForeArm:'LeftHand',RightArm:'RightForeArm',RightForeArm:'RightHand'};

// Map each target bone's actual child axis onto its simulated segment axis.
// Multiplying the simulation rotation by the raw avatar bind quaternion instead
// would add the avatar's bent/splayed bind stance to every simulated pose.
export function calibrateSegments(bindings,correspondence,state,conversion){
 const orientations=new Map(),segments=new Map(),report=[];
 for(const [name,child] of Object.entries(segmentChildren)){
  const parentIndex=state.names.indexOf(correspondence[name]),childIndex=state.names.indexOf(correspondence[child]);
  if(parentIndex<0||childIndex<0)throw Error(`Missing physical segment ${name}`);
  const binding=bindings.get(name),bindVector=bindings.get(child).w.clone().sub(binding.w),length=bindVector.length();
  const physicalQ=conversion.clone().multiply(new Quaternion().fromArray(state.rotations[parentIndex])).multiply(conversion.clone().invert());
  const axis=new Vector3().fromArray(state.positions[childIndex]).sub(new Vector3().fromArray(state.positions[parentIndex])).applyQuaternion(conversion).applyQuaternion(physicalQ.invert()).normalize();
  const alignment=new Quaternion().setFromUnitVectors(bindVector.clone().normalize(),axis);
  orientations.set(name,alignment.clone().multiply(binding.q).normalize());segments.set(name,{length,axis});
  report.push({name,bindOffsetDegrees:alignment.angleTo(new Quaternion())*180/Math.PI,length});
 }
 // One fixed mesh-origin calibration preserves the avatar's neutral sole height
 // after removing its bind-pose knee bends. This never moves the physical root
 // and never changes with contacts, gait, velocity or the current simulated pose.
 let hipLift=0;
 for(const side of ['Left','Right']){
  const upper=segments.get(side+'UpLeg'),lower=segments.get(side+'Leg');
  const alignedAnkle=bindings.get(side+'UpLeg').w.y+upper.length*upper.axis.y+lower.length*lower.axis.y;
  hipLift+=(bindings.get(side+'Foot').w.y-alignedAnkle)/2;
 }
 return {orientations,hipLift,report};
}
