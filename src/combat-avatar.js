import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {KTX2Loader} from 'three/addons/loaders/KTX2Loader.js';
import {MeshoptDecoder} from 'three/addons/libs/meshopt_decoder.module.js';
export async function loadCombatAvatar(renderer){
 const textures=new KTX2Loader().setTranscoderPath('/basis/').detectSupport(renderer);
 const loader=new GLTFLoader().setKTX2Loader(textures).setMeshoptDecoder(MeshoptDecoder);
 const gltf=await loader.loadAsync('/cybr-combat.glb');
 const model=gltf.scene;model.updateMatrixWorld(true);const bones={};model.traverse(o=>{if(o.isBone)bones[o.name]=o;if(o.isMesh){o.castShadow=true;o.receiveShadow=true;if(o.isSkinnedMesh)o.frustumCulled=false;}});
 // Embedded clips are deliberately never instantiated or played.
 const joints={};for(const [name,b] of Object.entries(bones))joints[name]={position:b.getWorldPosition(new THREE.Vector3()).toArray(),quaternion:b.getWorldQuaternion(new THREE.Quaternion()).toArray()};
 const bounds=new THREE.Box3().setFromObject(model);
 const rest=new Map(Object.entries(bones).map(([name,b])=>[name,{p:b.position.clone(),q:b.quaternion.clone(),s:b.scale.clone()}]));
 textures.dispose();return {model,bones,joints,bounds,rest,clipCount:gltf.animations.length};
}

export class PhysicalCombatSkin{
 constructor(asset,sim){
  for(const [name,rest] of asset.rest){const b=asset.bones[name];b.position.copy(rest.p);b.quaternion.copy(rest.q);b.scale.copy(rest.s);}
  this.model=asset.model;this.model.rotation.y=-Math.PI/2;this.model.updateMatrixWorld(true);this.bones=asset.bones;this.bind=new Map();
  for(const b of Object.values(this.bones))this.bind.set(b.name,{p:b.getWorldPosition(new THREE.Vector3()),q:b.getWorldQuaternion(new THREE.Quaternion())});
  const hip=this.bind.get('LeftUpLeg').p.clone().add(this.bind.get('RightUpLeg').p).multiplyScalar(.5);this.origin=hip.add(new THREE.Vector3(0,sim.hipDrop,0));
  this.controls=new Map();
  const torso=['Hips','Spine02','Spine01','Spine','neck','LeftShoulder','RightShoulder'];
  for(const name of torso)this.controls.set(name,{body:sim.root,offset:this.bind.get(name).p.clone().sub(this.origin),alignment:new THREE.Quaternion()});
  const head=sim.bodies.find(o=>o.name==='head').b;this.controls.set('Head',{body:head,offset:this.bind.get('Head').p.clone().sub(this.origin.clone().add(new THREE.Vector3(0,sim.hipDrop+.17,0))),alignment:new THREE.Quaternion()});
  for(let s=0;s<2;s++){
   const side=s?'Right':'Left';
   for(const [bone,child,bodyName,L] of [[side+'UpLeg',side+'Leg','thigh'+s,sim.legData[s].upper],[side+'Leg',side+'Foot','shin'+s,sim.legData[s].lower],[side+'Arm',side+'ForeArm','arm'+s,sim.armData[s].upper],[side+'ForeArm',side+'Hand','forearm'+s,sim.armData[s].lower]]){
    const direction=this.bind.get(child).p.clone().sub(this.bind.get(bone).p).normalize(),alignment=new THREE.Quaternion().setFromUnitVectors(direction,new THREE.Vector3(0,-1,0));
    this.controls.set(bone,{body:sim.bodies.find(o=>o.name===bodyName).b,offset:new THREE.Vector3(0,L/2,0),alignment});
   }
   this.controls.set(side+'Foot',{body:sim.feet[s],offset:new THREE.Vector3(-.035,sim.ankleHeight-.055,0),alignment:new THREE.Quaternion()});
   const forearm=this.controls.get(side+'ForeArm');this.controls.set(side+'Hand',{body:forearm.body,offset:new THREE.Vector3(0,-sim.armData[s].lower/2,0),alignment:forearm.alignment});
  }
  const depth=b=>{let d=0;while(b.parent){d++;b=b.parent;}return d;};this.ordered=Object.keys(Object.fromEntries(this.controls)).sort((a,b)=>depth(this.bones[a])-depth(this.bones[b]));
 }
 update(){
  for(const name of this.ordered){const bone=this.bones[name],control=this.controls.get(name),bodyQ=new THREE.Quaternion().copy(control.body.rotation()),worldP=control.offset.clone().applyQuaternion(bodyQ).add(control.body.translation()),worldQ=bodyQ.multiply(control.alignment).multiply(this.bind.get(name).q);
   bone.parent.updateWorldMatrix(true,false);bone.position.copy(bone.parent.worldToLocal(worldP));bone.quaternion.copy(bone.parent.getWorldQuaternion(new THREE.Quaternion()).invert().multiply(worldQ));bone.updateMatrixWorld(true);
  }
  this.model.updateMatrixWorld(true);
 }
}
