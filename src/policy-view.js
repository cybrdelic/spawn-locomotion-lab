import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {loadCombatAvatar} from './combat-avatar.js';
import {segmentChildren} from './segment-retarget.js';
import {ContactRetarget} from './contact-retarget.js';
const renderer=new THREE.WebGLRenderer({canvas:document.querySelector('canvas'),antialias:true});renderer.setPixelRatio(Math.min(2,devicePixelRatio));renderer.setClearColor('#101923');renderer.shadowMap.enabled=true;
const scene=new THREE.Scene();scene.add(new THREE.HemisphereLight('#e8f5ff','#344b47',2.2));const sun=new THREE.DirectionalLight('#fff1dd',3);sun.position.set(3,8,4);sun.castShadow=true;sun.shadow.mapSize.set(2048,2048);Object.assign(sun.shadow.camera,{left:-15,right:15,top:15,bottom:-15});scene.add(sun);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(100,100),new THREE.MeshStandardMaterial({color:'#243a43',roughness:.95}));floor.rotation.x=-Math.PI/2;floor.receiveShadow=true;scene.add(floor);scene.add(new THREE.GridHelper(50,100,'#426168','#2e4950'));
const radius=6,points=Array.from({length:129},(_,i)=>new THREE.Vector3(radius*Math.cos(i*Math.PI/64),.006,-radius*Math.sin(i*Math.PI/64)));
const pathRing=new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color:'#9bc5b1'}));scene.add(pathRing);
const camera=new THREE.PerspectiveCamera(38,innerWidth/innerHeight,.05,120);camera.position.set(9,2.1,4);const orbit=new OrbitControls(camera,renderer.domElement);orbit.target.set(6,.95,0);orbit.enableDamping=true;
const asset=await loadCombatAvatar(renderer);asset.model.rotation.y=-Math.PI/2;asset.model.updateMatrixWorld(true);scene.add(asset.model);
const bindings=new Map();for(const [name,bone] of Object.entries(asset.bones))bindings.set(name,{q:bone.getWorldQuaternion(new THREE.Quaternion()),w:bone.getWorldPosition(new THREE.Vector3())});
const correspondence={Hips:'pelvis',Spine02:'torso',Spine01:'torso',Spine:'torso',neck:'torso',Head:'head',LeftShoulder:'torso',RightShoulder:'torso',LeftArm:'left_upper_arm',LeftForeArm:'left_lower_arm',LeftHand:'left_hand',RightArm:'right_upper_arm',RightForeArm:'right_lower_arm',RightHand:'right_hand',LeftUpLeg:'left_thigh',LeftLeg:'left_shin',LeftFoot:'left_foot',RightUpLeg:'right_thigh',RightLeg:'right_shin',RightFoot:'right_foot'};
const conversion=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2);
const contactGeometry=await(await fetch('/physical-contact-rig.json')).json();
const skin=new ContactRetarget(asset,bindings,correspondence,conversion,contactGeometry);
let lastRenderedState;
let state,following=new THREE.Vector3(6,0,0),lastTime=-1,history=[],crouchRequested=false,requestedSpeed=0;
const viewOptions=new URLSearchParams(location.search);
const diagnosticMode=['support','mpc','motor','centroidal'].some(key=>viewOptions.has(key));
const unifiedMode=viewOptions.has('unified')||(!diagnosticMode&&!viewOptions.get('telemetry'));
const courseMode=viewOptions.has('course')||viewOptions.get('port')==='5191';
const physicsPort=new URLSearchParams(location.search).get('port')==='5191'?'5191':'5190';
const physicsEndpoint='http://127.0.0.1:'+physicsPort;
const sceneMeshes=new Map();
function updateScene(observation){
 for(const row of observation?.entities||[]){
  let mesh=sceneMeshes.get(row.name);
  if(!mesh){
   const geometry=row.kind==='sphere'?new THREE.SphereGeometry(row.radius,20,12):new THREE.BoxGeometry(2*row.halfSize[0],2*row.halfSize[2],2*row.halfSize[1]);
   mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:row.role==='disturbance'?'#dc713e':row.role==='vault'?'#b79c62':'#527b76',roughness:.82}));mesh.castShadow=true;mesh.receiveShadow=true;
   if(row.kind==='box')mesh.add(new THREE.LineSegments(new THREE.EdgesGeometry(geometry),new THREE.LineBasicMaterial({color:'#9dc7bd'})));
   scene.add(mesh);sceneMeshes.set(row.name,mesh);
  }
  mesh.visible=row.active!==false;
  mesh.position.set(row.position[0],row.position[2],-row.position[1]);
  mesh.quaternion.copy(conversion).multiply(new THREE.Quaternion().fromArray(row.rotation)).multiply(conversion.clone().invert());
 }
}
const telemetrySource=new URLSearchParams(location.search).get('telemetry');
async function intent(value){
 if(telemetrySource)return;
 if(value.reset){crouchRequested=false;requestedSpeed=0;}
 if('crouch'in value)crouchRequested=!!value.crouch;
 if('speed'in value)requestedSpeed=Number(value.speed);
 await fetch(physicsEndpoint+'/intent',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(value)});
}
document.querySelector('#standing').onclick=()=>{history=[];intent({speed:0,circle:!courseMode,reset:true,paused:false});};document.querySelector('#start').onclick=()=>{history=[];intent({speed:2,circle:true,reset:true,paused:false});};document.querySelector('#run').onclick=()=>intent({speed:2,paused:false});document.querySelector('#sprint').onclick=()=>intent({speed:3.5,paused:false});document.querySelector('#stop').onclick=()=>intent({speed:0,paused:false});document.querySelector('#pause').onclick=()=>intent({paused:true,evidenceTag:new URLSearchParams(location.search).get('review')});document.querySelector('#reset').onclick=()=>{history=[];intent({reset:true,paused:true});};
const supportMode=new URLSearchParams(location.search).has('support');
const locomotionMode=supportMode&&new URLSearchParams(location.search).has('locomotion');
const mpcMode=new URLSearchParams(location.search).has('mpc');
const muscleMode=new URLSearchParams(location.search).has('muscle');
const motorMode=new URLSearchParams(location.search).has('motor');
if(supportMode){
 document.querySelector('header small').textContent='LOCAL EXPERIMENT · EXPLICIT WHOLE-BODY SUPPORT';
 document.querySelector('header p').textContent='Measured support and predicted foot placement through one bounded torque solver. Single-step experiment; running remains unimplemented.';
 document.querySelector('footer').textContent='Drag to orbit · Stand and inspect actual support';
 for(const id of ['start','run','sprint','stop'])document.getElementById(id).hidden=true;
 for(const [label,offset] of [['Weight left',.035],['Center balance',0],['Weight right',-.035]]){
  const button=document.createElement('button');button.textContent=label;button.onclick=()=>intent({balance:offset,paused:false});
  document.getElementById('pause').before(button);
 }
 const stepButton=document.createElement('button');stepButton.textContent='Test corrective step';
 stepButton.onclick=()=>intent({step:true,paused:false});document.getElementById('pause').before(stepButton);
 if(locomotionMode){
  document.querySelector('header p').textContent='Private inverse-contact locomotion candidate. Finite joint effort; no root actuation or runtime clips. Not accepted running.';
  for(const button of document.querySelectorAll('aside button'))if(button.textContent.startsWith('Weight')||button.textContent==='Center balance'||button===stepButton)button.hidden=true;
  for(const id of ['run','sprint','stop'])document.getElementById(id).hidden=false;
  const slow=document.createElement('button');slow.textContent='Probe locomotion · 0.3 m/s';slow.onclick=()=>intent({speed:.3,paused:false});document.getElementById('run').before(slow);
 }
}
if(unifiedMode||muscleMode||new URLSearchParams(location.search).has('centroidal')){
 const crouch=document.createElement('button');crouch.textContent='Crouch';crouch.onclick=()=>intent({crouch:true,paused:false});document.getElementById('stop').after(crouch);
 const stand=document.createElement('button');stand.textContent='Stand upright';stand.onclick=()=>intent({crouch:false,paused:false});crouch.after(stand);
}
if(mpcMode){
 document.querySelector('header small').textContent='LOCAL EXPERIMENT · PHYSICAL TRAJECTORY MPC';
 document.querySelector('header p').textContent='One free-root physical controller. Joint-target proposals are tested through actual contact dynamics. Running remains under review.';
 const walk=document.createElement('button');walk.textContent='Walk · 0.5 m/s';walk.onclick=()=>intent({speed:.5,paused:false});document.getElementById('run').before(walk);
}
if(motorMode){
 document.querySelector('header small').textContent='LOCAL EXPERIMENT · CLOSED-LOOP PHYSICAL MOTOR PREDICTION';
 document.querySelector('header p').textContent='Measured-state motor prior, bounded joints and native contact prediction. Private candidate; running and braking remain under review.';
}
if(muscleMode){
 document.querySelector('header small').textContent='LOCAL EXPERIMENT · CONTACT-AWARE MUSCLES';
 document.querySelector('header p').textContent='One measured-state policy. Bounded joint targets, finite muscle effort, physical contact response. Private candidate under review.';
}
if(new URLSearchParams(location.search).has('foundation')){
 document.querySelector('header small').textContent='LOCAL EXPERIMENT · PRESERVED PHYSICAL FEEDBACK';
 document.querySelector('header p').textContent='One feedback actor on the actual bounded body. Finite joint muscles and measured contacts; no runtime clips. Source proportions; cybr appearance preserved. Candidate under review.';
}
if(courseMode){
 camera.position.set(-4,3.3,6);orbit.target.set(2,.95,0);following.set(0,0,0);
 document.getElementById('start').hidden=true;
 document.querySelector('header small').textContent='LOCAL EXPERIMENT · CONTACT & DISTURBANCE COURSE';
 document.querySelector('header p').textContent='Solid steps, vault barriers, a mantle and landing. Physical collision disturbances. The current floor checkpoint has not learned traversal yet.';
}
if(courseMode||unifiedMode){
 const grid=document.createElement('div');grid.style.cssText='display:grid;grid-template-columns:1fr 1fr;gap:4px';
 for(const side of ['left','right','front','back']){const button=document.createElement('button');button.textContent='Ball from '+side;button.onclick=()=>intent({disturbance:side,disturbanceSpeed:3,paused:false});grid.append(button);}
 document.getElementById('pause').before(grid);
 document.querySelector('aside').style.cssText='max-height:calc(100vh -70px);overflow:auto';
}
if(unifiedMode){
 document.querySelector('header small').textContent='LOCAL EXPERIMENT - UNIFIED PHYSICAL CONTACT INTENT';
 document.querySelector('header p').textContent='One feedback actor. Contact goals request jump, vault or climb; these skills remain untrained in the initial checkpoint.';
 const traverse=document.createElement('button');traverse.textContent='Request jump / vault / climb';traverse.onclick=()=>intent({traverse:true,paused:false});document.getElementById('pause').before(traverse);
 document.querySelector('footer').textContent='R run | Shift sprint | S stop | C crouch | Space contextual traversal | 1-4 physical disturbance';
}
addEventListener('keydown',e=>{if(supportMode||e.repeat||telemetrySource)return;if(unifiedMode){if(e.code==='Space'){e.preventDefault();intent({traverse:true,paused:false});}if(e.code==='KeyC')intent({crouch:!crouchRequested,paused:false});const side={'Digit1':'left','Digit2':'right','Digit3':'front','Digit4':'back'}[e.code];if(side)intent({disturbance:side,disturbanceSpeed:3,paused:false});}if(e.key==='Shift')intent({speed:3.5,paused:false});if(e.code==='KeyR')intent({speed:2,paused:false});if(e.code==='KeyS')intent({speed:0,paused:false});});
addEventListener('keyup',e=>{if(!supportMode&&!telemetrySource&&e.key==='Shift'&&requestedSpeed>0)intent({speed:2,paused:false});});
async function poll(){if(telemetrySource)return;try{const response=await fetch(physicsEndpoint+'/state');if(!response.ok)throw Error('Physics endpoint '+response.status);state=await response.json();if(state.time!==undefined&&state.time!==lastTime){lastTime=state.time;history.push({time:state.time,speed:Math.hypot(...state.velocity.slice(0,2)),comSpeed:Math.hypot(...(state.comVelocity||state.velocity).slice(0,2)),com:state.com,pathError:Math.abs(Math.hypot(state.root[0]-(state.circleCenter?.[0]||0),state.root[1]-(state.circleCenter?.[1]||0))-(state.radius||6)),rootHeight:state.root[2],turns:state.turns,requestedSpeed:state.requestedSpeed,footForces:state.names.map((n,i)=>n.includes('foot')?state.forces?.[i]:null).filter(Boolean)});if(history.length>5400)history.shift();}}catch(error){state={...state,connectionError:error.message};}setTimeout(poll,25);}poll();
if(telemetrySource){
 for(const button of document.querySelectorAll('aside button'))button.disabled=true;
 const recorded=await(await fetch(telemetrySource)).json();
 for(const button of document.querySelectorAll('button'))button.disabled=true;
 document.querySelector('header small').textContent='SKELETAL TELEMETRY REVIEW · NOT GAMEPLAY';
 document.querySelector('header p').textContent='Static poses from recorded physical states. This inspection view is not a native-input playtest.';
 if(recorded.scope?.includes('Training-reference')){document.querySelector('header small').textContent='MOCAP REFERENCE INSPECTION · NOT GAMEPLAY';document.querySelector('header p').textContent='Retargeted training data on the bounded rig. This view does not execute physics.';}
 window.telemetryReview={catalog:recorded.frames.map((f,i)=>({index:i,time:f.time,motion:f.sourceMotion,sourceFrame:f.sourceFrame,valid:f.sourcePoseWithinLimits})),select(time){const frame=recorded.frames.reduce((a,b)=>Math.abs(a.time-time)<Math.abs(b.time-time)?a:b);state={...frame,names:recorded.names,paused:true,turns:0,requestedSpeed:frame.requestedSpeed??recorded.requestedSpeed??0,circleCenter:recorded.circleCenter,radius:recorded.radius};return {time:state.time,root:state.root};}};
 window.telemetryReview.select(0);
}
window.policyLab={snapshot:()=>state,history:()=>history,retargetAudit:()=>{
 if(!lastRenderedState)return null;
 const s=lastRenderedState,rows=[];
 for(const [name,child] of Object.entries(segmentChildren)){
  const actual=asset.bones[child].getWorldPosition(new THREE.Vector3()).sub(asset.bones[name].getWorldPosition(new THREE.Vector3()));
  const expected=new THREE.Vector3().fromArray(s.positions[s.names.indexOf(correspondence[child])]).sub(new THREE.Vector3().fromArray(s.positions[s.names.indexOf(correspondence[name])])).applyQuaternion(conversion);
  rows.push({name,axisErrorDegrees:actual.angleTo(expected)*180/Math.PI,length:actual.length()});
 }
 return {...skin.last,segments:rows};
}};
function render(){requestAnimationFrame(render);
 if(state?.positions){
  updateScene(state.scene);pathRing.visible=!!state.circle;
  if(state.circleCenter){pathRing.position.set(state.circleCenter[0],0,-state.circleCenter[1]);pathRing.scale.setScalar((state.radius||6)/radius);}
  const root=new THREE.Vector3(state.root[0],state.root[2],-state.root[1]);
  skin.update(state);
  lastRenderedState=state;
  const position=new THREE.Vector3(root.x,0,root.z),delta=position.clone().sub(following);camera.position.add(delta);orbit.target.add(delta);following.copy(position);
  const forces=state.names.map((n,i)=>n.includes('foot')?Math.max(0,state.forces?.[i]?.[2]||0):null).filter(v=>v!==null);
  document.querySelector('#status').textContent=`${state.paused?'PAUSED':'SIMULATING'}\nTIME ${state.time.toFixed(2)} s\nINTENT ${state.requestedSpeed.toFixed(1)} m/s\nACTUAL ${Math.hypot(...state.velocity.slice(0,2)).toFixed(2)} m/s\nROOT HEIGHT ${state.root[2].toFixed(2)} m\nLAPS ${state.turns.toFixed(3)}\nFEET ${forces.map(f=>Math.round(f)+' N').join(' / ')}\nROOT ACTUATION 0\nANIMATION CLIPS 0`;
 }else document.querySelector('#status').textContent=state?.error||'Local physics initializing…';
 orbit.update();renderer.render(scene,camera);
}function resize(){renderer.setSize(innerWidth,innerHeight);camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();}addEventListener('resize',resize);resize();render();
