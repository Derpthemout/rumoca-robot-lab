import type {SteppingState} from './stepping';
import {MappingSession} from './localization';
import {OccupancyMap,type Point2} from './mapping';
import * as THREE from 'three';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {validateProfile,type RobotProfile,type Frame,type Vec3} from './model';
import {mountFrame,sampleSensor} from './sensors';
import {DEFAULT_SERVO,homePose,validatePose,validateServo,type RobotSnapshot,type ServoParameters} from './motion';
import type {runReachBaseline} from './reach-environment';
import './lab.css';
const $=<T extends HTMLElement>(id:string)=>document.getElementById(id) as T;
const worker=new Worker(new URL('./robot.worker.ts',import.meta.url),{type:'module'});
let sequence=0;const pending=new Map<number,{resolve:(v:any)=>void;reject:(e:Error)=>void}>();
worker.onmessage=({data})=>{const request=pending.get(data.id);if(!request)return;pending.delete(data.id);data.error?request.reject(Error(data.error)):request.resolve(data.result);};
worker.onerror=()=>{for(const r of pending.values())r.reject(Error('Robot worker failed. Reload the page.'));pending.clear();};
const call=(type:string,data:Record<string,unknown>={})=>new Promise<any>((resolve,reject)=>{const id=++sequence;pending.set(id,{resolve,reject});worker.postMessage({id,type,...data});});
const scene=new THREE.Scene();scene.background=new THREE.Color('#111923');
const view=$('viewport'),renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));view.append(renderer.domElement);
const camera=new THREE.PerspectiveCamera(45,1,.001,20);camera.up.set(0,0,1);camera.position.set(.5,-.65,.43);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.target.set(0,0,.12);orbit.enableDamping=true;
scene.add(new THREE.HemisphereLight(0xe5f7ff,0x4d6171,2));const light=new THREE.DirectionalLight(0xffffff,2);light.position.set(1,-1,2);scene.add(light);
const grid=new THREE.GridHelper(2,40,0x4b6676,0x263948);grid.rotation.x=Math.PI/2;scene.add(grid);
const obstacles:THREE.Object3D[]=[];
function box(size:Vec3,position:Vec3,color:number){const mesh=new THREE.Mesh(new THREE.BoxGeometry(...size),new THREE.MeshStandardMaterial({color}));mesh.position.set(...position);scene.add(mesh);obstacles.push(mesh);}
box([2,2,.02],[0,0,-.01],0x1e2c36);box([.15,.2,.24],[.4,0,.12],0x3e6675);box([.1,.15,.16],[.25,.35,.08],0x6d7765);box([1.5,.04,.4],[0,.65,.2],0x314958);
box([.04,1.5,.4],[-.8,0,.2],0x314958);box([.04,1.5,.4],[.8,0,.2],0x314958);box([1.6,.04,.4],[0,-.75,.2],0x314958);
const robot=new THREE.Group();scene.add(robot);
const visuals=new THREE.Group();scene.add(visuals);
const meshCache=new Map<string,THREE.BufferGeometry>();
async function loadVisuals(p:RobotProfile){
 const loader=new STLLoader();
 const geometries=await Promise.all((p.visuals??[]).map(async v=>{if(!meshCache.has(v.mesh)){const geometry=await loader.loadAsync(new URL('robots/microduck-meshes/'+v.mesh,new URL(import.meta.env.BASE_URL,location.href)).href);meshCache.set(v.mesh,geometry);}return meshCache.get(v.mesh)!;}));
 visuals.traverse(o=>{if(o instanceof THREE.Mesh)(o.material as THREE.Material).dispose();});visuals.clear();
 (p.visuals??[]).forEach((v,i)=>{const body=new THREE.Group();body.userData.link=p.links.findIndex(l=>l.name===v.link);body.matrixAutoUpdate=false;
 const mesh=new THREE.Mesh(geometries[i],new THREE.MeshStandardMaterial({color:new THREE.Color(...v.rgba.slice(0,3) as [number,number,number]),roughness:.65}));mesh.position.set(...v.position);const [w,x,y,z]=v.quaternion;mesh.quaternion.set(x,y,z,w).normalize();body.add(mesh);visuals.add(body);});
 robot.visible=!p.visuals?.length;
}
const target=new THREE.Mesh(new THREE.SphereGeometry(.008,16,12),new THREE.MeshBasicMaterial({color:0xffcc69}));scene.add(target);
const sensorAxes=new THREE.AxesHelper(.07);scene.add(sensorAxes);
const cloud=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({color:0x49e3d1,size:.003}));scene.add(cloud);
const linkMaterial=new THREE.LineBasicMaterial({color:0x58cfc0});const jointMaterial=new THREE.MeshStandardMaterial({color:0xf4c87b});

let profile:RobotProfile,q:number[]=[],commands:number[]=[],frames:Frame[]=[],source='',motionSource='',compilerVersion='';
let steppingRunning=false,steppingPreview=false;
let steppingRecords:Omit<SteppingState,'robot'>[]=[];let steppingJoints:{q:number[];velocity:number[];torque:number[]}[]=[];let contactModel='',steppingServo='';
const footprints=[0,1].map(()=>{const m=new THREE.Mesh(new THREE.BoxGeometry(.06,.036,.001),new THREE.MeshBasicMaterial({color:0x4ee0c5,transparent:true,opacity:.55}));m.visible=false;scene.add(m);return m;});
let busy=false,running=false,ready=false,simulationTime=0,lastSensorTime=-Infinity;
let servoParameters:ServoParameters={...DEFAULT_SERVO};
type Capture=ReturnType<typeof sampleSensor>&{simulationTime:number};
let lastCapture:Capture|undefined,lastRollout:ReturnType<typeof runReachBaseline>|undefined;
type RecordedState=Omit<RobotSnapshot,'frames'>;
let recording:RecordedState[]=[],recordedSensors:Capture[]=[];
const occupancy=new OccupancyMap();const mapping=new MappingSession(occupancy);let sweepSource='';let plannedRoute:Point2[]=[];
const MAX_RECORDING=3000;
const parameterFields=['inertia','kp','kd','damping','maxTorque'] as const;
const axes=['tx','ty','tz','sx','sy','sz','sr','sp','sw'];
const simulated=()=>$<HTMLInputElement>('animate').checked;
function numbers(ids:string[]):Vec3 {
  const values=ids.map(id=>{const input=$<HTMLInputElement>(id);if(input.value.trim()==='')throw Error('Enter all coordinates');return Number(input.value);});
  if(values.some(v=>!Number.isFinite(v)))throw Error('Enter finite coordinates');return values as Vec3;
}
function controls(){
  document.querySelectorAll<HTMLButtonElement|HTMLInputElement|HTMLSelectElement>('button,input,select').forEach(e=>{e.disabled=busy||running||!ready||steppingPreview;});
  $<HTMLButtonElement>('pause').disabled=!running;
  $<HTMLButtonElement>('rollout').disabled=busy||running||!lastRollout;
  $<HTMLButtonElement>('recording').disabled=busy||running||recording.length===0;
  $<HTMLButtonElement>('startStepping').disabled=busy||running||!ready;
  $<HTMLButtonElement>('stopStepping').disabled=!steppingRunning;
  $<HTMLButtonElement>('exitStepping').disabled=busy||running||!steppingPreview;
  $<HTMLButtonElement>('exportStepping').disabled=busy||running||!steppingRecords.length;
  if(steppingPreview){$<HTMLButtonElement>('rollout').disabled=true;$<HTMLButtonElement>('recording').disabled=true;}
  if(!ready&&!busy)$<HTMLButtonElement>('import').disabled=false;
}
async function action(fn:()=>Promise<void>){
  if(busy||running)return;busy=true;controls();
  try{await fn();}catch(error){$('status').textContent=String(error);}finally{busy=false;controls();}
}
function invalidateCapture(){
  lastCapture=undefined;cloud.geometry.dispose();cloud.geometry=new THREE.BufferGeometry();
  const canvas=$<HTMLCanvasElement>('sensor-view');canvas.getContext('2d')!.clearRect(0,0,canvas.width,canvas.height);
  $('sensor-status').textContent='Capture to refresh the sensor data.';
}
function clearRecords(){invalidateCapture();recording=[];recordedSensors=[];lastRollout=undefined;lastSensorTime=-Infinity;$('recording-status').textContent='No motion recorded yet.';}
function createSkeleton(){
  robot.traverse(o=>{if(o instanceof THREE.Mesh||o instanceof THREE.Line)o.geometry.dispose();});robot.clear();
  profile.links.forEach((link,i)=>{
    const point=new THREE.Mesh(new THREE.SphereGeometry(link.joint?.length ? .0035 : .006,12,8),jointMaterial);point.userData.frame=i;robot.add(point);
    const parent=profile.links.findIndex(l=>l.name===link.parent);
    if(parent>=0){const geometry=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]);const line=new THREE.Line(geometry,linkMaterial);line.userData={from:parent,to:i};robot.add(line);}
  });
}
function update(result:RobotSnapshot&{motionSource?:string}){
  q=result.q;commands=result.command??q;frames=result.frames;simulationTime=result.time??0;
  if(result.motionSource)motionSource=result.motionSource;
  visuals.children.forEach(body=>{const f=frames[body.userData.link],r=f.rotation;body.matrix.set(r[0],r[1],r[2],f.position[0],r[3],r[4],r[5],f.position[1],r[6],r[7],r[8],f.position[2],0,0,0,1);body.matrixWorldNeedsUpdate=true;});
  if(robot.visible)robot.children.forEach(o=>{
    if(o instanceof THREE.Mesh)o.position.set(...frames[o.userData.frame].position);
    else if(o instanceof THREE.Line){const positions=o.geometry.getAttribute('position');positions.setXYZ(0,...frames[o.userData.from].position);positions.setXYZ(1,...frames[o.userData.to].position);positions.needsUpdate=true;o.geometry.computeBoundingSphere();}
  });
  document.querySelectorAll<HTMLInputElement>('#joints input').forEach((input,i)=>{
    input.value=String(commands[i]);input.previousElementSibling!.textContent=`${profile.joints[i].name} · ${(q[i]*180/Math.PI).toFixed(1)}° → ${(commands[i]*180/Math.PI).toFixed(1)}°`;
  });
  $('time').textContent=`${simulationTime.toFixed(2)} s`;
  $('tracking').textContent=`${(Math.max(0,...q.map((v,i)=>Math.abs(commands[i]-v)))*180/Math.PI).toFixed(2)}°`;
  $('torque').textContent=`${Math.max(0,...result.torque.map(Math.abs)).toFixed(3)} Nm`;
  if(result.mode!=='servo'||!$<HTMLInputElement>('liveSensor').checked)invalidateCapture();updateMount();
}
function setTarget(position?:Vec3){
  const index=profile.sites.findIndex(s=>s.name===$<HTMLSelectElement>('effector').value);if(index<0)return;
  const p=position??frames[profile.links.length+index].position;
  ['tx','ty','tz'].forEach((id,i)=>{$<HTMLInputElement>(id).value=p[i].toFixed(5);});target.position.set(...p);
}
function updateMount(){
  if(!frames.length)return;const index=Number($<HTMLSelectElement>('mount').value);if(!frames[index])return;
  const f=mountFrame({frame:frames[index],offset:numbers(['sx','sy','sz']),rpy:numbers(['sr','sp','sw']).map(v=>v*Math.PI/180) as Vec3});
  const r=f.rotation;sensorAxes.matrixAutoUpdate=false;sensorAxes.matrix.set(r[0],r[1],r[2],f.position[0],r[3],r[4],r[5],f.position[1],r[6],r[7],r[8],f.position[2],0,0,0,1);sensorAxes.matrixWorldNeedsUpdate=true;return f;
}
async function load(value:unknown){
  const next=validateProfile(value),result=await call('init',{profile:next,base:new URL(import.meta.env.BASE_URL,location.href).href});
  mapping.reset();sweepSource='';plannedRoute=[];drawMap();profile=next;source=result.source;motionSource='';compilerVersion=result.compiler;servoParameters={...DEFAULT_SERVO};
  parameterFields.forEach(id=>{$<HTMLInputElement>(id).value=String(servoParameters[id]);});
  ['sx','sy','sz','sr','sp','sw'].forEach(id=>{$<HTMLInputElement>(id).value='0';});
  let visualWarning='';
  try{await loadVisuals(profile);}catch{visuals.clear();robot.visible=true;visualWarning=' Meshes unavailable; using skeleton view.';}
  createSkeleton();$('name').textContent=profile.name;$('legend-name').textContent=`${profile.name} · motion prototype`;
  $('description').textContent=profile.description;$('engine').textContent=`Rumoca ${compilerVersion} · Modelica → WASM`;
  $('joints').replaceChildren();profile.joints.forEach((j,i)=>{
    const label=document.createElement('label'),name=document.createElement('span'),input=document.createElement('input');input.type='range';input.min=String(j.limits[0]);input.max=String(j.limits[1]);input.step='.002';
    input.onchange=()=>action(async()=>{const next=commands.slice();next[i]=Number(input.value);const result=await call(simulated()?'target':'pose',{q:next});if(!simulated()){clearRecords();motionSource='';}update(result);$('status').textContent=simulated()?'Joint target set. Press Run or Step to move.':'Pose updated instantly.';});
    label.append(name,input);$('joints').append(label);
  });
  const options=(select:HTMLSelectElement,items:{name:string;value:string}[])=>select.replaceChildren(...items.map(item=>{const o=document.createElement('option');o.textContent=item.name;o.value=item.value;return o;}));
  options($<HTMLSelectElement>('effector'),profile.sites.map(s=>({name:s.name,value:s.name})));
  options($<HTMLSelectElement>('mount'),[...profile.links,...profile.sites].map((s,i)=>({name:s.name,value:String(i)})));
  $<HTMLSelectElement>('effector').value=profile.sites.some(s=>s.name==='left_foot')?'left_foot':profile.sites[0]?.name??'';
  const sensor=profile.sites.findIndex(s=>s.name==='head_camera');$<HTMLSelectElement>('mount').value=String(sensor<0?0:profile.links.length+sensor);
  clearRecords();update(result);setTarget();ready=true;$('status').textContent='Ready. Set a joint or IK target, then press Run.'+visualWarning;
}
function captureSensor(){
  const frame=updateMount();if(!frame)return;scene.updateMatrixWorld(true);
  lastCapture={...sampleSensor(frame,$<HTMLSelectElement>('sensor').value as 'depth'|'lidar',obstacles),simulationTime};lastSensorTime=simulationTime;
  if(lastCapture.kind==='lidar'&&$<HTMLInputElement>('mapEnabled').checked){
    try{if(sweepSource)throw Error('Clear map before using the mounted sensor after a sweep.');const record=mapping.integrate(lastCapture,lastCapture.simulationTime);showTracking(record);plannedRoute=[];drawMap();$('map-status').textContent=`${occupancy.scans} scans · 5 cm cells · ${mapping.mode} poses · gray = unknown`;}catch(error){$('map-status').textContent=String(error);}
  }
  cloud.geometry.dispose();cloud.geometry=new THREE.BufferGeometry().setFromPoints(lastCapture.points.map(p=>new THREE.Vector3(...p)));
  const canvas=$<HTMLCanvasElement>('sensor-view'),ctx=canvas.getContext('2d')!;ctx.fillStyle='#070e17';ctx.fillRect(0,0,canvas.width,canvas.height);
  if(lastCapture.kind==='depth')lastCapture.ranges.forEach((distance,i)=>{const value=distance===null?0:Math.round(255*(1-distance/lastCapture!.maxRange));ctx.fillStyle=`rgb(${value},${value},${value})`;ctx.fillRect((i%48)*6,Math.floor(i/48)*6,6,6);});
  else{ctx.fillStyle='#4de1cf';lastCapture.ranges.forEach((distance,i)=>{if(distance===null)return;const a=2*Math.PI*i/128,scale=85/lastCapture!.maxRange;ctx.fillRect(144+Math.cos(a)*distance*scale,96-Math.sin(a)*distance*scale,3,3);});}
  $('sensor-status').textContent=`t=${simulationTime.toFixed(2)} s · ${lastCapture.points.length} returns · ideal radial range, max 3 m · +X forward; no self-occlusion.`;
}
async function stepOnce(){
  if(recording.length>=MAX_RECORDING)throw Error('Recording limit reached (3,000 steps). Export it, then reset the episode.');
  const result:RobotSnapshot&{motionSource:string}=await call('step',{dt:.02});update(result);
  const {frames:unused,motionSource:unusedSource,...sample}=result;recording.push(sample);
  if($<HTMLInputElement>('liveSensor').checked&&simulationTime-lastSensorTime>=.2-1e-9){captureSensor();if(lastCapture)recordedSensors.push(lastCapture);}
  $('recording-status').textContent=`${recording.length} steps · ${recordedSensors.length} sensor captures · dt=20 ms`;
  return result;
}
$('play').onclick=async()=>{
  if(busy||running||!ready)return;running=true;controls();let failed=false;
  $('status').textContent='Running Rumoca joint dynamics. Pause to edit targets.';
  try{while(running){const started=performance.now();await stepOnce();if(running)await new Promise(resolve=>setTimeout(resolve,Math.max(0,20-(performance.now()-started))));}}
  catch(error){failed=true;$('status').textContent=String(error);}
  finally{running=false;controls();if(!failed)$('status').textContent=`Paused at ${simulationTime.toFixed(2)} s.`;}
};
$('pause').onclick=()=>{running=false;$('status').textContent='Pausing after the current simulation step…';};
$('step').onclick=()=>action(async()=>{await stepOnce();$('status').textContent=`Advanced to ${simulationTime.toFixed(2)} s.`;});
$('resetMotion').onclick=()=>action(async()=>{const result=await call('resetMotion');clearRecords();update(result);setTarget();$('status').textContent='Episode reset: initial pose, zero velocity, time zero.';});
$('configureMotion').onclick=()=>action(async()=>{
  const parameters={...servoParameters};parameterFields.forEach(id=>{parameters[id]=Number($<HTMLInputElement>(id).value);});validateServo(parameters);
  const result=await call('configureMotion',{parameters});servoParameters=parameters;clearRecords();update(result);setTarget();$('status').textContent='Servo parameters applied. New episode starts from this pose.';
});
$<HTMLSelectElement>('effector').onchange=()=>setTarget();
axes.forEach(id=>{$<HTMLInputElement>(id).onchange=()=>{try{target.position.set(...numbers(['tx','ty','tz']));updateMount();invalidateCapture();}catch(error){$('status').textContent=String(error);}};});
$<HTMLSelectElement>('mount').onchange=()=>{updateMount();invalidateCapture();};
$<HTMLSelectElement>('sensor').onchange=invalidateCapture;
$('solve').onclick=()=>action(async()=>{
  $('status').textContent='Solving bounded IK using Rumoca evaluations…';
  const result=await call('solve',{site:$<HTMLSelectElement>('effector').value,target:numbers(['tx','ty','tz']),animate:simulated()});
  if(!simulated()){clearRecords();motionSource='';}update(result);const s=result.solution;
  $('status').textContent=`${s.converged?'IK target found':'Closest IK target; requested position not reached'} · ${(s.error*1000).toFixed(2)} mm residual · ${s.iterations} iterations.${simulated()?' Press Run to move.':''}`;
});
$('reset').onclick=()=>action(async()=>{const result=await call('pose',{q:homePose(profile)});clearRecords();motionSource='';update(result);setTarget();$('status').textContent='Neutral pose restored; simulation clock reset.';});
$('capture').onclick=()=>action(async()=>captureSensor());
$('benchmark').onclick=()=>action(async()=>{
  const seed=Number($<HTMLInputElement>('seed').value);if(!Number.isInteger(seed)||seed<0||seed>0xffffffff)throw Error('Seed must be a uint32');
  $('status').textContent='Running a seeded reaching episode in Rumoca…';
  const result=await call('benchmark',{seed,site:$<HTMLSelectElement>('effector').value});clearRecords();lastRollout=result.rollout;update(result);setTarget(lastRollout!.initial.goal);
  const summary=lastRollout!.summary;$('status').textContent=`Reach baseline ${summary.terminated?'succeeded':'hit the time limit'} · ${(summary.finalDistance*1000).toFixed(2)} mm final error · ${summary.steps} steps. Rollout ready to export.`;
});
function download(name:string,data:string,type='application/json'){
  const url=URL.createObjectURL(new Blob([data],{type})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
$('model').onclick=()=>download(motionSource?'RobotServos.mo':'RobotKinematics.mo',motionSource||source,'text/plain');
$('rollout').onclick=()=>{if(lastRollout)download('rumoca-reach-rollout.json',JSON.stringify({...lastRollout,profile,compilerVersion,servoParameters,modelicaSource:motionSource},null,2));};
$('recording').onclick=()=>download('rumoca-motion-recording.json',JSON.stringify({format:'rumoca-motion-recording',version:1,profile,compilerVersion,servoParameters,modelicaSource:motionSource,units:{time:'s',q:'rad',velocity:'rad/s',torque:'Nm'},dt:.02,states:recording,sensors:recordedSensors},null,2));
$('export').onclick=()=>action(async()=>{
  // Compliant stops may briefly overshoot; an editable pose must obey limits.
  const pose=q.map((v,i)=>Math.max(profile.joints[i].limits[0],Math.min(v,profile.joints[i].limits[1])));
  download('robot-experiment.json',JSON.stringify({version:2,profile,q:pose,measuredQ:q,command:commands,servoParameters,sensor:lastCapture??null,mount:{kind:$<HTMLSelectElement>('sensor').value,frame:Number($<HTMLSelectElement>('mount').value),offset:numbers(['sx','sy','sz']),rpyDegrees:numbers(['sr','sp','sw'])},restoreSemantics:'pose clamped to joint limits and settings only; time and velocities reset; measuredQ preserves the captured pose'},null,2));
});
$('import').onclick=()=>$<HTMLInputElement>('file').click();
$<HTMLInputElement>('file').onchange=()=>action(async()=>{
  const file=$<HTMLInputElement>('file').files?.[0];if(!file)return;if(file.size>2000000)throw Error('Profile limit: 2 MB');
  const data=JSON.parse(await file.text()),p=validateProfile(data.profile??data);
  if(data.q)validatePose(p,data.q);if(data.command)validatePose(p,data.command);if(data.servoParameters)validateServo(data.servoParameters);
  await load(p);if(data.q)update(await call('pose',{q:data.q}));
  if(data.servoParameters){servoParameters=validateServo(data.servoParameters);update(await call('configureMotion',{parameters:servoParameters}));parameterFields.forEach(id=>{$<HTMLInputElement>(id).value=String(servoParameters[id]);});}
  if(data.command)update(await call('target',{q:data.command}));
  if(data.mount&&Number.isInteger(data.mount.frame)&&data.mount.frame>=0&&data.mount.frame<frames.length&&[data.mount.offset,data.mount.rpyDegrees].every(v=>Array.isArray(v)&&v.length===3&&v.every(Number.isFinite))){
    $<HTMLSelectElement>('mount').value=String(data.mount.frame);if(['depth','lidar'].includes(data.mount.kind))$<HTMLSelectElement>('sensor').value=data.mount.kind;
    ['sx','sy','sz'].forEach((id,i)=>{$<HTMLInputElement>(id).value=String(data.mount.offset[i]);});['sr','sp','sw'].forEach((id,i)=>{$<HTMLInputElement>(id).value=String(data.mount.rpyDegrees[i]);});updateMount();
  }
  setTarget();$<HTMLInputElement>('file').value='';$('status').textContent='Restored pose and settings at time zero. Transient velocities are not resumed.';
});
$('startStepping').onclick=()=>action(async()=>{
  const initial:SteppingState&{contactSource:string}=await call('startStepping');contactModel=initial.contactSource;steppingServo='';steppingRecords=[];steppingJoints=[];steppingPreview=true;steppingRunning=true;clearRecords();mapping.reset();sweepSource='';plannedRoute=[];drawMap();controls();
  const display=(s:SteppingState)=>{update(s.robot);footprints.forEach((m,i)=>{m.visible=true;m.position.set(s.feet[i][0],s.feet[i][1],.001);(m.material as THREE.MeshBasicMaterial).color.setHex((i===0?s.base.leftN:s.base.rightN)>.3?0x4ee0c5:0xffcc69);});
    $('stepping-status').textContent=`${s.phase} · ${s.steps}/4 steps · t=${s.base.time.toFixed(2)} s · contact ${s.base.leftN.toFixed(1)} / ${s.base.rightN.toFixed(1)} N · support-box margin ${(s.supportMargin*1000).toFixed(1)} mm · IK ${(s.ikError*1000).toFixed(1)} mm`;
  };
  display(initial);
  try{while(steppingRunning){const started=performance.now(),s:SteppingState&{servoSource:string}=await call('stepGait');steppingServo=s.servoSource;display(s);
    const {robot,servoSource:unusedServo,...state}=s;steppingRecords.push(state);steppingJoints.push({q:robot.q,velocity:robot.velocity,torque:robot.torque});
    if(s.done){$('status').textContent='Four reduced-order steps completed. Return to joint lab to resume normal controls.';break;}
    if(steppingRecords.length>=1000)throw Error('Stepping time limit reached');
    await new Promise(resolve=>setTimeout(resolve,Math.max(0,20-(performance.now()-started))));
  }}catch(error){$('stepping-status').textContent=`Stopped: ${String(error)}`;throw error;}
  finally{steppingRunning=false;controls();}
});
$('stopStepping').onclick=()=>{steppingRunning=false;$('status').textContent='Stopping after the current contact step.';};
$('exitStepping').onclick=()=>action(async()=>{const result=await call('exitStepping');steppingPreview=false;footprints.forEach(m=>m.visible=false);update(result);setTarget();$('status').textContent='Returned to the saved joint-lab state.';});
$('exportStepping').onclick=()=>download('rumoca-stepping.json',JSON.stringify({format:'rumoca-reduced-order-stepping',version:1,profile,compilerVersion,dt:.02,assumptions:{massKg:.6,stiffnessNPerM:1500,dampingNsM:20,friction:.6,orientation:'constrained upright',supportHalfSizeMetres:[.03,.018],model:'separate joint servos and centroidal contact body; uncalibrated; no route execution'},contactModel,servoModel:steppingServo,states:steppingRecords,joints:steppingJoints},null,2));
function showTracking(record:MappingSession['history'][number]|undefined){
  $('tracking-status').textContent=record?`Position drift ${(record.positionError*1000).toFixed(1)} mm · yaw drift ${(record.yawError*180/Math.PI).toFixed(2)}° · residual ${(record.rmse*1000).toFixed(1)} mm · ${(record.overlap*100).toFixed(0)}% overlap`:'Ground-truth mapping. Select LiDAR odometry to estimate motion.';
}
$<HTMLSelectElement>('poseMode').onchange=()=>{mapping.reset($<HTMLSelectElement>('poseMode').value as 'truth'|'estimated');sweepSource='';plannedRoute=[];drawMap();$('tracking-status').textContent='Map reset. First valid scan establishes the pose anchor.';};
$('sweep').onclick=()=>action(async()=>{
  const sweep=await call('sensorSweep');mapping.reset('estimated');sweepSource=sweep.source;$<HTMLSelectElement>('poseMode').value='estimated';plannedRoute=[];
  scene.updateMatrixWorld(true);
  try{for(const sample of sweep.samples){const scan=sampleSensor(sample.frame,'lidar',obstacles);showTracking(mapping.integrate(scan,sample.time));drawMap();await new Promise(resolve=>setTimeout(resolve,20));}
    $('map-status').textContent=`${occupancy.scans} scans from a Rumoca virtual sensor sweep. Export map includes drift history and Modelica. Clear map before switching back to the mounted sensor.`;
  }catch(error){$('map-status').textContent=String(error);throw error;}
});
function drawMap(){
  const canvas=$<HTMLCanvasElement>('map-view'),ctx=canvas.getContext('2d')!,scale=canvas.width/occupancy.size;
  for(let y=0;y<occupancy.size;y++)for(let x=0;x<occupancy.size;x++){const v=occupancy.evidence[y*occupancy.size+x];ctx.fillStyle=v>0?'#111923':v<0?'#bcd9d4':'#53616e';ctx.fillRect(x*scale,(occupancy.size-1-y)*scale,Math.ceil(scale),Math.ceil(scale));}
  const pixel=(p:Point2)=>[(p[0]-occupancy.origin[0])/occupancy.resolution*scale,canvas.height-(p[1]-occupancy.origin[1])/occupancy.resolution*scale];
  if(plannedRoute.length){ctx.strokeStyle='#ffcc69';ctx.lineWidth=2;ctx.beginPath();plannedRoute.forEach((p,i)=>{const [x,y]=pixel(p);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.stroke();}
  for(const [key,color] of [['truth','#49e3d1'],['estimate','#f17fca']] as const){ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();mapping.history.forEach((record,i)=>{const p=record[key],v=pixel([p.x,p.y]);if(i)ctx.lineTo(v[0],v[1]);else ctx.moveTo(v[0],v[1]);});ctx.stroke();}
  ctx.fillStyle='#49e3d1';const [x,y]=pixel([0,0]);ctx.fillRect(x-2,y-2,4,4);
}
$('mapSensor').onclick=()=>action(async()=>{
  $<HTMLSelectElement>('sensor').value='lidar';$<HTMLSelectElement>('mount').value=String(profile.links.findIndex(l=>l.parent===null));
  ['sx','sy','sr','sp','sw'].forEach(id=>{$<HTMLInputElement>(id).value='0';});$<HTMLInputElement>('sz').value='.1';
  mapping.reset();sweepSource='';plannedRoute=[];drawMap();updateMount();invalidateCapture();$('map-status').textContent='Level LiDAR selected, 0.1 m above root. Click Capture sensor.';
});
$('clearMap').onclick=()=>action(async()=>{mapping.reset();sweepSource='';plannedRoute=[];drawMap();$('map-status').textContent='Map cleared. Capture a level LiDAR scan.';});
$('planRoute').onclick=()=>action(async()=>{
  plannedRoute=[];drawMap();
  const read=(id:string)=>{const v=$<HTMLInputElement>(id).value;if(!v.trim()||!Number.isFinite(Number(v)))throw Error('Enter finite route settings');return Number(v);};
  try{plannedRoute=occupancy.plan([read('routeSX'),read('routeSY')],[read('routeGX'),read('routeGY')],read('routeRadius'));drawMap();$('map-status').textContent=`Route preview: ${plannedRoute.length} cell centers; unknown cells blocked. No walking execution.`;}catch(error){$('map-status').textContent=String(error);}
});
['routeSX','routeSY','routeGX','routeGY','routeRadius'].forEach(id=>{$<HTMLInputElement>(id).onchange=()=>{plannedRoute=[];drawMap();$('map-status').textContent='Route settings changed. Plan again.';};});
$('exportMap').onclick=()=>download('robot-occupancy-map.json',JSON.stringify({...mapping.export(),sweepModelica:sweepSource||undefined,route:plannedRoute,footprintRadius:plannedRoute.length?Number($<HTMLInputElement>('routeRadius').value):null,routeSemantics:'cell-center path on a horizontal sensor slice; no locomotion execution'},null,2));
new ResizeObserver(()=>{renderer.setSize(view.clientWidth,view.clientHeight);camera.aspect=view.clientWidth/view.clientHeight;camera.updateProjectionMatrix();}).observe(view);
renderer.setAnimationLoop(()=>{orbit.update();renderer.render(scene,camera);});
window.addEventListener('pagehide',()=>{running=false;worker.terminate();renderer.dispose();});
void action(async()=>{const response=await fetch(new URL('robots/microduck.json',new URL(import.meta.env.BASE_URL,location.href)));if(!response.ok)throw Error('Microduck model unavailable');await load(await response.json());});
