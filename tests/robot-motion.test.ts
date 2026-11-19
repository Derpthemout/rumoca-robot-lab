import {beforeAll,it,expect} from 'vitest';
import {readFileSync} from 'node:fs';
import init,* as rumoca from '@cognipilot/rumoca';
import {RobotRuntime,DEFAULT_SERVO,servoSource} from '../src/robotics/motion';
import {validateProfile,type RobotProfile} from '../src/robotics/model';
const profile=validateProfile(JSON.parse(readFileSync('public/robots/microduck.json','utf8').replace(/^\uFEFF/,'')));
const single:RobotProfile={version:1,name:'servo fixture',description:'',joints:[{name:'joint',axis:[0,0,1],position:[0,0,0],limits:[-1,1]}],links:[{name:'base',parent:null,position:[0,0,0],quaternion:[1,0,0,0],joint:'joint'}],sites:[{name:'tip',link:'base',position:[.1,0,0],quaternion:[1,0,0,0]}]};
beforeAll(async()=>{await init({module_or_path:readFileSync(new URL('./rumoca_bind_wasm_bg.wasm',import.meta.resolve('@cognipilot/rumoca')))});});
it('integrates a saturated joint response in Rumoca, preserves time and replays after reset',()=>{
 const runtime=new RobotRuntime(rumoca,single);
 try{
  expect(runtime.setCommand([.8]).q).toEqual([0]);
  const first=runtime.step(.02);expect(first.time).toBeCloseTo(.02,12);expect(first.q[0]).toBeGreaterThan(0);expect(first.q[0]).toBeLessThan(.8);
  expect(Math.abs(first.torque[0])).toBeLessThanOrEqual(DEFAULT_SERVO.maxTorque);
  const path=[first];for(let i=1;i<100;i++)path.push(runtime.step(.02));
  expect(path.at(-1)!.q[0]).toBeCloseTo(.8,4);expect(path.at(-1)!.time).toBeCloseTo(2,10);
  expect(path.every(s=>s.q[0]<=1.00001&&s.q[0]>=-1.00001)).toBe(true);
  runtime.reset();expect(runtime.snapshot().time).toBe(0);expect(runtime.snapshot().velocity).toEqual([0]);runtime.setCommand([.8]);
  for(const expected of path){const actual=runtime.step(.02);expect(actual.q[0]).toBeCloseTo(expected.q[0],9);}
 }finally{runtime.free();}
},60000);
it('rejects invalid steps and commands without advancing or changing the command',()=>{
 const runtime=new RobotRuntime(rumoca,single);try{
  runtime.setCommand([.3]);runtime.step(.02);const before=runtime.snapshot();
  for(const dt of [0,-1,NaN,Infinity,.2])expect(()=>runtime.step(dt)).toThrow();
  for(const command of [[NaN],[2],[],[0,0]])expect(()=>runtime.setCommand(command)).toThrow();
  expect(runtime.snapshot()).toEqual(before);
  expect(()=>runtime.configure({...DEFAULT_SERVO,inertia:0})).toThrow();expect(runtime.snapshot()).toEqual(before);
 }finally{runtime.free();}
});
it('runs Microduck joint motion and moving camera frames with the same generated Modelica',()=>{
 const runtime=new RobotRuntime(rumoca,profile);try{
  const start=runtime.snapshot(),command=start.q.slice();command[6]=.15;
  runtime.setCommand(command);let state=start;for(let i=0;i<50;i++)state=runtime.step(.02);
  expect(state.q[6]).toBeCloseTo(.15,4);expect(state.time).toBeCloseTo(1,10);
  const camera=profile.links.length+profile.sites.findIndex(s=>s.name==='head_camera');
  expect(state.frames[camera].rotation).not.toEqual(start.frames[camera].rotation);
  expect(servoSource(profile,start.q)).toContain('der(velocity6)');
 }finally{runtime.free();}
},120000);
it('a gentler torque limit changes transient response, not the commanded pose',()=>{
 const fast=new RobotRuntime(rumoca,single),slow=new RobotRuntime(rumoca,single);try{
  slow.configure({...DEFAULT_SERVO,maxTorque:.02});fast.setCommand([.8]);slow.setCommand([.8]);
  const a=fast.step(.1),b=slow.step(.1);expect(a.q[0]).toBeGreaterThan(b.q[0]);expect(a.command).toEqual(b.command);
 }finally{fast.free();slow.free();}
});
import {ReachEnvironment,runReachBaseline} from '../src/robotics/reach-environment';
it('reproduces seeded reaching episodes and separates success from timeout',()=>{
  const runtime=new RobotRuntime(rumoca,single);try{
    const env=new ReachEnvironment(runtime),initial=env.reset({seed:42,site:'tip'}),baseline=env.baselineAction();
    const first=[];let transition;do{transition=env.step(baseline.action);first.push(transition);}while(!transition.terminated&&!transition.truncated);
    expect(transition.terminated).toBe(true);expect(transition.truncated).toBe(false);
    expect(()=>env.step(baseline.action)).toThrow('Episode ended');
    expect(env.reset({seed:42,site:'tip'})).toEqual(initial);
    for(const expected of first){const actual=env.step(baseline.action);expect(actual.reward).toBeCloseTo(expected.reward,9);expect(actual.observation.q[0]).toBeCloseTo(expected.observation.q[0],9);}
    env.reset({seed:42,site:'tip',goal:[10,10,10],horizon:1});const timeout=env.step([0]);expect(timeout.terminated).toBe(false);expect(timeout.truncated).toBe(true);
    const before=runtime.snapshot();expect(()=>env.reset({seed:-1,site:'tip'})).toThrow();expect(runtime.snapshot()).toEqual(before);
  }finally{runtime.free();}
},60000);
it('rejects invalid training actions without consuming time or poisoning the episode',()=>{
  const runtime=new RobotRuntime(rumoca,single);try{
    const env=new ReachEnvironment(runtime);env.reset({seed:8,site:'tip'});const before=runtime.snapshot();
    for(const action of [[NaN],[2],[],[0,0]])expect(()=>env.step(action)).toThrow();expect(runtime.snapshot()).toEqual(before);
    expect(env.step([0]).observation.time).toBeCloseTo(.02,12);
  }finally{runtime.free();}
});
it('records a real Microduck reach baseline with action, reward and termination metadata',()=>{
  const runtime=new RobotRuntime(rumoca,profile);try{
    const rollout=runReachBaseline(runtime,{seed:42,site:'left_foot'});
    expect(rollout.format).toBe('rumoca-reach-rollout');expect(rollout.policy).toContain('not learned');expect(rollout.summary.terminated).toBe(true);
    expect(rollout.summary.finalDistance).toBeLessThan(.001);expect(rollout.transitions.every(t=>Number.isFinite(t.reward)&&t.action.length===14)).toBe(true);
  }finally{runtime.free();}
},120000);
import {BoxGeometry,Mesh,MeshBasicMaterial} from 'three';
import {sampleSensor,mountFrame} from '../src/robotics/sensors';
import {OccupancyMap} from '../src/robotics/mapping';
it('maps real raycasts from a Rumoca Microduck frame and plans the default route',()=>{
 const runtime=new RobotRuntime(rumoca,profile);
 const obstacles=[[[.15,.2,.24],[.4,0,.12]],[[.1,.15,.16],[.25,.35,.08]],[[1.5,.04,.4],[0,.65,.2]]].map(([size,p])=>{const mesh=new Mesh(new BoxGeometry(...size),new MeshBasicMaterial());mesh.position.set(p[0],p[1],p[2]);mesh.updateMatrixWorld();return mesh;});
 try{
   const root=profile.links.findIndex(l=>l.parent===null),frame=mountFrame({frame:runtime.snapshot().frames[root],offset:[0,0,.1],rpy:[0,0,0]});
   const scan=sampleSensor(frame,'lidar',obstacles),map=new OccupancyMap();map.integrate(scan);
   expect(scan.points.length).toBeGreaterThan(0);expect(map.plan([0,0],[-.5,0],.1).length).toBeGreaterThan(1);
 }finally{runtime.free();obstacles.forEach(m=>{m.geometry.dispose();m.material.dispose();});}
},120000);
import {sensorSweep} from '../src/robotics/sensor-sweep';
import {MappingSession} from '../src/robotics/localization';
it('runs a real Rumoca sensor sweep through raycasts and estimated-pose mapping',()=>{
 const sweep=sensorSweep(rumoca),mapper=new MappingSession(new OccupancyMap());mapper.reset('estimated');
 const obstacles=[[[.15,.2,.24],[.4,0,.12]],[[.1,.15,.16],[.25,.35,.08]],[[1.5,.04,.4],[0,.65,.2]],[[.04,1.5,.4],[-.8,0,.2]],[[.04,1.5,.4],[.8,0,.2]],[[1.6,.04,.4],[0,-.75,.2]]].map(([size,p])=>{const mesh=new Mesh(new BoxGeometry(...size),new MeshBasicMaterial());mesh.position.set(p[0],p[1],p[2]);mesh.updateMatrixWorld();return mesh;});
 try{
  for(const sample of sweep.samples)mapper.integrate(sampleSensor(sample.frame,'lidar',obstacles),sample.time);
  expect(mapper.history.at(-1)!.time).toBeCloseTo(2,9);expect(mapper.map.scans).toBe(41);expect(sweep.samples.at(-1)!.time).toBeCloseTo(2,9);expect(sweep.samples.at(-1)!.frame.position[0]).toBeCloseTo(.06,9);
  const last=mapper.history.at(-1)!;expect(last.positionError).toBeLessThan(.025);expect(last.yawError).toBeLessThan(.025);expect(mapper.map.plan([0,0],[-.5,0],.1).length).toBeGreaterThan(1);
  console.log(`Sweep: ${mapper.map.scans} scans; drift ${(last.positionError*1000).toFixed(2)} mm / ${(last.yawError*180/Math.PI).toFixed(3)} deg`);
 }finally{obstacles.forEach(m=>{m.geometry.dispose();m.material.dispose();});}
},120000);
