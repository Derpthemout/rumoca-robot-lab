import {beforeAll,it,expect} from 'vitest';
import {readFileSync} from 'node:fs';
import init,* as rumoca from '@cognipilot/rumoca';
import {modelicaSource,validateProfile,evaluate,chain,type RobotProfile} from '../src/robotics/model';
import {solveIK} from '../src/robotics/ik';
const profile=validateProfile(JSON.parse(readFileSync('public/robots/microduck.json','utf8').replace(/^\uFEFF/,'')));
const initial=profile.joints.map(j=>Math.max(j.limits[0],Math.min(0,j.limits[1])));
function create(p:RobotProfile){return rumoca.WasmSimulationSession.withInteractiveOptions(modelicaSource(p),'RobotKinematics',.01,'rk-like',1e-8,1e-6,JSON.stringify(p.joints.map((_,i)=>[`q${i}`,initial[i]??0])));}
beforeAll(async()=>{await init({module_or_path:readFileSync(new URL('./rumoca_bind_wasm_bg.wasm',import.meta.resolve('@cognipilot/rumoca')))});});
it('compiles the real Microduck tree in Rumoca and solves a reachable foot target',()=>{
 const session=create(profile);
 try{
  const frames=evaluate(session,profile,initial);expect(frames).toHaveLength(profile.links.length+profile.sites.length);
  expect(profile.joints).toHaveLength(14);
  const site=profile.sites.findIndex(s=>s.name==='left_foot'),index=profile.links.length+site;
  const desired=initial.slice();const active=chain(profile,'left_foot');desired[active[2]]=.12;desired[active[3]]=-.1;
  const target=evaluate(session,profile,desired)[index].position;
  const result=solveIK(q=>evaluate(session,profile,q)[index].position,initial,profile.joints.map(j=>j.limits),active,target);
  expect(result.converged).toBe(true);expect(result.error).toBeLessThan(.0005);
  expect(result.q.every((q,i)=>q>=profile.joints[i].limits[0]&&q<=profile.joints[i].limits[1])).toBe(true);
  const impossible=solveIK(q=>evaluate(session,profile,q)[index].position,initial,profile.joints.map(j=>j.limits),active,[100,100,100]);
  expect(impossible.converged).toBe(false);expect(Number.isFinite(impossible.error)).toBe(true);
  expect(()=>evaluate(session,profile,initial.map(()=>NaN))).toThrow();
 }finally{session.free();}
},120000);
it('matches a known one-joint transform including an offset pivot',()=>{
 const p:RobotProfile={version:1,name:'fixture',description:'',joints:[{name:'hinge',axis:[0,0,1],position:[.1,0,0],limits:[-3.2,3.2]}],links:[{name:'root',parent:null,position:[0,0,0],quaternion:[1,0,0,0],joint:'hinge'}],sites:[{name:'tip',link:'root',position:[1,0,0],quaternion:[1,0,0,0]}]};
 const session=create(p);try{const frame=evaluate(session,p,[Math.PI/2])[1];expect(frame.position[0]).toBeCloseTo(.1,8);expect(frame.position[1]).toBeCloseTo(.9,8);}finally{session.free();}
});
it('rejects invalid tree and joint limits before compilation',()=>{
 expect(()=>validateProfile({...profile,links:[{...profile.links[0],parent:'missing'}]})).toThrow();
 expect(()=>validateProfile({...profile,joints:[{...profile.joints[0],limits:[1,-1]}]})).toThrow();
});


import {BoxGeometry,Mesh,MeshBasicMaterial} from 'three';
import {mountFrame,sampleSensor} from '../src/robotics/sensors';
it('rotates sensor offsets with the parent and measures known scene ranges',()=>{
 const frame=mountFrame({frame:{position:[1,2,3],rotation:[0,-1,0,1,0,0,0,0,1]},offset:[.2,0,0],rpy:[0,0,0]});
 expect(frame.position).toEqual([1,2.2,3]);
 const wall=new Mesh(new BoxGeometry(.1,2,2),new MeshBasicMaterial());wall.position.set(1,0,0);wall.updateMatrixWorld(true);
 const capture=sampleSensor({position:[0,0,0],rotation:[1,0,0,0,1,0,0,0,1]},'lidar',[wall]);
 expect(capture.ranges[0]).toBeCloseTo(.95,6);expect(capture.ranges[64]).toBe(null);
});
