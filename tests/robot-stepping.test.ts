import {beforeAll,it,expect} from 'vitest';
import {readFileSync} from 'node:fs';
import init,* as rumoca from '@cognipilot/rumoca';
import {GroundContact,SteppingExperiment} from '../src/robotics/stepping';
import {validateProfile} from '../src/robotics/model';
const profile=validateProfile(JSON.parse(readFileSync('public/robots/microduck.json','utf8').replace(/^\uFEFF/,'')));
beforeAll(async()=>{await init({module_or_path:readFileSync(new URL('./rumoca_bind_wasm_bg.wasm',import.meta.resolve('@cognipilot/rumoca')))});});
it('falls under gravity, settles on compliant contacts, and bounds tangential force',()=>{
 const sim=new GroundContact(rumoca,.02,0,0);
 try{const first=sim.step(0,0,0,0,[0,0]);expect(first.vz).toBeLessThan(0);expect(first.leftN).toBe(0);let s=first;for(let i=0;i<150;i++)s=sim.step(0,0,0,0,[0,0]);expect(s.leftN+s.rightN).toBeCloseTo(.6*9.81,2);expect(Math.abs(s.vz)).toBeLessThan(.001);
 s=sim.step(0,0,0,0,[1,1]);expect(Math.hypot(s.fx,s.fy)).toBeLessThanOrEqual(.6*(s.leftN+s.rightN)+1e-8);
 }finally{sim.free();}
},120000);
it('performs four support-gated Microduck steps with finite contact and joint states',()=>{
 const sim=new SteppingExperiment(rumoca,profile);let maxLift=0,maxIK=0,s=sim.snapshot();const initialX=s.feet.map(p=>p[0]);const lifts=[0,0];let transitions=0;
 try{for(let i=0;i<1000&&!s.done;i++){const previous=s;s=sim.step();maxIK=Math.max(maxIK,s.ikError);if(s.phase==='swing'){maxLift=Math.max(maxLift,...s.feet.map(p=>p[2]));s.feet.forEach((p,i)=>{lifts[i]=Math.max(lifts[i],p[2]);});if(previous.phase!=='swing')transitions++;}expect(s.base.leftN).toBeGreaterThanOrEqual(0);expect(s.base.rightN).toBeGreaterThanOrEqual(0);}
 console.log(JSON.stringify({steps:s.steps,phase:s.phase,time:s.base.time,maxLift,maxIK,lifts,transitions,feet:s.feet,base:s.base}));expect(s.done).toBe(true);expect(s.steps).toBe(4);expect(maxLift).toBeGreaterThan(.005);expect(lifts.every(v=>v>.005)).toBe(true);expect(transitions).toBe(4);expect(maxIK).toBeLessThan(.005);s.feet.forEach((p,i)=>expect(p[0]-initialX[i]).toBeGreaterThan(.02));
 }finally{sim.free();}
},120000);

it('removes contact forces when the feet leave the ground and rejects invalid inputs without advancing',()=>{
 const sim=new GroundContact(rumoca,-.002,0,0);
 try{const before=sim.snapshot();expect(()=>sim.step(NaN,0,0,0,[0,0])).toThrow();expect(sim.snapshot()).toEqual(before);
 const airborne=sim.step(1,1,0,0,[1,1]);expect(airborne.leftN+airborne.rightN).toBe(0);expect(airborne.fx).toBe(0);expect(airborne.fy).toBe(0);expect(airborne.vz).toBeCloseTo(-9.81*.02,6);
 }finally{sim.free();}
},120000);
