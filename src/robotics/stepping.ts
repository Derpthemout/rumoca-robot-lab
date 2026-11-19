import type {WasmSimulationSession} from '@cognipilot/rumoca';
import {RobotRuntime,type RobotCompiler,type RobotSnapshot} from './motion';
import {chain,type RobotProfile,type Vec3} from './model';
export function contactSource(initialZ:number){
 if(!Number.isFinite(initialZ))throw Error('Invalid initial height');
 return `model GroundContact
 input Real leftZ; input Real rightZ; input Real leftV; input Real rightV;
 input Real targetX; input Real targetY;
 Real x(start=0,fixed=true); Real y(start=0,fixed=true); Real z(start=${initialZ},fixed=true);
 Real vx(start=0,fixed=true); Real vy(start=0,fixed=true); Real vz(start=0,fixed=true);
 output Real leftN; output Real rightN; output Real fx; output Real fy;
 Real ax; Real ay; Real scale;
equation
 der(x)=vx; der(y)=vy; der(z)=vz;
 leftN=if z+leftZ<0 then max(0,-1500*(z+leftZ)-20*(vz+leftV)) else 0;
 rightN=if z+rightZ<0 then max(0,-1500*(z+rightZ)-20*(vz+rightV)) else 0;
 ax=0.6*(40*(targetX-x)-12*vx); ay=0.6*(40*(targetY-y)-12*vy);
 scale=min(1,0.6*(leftN+rightN)/max(0.000001,sqrt(ax*ax+ay*ay)));
 fx=scale*ax; fy=scale*ay;
 0.6*der(vx)=fx; 0.6*der(vy)=fy; 0.6*der(vz)=leftN+rightN-0.6*9.81;
end GroundContact;`;
}
export interface ContactState {time:number;x:number;y:number;z:number;vx:number;vy:number;vz:number;leftN:number;rightN:number;fx:number;fy:number}
export class GroundContact {
 readonly source:string;private session:WasmSimulationSession;
 constructor(compiler:RobotCompiler,initialZ:number,leftZ:number,rightZ:number){this.source=contactSource(initialZ);this.session=compiler.WasmSimulationSession.withInteractiveOptions(this.source,'GroundContact',.0005,'rk-like',1e-9,1e-7,JSON.stringify([['leftZ',leftZ],['rightZ',rightZ],['leftV',0],['rightV',0],['targetX',0],['targetY',0]]));}
 step(leftZ:number,rightZ:number,leftV:number,rightV:number,target: [number,number],dt=.02){
  if(![leftZ,rightZ,leftV,rightV,...target,dt].every(Number.isFinite)||dt<=0||dt>.02)throw Error('Invalid contact input');
  this.session.set_inputs(JSON.stringify([['leftZ',leftZ],['rightZ',rightZ],['leftV',leftV],['rightV',rightV],['targetX',target[0]],['targetY',target[1]]]));
  this.session.advance_to(Math.round((this.session.time()+dt)*1e12)/1e12);return this.snapshot();
 }
 snapshot():ContactState{const s=JSON.parse(this.session.state_json()),v=s.values;const state={time:s.time,...Object.fromEntries(['x','y','z','vx','vy','vz','leftN','rightN','fx','fy'].map(k=>[k,v[k]]))} as ContactState;if(!Object.values(state).every(Number.isFinite))throw Error('Nonfinite contact state');return state;}
 free(){this.session.free();}
}
export interface SteppingState {robot:RobotSnapshot;base:ContactState;phase:string;steps:number;supportMargin:number;feet:Vec3[];desiredFeet:Vec3[];ikError:number;done:boolean}
/** Reduced-order centroidal contact experiment. Orientation is constrained upright.
 * Joint servos and the contact body are co-simulated at 20 ms, not coupled multibody dynamics. */
export class SteppingExperiment {
 readonly runtime:RobotRuntime;readonly contact:GroundContact;
 private sites=['left_foot','right_foot'];private indices:number[];private footTargets:Vec3[];private previous:Vec3[];
 private nominalZ=0;private phase='settle';private phaseTime=0;private steps=0;private swing=0;private start:Vec3=[0,0,0];private last:SteppingState;private fault=false;
 constructor(compiler:RobotCompiler,profile:RobotProfile){
  if(!this.sites.every(name=>profile.sites.some(s=>s.name===name)))throw Error('Stepping requires left_foot and right_foot sites');
  this.runtime=new RobotRuntime(compiler,profile);let initial=this.runtime.snapshot();this.indices=this.sites.map(name=>profile.links.length+profile.sites.findIndex(s=>s.name===name));
    // Begin from a modest crouch and narrower stance, within the imported joint limits.
  for(let i=0;i<2;i++){const p=initial.frames[this.indices[i]].position;const target:Vec3=[p[0],p[1]+(i===0?-.015:.015),p[2]+.012];const solved=this.runtime.solve(this.sites[i],target,false);if(solved.solution.error>.002){this.runtime.free();throw Error('Profile cannot reach the initial crouched stance');}initial=this.runtime.snapshot();}
  this.previous=this.indices.map(i=>initial.frames[i].position.slice() as Vec3);this.footTargets=this.previous.map(p=>[p[0],p[1],0]);
  this.nominalZ=-Math.min(...this.previous.map(p=>p[2]));try{this.contact=new GroundContact(compiler,.01-Math.min(...this.previous.map(p=>p[2])),this.previous[0][2],this.previous[1][2]);}catch(error){this.runtime.free();throw error;}
  this.last=this.makeState(initial,this.contact.snapshot(),0);this.start=this.footTargets[0].slice() as Vec3;
 }
 private makeState(robot:RobotSnapshot,base:ContactState,ikError:number):SteppingState{
  const frames=robot.frames.map(f=>({...f,position:[f.position[0]+base.x,f.position[1]+base.y,f.position[2]+base.z] as Vec3})),feet=this.indices.map(i=>frames[i].position);
  const active=feet.filter((_,i)=>(i===0?base.leftN:base.rightN)>.3);
  const margin=active.length?Math.min(base.x-Math.min(...active.map(p=>p[0]-.03)),Math.max(...active.map(p=>p[0]+.03))-base.x,base.y-Math.min(...active.map(p=>p[1]-.018)),Math.max(...active.map(p=>p[1]+.018))-base.y):-1;
  return {robot:{...robot,frames},base,feet,desiredFeet:this.footTargets.map(p=>p.slice() as Vec3),phase:this.phase,steps:this.steps,supportMargin:margin,ikError,done:this.phase==='complete'};
 }
 snapshot(){return this.last;}
 step():SteppingState{
  if(this.fault)throw Error('Stepping faulted; reset experiment');if(this.last.done)return this.last;
  try{return this.advance();}catch(error){this.fault=true;throw error;}
 }
 private advance(){
  const dt=.02;this.phaseTime+=dt;const base=this.contact.snapshot();let error=0;
  const support=1-this.swing;
  const target: [number,number]=this.phase==='settle'||this.phase==='complete'?[(this.footTargets[0][0]+this.footTargets[1][0])/2,(this.footTargets[0][1]+this.footTargets[1][1])/2]:[this.footTargets[support][0]+.016,this.footTargets[support][1]-Math.sign(this.footTargets[support][1])*.016];
  if(this.phase==='swing'){
   const t=Math.min(1,this.phaseTime/1.2),smooth=t*t*(3-2*t);
   this.footTargets[this.swing]=[this.start[0]+.015*smooth,this.start[1],.012*Math.sin(Math.PI*t)**2];
   if(t>=1){this.phase='touchdown';this.phaseTime=0;}
  }
  if(this.phase!=='settle'){
   const total=Math.max(1,Math.min(10,.6*(9.81+30*(this.nominalZ-base.z)-8*base.vz)));
   const swingWeight=this.phase==='swing'?0:this.phase==='touchdown'?.5:.5*(1-Math.min(1,this.phaseTime/.8));
   this.footTargets[support][2]=-total*(1-swingWeight)/1500;
   if(this.phase!=='swing')this.footTargets[this.swing][2]=-total*swingWeight/1500;
   const command=this.runtime.snapshot().q.slice();
   this.sites.forEach((name,i)=>{const p=this.footTargets[i],solution=this.runtime.solve(name,[p[0]-base.x,p[1]-base.y,p[2]-base.z]).solution;error=Math.max(error,solution.error);for(const j of chain(this.runtime.profile,name))command[j]=solution.q[j];});
   if(error>.005)throw Error('Foot IK error exceeded 5 mm; experiment stopped');this.runtime.setCommand(command);
  }
  const robot=this.runtime.step(dt),local=this.indices.map(i=>robot.frames[i].position);
  const next=this.contact.step(local[0][2],local[1][2],(local[0][2]-this.previous[0][2])/dt,(local[1][2]-this.previous[1][2])/dt,target,dt);this.previous=local.map(p=>p.slice() as Vec3);
  if(Math.abs(next.vz)>1||next.z<-.15)throw Error('Ground support lost; experiment stopped');
  if(this.phase==='settle'&&this.phaseTime>=1){this.phase='shift';this.phaseTime=0;}
  else if(this.phase==='shift'){
   const foot=[local[support][0]+next.x,local[support][1]+next.y],normal=support===0?next.leftN:next.rightN;
   if(this.phaseTime>.8&&normal>.3&&Math.abs(next.x-foot[0])<.027&&Math.abs(next.y-foot[1])<.0175){this.phase='swing';this.phaseTime=0;this.start=this.footTargets[this.swing].slice() as Vec3;}
   else if(this.phaseTime>4)throw Error('Support transfer timed out; foot was not lifted');
  }else if(this.phase==='touchdown'){
   const normal=this.swing===0?next.leftN:next.rightN;
   if(this.phaseTime>.3&&normal>.3){this.steps++;this.swing=1-this.swing;this.phase=this.steps>=4?'complete':'shift';this.phaseTime=0;}
   else if(this.phaseTime>3)throw Error('Touchdown timed out');
  }
  this.last=this.makeState(robot,next,error);return this.last;
 }
 free(){this.runtime.free();this.contact.free();}
}
