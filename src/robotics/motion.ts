import type {WasmSimulationSession} from '@cognipilot/rumoca';
import {chain,evaluate,modelicaSource,validateProfile,type Frame,type RobotProfile,type Vec3} from './model';
import {solveIK} from './ik';

export interface ServoParameters {
  inertia:number; kp:number; kd:number; damping:number; maxTorque:number;
  stopStiffness:number; stopDamping:number;
}
/** Teaching parameters, not identified Microduck motor or rigid-body parameters. */
export const DEFAULT_SERVO:Readonly<ServoParameters>=Object.freeze({
  inertia:.002,kp:1,kd:.04,damping:.05,maxTorque:.75,stopStiffness:20,stopDamping:.2,
});
export function validateServo(p:ServoParameters):ServoParameters {
  const bounds:Record<keyof ServoParameters,[number,number]>={inertia:[.0001,.1],kp:[.01,10],kd:[0,2],damping:[.001,2],maxTorque:[.01,5],stopStiffness:[1,100],stopDamping:[0,5]};
  for(const key of Object.keys(bounds) as (keyof ServoParameters)[]){
    if(typeof p?.[key]!=='number'||!Number.isFinite(p[key])||p[key]<bounds[key][0]||p[key]>bounds[key][1])throw Error(`Invalid servo parameter: ${key}`);
  }
  return Object.fromEntries(Object.keys(bounds).map(key=>[key,p[key as keyof ServoParameters]])) as unknown as ServoParameters;
}
export function validatePose(p:RobotProfile,q:unknown):asserts q is number[]{
  if(!Array.isArray(q)||q.length!==p.joints.length||q.some((v,i)=>typeof v!=='number'||!Number.isFinite(v)||v<p.joints[i].limits[0]||v>p.joints[i].limits[1]))throw Error('Joint command outside limits');
}
export function homePose(p:RobotProfile){return p.joints.map(j=>Math.max(j.limits[0],Math.min(0,j.limits[1])));}

/** Second-order decoupled joint servos with torque saturation and compliant stops.
 * All differential equations and frame transforms execute inside Rumoca. */
export function servoSource(p:RobotProfile,initial:number[],parameters:ServoParameters=DEFAULT_SERVO):string {
  validatePose(p,initial);const s=validateServo(parameters);
  let source=modelicaSource(p).replaceAll('RobotKinematics','RobotServos');
  const equations:string[]=[];
  p.joints.forEach((joint,i)=>{
    source=source.replace(`input Real q${i};`,[
      `input Real command${i};`,
      `output Real q${i}(start=${initial[i]},fixed=true);`,
      `output Real velocity${i}(start=0,fixed=true);`,
      `output Real torque${i};`,
      `Real stop${i};`,
    ].join('\n'));
    const [lo,hi]=joint.limits;
    equations.push(
      `der(q${i}) = velocity${i};`,
      `torque${i} = max(-${s.maxTorque}, min(${s.maxTorque}, ${s.kp}*(command${i}-q${i})-${s.kd}*velocity${i}));`,
      `stop${i} = if q${i} < (${lo}) then ${s.stopStiffness}*((${lo})-q${i})-${s.stopDamping}*min(0,velocity${i}) else if q${i} > (${hi}) then ${s.stopStiffness}*((${hi})-q${i})-${s.stopDamping}*max(0,velocity${i}) else 0;`,
      `${s.inertia}*der(velocity${i}) = torque${i}-${s.damping}*velocity${i}+stop${i};`,
    );
  });
  return source.replace('end RobotServos;',equations.join('\n')+'\nend RobotServos;');
}
export interface RobotSnapshot {
  time:number;q:number[];velocity:number[];torque:number[];command:number[];frames:Frame[];
  mode:'kinematic'|'servo';limitPenetration:number;
}
export type RobotCompiler=Pick<typeof import('@cognipilot/rumoca'),'WasmSimulationSession'>;

export class RobotRuntime {
  readonly profile:RobotProfile;
  private fk:WasmSimulationSession;
  private motion?:WasmSimulationSession;
  private pose:number[];
  private command:number[];
  private initial:number[];
  private parameters:ServoParameters={...DEFAULT_SERVO};
  private fault=false;
  motionSource='';
  readonly kinematicSource:string;
  constructor(private compiler:RobotCompiler,profile:RobotProfile){
    this.profile=validateProfile(profile);this.initial=homePose(this.profile);this.pose=this.initial.slice();this.command=this.initial.slice();
    this.kinematicSource=modelicaSource(this.profile);
    this.fk=compiler.WasmSimulationSession.withInteractiveOptions(this.kinematicSource,'RobotKinematics',.01,'rk-like',1e-8,1e-6,JSON.stringify(this.pose.map((v,i)=>[`q${i}`,v])));
    try{this.snapshot();}catch(error){this.fk.free();throw error;}
  }
  free(){this.motion?.free();this.motion=undefined;this.fk.free();}
  setPose(q:number[]){
    validatePose(this.profile,q);
    // Evaluate before replacing the current experiment, so invalid data is harmless.
    const frames=evaluate(this.fk,this.profile,q);
    this.motion?.free();this.motion=undefined;this.motionSource='';this.fault=false;
    this.pose=q.slice();this.command=q.slice();this.initial=q.slice();
    return {time:0,q:q.slice(),velocity:q.map(()=>0),torque:q.map(()=>0),command:q.slice(),frames,mode:'kinematic' as const,limitPenetration:0};
  }
  setCommand(q:number[]){validatePose(this.profile,q);if(this.fault)throw Error('Simulation faulted; reset before continuing');this.command=q.slice();return this.snapshot();}
  configure(parameters:ServoParameters){
    const next=validateServo(parameters);
    // Reconfiguration intentionally starts a new episode at the current pose.
    const current=this.snapshot();validatePose(this.profile,current.q);
    const source=servoSource(this.profile,current.q,next);
    const candidate=this.compileMotion(source,current.q);
    this.motion?.free();this.motion=candidate;this.motionSource=source;this.parameters=next;
    this.initial=current.q.slice();this.pose=current.q.slice();this.command=current.q.slice();this.fault=false;
    return this.snapshot();
  }
  private compileMotion(source:string,initial:number[]){
    return this.compiler.WasmSimulationSession.withInteractiveOptions(source,'RobotServos',.001,'rk-like',1e-9,1e-7,JSON.stringify(initial.map((v,i)=>[`command${i}`,v])));
  }
  private ensureMotion(){
    if(this.motion)return;
    this.motionSource=servoSource(this.profile,this.pose,this.parameters);
    this.motion=this.compileMotion(this.motionSource,this.pose);this.initial=this.pose.slice();
  }
  reset(){
    this.motion?.reset();this.pose=this.initial.slice();this.command=this.initial.slice();this.fault=false;
    this.motion?.set_inputs(JSON.stringify(this.command.map((v,i)=>[`command${i}`,v])));
    return this.snapshot();
  }
  step(dt=.02){
    if(!Number.isFinite(dt)||dt<=0||dt>.1)throw Error('Step must be greater than zero and at most 0.1 seconds');
    if(this.fault)throw Error('Simulation faulted; reset before continuing');
    this.ensureMotion();
    try{
      const goal=Math.round((this.motion!.time()+dt)*1e12)/1e12;
      this.motion!.set_inputs(JSON.stringify(this.command.map((v,i)=>[`command${i}`,v])));
      this.motion!.advance_to(goal);
      const state=this.snapshot();
      if(Math.abs(state.time-goal)>1e-9)throw Error('Rumoca did not reach the requested timestamp');
      if(state.limitPenetration>.05)throw Error('Compliant joint stop exceeded 0.05 radians; reset and reduce gains');
      this.pose=state.q.slice();return state;
    }catch(error){this.fault=true;throw error;}
  }
  snapshot():RobotSnapshot {
    if(!this.motion)return {time:0,q:this.pose.slice(),velocity:this.pose.map(()=>0),torque:this.pose.map(()=>0),command:this.command.slice(),frames:evaluate(this.fk,this.profile,this.pose),mode:'kinematic',limitPenetration:0};
    const {time,values}=JSON.parse(this.motion.state_json());
    if(!Number.isFinite(time))throw Error('Nonfinite simulation time');
    const read=(name:string)=>{const value=values[name];if(typeof value!=='number'||!Number.isFinite(value))throw Error(`Nonfinite simulation output: ${name}`);return value;};
    const q=this.profile.joints.map((_,i)=>read(`q${i}`));
    const frames=[...this.profile.links,...this.profile.sites].map((_,i)=>({position:[1,2,3].map(k=>read(`p${i}[${k}]`)) as Vec3,rotation:[1,2,3].flatMap(r=>[1,2,3].map(c=>read(`r${i}[${r},${c}]`)))}));
    return {time,q,velocity:q.map((_,i)=>read(`velocity${i}`)),torque:q.map((_,i)=>read(`torque${i}`)),command:this.command.slice(),frames,mode:'servo',limitPenetration:Math.max(0,...q.map((v,i)=>Math.max(this.profile.joints[i].limits[0]-v,v-this.profile.joints[i].limits[1])))};
  }
  positionFor(q:number[],site:string):Vec3 {
    validatePose(this.profile,q);const index=this.profile.sites.findIndex(s=>s.name===site);if(index<0)throw Error('Unknown site');
    return evaluate(this.fk,this.profile,q)[this.profile.links.length+index].position;
  }
  solve(site:string,target:Vec3,applyAsCommand=true){
    const index=this.profile.sites.findIndex(s=>s.name===site);if(index<0)throw Error('Unknown end effector');
    const current=this.snapshot();
    const start=current.q.map((v,i)=>Math.max(this.profile.joints[i].limits[0],Math.min(v,this.profile.joints[i].limits[1])));
    const solution=solveIK(q=>evaluate(this.fk,this.profile,q)[this.profile.links.length+index].position,start,this.profile.joints.map(j=>j.limits),chain(this.profile,site),target);
    const snapshot=applyAsCommand?this.setCommand(solution.q):this.setPose(solution.q);
    return {...snapshot,solution:{q:solution.q,error:solution.error,iterations:solution.iterations,converged:solution.converged}};
  }
}
