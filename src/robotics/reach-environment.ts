import {chain,type Vec3} from './model';
import {homePose,RobotRuntime,type RobotSnapshot} from './motion';

export interface ReachOptions {seed:number;site:string;goal?:Vec3;horizon?:number;dt?:number}
export interface ReachObservation {q:number[];velocity:number[];goal:Vec3;endEffector:Vec3;time:number}
export interface ReachTransition {
  observation:ReachObservation;action:number[];reward:number;terminated:boolean;truncated:boolean;
  info:{distance:number;step:number;limitPenetration:number};
}
function random(seed:number){let state=seed>>>0;return ()=>{state=(Math.imul(state,1664525)+1013904223)>>>0;return state/4294967296;};}
/** Fixed-base reaching task. A training interface, not a locomotion environment. */
export class ReachEnvironment {
  private goal:Vec3=[0,0,0];private site='';private index=-1;private stepCount=0;
  private dt=.02;private horizon=150;private done=true;private previousDistance=0;
  constructor(readonly runtime:RobotRuntime){runtime.setPose(homePose(runtime.profile));}
  reset(options:ReachOptions):ReachObservation {
    if(!Number.isInteger(options.seed)||options.seed<0||options.seed>0xffffffff)throw Error('Seed must be a uint32');
    const siteIndex=this.runtime.profile.sites.findIndex(s=>s.name===options.site);
    if(siteIndex<0)throw Error('Unknown reaching site');
    const dt=options.dt??.02,horizon=options.horizon??150;
    if(!Number.isFinite(dt)||dt<=0||dt>.1||!Number.isInteger(horizon)||horizon<1||horizon>1000)throw Error('Invalid episode limits');
    if(options.goal&&(!Array.isArray(options.goal)||options.goal.length!==3||!options.goal.every(Number.isFinite)))throw Error('Goal must contain three finite coordinates');
    let goal=options.goal?.slice() as Vec3|undefined;
    if(!goal){
      const rng=random(options.seed),q=homePose(this.runtime.profile);
      for(const index of chain(this.runtime.profile,options.site)){const [lo,hi]=this.runtime.profile.joints[index].limits;q[index]=Math.max(lo,Math.min(hi,q[index]+(rng()-.5)*.5));}
      goal=this.runtime.positionFor(q,options.site);
    }
    const snapshot=this.runtime.reset();this.goal=goal;this.site=options.site;this.index=this.runtime.profile.links.length+siteIndex;
    this.dt=dt;this.horizon=horizon;this.stepCount=0;this.done=false;
    this.previousDistance=this.distance(snapshot);return this.observation(snapshot);
  }
  /** Absolute joint targets normalized to [-1, 1], mapped to each joint's limits. */
  step(action:number[]):ReachTransition {
    if(this.done)throw Error('Episode ended or not reset');
    if(!Array.isArray(action)||action.length!==this.runtime.profile.joints.length||action.some(v=>typeof v!=='number'||!Number.isFinite(v)||v< -1||v>1))throw Error('Action must have one finite value in [-1, 1] per joint');
    const command=action.map((v,i)=>{const [lo,hi]=this.runtime.profile.joints[i].limits;return lo+(v+1)*.5*(hi-lo);});
    this.runtime.setCommand(command);const snapshot=this.runtime.step(this.dt);this.stepCount++;
    const distance=this.distance(snapshot),settled=Math.max(0,...snapshot.velocity.map(Math.abs))<.05;
    const terminated=distance<.001&&settled;
    const truncated=!terminated&&this.stepCount>=this.horizon;
    const effort=snapshot.torque.reduce((sum,t)=>sum+t*t,0);
    const reward=this.previousDistance-distance-this.dt*(distance+.0001*effort)+(terminated?1:0);
    this.previousDistance=distance;this.done=terminated||truncated;
    return {observation:this.observation(snapshot),action:action.slice(),reward,terminated,truncated,info:{distance,step:this.stepCount,limitPenetration:snapshot.limitPenetration}};
  }
  baselineAction(){
    if(this.done)throw Error('Reset the environment first');
    const result=this.runtime.solve(this.site,this.goal);
    const action=result.solution.q.map((q,i)=>{const [lo,hi]=this.runtime.profile.joints[i].limits;return Math.max(-1,Math.min(1,2*(q-lo)/(hi-lo)-1));});
    return {action,ik:result.solution};
  }
  private distance(s:RobotSnapshot){return Math.hypot(...s.frames[this.index].position.map((v,i)=>this.goal[i]-v));}
  private observation(s:RobotSnapshot):ReachObservation {return {q:s.q.slice(),velocity:s.velocity.slice(),goal:this.goal.slice() as Vec3,endEffector:s.frames[this.index].position.slice() as Vec3,time:s.time};}
}
export function runReachBaseline(runtime:RobotRuntime,options:ReachOptions){
  const environment=new ReachEnvironment(runtime),initial=environment.reset(options),baseline=environment.baselineAction();
  const transitions:ReachTransition[]=[];
  do{transitions.push(environment.step(baseline.action));}while(!transitions.at(-1)!.terminated&&!transitions.at(-1)!.truncated);
  return {format:'rumoca-reach-rollout',version:1,task:'fixed-base-position-reaching',policy:'bounded-IK-target-hold (not learned)',seed:options.seed,site:options.site,dt:options.dt??.02,actionConvention:'absolute joint target normalized to [-1,1]',initial,ik:baseline.ik,transitions,
    summary:{steps:transitions.length,return:transitions.reduce((sum,t)=>sum+t.reward,0),finalDistance:transitions.at(-1)!.info.distance,terminated:transitions.at(-1)!.terminated,truncated:transitions.at(-1)!.truncated}};
}
