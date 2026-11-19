/// <reference lib="webworker" />
import {compilerAsset} from '../compiler-assets';
import {SteppingExperiment} from './stepping';
import {sensorSweep} from './sensor-sweep';
import {RobotRuntime} from './motion';
import {runReachBaseline} from './reach-environment';
let stepping:SteppingExperiment|undefined;
let compiler:typeof import('@cognipilot/rumoca'),runtime:RobotRuntime|undefined;
async function execute(data:any){
  if(data.type==='init'){
    compiler??=await import(/* @vite-ignore */ compilerAsset(data.base,'js'));
    await compiler.default({module_or_path:compilerAsset(data.base,'wasm')});
    const next=new RobotRuntime(compiler,data.profile);runtime?.free();runtime=next;stepping?.free();stepping=undefined;
    return {...runtime.snapshot(),source:runtime.kinematicSource,compiler:compiler.get_version()};
  }
  if(!runtime)throw Error('Load a robot first');
  if(data.type==='startStepping'){const next=new SteppingExperiment(compiler,runtime.profile);stepping?.free();stepping=next;return {...stepping.snapshot(),contactSource:stepping.contact.source};}
  if(data.type==='stepGait'){if(!stepping)throw Error('Start stepping first');return {...stepping.step(),servoSource:stepping.runtime.motionSource};}
  if(data.type==='exitStepping'){stepping?.free();stepping=undefined;return runtime.snapshot();}
  if(data.type==='sensorSweep')return sensorSweep(compiler);
  if(data.type==='pose')return runtime.setPose(data.q);
  if(data.type==='target')return runtime.setCommand(data.q);
  if(data.type==='step')return {...runtime.step(data.dt),motionSource:runtime.motionSource};
  if(data.type==='resetMotion')return runtime.reset();
  if(data.type==='configureMotion')return {...runtime.configure(data.parameters),motionSource:runtime.motionSource};
  if(data.type==='solve')return runtime.solve(data.site,data.target,data.animate!==false);
  if(data.type==='benchmark'){
    const rollout=runReachBaseline(runtime,{seed:data.seed,site:data.site,goal:data.goal,horizon:150,dt:.02});
    return {...runtime.snapshot(),rollout,motionSource:runtime.motionSource};
  }
  throw Error('Unknown robot command');
}
let queue=Promise.resolve();
self.onmessage=({data})=>{queue=queue.then(async()=>{try{self.postMessage({id:data.id,result:await execute(data)});}catch(error){self.postMessage({id:data.id,error:String(error)});}});};
