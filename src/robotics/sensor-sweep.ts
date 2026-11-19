import type {Frame} from './model';
import type {RobotCompiler} from './motion';
export const SWEEP_SOURCE=`model SensorSweep
 Real x(start=0,fixed=true);
 Real y(start=0,fixed=true);
 Real yaw(start=0,fixed=true);
 output Real c;
 output Real s;
equation
 der(x)=0.03;
 der(y)=0.01;
 der(yaw)=0.02;
 c=cos(yaw);
 s=sin(yaw);
end SensorSweep;`;
/** Virtual level sensor rig, independent of the fixed robot; not locomotion. */
export function sensorSweep(compiler:RobotCompiler){
 const session=compiler.WasmSimulationSession.withInteractiveOptions(SWEEP_SOURCE,'SensorSweep',.005,'rk-like',1e-9,1e-7,'[]');
 try{
  const samples:{time:number;frame:Frame}[]=[];
  for(let i=0;i<=40;i++){
   if(i)session.advance_to(i*.05);
   const state=JSON.parse(session.state_json()),v=state.values;
   if(![state.time,v.x,v.y,v.yaw,v.c,v.s].every(Number.isFinite))throw Error('Invalid Rumoca sweep state');
   samples.push({time:state.time,frame:{position:[v.x,v.y,.22],rotation:[v.c,-v.s,0,v.s,v.c,0,0,0,1]}});
  }return {source:SWEEP_SOURCE,samples};
 }finally{session.free();}
}
