import {OccupancyMap,type PlanarScan} from './mapping';
import {LidarOdometry,type Pose2,type TrackingResult} from './odometry';
export class MappingSession {
 readonly tracker=new LidarOdometry();mode:'truth'|'estimated'='truth';private initialized=false;private height:number|undefined;private lost=false;
 readonly history:{time?:number;truth:Pose2;estimate:Pose2;positionError:number;yawError:number;rmse:number;overlap:number}[]=[];
 constructor(readonly map:OccupancyMap){}
 reset(mode=this.mode){this.mode=mode;this.map.clear();this.tracker.reset();this.history.length=0;this.initialized=false;this.height=undefined;this.lost=false;}
 integrate(scan:PlanarScan,time?:number){
  if(time!==undefined&&(!Number.isFinite(time)||time<0))throw Error('Invalid scan timestamp');
  if(this.lost)throw Error('Tracking stopped. Clear map to reinitialize; rejected scans are not fused.');
  if(this.history.length>=3000)throw Error('Tracking history full. Export and clear map.');
  // Validate horizontal scan geometry before changing the estimator or map.
  new OccupancyMap().integrate(scan);
  if(this.height!==undefined&&Math.abs(scan.frame.position[2]-this.height)>.005)throw Error('Scan height changed. Clear map first.');
  const truth={x:scan.frame.position[0],y:scan.frame.position[1],yaw:Math.atan2(scan.frame.rotation[3],scan.frame.rotation[0])};
  if(this.mode==='truth'){this.map.integrate(scan);this.height??=scan.frame.position[2];return undefined;}
  let result:TrackingResult;
  try{
   // Only the first scan gets a known anchor. Subsequent optimizer inputs are ranges only.
   result=this.initialized?this.tracker.update({ranges:scan.ranges,maxRange:scan.maxRange}):this.tracker.initialize({ranges:scan.ranges,maxRange:scan.maxRange},truth);
   const {x,y,yaw}=result.pose,c=Math.cos(yaw),s=Math.sin(yaw);
   this.map.integrate({...scan,frame:{position:[x,y,scan.frame.position[2]],rotation:[c,-s,0,s,c,0,0,0,1]}});
  }catch(error){this.lost=true;throw error;}
  this.initialized=true;this.height??=scan.frame.position[2];
  const record={time,truth,estimate:result.pose,positionError:Math.hypot(result.pose.x-truth.x,result.pose.y-truth.y),yawError:Math.abs(Math.atan2(Math.sin(result.pose.yaw-truth.yaw),Math.cos(result.pose.yaw-truth.yaw))),rmse:result.rmse,overlap:result.overlap};
  this.history.push(record);return record;
 }
 export(){return {...this.map.export(),format:'rumoca-robot-occupancy',version:2,poseSource:this.mode==='truth'?'Rumoca ground truth':'range-only local LiDAR odometry; first pose anchored to Rumoca',trackingStatus:this.mode==='truth'?'ground-truth':this.lost?'lost':this.initialized?'tracking':'uninitialized',history:this.history,limitations:'horizontal slice; local odometry only; no loop closure, relocalization or walking'};}
}
