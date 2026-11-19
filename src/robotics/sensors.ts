import {Euler,Matrix4,Quaternion,Raycaster,Vector3,type Object3D} from 'three';
import type {Frame,Vec3} from './model';
export interface SensorMount {frame:Frame;offset:Vec3;rpy:Vec3}
export function mountFrame(mount:SensorMount):Frame {
 const r=mount.frame.rotation,p=mount.frame.position;
 const parent=new Matrix4().set(r[0],r[1],r[2],p[0],r[3],r[4],r[5],p[1],r[6],r[7],r[8],p[2],0,0,0,1);
 const local=new Matrix4().compose(new Vector3(...mount.offset),new Quaternion().setFromEuler(new Euler(...mount.rpy,'ZYX')),new Vector3(1,1,1));
 const e=parent.multiply(local).elements;
 return {position:[e[12],e[13],e[14]],rotation:[e[0],e[4],e[8],e[1],e[5],e[9],e[2],e[6],e[10]]};
}
/** Ideal geometric samples; +X forward, +Y left, +Z up. No simulated timing/noise. */
export function sampleSensor(frame:Frame,kind:'depth'|'lidar',obstacles:Object3D[],maxRange=3){
 const width=kind==='depth'?48:128,height=kind==='depth'?32:1,ray=new Raycaster(),origin=new Vector3(...frame.position),r=frame.rotation;
 const ranges:(number|null)[]=[],points:Vec3[]=[];
 for(let y=0;y<height;y++)for(let x=0;x<width;x++){
  const local=kind==='lidar'?new Vector3(Math.cos(2*Math.PI*x/width),Math.sin(2*Math.PI*x/width),0):new Vector3(1,(.5-(x+.5)/width)*2*Math.tan(Math.PI/6),(.5-(y+.5)/height)*2*Math.tan(Math.PI/6)*height/width).normalize();
  const d=new Vector3(r[0]*local.x+r[1]*local.y+r[2]*local.z,r[3]*local.x+r[4]*local.y+r[5]*local.z,r[6]*local.x+r[7]*local.y+r[8]*local.z);
  ray.set(origin,d);ray.near=.005;ray.far=maxRange;const hit=ray.intersectObjects(obstacles,false)[0];
  ranges.push(hit?hit.distance:null);if(hit)points.push(hit.point.toArray() as Vec3);
 }
 return {kind,width,height,maxRange,rangeConvention:'radial-metres',frame,points,ranges};
}
