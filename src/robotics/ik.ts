import {Matrix3,Vector3} from 'three';
import type {Vec3} from './model';
export interface IKResult {q:number[];error:number;iterations:number;converged:boolean}
/** Bounded damped least squares; FK is supplied by the Rumoca session. */
export function solveIK(fk:(q:number[])=>Vec3,initial:number[],limits:readonly [number,number][],active:number[],target:Vec3):IKResult {
 if(!target.every(Number.isFinite)||initial.length!==limits.length||active.some(i=>!Number.isInteger(i)||i<0||i>=initial.length))throw Error('Invalid IK inputs');
 let q=initial.slice(),pos=fk(q),error=Math.hypot(...pos.map((v,i)=>target[i]-v));let iteration=0;
 for(;iteration<60&&error>0.0005;iteration++){
  const jac=active.map(index=>{const up=limits[index][1]-q[index],down=q[index]-limits[index][0];const h=up>=down?Math.min(1e-5,up):-Math.min(1e-5,down);if(Math.abs(h)<1e-14)return [0,0,0];const pert=q.slice();pert[index]+=h;const p=fk(pert);return p.map((v,k)=>(v-pos[k])/h);});
  const a=Array.from({length:3},(_,r)=>Array.from({length:3},(_,c)=>jac.reduce((sum,j)=>sum+j[r]*j[c],0)+(r===c?0.000025:0)));
  const m=new Matrix3().set(...a.flat() as [number,number,number,number,number,number,number,number,number]).invert();
  const v=new Vector3(...target).sub(new Vector3(...pos)).applyMatrix3(m).toArray();
  const delta=jac.map(j=>Math.max(-.2,Math.min(.2,j.reduce((sum,x,k)=>sum+x*v[k],0))));
  let improved=false;
  for(let scale=1;scale>=1/32;scale/=2){
   const candidate=q.slice();active.forEach((index,j)=>{candidate[index]=Math.max(limits[index][0],Math.min(limits[index][1],q[index]+scale*delta[j]));});
   const next=fk(candidate),distance=Math.hypot(...next.map((v,i)=>target[i]-v));
   if(distance<error-1e-10){q=candidate;pos=next;error=distance;improved=true;break;}
  }
  if(!improved)break;
 }
 fk(q);return {q,error,iterations:iteration,converged:error<=0.0005};
}
