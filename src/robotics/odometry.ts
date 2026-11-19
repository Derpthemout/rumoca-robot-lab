import type {Point2} from './mapping';
export interface Pose2 {x:number;y:number;yaw:number}
export interface RangeScan {ranges:(number|null)[];maxRange:number}
export interface TrackingResult {pose:Pose2;rmse:number;overlap:number;matches:number;status:'initialized'|'tracked'}
const transform=(p:Point2,t:Pose2):Point2=>[t.x+Math.cos(t.yaw)*p[0]-Math.sin(t.yaw)*p[1],t.y+Math.sin(t.yaw)*p[0]+Math.cos(t.yaw)*p[1]];
function points(scan:RangeScan):Point2[]{
 if(scan.ranges.length!==128||!Number.isFinite(scan.maxRange)||scan.maxRange<=0||scan.maxRange>10||scan.ranges.some(v=>v!==null&&(!Number.isFinite(v)||v<=0||v>scan.maxRange)))throw Error('Invalid odometry ranges');
 return scan.ranges.flatMap((v,i)=>v===null?[]:[[v*Math.cos(2*Math.PI*i/128),v*Math.sin(2*Math.PI*i/128)] as Point2]);
}
function spread(p:Point2[]){
 const mean=p.reduce((s,v)=>[s[0]+v[0]/p.length,s[1]+v[1]/p.length],[0,0]);let xx=0,xy=0,yy=0;
 for(const v of p){const x=v[0]-mean[0],y=v[1]-mean[1];xx+=x*x;xy+=x*y;yy+=y*y;}
 const trace=xx+yy;return trace===0?0:(trace-Math.sqrt((xx-yy)**2+4*xy*xy))/(2*trace);
}
/** Local point-to-polyline ICP. No truth poses or world hit points enter updates. */
export class LidarOdometry {
 private reference:Point2[]=[];private pose:Pose2={x:0,y:0,yaw:0};
 reset(){this.reference=[];this.pose={x:0,y:0,yaw:0};}
 initialize(scan:RangeScan,anchor:Pose2):TrackingResult{
  const p=points(scan);if(!Object.values(anchor).every(Number.isFinite))throw Error('Invalid initial anchor');
  if(p.length<12||spread(p)<.005)throw Error('Insufficient non-collinear LiDAR geometry');
  this.pose={...anchor};this.reference=p.map(v=>transform(v,anchor));return {pose:{...anchor},rmse:0,overlap:1,matches:p.length,status:'initialized'};
 }
 update(scan:RangeScan):TrackingResult{
  if(!this.reference.length)throw Error('Initialize tracking first');const p=points(scan);
  if(p.length<12||spread(p)<.005)throw Error('Tracking lost: insufficient non-collinear returns');
  let estimate={...this.pose};
  const correspond=(t:Pose2)=>p.map(v=>{const a=transform(v,t);let d=.15**2,b:Point2|undefined;for(let i=0;i<this.reference.length;i++){
    const r=this.reference[i],next=this.reference[(i+1)%this.reference.length],vx=next[0]-r[0],vy=next[1]-r[1],length=vx*vx+vy*vy;
    const u=length>1e-10&&length<.2**2?Math.max(0,Math.min(1,((a[0]-r[0])*vx+(a[1]-r[1])*vy)/length)):0;
    const candidate:Point2=[r[0]+u*vx,r[1]+u*vy],distance=(a[0]-candidate[0])**2+(a[1]-candidate[1])**2;if(distance<d){d=distance;b=candidate;}
   }return b?{a,b,d}:null;}).filter(v=>v!==null);
  for(let iteration=0;iteration<40;iteration++){
   const all=correspond(estimate);if(all.length<12||all.length/p.length<.5)throw Error('Tracking lost: insufficient scan overlap');
   const pairs=all.sort((a,b)=>a.d-b.d).slice(0,Math.max(12,Math.floor(all.length*.85)));
   const ac:Point2=[0,0],bc:Point2=[0,0];for(const {a,b} of pairs){ac[0]+=a[0]/pairs.length;ac[1]+=a[1]/pairs.length;bc[0]+=b[0]/pairs.length;bc[1]+=b[1]/pairs.length;}
   if(spread(pairs.map(v=>v.b))<.005)throw Error('Tracking lost: ambiguous matched geometry');
   let dot=0,cross=0;for(const {a,b} of pairs){const x=a[0]-ac[0],y=a[1]-ac[1],u=b[0]-bc[0],v=b[1]-bc[1];dot+=x*u+y*v;cross+=x*v-y*u;}
   const yaw=Math.atan2(cross,dot),c=Math.cos(yaw),s=Math.sin(yaw),dx=bc[0]-c*ac[0]+s*ac[1],dy=bc[1]-s*ac[0]-c*ac[1];
   estimate={x:c*estimate.x-s*estimate.y+dx,y:s*estimate.x+c*estimate.y+dy,yaw:estimate.yaw+yaw};
   if(Math.hypot(dx,dy)<1e-6&&Math.abs(yaw)<1e-6)break;
  }
  const matched=correspond(estimate),rmse=Math.sqrt(matched.reduce((s,p)=>s+p.d,0)/matched.length),overlap=matched.length/p.length;
  if(!Number.isFinite(rmse)||rmse>.04||overlap<.5||Math.hypot(estimate.x-this.pose.x,estimate.y-this.pose.y)>.12||Math.abs(estimate.yaw-this.pose.yaw)>.2)throw Error('Tracking lost: residual or motion exceeds local tracking bounds');
  this.pose=estimate;this.reference=p.map(v=>transform(v,estimate));
  return {pose:{...estimate},rmse,overlap,matches:matched.length,status:'tracked'};
 }
}
